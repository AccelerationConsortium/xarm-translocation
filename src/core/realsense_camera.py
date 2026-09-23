"""Intel RealSense depth camera attached to this device PC.

The D435i (and any other librealsense-supported camera) is *local* USB
hardware, unlike the lab PTZ camera in ``camera_tracker.py`` which is a
network camera driven through the dashboard. This module owns the
librealsense pipeline and hands the API server ready-to-serve artefacts:

* :meth:`RealSenseCamera.describe` -- device enumeration + stream state for
  ``GET /realsense/<id>/status`` and the ``details.realsense.cameras.<id>``
  block on ``/status``.
* :meth:`RealSenseCamera.jpeg` / :meth:`mjpeg_frames` -- colour or colourised
  depth as JPEG (snapshot) or a multipart MJPEG generator (live preview).
* :meth:`RealSenseCamera.depth_png` -- the raw 16-bit depth map, lossless,
  for downstream CV.
* :meth:`RealSenseCamera.depth_at` -- metric distance (and a camera-frame 3-D
  point) for one pixel, the primitive a "did the arm really get there"
  vision check or a plate-locator builds on.

Design constraints, shared with the other optional subsystems:

1. **Optional at every layer.** ``pyrealsense2`` is an extra
   (``uv sync --extra realsense``); when it is missing, or the YAML says
   ``enabled: false``, or no camera is plugged in, construction still
   succeeds and every accessor answers with a *reason* instead of raising
   into the arm's control path. ``/status`` omits the block entirely when
   the feature is unconfigured, so unmigrated deployments see no change.
2. **Never on the asyncio loop.** ``wait_for_frames`` blocks; a daemon
   capture thread owns the pipeline and publishes the latest
   :class:`FrameBundle` under a lock. Request handlers only read it.
3. **Injectable backend.** ``rs_module`` (the ``pyrealsense2`` namespace)
   and ``np_module`` are constructor parameters so the unit tests drive the
   whole lifecycle with a fake -- no hardware, no DLL.
4. **Camera health never changes ``equipment_status``.** Arm motion does
   not depend on the camera (yet), so an unplugged camera is reported on
   ``components.realsense_<id>`` + ``details.realsense`` and leaves the
   top-level state alone -- the same reasoning STATUS_SPEC §2.2 applies to
   the sash interlock being blind.
5. **Many cameras, addressed by a device-local id.** The YAML lists
   ``cameras:``; this module turns that list into a registry keyed by
   ``id`` (see :func:`configure_cameras`). The id is the first path segment
   of every route for that camera and the top directory of its captures,
   so it is validated against :data:`CAMERA_ID_RE` at load time rather than
   sanitised at every use. A malformed, duplicate or ambiguous entry is
   skipped with a logged reason -- a bad line in a config file must not stop
   the arm service from booting.

Configuration lives in ``src/settings/realsense.yaml``; see that file for
the field documentation.
"""

from __future__ import annotations

import io
import logging
import os
import re
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterator, List, Optional, Tuple

logger = logging.getLogger("xarm.realsense")

_DEVICE_LIST_TTL_S = 5.0     # enumeration is a USB round-trip; cache it

# A camera id is device-local, lands in URLs (/realsense/<id>/status) and in
# the capture store's directory layout, so it is deliberately narrow: no
# dots, no slashes, no upper case, nothing that could be read as a path.
CAMERA_ID_RE = re.compile(r"^[a-z0-9][a-z0-9_-]{0,31}$")
_STREAM_KINDS = ("color", "depth")
_MJPEG_BOUNDARY = "xarm-realsense-frame"


class RealSenseError(RuntimeError):
    """Base class for camera failures the API surfaces to the caller."""


class RealSenseUnavailable(RealSenseError):
    """Not installed / disabled / no camera plugged in. HTTP 503 material."""


class RealSenseNotStreaming(RealSenseError):
    """A frame was asked for while the pipeline is stopped. HTTP 409 material."""


@dataclass
class FrameBundle:
    """The most recent set of frames, already converted to numpy arrays."""

    color: Any                  # HxWx3 uint8 BGR, or None if colour is disabled
    depth: Any                  # HxW uint16 (units of depth_scale metres), or None
    depth_color: Any            # HxWx3 uint8 RGB colourised depth, or None
    depth_scale: float          # metres per depth unit
    intrinsics: Dict[str, Any]  # of the stream the depth map is expressed in
    frame_number: int
    timestamp_ms: float         # device timestamp
    captured_at: float          # time.monotonic() on this host


class RealSenseCamera:
    """Own one RealSense pipeline; publish frames and health.

    ``config`` is the parsed ``realsense.yaml`` (or ``None`` for a disabled
    no-op). ``rs_module`` / ``np_module`` default to importing
    ``pyrealsense2`` / ``numpy`` lazily; tests pass fakes.
    """

    def __init__(
        self,
        config: Optional[Dict[str, Any]],
        *,
        camera_id: str = "",
        rs_module: Any = None,
        np_module: Any = None,
    ):
        config = config or {}
        # Device-local handle: the first path segment of every route for this
        # camera and the top directory of its captures. Taken from the
        # explicit argument (what :func:`configure_cameras` passes) or the
        # entry's own ``id``; validated by the registry, not here, so a
        # hand-built camera in a test never has to care.
        self.camera_id = str(camera_id or config.get("id") or "camera").strip()
        self.enabled = bool(config.get("enabled", False))
        self.serial = str(config.get("serial", "") or "").strip() or None
        self.label = str(config.get("label", "") or "").strip() or "RealSense camera"
        # Button-sized name for the panel's camera toggle ("RS D405").
        self.short_label = str(config.get("short_label", "") or "").strip() or self.camera_id
        # Where the camera sits and which way it looks. Descriptive only: it
        # never alters the frames. Reported on every surface and in each
        # capture's meta.json so a frame can be interpreted without knowing
        # which lens was on the bench that day.
        self.mount = _mount_dict(config.get("mount"))
        self.autostart = bool(config.get("autostart", False))
        self.start_on_demand = bool(config.get("start_on_demand", True))
        self.idle_timeout_s = _as_float(config.get("idle_timeout_seconds"), 0.0)
        self.align_depth_to_color = bool(config.get("align_depth_to_color", True))
        self.jpeg_quality = int(min(95, max(1, _as_float(config.get("jpeg_quality"), 80))))
        self.frame_timeout_ms = int(_as_float(config.get("frame_timeout_ms"), 5000))
        self.max_frame_failures = int(_as_float(config.get("max_consecutive_frame_failures"), 5))
        self.color_profile = _stream_profile(config.get("color"), 640, 480, 30)
        self.depth_profile = _stream_profile(config.get("depth"), 640, 480, 30)

        # Backend. Import lazily so the module (and the whole API server)
        # stays importable on a machine without the extra installed.
        self._rs = rs_module
        self._np = np_module
        self.installed = False
        self.library_version: Optional[str] = None
        self.install_error: Optional[str] = None
        if self.enabled:
            self._import_backend()

        # Pipeline state. Everything below is guarded by _lock.
        self._lock = threading.RLock()
        self._frame_ready = threading.Condition(self._lock)
        self._pipeline = None
        self._align = None
        self._colorizer = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._state = "off"                     # off | starting | streaming | error
        self._latest: Optional[FrameBundle] = None
        self._device_info: Optional[Dict[str, Any]] = None
        self._depth_scale: Optional[float] = None
        self._intrinsics: Dict[str, Dict[str, Any]] = {}
        self._frames_captured = 0
        self._fps_ema: Optional[float] = None
        self._last_consumer_at = time.monotonic()
        self._last_error: Optional[str] = None
        self._started_at: Optional[float] = None

        # Enumeration cache (see _DEVICE_LIST_TTL_S).
        self._devices_cache: Optional[List[Dict[str, Any]]] = None
        self._devices_cache_at = 0.0

    # ------------------------------------------------------------------
    # Construction helpers
    # ------------------------------------------------------------------

    def _import_backend(self) -> None:
        try:
            if self._rs is None:
                import pyrealsense2 as rs  # type: ignore[import-not-found]

                self._rs = rs
            if self._np is None:
                import numpy as np

                self._np = np
            self.installed = True
            self.library_version = str(getattr(self._rs, "__version__", None) or "") or None
        except Exception as exc:  # noqa: BLE001 - missing extra is an expected state
            self.installed = False
            self.install_error = f"{type(exc).__name__}: {exc}"

    @property
    def configured(self) -> bool:
        """``enabled: true`` in the YAML. Decides whether /status carries a block."""
        return self.enabled

    @property
    def streaming(self) -> bool:
        with self._lock:
            return self._state == "streaming"

    @property
    def state(self) -> str:
        with self._lock:
            return self._state

    # ------------------------------------------------------------------
    # Enumeration
    # ------------------------------------------------------------------

    def list_devices(self, *, force: bool = False) -> List[Dict[str, Any]]:
        """Every RealSense on the USB bus, streaming or not. Cached briefly.

        Returns ``[]`` (never raises) when the backend is missing or the
        query fails -- the caller reads ``describe()['reason']`` for why.
        """
        if not (self.enabled and self.installed):
            return []
        now = time.monotonic()
        if (not force and self._devices_cache is not None
                and (now - self._devices_cache_at) < _DEVICE_LIST_TTL_S):
            return list(self._devices_cache)
        devices: List[Dict[str, Any]] = []
        try:
            for dev in self._rs.context().query_devices():
                devices.append(self._device_dict(dev))
        except Exception as exc:  # noqa: BLE001 - enumeration is best-effort
            logger.warning("RealSense enumeration failed: %s", exc)
        self._devices_cache = devices
        self._devices_cache_at = now
        return list(devices)

    def _device_dict(self, dev: Any) -> Dict[str, Any]:
        rs = self._rs
        info: Dict[str, Any] = {}
        for key in ("name", "serial_number", "firmware_version",
                    "usb_type_descriptor", "product_line", "product_id"):
            member = getattr(rs.camera_info, key, None)
            try:
                info[key] = dev.get_info(member) if member is not None and dev.supports(member) else None
            except Exception:  # noqa: BLE001
                info[key] = None
        return {
            "name": info["name"],
            "serial": info["serial_number"],
            "firmware": info["firmware_version"],
            "usb_type": info["usb_type_descriptor"],
            "product_line": info["product_line"],
            "product_id": info["product_id"],
        }

    def _pick_device(self) -> Dict[str, Any]:
        devices = self.list_devices(force=True)
        if not devices:
            raise RealSenseUnavailable(
                "no RealSense device connected (check the USB 3 cable and port; "
                "the camera should appear in Device Manager even without this service)"
            )
        if self.serial:
            for dev in devices:
                if dev.get("serial") == self.serial:
                    return dev
            raise RealSenseUnavailable(
                f"RealSense serial {self.serial!r} not found; connected: "
                + ", ".join(str(d.get("serial")) for d in devices)
            )
        return devices[0]

    def _own_devices(self, devices: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """The entries in ``devices`` that are THIS camera: the serial match
        when a serial is configured, else everything on the bus."""
        if not self.serial:
            return list(devices)
        return [d for d in devices if d.get("serial") == self.serial]

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> Dict[str, Any]:
        """Open the pipeline and begin capturing. Idempotent while streaming.

        Raises :class:`RealSenseUnavailable` when the feature is disabled, the
        extra is missing, or no camera matches; :class:`RealSenseError` when
        librealsense refuses the stream profiles (typical on a USB 2 link).
        """
        if not self.enabled:
            raise RealSenseUnavailable("RealSense camera disabled (enabled: false in realsense.yaml)")
        if not self.installed:
            raise RealSenseUnavailable(
                f"pyrealsense2 not installed ({self.install_error}); run `uv sync --extra realsense`"
            )
        with self._lock:
            if self._state in ("streaming", "starting"):
                return self.describe()
            self._state = "starting"
            self._last_error = None
        try:
            device = self._pick_device()
            rs = self._rs
            cfg = rs.config()
            if device.get("serial"):
                cfg.enable_device(str(device["serial"]))
            if self.depth_profile["enabled"]:
                cfg.enable_stream(rs.stream.depth, self.depth_profile["width"],
                                  self.depth_profile["height"], rs.format.z16,
                                  self.depth_profile["fps"])
            if self.color_profile["enabled"]:
                cfg.enable_stream(rs.stream.color, self.color_profile["width"],
                                  self.color_profile["height"], rs.format.bgr8,
                                  self.color_profile["fps"])
            pipeline = rs.pipeline()
            profile = pipeline.start(cfg)

            dev = profile.get_device()
            depth_scale = None
            if self.depth_profile["enabled"]:
                try:
                    depth_scale = float(dev.first_depth_sensor().get_depth_scale())
                except Exception:  # noqa: BLE001 - fall back to the D4xx default
                    depth_scale = 0.001
            intrinsics: Dict[str, Dict[str, Any]] = {}
            for kind in _STREAM_KINDS:
                if not getattr(self, f"{kind}_profile")["enabled"]:
                    continue
                try:
                    vsp = profile.get_stream(getattr(rs.stream, kind)).as_video_stream_profile()
                    intrinsics[kind] = _intrinsics_dict(vsp.get_intrinsics(), kind)
                except Exception as exc:  # noqa: BLE001
                    logger.warning("could not read %s intrinsics: %s", kind, exc)

            align = None
            if (self.align_depth_to_color and self.depth_profile["enabled"]
                    and self.color_profile["enabled"]):
                align = rs.align(rs.stream.color)
            colorizer = rs.colorizer() if self.depth_profile["enabled"] else None

            with self._lock:
                self._pipeline = pipeline
                self._align = align
                self._colorizer = colorizer
                self._device_info = self._device_dict(dev)
                self._depth_scale = depth_scale
                self._intrinsics = intrinsics
                self._latest = None
                self._frames_captured = 0
                self._fps_ema = None
                self._started_at = time.monotonic()
                self._last_consumer_at = time.monotonic()
                self._stop_event = threading.Event()
                self._thread = threading.Thread(
                    target=self._capture_loop, name="xarm-realsense-capture", daemon=True
                )
                self._state = "streaming"
                self._thread.start()
            usb = (self._device_info or {}).get("usb_type") or ""
            logger.info("RealSense streaming: %s serial=%s usb=%s",
                        self._device_info.get("name"), self._device_info.get("serial"), usb)
            if str(usb).startswith("2"):
                logger.warning("RealSense is on a USB %s link; depth+colour will be limited", usb)
            return self.describe()
        except RealSenseError:
            with self._lock:
                self._state = "off"
            raise
        except Exception as exc:  # noqa: BLE001 - librealsense raises plain RuntimeError
            with self._lock:
                self._state = "error"
                self._last_error = f"{type(exc).__name__}: {exc}"
            raise RealSenseError(f"RealSense pipeline failed to start: {exc}") from exc

    def stop(self) -> None:
        """Stop capturing and release the device. Safe to call when stopped."""
        with self._lock:
            thread = self._thread
            pipeline = self._pipeline
            self._stop_event.set()
            self._thread = None
            self._pipeline = None
            self._align = None
            self._colorizer = None
            if self._state != "error":
                self._state = "off"
            self._frame_ready.notify_all()
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=max(2.0, self.frame_timeout_ms / 1000.0 + 1.0))
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception as exc:  # noqa: BLE001 - already gone is fine
                logger.debug("pipeline.stop raised (ignored): %s", exc)
            logger.info("RealSense stopped")

    def ensure_started(self) -> None:
        """Start on demand if the config allows; else demand an explicit start."""
        if self.streaming:
            return
        if not self.start_on_demand:
            raise RealSenseNotStreaming(
                f"RealSense pipeline is stopped; POST /realsense/{self.camera_id}/start "
                "(start_on_demand is off)"
            )
        self.start()

    # ------------------------------------------------------------------
    # Capture thread
    # ------------------------------------------------------------------

    def _capture_loop(self) -> None:
        np = self._np
        failures = 0
        last_t: Optional[float] = None
        while not self._stop_event.is_set():
            with self._lock:
                pipeline, align, colorizer = self._pipeline, self._align, self._colorizer
            if pipeline is None:
                break
            try:
                frames = pipeline.wait_for_frames(self.frame_timeout_ms)
                if align is not None:
                    frames = align.process(frames)
                depth_frame = frames.get_depth_frame() if self.depth_profile["enabled"] else None
                color_frame = frames.get_color_frame() if self.color_profile["enabled"] else None
                if ((self.depth_profile["enabled"] and not depth_frame)
                        or (self.color_profile["enabled"] and not color_frame)):
                    continue  # partial frameset; librealsense delivers the next one shortly

                color = np.asanyarray(color_frame.get_data()).copy() if color_frame else None
                depth = np.asanyarray(depth_frame.get_data()).copy() if depth_frame else None
                depth_color = None
                if depth_frame and colorizer is not None:
                    depth_color = np.asanyarray(colorizer.colorize(depth_frame).get_data()).copy()

                ref = color_frame or depth_frame
                intr_kind = "color" if (align is not None or not depth_frame) else "depth"
                bundle = FrameBundle(
                    color=color,
                    depth=depth,
                    depth_color=depth_color,
                    depth_scale=self._depth_scale or 0.001,
                    intrinsics=self._intrinsics.get(intr_kind, {}),
                    frame_number=int(_call_or(ref, "get_frame_number", 0)),
                    timestamp_ms=float(_call_or(ref, "get_timestamp", 0.0)),
                    captured_at=time.monotonic(),
                )
                failures = 0
                now = bundle.captured_at
                with self._lock:
                    self._latest = bundle
                    self._frames_captured += 1
                    if last_t is not None and now > last_t:
                        inst = 1.0 / (now - last_t)
                        self._fps_ema = inst if self._fps_ema is None else (0.9 * self._fps_ema + 0.1 * inst)
                    self._frame_ready.notify_all()
                    idle_for = now - self._last_consumer_at
                last_t = now

                if self.idle_timeout_s > 0 and idle_for > self.idle_timeout_s:
                    logger.info("RealSense idle for %.0fs; stopping pipeline", self.idle_timeout_s)
                    self.stop()
                    return
            except Exception as exc:  # noqa: BLE001 - a dropped frame is not fatal
                if self._stop_event.is_set():
                    break
                failures += 1
                logger.warning("RealSense frame failure %d/%d: %s",
                               failures, self.max_frame_failures, exc)
                if failures >= self.max_frame_failures:
                    with self._lock:
                        self._state = "error"
                        self._last_error = (
                            f"camera lost after {failures} consecutive frame failures: {exc}"
                        )
                    self.stop()
                    return

    # ------------------------------------------------------------------
    # Frame access
    # ------------------------------------------------------------------

    def latest(self, *, mark_consumer: bool = True) -> FrameBundle:
        """The most recent frame bundle. Raises if not streaming / no frame yet."""
        with self._lock:
            if self._state != "streaming":
                raise RealSenseNotStreaming(self._last_error or "RealSense pipeline is stopped")
            if mark_consumer:
                self._last_consumer_at = time.monotonic()
            if self._latest is None:
                # First frame after start: give the capture thread a moment.
                self._frame_ready.wait(timeout=self.frame_timeout_ms / 1000.0)
            if self._latest is None:
                raise RealSenseNotStreaming("no frame received yet")
            return self._latest

    def wait_for_new_frame(self, after: int, timeout_s: float) -> Optional[FrameBundle]:
        """Block until a frame newer than ``after`` (a frames_captured count) lands."""
        deadline = time.monotonic() + timeout_s
        with self._lock:
            while self._state == "streaming" and self._frames_captured <= after:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._frame_ready.wait(timeout=remaining)
            if self._state != "streaming":
                return None
            self._last_consumer_at = time.monotonic()
            return self._latest

    @property
    def frames_captured(self) -> int:
        with self._lock:
            return self._frames_captured

    def encode_jpeg(self, bundle: FrameBundle, kind: str = "color") -> bytes:
        """Encode one *given* bundle as JPEG.

        Split out from :meth:`jpeg` so a caller that needs colour and depth
        from the *same* frameset (a capture record) encodes both from one
        bundle instead of calling ``latest()`` twice and silently pairing
        two different moments.
        """
        kind = _check_kind(kind)
        array = bundle.color if kind == "color" else bundle.depth_color
        if array is None:
            raise RealSenseError(f"{kind} stream is disabled in realsense.yaml")
        if kind == "color":
            array = array[..., ::-1]  # BGR -> RGB for the encoder
        return _encode_image(array, "JPEG", quality=self.jpeg_quality)

    def encode_depth_png(self, bundle: FrameBundle) -> bytes:
        """Encode one given bundle's raw depth as a lossless 16-bit PNG."""
        if bundle.depth is None:
            raise RealSenseError("depth stream is disabled in realsense.yaml")
        return _encode_image(bundle.depth, "PNG", sixteen_bit=True)

    def jpeg(self, kind: str = "color") -> Tuple[bytes, FrameBundle]:
        """Encode the latest colour (BGR) or colourised depth frame as JPEG."""
        bundle = self.latest()
        return self.encode_jpeg(bundle, kind), bundle

    def depth_png(self) -> Tuple[bytes, FrameBundle]:
        """The raw 16-bit depth map as a lossless PNG (units: depth_scale metres)."""
        bundle = self.latest()
        return self.encode_depth_png(bundle), bundle

    def mjpeg_frames(self, kind: str = "color", max_fps: float = 10.0) -> Iterator[bytes]:
        """Yield ``multipart/x-mixed-replace`` parts until the pipeline stops.

        Each part carries one JPEG. Paced to ``max_fps`` so a panel preview
        does not re-encode all 30 device frames a second.
        """
        kind = _check_kind(kind)
        min_interval = 1.0 / max(0.5, float(max_fps))
        seen = -1
        last_sent = 0.0
        while True:
            bundle = self.wait_for_new_frame(seen, timeout_s=max(1.0, self.frame_timeout_ms / 1000.0))
            if bundle is None:
                if not self.streaming:
                    return
                continue
            seen = self.frames_captured
            now = time.monotonic()
            if now - last_sent < min_interval:
                continue
            last_sent = now
            data, _ = self.jpeg(kind)
            yield (
                f"--{_MJPEG_BOUNDARY}\r\n"
                f"Content-Type: image/jpeg\r\n"
                f"Content-Length: {len(data)}\r\n"
                f"X-Frame-Number: {bundle.frame_number}\r\n\r\n"
            ).encode("ascii") + data + b"\r\n"

    @staticmethod
    def mjpeg_content_type() -> str:
        return f"multipart/x-mixed-replace; boundary={_MJPEG_BOUNDARY}"

    def depth_at(self, x: int, y: int, *, window: int = 1) -> Dict[str, Any]:
        """Metric distance at pixel (x, y) of the depth map, plus a 3-D point.

        ``window`` (odd, >= 1) takes the median of the non-zero depth values
        in a window x window patch -- RealSense depth is noisy per-pixel and
        has zero-valued holes, so a 5x5 median is what a robot should use.
        The 3-D point is a pinhole deprojection in the camera frame of the
        stream the depth map is expressed in (colour when aligned): +X
        right, +Y down, +Z out of the lens, metres. Distortion is ignored,
        which is exact for the rectified depth stream and within a pixel for
        the D4xx colour stream.
        """
        bundle = self.latest()
        if bundle.depth is None:
            raise RealSenseError("depth stream is disabled in realsense.yaml")
        depth = bundle.depth
        h, w = depth.shape[:2]
        x, y = int(x), int(y)
        if not (0 <= x < w and 0 <= y < h):
            raise ValueError(f"pixel ({x}, {y}) outside the {w}x{h} depth map")
        window = max(1, int(window)) | 1  # force odd
        half = window // 2
        patch = depth[max(0, y - half):y + half + 1, max(0, x - half):x + half + 1]
        np = self._np
        valid = patch[patch > 0]
        raw = float(np.median(valid)) if valid.size else 0.0
        distance = raw * bundle.depth_scale
        point = None
        intr = bundle.intrinsics or {}
        if distance > 0 and intr.get("fx") and intr.get("fy"):
            point = [
                (x - float(intr["ppx"])) / float(intr["fx"]) * distance,
                (y - float(intr["ppy"])) / float(intr["fy"]) * distance,
                distance,
            ]
        return {
            "pixel": [x, y],
            "window": window,
            "valid_samples": int(valid.size),
            "distance_m": distance if distance > 0 else None,
            "point_m": point,
            "frame_number": bundle.frame_number,
            "frame": intr.get("stream") or ("color" if self._align is not None else "depth"),
        }

    # ------------------------------------------------------------------
    # Reporting
    # ------------------------------------------------------------------

    def intrinsics(self) -> Dict[str, Any]:
        """Per-stream pinhole intrinsics + depth scale (populated while streaming)."""
        with self._lock:
            return {
                "streaming": self._state == "streaming",
                "depth_scale_m": self._depth_scale,
                "aligned_to": "color" if self._align is not None else None,
                "streams": dict(self._intrinsics),
            }

    def describe(self) -> Dict[str, Any]:
        """Everything the panel / ``GET /realsense/<id>/status`` needs. Never raises."""
        devices = self.list_devices()
        with self._lock:
            state = self._state
            latest = self._latest
            info: Dict[str, Any] = {
                "camera_id": self.camera_id,
                "configured": self.configured,
                "installed": self.installed,
                "library_version": self.library_version,
                "label": self.label,
                "short_label": self.short_label,
                "mount": dict(self.mount),
                "state": state,
                "streaming": state == "streaming",
                "start_on_demand": self.start_on_demand,
                "device": dict(self._device_info) if self._device_info else None,
                "devices": devices,
                "streams": {
                    "color": dict(self.color_profile),
                    "depth": dict(self.depth_profile),
                    "align_depth_to_color": self.align_depth_to_color,
                },
                "depth_scale_m": self._depth_scale,
                "frames_captured": self._frames_captured,
                "fps_measured": round(self._fps_ema, 1) if self._fps_ema else None,
                "last_frame_age_s": (round(time.monotonic() - latest.captured_at, 2)
                                     if latest else None),
                "uptime_s": (round(time.monotonic() - self._started_at, 1)
                             if self._started_at and state == "streaming" else None),
                "last_error": self._last_error,
                "present": state == "streaming" or bool(self._own_devices(devices)),
                "warnings": [],
                "reason": None,
            }
        usb = ((info["device"] or {}).get("usb_type") or "")
        if str(usb).startswith("2"):
            info["warnings"].append(
                f"USB {usb} link: use a USB 3 port/cable for full depth+colour rate"
            )
        if not self.configured:
            info["reason"] = "RealSense camera disabled (enabled: false in realsense.yaml)"
        elif not self.installed:
            info["reason"] = (
                f"pyrealsense2 not installed ({self.install_error}); run `uv sync --extra realsense`"
            )
        elif state == "error":
            info["reason"] = info["last_error"] or "camera error"
        elif state != "streaming" and not devices:
            info["reason"] = "no RealSense device connected"
        elif state != "streaming" and not info["present"]:
            # Another RealSense is on the bus, but not this one: without this
            # an unplugged camera would read "starts on first request".
            info["reason"] = f"RealSense sn {self.serial} not connected (on the bus: " + ", ".join(
                str(d.get("serial")) for d in devices) + ")"
        elif state != "streaming":
            info["reason"] = "pipeline stopped" + (
                " (starts on first request)" if self.start_on_demand else ""
            )
        return info

    def component_status(self) -> Optional[Dict[str, Any]]:
        """``components.realsense_<id>`` fields, or None when unconfigured."""
        if not self.configured:
            return None
        d = self.describe()
        present = d["present"]
        if d["streaming"]:
            state = "streaming"
        elif d["state"] == "error":
            state = "error"
        elif not d["installed"]:
            state = "driver_missing"
        elif present:
            state = "idle"
        else:
            state = "disconnected"
        own = self._own_devices(d["devices"])
        dev = d["device"] or (own[0] if own else {})
        bits = [self.label]
        if dev.get("name"):
            bits.append(str(dev["name"]))
        if dev.get("serial"):
            bits.append(f"sn {dev['serial']}")
        if d["fps_measured"]:
            bits.append(f"{d['fps_measured']} fps")
        if d["reason"] and not d["streaming"]:
            bits.append(d["reason"])
        return {"connected": present, "state": state, "message": " · ".join(bits)}

    def status_block(self) -> Optional[Dict[str, Any]]:
        """Compact ``details.realsense.cameras.<id>`` block, or None when unconfigured."""
        if not self.configured:
            return None
        d = self.describe()
        return {
            "camera_id": self.camera_id,
            "label": self.label,
            "mount": dict(self.mount),
            "state": d["state"],
            "installed": d["installed"],
            "device": d["device"],
            "devices": d["devices"],
            "streams": d["streams"],
            "fps_measured": d["fps_measured"],
            "frames_captured": d["frames_captured"],
            "last_frame_age_s": d["last_frame_age_s"],
            "warnings": d["warnings"],
            "reason": d["reason"],
        }


# ----------------------------------------------------------------------
# Process-wide registry
# ----------------------------------------------------------------------
#
# The cameras outlive any one arm connection: an operator wants to see the
# bench before /connect and after /disconnect. The API server configures the
# registry once at import; status_builder reads it through cameras() so the
# /status envelope and /realsense/* never disagree.
#
# Keyed by the device-local id from the YAML, which is also the first path
# segment of every route for that camera. Nothing here raises: a config file
# that cannot be read, or an entry that cannot be trusted, yields an empty (or
# smaller) registry plus a reason the API hands back on GET /realsense/cameras.

_cameras: "Dict[str, RealSenseCamera]" = {}
_reason: Optional[str] = None


def load_config(path: str) -> Tuple[Dict[str, Any], Optional[str]]:
    """Parse ``realsense.yaml``. Returns ``(config, reason_it_is_empty)``."""
    try:
        import yaml  # local import: keeps the module importable without PyYAML

        with open(path, "r") as handle:
            loaded = yaml.safe_load(handle)
    except FileNotFoundError:
        logger.info("no config at %s; no RealSense cameras", path)
        return {}, f"no RealSense config at {path}"
    except Exception as exc:  # noqa: BLE001 - never break service boot
        logger.warning("failed to load %s: %s; no RealSense cameras", path, exc)
        return {}, f"could not read {path}: {exc}"
    if not isinstance(loaded, dict):
        return {}, f"{path} does not contain a mapping"
    return loaded, None


def build_cameras(
    config: Optional[Dict[str, Any]], **kwargs: Any
) -> Tuple["Dict[str, RealSenseCamera]", Optional[str]]:
    """Turn a parsed config into ``{camera_id: RealSenseCamera}`` + a reason.

    Every rejection is a log line and a skipped entry, never an exception:
    this runs at import time in the arm's own process, and a typo in a camera
    id must not be able to stop the service from booting. The rules:

    * ``enabled: false`` at the top level, or an empty ``cameras:`` list,
      means no cameras at all (the reason says which).
    * an id must match :data:`CAMERA_ID_RE` -- it is interpolated into URLs
      and into the capture store's paths.
    * duplicate ids are refused rather than resolved, because which of the
      two won would depend on file order.
    * ``serial`` is required as soon as more than one entry is configured:
      "the first device librealsense enumerates" is not stable across
      reboots, and an id silently pointing at the wrong lens is worse than a
      camera that is missing.
    """
    config = config or {}
    if not bool(config.get("enabled", False)):
        return {}, "RealSense disabled (enabled: false in realsense.yaml)"
    entries = config.get("cameras")
    if not isinstance(entries, list) or not entries:
        return {}, "no cameras configured (cameras: is empty in realsense.yaml)"

    built: Dict[str, RealSenseCamera] = {}
    skipped: List[str] = []
    require_serial = len(entries) > 1
    for index, entry in enumerate(entries):
        if not isinstance(entry, dict):
            skipped.append(f"cameras[{index}] is not a mapping")
            continue
        camera_id = str(entry.get("id", "") or "").strip()
        if not CAMERA_ID_RE.match(camera_id):
            skipped.append(f"cameras[{index}] id {camera_id!r} does not match {CAMERA_ID_RE.pattern}")
            continue
        if camera_id in built:
            skipped.append(f"duplicate camera id {camera_id!r}")
            continue
        if not bool(entry.get("enabled", True)):
            skipped.append(f"camera {camera_id!r} is disabled in realsense.yaml")
            continue
        serial = str(entry.get("serial", "") or "").strip()
        if require_serial and not serial:
            skipped.append(
                f"camera {camera_id!r} has no serial; a serial is required with "
                f"{len(entries)} cameras configured, or enumeration is ambiguous"
            )
            continue
        settings = dict(entry)
        settings["enabled"] = True           # the top-level switch already said yes
        built[camera_id] = RealSenseCamera(settings, camera_id=camera_id, **kwargs)

    for note in skipped:
        logger.warning("RealSense config: skipped %s", note)
    if built:
        return built, None
    return {}, "; ".join(skipped) or "no usable camera entries in realsense.yaml"


def configure_cameras(config_path: str, **kwargs: Any) -> "Dict[str, RealSenseCamera]":
    """Build the process-wide registry from ``realsense.yaml``. Never raises."""
    global _cameras, _reason
    config, reason = load_config(config_path)
    if reason is not None:
        _cameras, _reason = {}, reason
        return _cameras
    _cameras, _reason = build_cameras(config, **kwargs)
    return _cameras


def set_cameras(mapping: Optional["Dict[str, RealSenseCamera]"], *,
                reason: Optional[str] = None) -> None:
    """Install (or clear, with None/{}) the registry. Tests use this."""
    global _cameras, _reason
    _cameras = dict(mapping or {})
    _reason = None if _cameras else (reason or "no RealSense cameras configured")


def cameras() -> "Dict[str, RealSenseCamera]":
    """The registry, id -> camera. Empty when nothing is configured."""
    return dict(_cameras)


def camera(camera_id: str) -> Optional["RealSenseCamera"]:
    """One camera by id, or None when that id is not configured."""
    return _cameras.get(str(camera_id))


def default_camera_id() -> Optional[str]:
    """The id to assume when a caller names none.

    Exactly one configured camera means there is nothing to be ambiguous
    about; two or more means a caller that did not say which one did not
    say enough, and the API answers 400 rather than guessing.
    """
    if len(_cameras) == 1:
        return next(iter(_cameras))
    return None


def configuration_reason() -> Optional[str]:
    """Why the registry is empty, or None when it is not."""
    return _reason if not _cameras else None


def default_config_path() -> str:
    return os.path.join(os.path.dirname(os.path.dirname(__file__)), "settings", "realsense.yaml")


# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------


def _as_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _mount_dict(block: Any) -> Dict[str, Any]:
    """Normalise a ``mount`` entry to ``{"location": str|None, "facing": str|None}``."""
    block = block if isinstance(block, dict) else {}
    out: Dict[str, Any] = {}
    for key in ("location", "facing"):
        value = str(block.get(key, "") or "").strip()
        out[key] = value or None
    return out


def _stream_profile(block: Any, width: int, height: int, fps: int) -> Dict[str, Any]:
    block = block if isinstance(block, dict) else {}
    return {
        "enabled": bool(block.get("enabled", True)),
        "width": int(_as_float(block.get("width"), width)),
        "height": int(_as_float(block.get("height"), height)),
        "fps": int(_as_float(block.get("fps"), fps)),
    }


def _check_kind(kind: str) -> str:
    kind = str(kind or "color").lower()
    if kind not in _STREAM_KINDS:
        raise ValueError(f"stream must be one of {_STREAM_KINDS}, got {kind!r}")
    return kind


def _call_or(obj: Any, method: str, default: Any) -> Any:
    try:
        return getattr(obj, method)()
    except Exception:  # noqa: BLE001
        return default


def _intrinsics_dict(intr: Any, stream: str) -> Dict[str, Any]:
    model = getattr(intr, "model", None)
    return {
        "stream": stream,
        "width": int(getattr(intr, "width", 0)),
        "height": int(getattr(intr, "height", 0)),
        "fx": float(getattr(intr, "fx", 0.0)),
        "fy": float(getattr(intr, "fy", 0.0)),
        "ppx": float(getattr(intr, "ppx", 0.0)),
        "ppy": float(getattr(intr, "ppy", 0.0)),
        "model": str(model).split(".")[-1] if model is not None else None,
        "coeffs": [float(c) for c in (getattr(intr, "coeffs", None) or [])],
    }


def _encode_image(array: Any, fmt: str, *, quality: int = 80, sixteen_bit: bool = False) -> bytes:
    from PIL import Image  # local import: Pillow is part of the optional extra

    if sixteen_bit:
        image = Image.fromarray(array.astype("uint16"))  # mode I;16 -> 16-bit PNG
    else:
        image = Image.fromarray(array)
    buf = io.BytesIO()
    if fmt == "JPEG":
        image.save(buf, format="JPEG", quality=int(quality))
    else:
        image.save(buf, format=fmt)
    return buf.getvalue()
