"""Lab PTZ camera and RealSense routes for the shared panel's camera tiles.

Both are optional local-config blocks and neither loads a camera SDK here:

* Lab camera (the bench Tapo): the xArm's CameraTracker reads its live state
  from the dashboard's open /api/equipment snapshot (availability, lenses,
  presets, stream source). The panel's video uses the dashboard's
  authenticated viewing sessions on the shared page origin. PTZ and preset
  recalls go through the dashboard's audited control passthrough with the
  operator's own credential (their ac_auth session cookie, or a machine
  principal's X-Api-Key), so this service holds no camera credential and the
  dashboard authorizes and audits the real person. There is no motion graph
  runner here, so "Follow arm" is not offered.
* RealSense: owned by the standalone SDL camera service and reached through
  the xArm's remote facade (core.remote_realsense), with the xArm's
  /realsense/<id>/* routes, payloads and status codes.

Gating follows the xArm: discovery, status, depth readings and intrinsics are
open reads; anything that starts a camera, ships pixels or moves the PTZ needs
the edge's shared-secret identity. A camera outage never changes the robot's
equipment_status.

Captures (the xArm's capture records) are kept by the camera service's store:
POST /control/realsense[/<id>]/capture grabs one aligned frameset there with
the arm's pose from cached telemetry. Taking or deleting one is the only
claim-gated act here (with control on; otherwise the edge identity), and it
is refused while the arm moves, since a frame grabbed mid-move is blurred and
its pose already stale. Listings and meta are open reads; the image files
need the edge identity like every other frame.
"""

from __future__ import annotations

import asyncio
import logging

import httpx
from fastapi import Depends, HTTPException, Request
from datetime import datetime, timezone

from fastapi.responses import Response, StreamingResponse
from pydantic import BaseModel, ConfigDict, Field
from core.camera_tracker import CameraTracker
from core.realsense_camera import RealSenseNotStreaming, RealSenseUnavailable
from core.realsense_captures import COLOR_NAME, DEPTH_NAME, CaptureNotFound
from core.remote_realsense import StreamParts

from .edge import SECRET_ENV, configured_secret, edge_identity

log = logging.getLogger(__name__)
# The dashboard's session cookie (ac_auth); the only cookie ever forwarded.
SESSION_COOKIE = "ac_auth_session"
TAGS = ["cameras"]


class CameraFollowRequest(BaseModel):
    """Body of POST /camera/follow."""

    enabled: bool = Field(description="Requested follow state; always refused here")


class PresetRequest(BaseModel):
    """Body of POST /camera/preset."""

    preset_id: str = Field(min_length=1, max_length=64, description="A saved preset id from /camera/config presets")


class CaptureRequest(BaseModel):
    """Body of POST /control/realsense/capture (the xArm's). Every field is optional."""

    model_config = ConfigDict(extra="forbid")
    camera: str | None = Field(
        default=None,
        max_length=32,
        description="Camera id for the fixed-path alias; omit it while exactly one camera is configured. "
        "The nested route POST /control/realsense/{camera_id}/capture ignores it.",
    )
    label: str | None = Field(default=None, max_length=200, description="Free-form name for this capture")
    node_id: str | None = Field(
        default=None, max_length=64, description="Motion-graph node this capture belongs to; defaults to the arm's"
    )
    tags: list[str] | None = Field(default=None, max_length=32, description="Free-form tags for later filtering")
    protected: bool = Field(default=False, description="Exempt this capture from the store's retention bounds")


def _upstream_reason(response):
    try:
        detail = response.json().get("detail")
    except (ValueError, AttributeError):
        detail = None
    if isinstance(detail, dict):
        detail = detail.get("reason") or detail.get("message") or detail.get("error")
    return str(detail or response.text[:300] or f"HTTP {response.status_code}")


def _camera_error(exc):
    """Map camera exceptions to the status codes the panel keys on."""
    if isinstance(exc, RealSenseUnavailable):
        return HTTPException(status_code=503, detail={"error": "realsense_unavailable", "reason": str(exc)})
    if isinstance(exc, CaptureNotFound):
        return HTTPException(status_code=404, detail={"error": "capture_not_found", "reason": str(exc)})
    if isinstance(exc, RealSenseNotStreaming):
        return HTTPException(status_code=409, detail={"error": "realsense_not_streaming", "reason": str(exc)})
    if isinstance(exc, ValueError):
        return HTTPException(status_code=400, detail={"error": "bad_request", "reason": str(exc)})
    return HTTPException(status_code=502, detail={"error": "realsense_error", "reason": str(exc)})


def _stream_kind(stream):
    kind = (stream or "color").lower()
    if kind not in ("color", "depth"):
        raise HTTPException(
            status_code=400,
            detail={"error": "bad_request", "reason": "stream must be 'color' or 'depth'"},
        )
    return kind


def _urls(camera_id):
    """Every route for one camera, so clients never build paths themselves."""
    base = f"/realsense/{camera_id}"
    return {
        "status": f"{base}/status",
        "start": f"{base}/start",
        "stop": f"{base}/stop",
        "snapshot": f"{base}/snapshot.jpg",
        "depth_png": f"{base}/depth.png",
        "stream": f"{base}/stream.mjpg",
        "depth": f"{base}/depth",
        "intrinsics": f"{base}/intrinsics",
        "calibration": f"{base}/calibration",
        "captures": f"{base}/captures",
        "capture": f"/control/realsense/{camera_id}/capture",
    }


def _capture_urls(camera_id, capture_id, meta):
    base = f"/realsense/{camera_id}/captures/{capture_id}"
    urls = {"meta": base}
    for name, key in ((COLOR_NAME, "color"), (DEPTH_NAME, "depth")):
        if name in ((meta or {}).get("files") or {}):
            urls[key] = f"{base}/{name}"
    return urls


class Cameras:
    """Builds the camera clients from Settings and installs their routes.

    ``stereo`` injects a prebuilt ``(service, {id: camera})`` pair and
    ``transport`` the httpx transport for dashboard calls (tests).
    """

    def __init__(self, settings, *, edge_secret=None, stereo=None, transport=None):
        self.secret = configured_secret(edge_secret)
        self.lab = settings.lab_camera
        self._transport = transport
        tracker_config = {}
        if self.lab is not None:
            tracker_config = {
                "enabled": True,
                "dashboard_base_url": self.lab.dashboard_base_url,
                "camera_id": self.lab.camera_id,
                "lens": self.lab.lens or "",
                "request_timeout_seconds": self.lab.request_timeout_s,
                "follow_by_default": False,
            }
        # Read view only: notify_node is never called, so it never pans.
        self.tracker = CameraTracker(
            tracker_config, fetcher=self._get_json, sender=lambda *_: None, environ={}
        )
        self.service = None
        self.cameras = {}
        # The camera service's capture store (core.remote_realsense.RemoteStore).
        self.store = None
        self.reason = "no camera_service in the local config"
        if stereo is not None:
            self.service, self.cameras, *rest = stereo
            self.store = rest[0] if rest else None
            self.reason = None
        elif settings.camera_service is not None:
            try:
                from core.remote_realsense import configure

                entries = [c.model_dump(exclude_none=True) for c in settings.camera_service.cameras]
                self.service, self.cameras, self.store = configure(
                    settings.camera_service.service_file, {"cameras": entries}
                )
                self.reason = None
            except Exception as exc:
                # A camera misconfiguration must not take robot observation down.
                log.exception("Camera service configuration rejected")
                self.reason = f"camera service configuration rejected: {exc}"

    # ── lifecycle ───────────────────────────────────────────────────
    def start(self):
        if self.service is not None:
            self.service.start_polling()

    def close(self):
        if self.service is not None:
            self.service.close()

    # ── dashboard transport ─────────────────────────────────────────
    def _client(self, timeout):
        # One client per call: nothing (in particular no operator cookie a
        # response might set) survives into another caller's request.
        return httpx.Client(transport=self._transport, timeout=timeout, trust_env=False)

    def _get_json(self, url, timeout):
        with self._client(timeout) as client:
            response = client.get(url)
            response.raise_for_status()
            return response.json()

    def _forward(self, action, payload, credential):
        url = f"{self.lab.dashboard_base_url}/api/equipment/{self.lab.camera_id}/control/{action}"
        with self._client(self.lab.request_timeout_s) as client:
            return client.post(url, json=payload, headers=credential)

    # ── gates ───────────────────────────────────────────────────────
    def require_viewer(self, request: Request):
        if not self.secret:
            raise HTTPException(
                status_code=503,
                detail={
                    "error": "edge_identity_not_configured",
                    "reason": "Camera video and PTZ need the dashboard edge identity",
                    "hint": f"set {SECRET_ENV} for the service",
                },
            )
        identity = edge_identity(request, self.secret)
        if identity is None:
            raise HTTPException(
                status_code=401,
                detail={
                    "error": "login_required",
                    "reason": "Open this panel through the lab dashboard and sign in to use the camera",
                },
            )
        return identity

    @staticmethod
    def operator_credential(request):
        """The caller's own dashboard credential, never a stored one."""
        api_key = request.headers.get("x-api-key")
        if api_key:
            return {"X-Api-Key": api_key}
        token = request.cookies.get(SESSION_COOKIE)
        if token:
            return {"Cookie": f"{SESSION_COOKIE}={token}"}
        return None

    def camera(self, camera_id):
        """One RealSense by id; 404 bodies name the ids that do exist."""
        if not self.cameras:
            raise HTTPException(
                status_code=404,
                detail={"error": "realsense_not_configured", "hint": self.reason},
            )
        camera = self.cameras.get(camera_id)
        if camera is None:
            raise HTTPException(
                status_code=404,
                detail={"error": "camera_not_found", "camera_id": camera_id, "cameras": sorted(self.cameras)},
            )
        return camera

    def default_id(self):
        return next(iter(self.cameras)) if len(self.cameras) == 1 else None

    def captures_available(self):
        return self.store is not None and bool(self.cameras)

    def capture_store(self):
        if not self.captures_available():
            raise HTTPException(
                status_code=404,
                detail={"error": "captures_not_configured", "hint": self.reason or "no camera service store"},
            )
        return self.store

    # ── /status contributions (cached telemetry only, never a request) ──
    def components(self):
        out = {}
        for camera_id, camera in self.cameras.items():
            try:
                out[f"realsense_{camera_id}"] = camera.component_status()
            except Exception:
                log.exception("RealSense status block failed")
        return out

    def details(self):
        if not self.cameras:
            return {}
        blocks = {}
        for camera_id, camera in self.cameras.items():
            try:
                blocks[camera_id] = camera.status_block()
            except Exception:
                log.exception("RealSense status block failed")
        return {"realsense": {"default": self.default_id(), "cameras": blocks}}

    # ── routes ──────────────────────────────────────────────────────
    def install(self, app, *, take=None, arm_context=None, arm_moving=None):
        """Install the camera routes. ``take`` gates taking and deleting a
        capture (the arm's login + claim when control is on; the viewer gate
        otherwise); ``arm_context()`` is the cached arm state stamped on a
        capture and ``arm_moving()`` whether a move is in flight."""
        viewer = [Depends(self.require_viewer)]
        take = take or viewer

        @app.get("/camera/config", tags=TAGS)
        async def camera_config():
            """Whether the panel shows the lab camera tile, and its live state.

            ``configured`` decides whether the tile appears; ``available`` (a
            short-cached probe of the camera through the dashboard) whether the
            stream and PTZ are usable now. ``stream_url`` and ``lenses`` carry
            the go2rtc source names the panel opens dashboard viewing sessions
            for. Never raises. Following the arm is not supported here.
            """
            info = await asyncio.to_thread(self.tracker.availability)
            info.update(connected=False, following=False, follow_supported=False)
            return info

        @app.post("/camera/follow", tags=TAGS)
        def camera_follow(_body: CameraFollowRequest):
            raise HTTPException(
                status_code=409,
                detail={
                    "error": "follow_not_supported",
                    "reason": "This service runs no motion graph, so the camera cannot follow the arm",
                },
            )

        async def control(request, action, payload):
            if self.lab is None:
                raise HTTPException(status_code=404, detail="camera tracking not configured")
            credential = self.operator_credential(request)
            if credential is None:
                raise HTTPException(
                    status_code=401,
                    detail={"error": "login_required", "reason": "No dashboard session to authorize camera control"},
                )
            try:
                response = await asyncio.to_thread(self._forward, action, payload, credential)
            except httpx.HTTPError as exc:
                raise HTTPException(
                    status_code=502,
                    detail={"error": "camera_control_failed", "reason": f"dashboard unreachable: {type(exc).__name__}"},
                ) from exc
            if response.is_success:
                return {"ok": True}
            # The dashboard's refusals (401 not signed in, 403 no role on the
            # camera, 409 privacy mode, 422 bad body) are the operator's to read.
            status = response.status_code if 400 <= response.status_code < 500 else 502
            raise HTTPException(
                status_code=status,
                detail={"error": "camera_control_refused" if status != 502 else "camera_control_failed",
                        "reason": _upstream_reason(response)},
            )

        @app.post("/camera/ptz", tags=TAGS, dependencies=viewer)
        async def camera_ptz(request: Request, body: dict):
            """Operator PTZ, forwarded verbatim to the dashboard's audited
            control passthrough as the operator: a continuous move
            ``{direction, speed, duration_ms}`` or a stop ``{pan, tilt, zoom}``."""
            return await control(request, "ptz", body)

        @app.post("/camera/preset", tags=TAGS, dependencies=viewer)
        async def camera_preset(request: Request, body: PresetRequest):
            """Recall a saved camera preset through the same audited route."""
            return await control(request, "preset/goto", {"preset_id": body.preset_id})

        @app.get("/realsense/cameras", tags=TAGS)
        async def realsense_cameras():
            """The RealSense cameras on this PC and where to reach each one.

            Never 404s: no configured camera is an empty list plus ``reason``.
            ``default`` is the id a caller may omit naming (null with several).
            """

            def listing():
                out = []
                for camera_id, camera in sorted(self.cameras.items()):
                    described = camera.describe()
                    out.append({
                        "id": camera_id,
                        "label": camera.label,
                        "short_label": getattr(camera, "short_label", None) or camera_id,
                        "mount": dict(getattr(camera, "mount", None) or {}),
                        "state": described["state"],
                        "streaming": described["streaming"],
                        "start_on_demand": camera.start_on_demand,
                        "device": described["device"],
                        "urls": _urls(camera_id),
                    })
                return out

            return {
                "cameras": await asyncio.to_thread(listing),
                "default": self.default_id(),
                "reason": None if self.cameras else self.reason,
            }

        @app.get("/cameras", tags=TAGS)
        async def all_cameras():
            """Discover this panel's cameras by capability."""
            rs = await realsense_cameras()
            capabilities = ["color", "depth", "intrinsics", "calibration", "snapshot", "mjpeg"]
            if self.store is not None:
                capabilities.append("captures")
            cameras = [dict(camera, kind="realsense", capabilities=list(capabilities)) for camera in rs["cameras"]]
            lab = {"configured": self.lab is not None}
            if self.lab is not None:
                lab.update(
                    camera_id=self.lab.camera_id,
                    urls={"config": "/camera/config", "ptz": "/camera/ptz", "preset": "/camera/preset"},
                )
            return {
                "cameras": cameras,
                "sources": {
                    "realsense": {"reason": rs["reason"]},
                    "usb": {"available": False, "reason": "USB cameras are owned by the standalone camera service"},
                    "lab": lab,
                },
            }

        @app.get("/realsense/{camera_id}/status", tags=TAGS)
        async def realsense_status(camera_id: str):
            """Cached camera-service telemetry for one camera; never starts it."""
            camera = self.camera(camera_id)
            return await asyncio.to_thread(camera.describe)

        @app.post("/realsense/{camera_id}/start", tags=TAGS, dependencies=viewer)
        async def realsense_start(camera_id: str):
            camera = self.camera(camera_id)
            try:
                return await asyncio.to_thread(camera.start)
            except Exception as exc:
                raise _camera_error(exc) from exc

        @app.post("/realsense/{camera_id}/stop", tags=TAGS, dependencies=viewer)
        async def realsense_stop(camera_id: str):
            camera = self.camera(camera_id)
            try:
                await asyncio.to_thread(camera.stop)
            except Exception as exc:
                raise _camera_error(exc) from exc
            return await asyncio.to_thread(camera.describe)

        @app.get("/realsense/{camera_id}/snapshot.jpg", tags=TAGS, dependencies=viewer)
        async def realsense_snapshot(camera_id: str, stream: str | None = None):
            """One JPEG of the colour image or (``?stream=depth``) the
            colourised depth map; the camera service starts on demand."""
            camera = self.camera(camera_id)
            kind = _stream_kind(stream)
            try:
                data, frame = await asyncio.to_thread(camera.jpeg, kind)
            except Exception as exc:
                raise _camera_error(exc) from exc
            return Response(content=data, media_type="image/jpeg", headers={
                "Cache-Control": "no-store",
                "X-Frame-Number": str(frame.frame_number),
                "X-Frame-Timestamp-Ms": str(frame.timestamp_ms),
            })

        @app.get("/realsense/{camera_id}/depth.png", tags=TAGS, dependencies=viewer)
        async def realsense_depth_png(camera_id: str):
            """Raw 16-bit depth, lossless; pixel x depth_scale_m is metres."""
            camera = self.camera(camera_id)
            try:
                data, frame = await asyncio.to_thread(camera.depth_png)
            except Exception as exc:
                raise _camera_error(exc) from exc
            return Response(content=data, media_type="image/png", headers={
                "Cache-Control": "no-store",
                "X-Frame-Number": str(frame.frame_number),
                "X-Depth-Scale-M": str(frame.depth_scale),
            })

        @app.get("/realsense/{camera_id}/stream.mjpg", tags=TAGS, dependencies=viewer)
        async def realsense_stream(camera_id: str, stream: str | None = None, fps: float = 10.0):
            """Live MJPEG preview for an <img>, paced to ``fps`` (0.5..30)."""
            camera = self.camera(camera_id)
            kind = _stream_kind(stream)
            fps = max(0.5, min(30.0, float(fps)))
            # Each part blocks on the camera service; pull it on a worker so
            # the event loop stays free. The first part is read before the
            # response starts, so an upstream refusal is a status code rather
            # than an empty 200 stream.
            parts = StreamParts(camera.mjpeg_frames(kind, max_fps=fps))
            try:
                first = await asyncio.to_thread(parts.next)
            except Exception as exc:
                parts.close_soon()
                raise _camera_error(exc) from exc

            async def body():
                # A viewer leaving cancels this generator; close the upstream
                # stream then rather than leaving it to the garbage collector.
                # close_soon(), not an await: awaiting inside a cancelled
                # task's finally is itself cancelled.
                try:
                    chunk = first
                    while chunk is not None:
                        yield chunk
                        try:
                            chunk = await asyncio.to_thread(parts.next)
                        except RealSenseUnavailable:
                            return  # upstream ended mid-stream; the <img> reconnects
                finally:
                    parts.close_soon()

            return StreamingResponse(body(), media_type=camera.mjpeg_content_type(),
                                     headers={"Cache-Control": "no-store"})

        @app.get("/realsense/{camera_id}/depth", tags=TAGS)
        async def realsense_depth_at(camera_id: str, x: int, y: int, window: int = 5):
            """Metric distance at pixel (x, y) plus a camera-frame 3-D point.
            ``window`` (odd) is the median patch. 409 while stopped, so an open
            read never switches the camera on."""
            camera = self.camera(camera_id)
            try:
                return await asyncio.to_thread(camera.depth_at, x, y, window=window)
            except Exception as exc:
                raise _camera_error(exc) from exc

        @app.get("/realsense/{camera_id}/intrinsics", tags=TAGS)
        async def realsense_intrinsics(camera_id: str):
            """Pinhole intrinsics per stream plus depth scale, while streaming."""
            camera = self.camera(camera_id)
            try:
                return await asyncio.to_thread(camera.intrinsics)
            except Exception as exc:
                raise _camera_error(exc) from exc

        @app.get("/realsense/{camera_id}/calibration", tags=TAGS)
        async def realsense_calibration(camera_id: str, refresh: bool = False):
            """Stored device calibration (per-profile intrinsics, all stream
            extrinsics, stereo baseline, IMU on a D435i) from the camera
            service; no pipeline start. ``refresh=true`` re-reads it."""
            camera = self.camera(camera_id)
            try:
                return await asyncio.to_thread(camera.calibration, refresh=refresh)
            except Exception as exc:
                raise _camera_error(exc) from exc

        # Only with a camera service: an unconfigured service has no /control route.
        if self.store is not None:
            self._install_captures(app, viewer, take, arm_context, arm_moving)
        return self

    def _install_captures(self, app, viewer, take, arm_context, arm_moving):
        def requester(request):
            identity = getattr(request.state, "identity", None) or edge_identity(request, self.secret)
            return identity["email"] if identity else "unknown"

        async def capture(request, camera_id, body):
            self.capture_store()
            camera = self.camera(camera_id)
            if arm_moving is not None and arm_moving():
                raise HTTPException(
                    status_code=409,
                    detail={
                        "error": "arm_moving",
                        "reason": "A move is in flight; a frame grabbed now is blurred and its pose already stale",
                    },
                )
            arm = dict(arm_context()) if arm_context is not None else {"connected": False}
            if body.node_id is not None:
                arm["node_id"] = body.node_id
            context = {
                "arm": arm,
                "requested_by": requester(request),
                "context_sampled_at": datetime.now(timezone.utc).isoformat(),
                "synchronization": "cached robot telemetry, not hardware synchronized",
            }
            try:
                result = await asyncio.to_thread(
                    camera.capture_remote, label=body.label, tags=list(body.tags or []),
                    protected=body.protected, context=context,
                )
            except Exception as exc:
                raise _camera_error(exc) from exc
            result["urls"] = _capture_urls(camera_id, result["capture_id"], result.get("meta"))
            return result

        @app.post("/control/realsense/{camera_id}/capture", tags=TAGS, dependencies=take)
        async def realsense_capture(camera_id: str, request: Request, body: CaptureRequest | None = None):
            """Grab one aligned colour + 16-bit depth frameset from this camera
            and keep it in the camera service's store with the arm's pose.
            Starts the camera on demand. 409 while the arm moves."""
            return await capture(request, camera_id, body or CaptureRequest())

        @app.post("/control/realsense/capture", tags=TAGS, dependencies=take)
        async def realsense_capture_alias(request: Request, body: CaptureRequest | None = None):
            """The skill-facing alias: the camera is named in the body (a skill
            endpoint is sent verbatim, so it cannot fill a path parameter).
            Omitted means the sole camera; with several, 400 camera_required."""
            body = body or CaptureRequest()
            camera_id = (body.camera or "").strip() or self.default_id()
            if camera_id is None:
                raise HTTPException(
                    status_code=400,
                    detail={
                        "error": "camera_required",
                        "hint": 'more than one camera is configured; set "camera" in the body',
                        "cameras": sorted(self.cameras),
                    },
                )
            return await capture(request, camera_id, body)

        @app.get("/realsense/captures", tags=TAGS)
        async def realsense_captures(
            limit: int = 50, node_id: str | None = None, label: str | None = None, since: str | None = None
        ):
            """Capture metadata across every camera, newest first. Open read."""
            store = self.capture_store()
            limit = max(1, min(500, int(limit)))
            try:
                items = await asyncio.to_thread(
                    store.list_captures, camera_id=None, limit=limit, node_id=node_id, label=label, since=since
                )
            except Exception as exc:
                raise _camera_error(exc) from exc
            return {"captures": items, "count": len(items), "retention": store.describe()}

        @app.get("/realsense/{camera_id}/captures", tags=TAGS)
        async def realsense_camera_captures(
            camera_id: str, limit: int = 50, node_id: str | None = None, label: str | None = None,
            since: str | None = None,
        ):
            """The same listing, narrowed to one camera."""
            store = self.capture_store()
            self.camera(camera_id)
            limit = max(1, min(500, int(limit)))
            try:
                items = await asyncio.to_thread(
                    store.list_captures, camera_id=camera_id, limit=limit, node_id=node_id, label=label,
                    since=since,
                )
            except Exception as exc:
                raise _camera_error(exc) from exc
            return {"camera_id": camera_id, "captures": items, "count": len(items)}

        @app.get("/realsense/{camera_id}/captures/{capture_id}", tags=TAGS)
        async def realsense_capture_meta(camera_id: str, capture_id: str):
            """One capture's meta.json. Open read."""
            store = self.capture_store()
            self.camera(camera_id)
            try:
                meta = await asyncio.to_thread(store.get, camera_id, capture_id)
            except Exception as exc:
                raise _camera_error(exc) from exc
            return {"capture_id": capture_id, "camera_id": camera_id,
                    "urls": _capture_urls(camera_id, capture_id, meta), "meta": meta}

        @app.get("/realsense/{camera_id}/captures/{capture_id}/{filename}", tags=TAGS, dependencies=viewer)
        async def realsense_capture_file(camera_id: str, capture_id: str, filename: str):
            """One capture artefact, color.jpg or depth.png (16-bit)."""
            store = self.capture_store()
            self.camera(camera_id)
            if filename not in (COLOR_NAME, DEPTH_NAME):
                raise HTTPException(status_code=404, detail={"error": "capture_file_not_found", "filename": filename})
            try:
                data = await asyncio.to_thread(store.get_file, camera_id, capture_id, filename)
            except Exception as exc:
                raise _camera_error(exc) from exc
            media = "image/jpeg" if filename == COLOR_NAME else "image/png"
            return Response(content=data, media_type=media, headers={"Cache-Control": "no-store"})

        @app.delete("/control/realsense/{camera_id}/captures/{capture_id}", tags=TAGS, dependencies=take)
        async def realsense_capture_delete(camera_id: str, capture_id: str):
            """Delete one capture (the camera service also requires its admin token)."""
            store = self.capture_store()
            self.camera(camera_id)
            try:
                deleted = await asyncio.to_thread(store.delete, camera_id, capture_id)
            except Exception as exc:
                raise _camera_error(exc) from exc
            if not deleted:
                raise HTTPException(status_code=404, detail={"error": "capture_not_found", "capture_id": capture_id})
            return {"ok": True, "camera_id": camera_id, "capture_id": capture_id}
