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
the edge's shared-secret identity. Nothing here is claim-gated, and a camera
outage never changes the robot's equipment_status.
"""

from __future__ import annotations

import asyncio
import logging

import httpx
from fastapi import Depends, HTTPException, Request
from fastapi.responses import Response, StreamingResponse
from pydantic import BaseModel, Field
from core.camera_tracker import CameraTracker
from core.realsense_camera import RealSenseNotStreaming, RealSenseUnavailable

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
    }


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
        self.reason = "no camera_service in the local config"
        if stereo is not None:
            self.service, self.cameras = stereo
            self.reason = None
        elif settings.camera_service is not None:
            try:
                from core.remote_realsense import configure

                entries = [c.model_dump(exclude_none=True) for c in settings.camera_service.cameras]
                self.service, self.cameras, _store = configure(
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
    def install(self, app):
        viewer = [Depends(self.require_viewer)]

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
            cameras = [
                dict(camera, kind="realsense", capabilities=["color", "depth", "intrinsics", "snapshot", "mjpeg"])
                for camera in rs["cameras"]
            ]
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
            parts = camera.mjpeg_frames(kind, max_fps=fps)
            try:
                first = await asyncio.to_thread(next, parts, None)
            except Exception as exc:
                raise _camera_error(exc) from exc

            async def body():
                chunk = first
                while chunk is not None:
                    yield chunk
                    try:
                        chunk = await asyncio.to_thread(next, parts, None)
                    except RealSenseUnavailable:
                        return  # upstream ended mid-stream; the <img> reconnects

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

        return self
