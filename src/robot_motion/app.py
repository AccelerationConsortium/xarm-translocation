"""Side-effect-free STATUS_SPEC observation service serving the shared web UI."""

from __future__ import annotations

import asyncio
import json
import logging
import socket
import time
from contextlib import asynccontextmanager
from datetime import datetime, timezone
from importlib.resources import files
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, PlainTextResponse
from sdl_lab_contract import ComponentStatus, EquipmentStatus, HealthResponse, ProbeResponse
from core.motion_graph import GraphError

from . import __version__
from .cameras import Cameras
from .config import MODELS, Settings
from .drivers import inventory
from .drivers.ur import URObserver
from .edge import configured_secret, edge_identity
from .graph import Graph, PreviewRequest
from .gripper import component_status as gripper_component

log = logging.getLogger(__name__)
# The shared xArm web UI (src/web) is served byte-for-byte from this allowlist.
# Never mount the whole package: that would also publish server.py. Query
# strings (the UI's ?v= cache keys) are ignored by path routing.
SHARED_UI_FILES = {
    "index.html": "text/html",
    "graph.html": "text/html",
    "main.js": "text/javascript",
    "graph.js": "text/javascript",
    "workspace.js": "text/javascript",
    "camera-player.js": "text/javascript",
    "realsense-card.js": "text/javascript",
    "cytoscape.min.js": "text/javascript",
    "style.css": "text/css",
    "graph.css": "text/css",
    "workspace.css": "text/css",
}
NOT_PRESENT = "Not present on this service; shared UI compatibility answer only."
NOTICE = (
    "Prototype only. UR physical control is limited to config-gated, claimed, "
    "identity-checked arm moves inside commissioned joint and workspace limits "
    "and Robotiq gripper commands, and is absent unless the local config enables "
    "it. MG400 support is planned. Existing xArm control uses the legacy application."
)
TELEMETRY_UNAVAILABLE = {"valid": False, "source": "rtde_receive"}
SAFETY = (
    "Topology preview only: no collision, reachability, joint-limit, payload, "
    "TCP calibration, gripper, or physical clearance validation; not executable authorization."
)


def create_app(
    settings: Settings | None = None,
    *,
    observer=None,
    control_session_factory=None,
    edge_secret=None,
    stereo_cameras=None,
    camera_transport=None,
    gripper=None,
):
    settings = settings or Settings()
    observation = None
    observed_at = None
    observed_time = None
    activity_since = None
    previous_activity = "unknown"
    started = time.monotonic()
    graph = None
    if settings.graph_file:
        graph = Graph.model_validate_json(
            Path(settings.graph_file).read_text(encoding="utf-8-sig")
        )
        if settings.model and graph.robot_model != settings.model:
            raise ValueError("Configured graph belongs to a different robot model")

    if observer is None and settings.observe:
        if settings.ur_transport == "rtde":
            from .drivers.ur_rtde import URRTDEObserver

            observer = URRTDEObserver(settings)
        else:
            observer = URObserver(settings)
    # RTDE observers split the read: Dashboard state on the slow poll, joints,
    # TCP and force on a fast one, so live values do not wait 10 s.
    fast_telemetry = (
        settings.observe
        and settings.ur_transport == "rtde"
        and callable(getattr(observer, "read_telemetry", None))
        and callable(getattr(observer, "read_dashboard", None))
    )
    telemetry = {"sample": None, "error": None, "at": None}

    async def threaded(read):
        task = asyncio.create_task(asyncio.to_thread(read))
        try:
            return await asyncio.shield(task)
        except asyncio.CancelledError:
            # Cancelling to_thread does not stop the native SDK call.
            # Drain it before disconnecting the receiver at shutdown.
            await asyncio.gather(task, return_exceptions=True)
            raise

    async def poll():
        nonlocal observation, observed_at, observed_time, activity_since, previous_activity
        read = observer.read_dashboard if fast_telemetry else observer.read
        while True:
            try:
                result = await threaded(read)
            except Exception:
                log.exception("Read-only robot observation failed")
                result = {
                    "equipment_status": "unknown",
                    "activity": "unknown",
                    "message": "Robot observation unavailable; see service log",
                    "components": {},
                    "details": {},
                }
            now = datetime.now(timezone.utc)
            activity = result["activity"]
            if activity != previous_activity:
                previous_activity, activity_since = activity, now
            observation, observed_at = result, time.monotonic()
            observed_time = now
            await asyncio.sleep(settings.poll_interval_s)

    async def poll_telemetry():
        while True:
            try:
                sample, error = await threaded(observer.read_telemetry), None
            except Exception as exc:  # noqa: BLE001 - a broken stream reads as absent
                sample, error = None, str(exc)
                if error != telemetry["error"]:
                    log.exception("RTDE receive-only telemetry unavailable")
            if error is None and telemetry["error"] is not None:
                log.info("RTDE receive-only telemetry restored")
            telemetry.update(sample=sample, error=error, at=time.monotonic())
            await asyncio.sleep(settings.telemetry_interval_s)

    def with_telemetry(current, stale):
        """Overlay the fast sample on the Dashboard observation. A missing or
        old sample is reported as unavailable, never as the last position."""
        if not fast_telemetry or stale:
            return current
        age_limit = max(3 * settings.telemetry_interval_s, 2.0)
        fresh = telemetry["at"] is not None and time.monotonic() - telemetry["at"] <= age_limit
        sample = telemetry["sample"] if fresh else None
        component = (
            ComponentStatus(connected=True, state="receiving")
            if sample
            else ComponentStatus(
                connected=False, state="unknown", message="RTDE telemetry unavailable; see service log"
            )
        )
        return {
            **current,
            "components": {**current["components"], "telemetry": component},
            "details": {**current["details"], "telemetry": sample or dict(TELEMETRY_UNAVAILABLE)},
        }

    cameras = Cameras(
        settings, edge_secret=edge_secret, stereo=stereo_cameras, transport=camera_transport
    )
    if gripper is None and settings.gripper is not None:
        from .drivers.robotiq import RobotiqGripper

        gripper = RobotiqGripper(settings.gripper, settings.robot_host)

    @asynccontextmanager
    async def lifespan(_app):
        task = asyncio.create_task(poll()) if settings.observe else None
        telemetry_task = asyncio.create_task(poll_telemetry()) if fast_telemetry else None
        # Camera telemetry is polled on its own thread; /status only reads it.
        cameras.start()
        if gripper is not None:
            gripper.start()
        try:
            yield
        finally:
            # The arm first: stop any move and close its session while the
            # service manager's shutdown allowance is still running.
            if control is not None:
                await asyncio.to_thread(control.shutdown)
            for running in (task, telemetry_task):
                if running:
                    running.cancel()
                    try:
                        await running
                    except asyncio.CancelledError:
                        pass
            if observer is not None and callable(getattr(observer, "close", None)):
                await asyncio.to_thread(observer.close)
            await asyncio.to_thread(cameras.close)
            if gripper is not None:
                await asyncio.to_thread(gripper.close)

    app = FastAPI(
        title="Robot Motion", version=__version__, lifespan=lifespan, description=NOTICE
    )
    app.state.settings = settings

    def current_observation():
        stale = (
            observed_at is None
            or time.monotonic() - observed_at > settings.poll_interval_s + 30
        )
        current = (
            observation
            if observation is not None and not stale
            else {
                "equipment_status": "unknown",
                "activity": "unknown",
                "components": {},
                "details": {},
                "message": (
                    "No fresh robot observation"
                    if settings.observe
                    else "Observation disabled; no robot connection"
                ),
            }
        )
        return current, stale

    control = None
    if settings.control_enabled:
        from .control import URControl

        control = URControl(
            settings,
            session_factory=control_session_factory,
            edge_secret=edge_secret,
            observe=current_observation,
            gripper=gripper,
        ).install(app)

    @app.get("/", response_model=ProbeResponse)
    def probe():
        return ProbeResponse(
            equipment_id=settings.equipment_id,
            equipment_name=settings.equipment_name,
            protocol_version="1.2",
        )

    @app.get("/health", response_model=HealthResponse)
    def health():
        return HealthResponse(status="healthy")

    def gripper_details():
        # Cached by the driver's poll; never a request to the gripper here.
        if gripper is None:
            return {}
        from .gripper import connection_details

        force = (
            control.gripper_control.force_pct
            if control is not None and control.gripper_control is not None
            else gripper.settings.default_force_pct
        )
        return connection_details(gripper, force)

    def measured(details):
        # The shared UI reads xArm-named keys. Fill them only from a valid RTDE
        # sample so a missing or stale stream never shows as a position.
        telemetry = details.get("telemetry") or {}
        valid = telemetry.get("valid") is True
        return {
            "current_joints": telemetry.get("joints_deg") if valid else None,
            "current_position": telemetry.get("tcp_mm_rpy_deg") if valid else None,
            "num_joints": MODELS[settings.model]["joints"] if settings.model else None,
            "connection_details": (
                {
                    "host": settings.robot_host,
                    "port": 30004 if settings.ur_transport == "rtde" else 29999,
                    "profile_name": settings.model,
                    **gripper_details(),
                }
                if settings.observe
                else None
            ),
        }

    @app.get("/status", response_model=EquipmentStatus)
    def status():
        current, stale = current_observation()
        current = with_telemetry(current, stale)
        control_state = control.state() if control is not None else None
        return EquipmentStatus(
            protocol_version="1.2",
            equipment_id=settings.equipment_id,
            equipment_name=settings.equipment_name,
            equipment_kind="robot_arm",
            equipment_version=__version__,
            host=socket.gethostname(),
            device_time=datetime.now(timezone.utc),
            uptime_seconds=time.monotonic() - started,
            equipment_status=current["equipment_status"],
            activity=current["activity"],
            activity_since=activity_since if not stale else None,
            message=current["message"],
            # Cameras are components, not state inputs: arm observation does
            # not depend on them, so an outage never moves equipment_status.
            components={
                **cameras.components(),
                **current["components"],
                **({"gripper": gripper_component(gripper)} if gripper is not None else {}),
            },
            last_error=current.get("last_error"),
            allowed_actions=control_state["allowed_actions"] if control_state else [],
            details={
                **cameras.details(),
                **current["details"],
                **measured(current["details"]),
                **({"gripper": gripper.summary()} if gripper is not None else {}),
                "monitoring_only": control is None,
                "control_enabled": control is not None,
                "control_implementation": (
                    "+".join(
                        part
                        for part, present in (
                            ("joint_step", settings.control.joint_step is not None),
                            ("motion", settings.control.motion is not None),
                            ("gripper", gripper is not None),
                        )
                        if present
                    )
                    if control is not None
                    else "not_implemented"
                ),
                "claimed_by": control_state["claimed_by"] if control_state else None,
                "control_session": control_state["control_session"] if control_state else None,
                # The panel's Manual switch reads this (the xArm's field name).
                "manual_mode": bool(control_state and control_state["control_session"].get("manual_mode")),
                "driver": settings.driver,
                "model": settings.model,
                "primary_operation": "controller program playing; not proof of physical arm movement",
                "notice": NOTICE,
                "graph_loaded": graph is not None,
                "observation_enabled": settings.observe,
                "observed_time": (
                    observed_time.isoformat()
                    if observed_time is not None and not stale
                    else None
                ),
                "observation_transport": (
                    settings.ur_transport if settings.driver == "ur" else None
                ),
            },
        )

    @app.get("/drivers")
    def drivers():
        return inventory()

    @app.get("/graph")
    def configured_graph():
        return {"graph": graph.model_dump() if graph else None, "notice": SAFETY}

    @app.post("/graph/validate")
    def validate_graph(body: Graph):
        return {
            "valid_topology": True,
            "physical_validation": False,
            "nodes": len(body.nodes),
            "edges": len(body.edges),
            "notice": SAFETY,
        }

    @app.post("/graph/preview")
    def preview(body: PreviewRequest):
        try:
            path = body.graph.path(body.source, body.target)
        except GraphError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc
        return {
            "path": [body.source, *path],
            "executed": False,
            "physical_validation": False,
            "notice": SAFETY,
        }

    @app.get("/agent-docs", response_class=PlainTextResponse)
    def guide():
        return PlainTextResponse(
            files("robot_motion")
            .joinpath("docs/AGENT_GUIDE.md")
            .read_text(encoding="utf-8"),
            media_type="text/markdown",
        )

    @app.get("/agent-docs/api-reference", response_class=PlainTextResponse)
    def reference():
        schema = app.openapi()
        lines = ["# Robot Motion API reference", "", NOTICE, ""]
        for path, methods in schema["paths"].items():
            for method, operation in methods.items():
                lines += [
                    f"## {method.upper()} {path}",
                    "",
                    json.dumps(operation, indent=2),
                    "",
                ]
        lines += ["## Schemas", "", json.dumps(schema.get("components", {}), indent=2)]
        return PlainTextResponse("\n".join(lines), media_type="text/markdown")

    @app.get("/llms.txt", response_class=PlainTextResponse)
    def llms():
        return "# Robot Motion\n\n- [Guide](/agent-docs)\n- [Reference](/agent-docs/api-reference)\n- [OpenAPI](/openapi.json)\n- [Status](/status)\n- [Drivers](/drivers)\n- [Cameras](/cameras)\n"

    # Without control_enabled there are no /control routes: claims are N/A and
    # the shared UI's Take Control / motion buttons get 404s. The legacy xArm
    # application retains its own hard claims.

    # Read-only polls the shared UI issues on load. Each answers truthfully
    # that the feature is absent so the page renders without inventing state.
    shared = {"tags": ["shared-ui"], "summary": NOT_PRESENT}

    @app.get("/graph/layout", **shared)
    def graph_layout():
        return {"positions": {}, "expanded": {}, "pan": None, "zoom": None}

    @app.get("/locations", **shared)
    @app.get("/track/locations", **shared)
    def named_locations():
        return {"locations": [], "positions": {}}

    @app.get("/interlocks/sash", **shared)
    def sash_interlock():
        return {"configured": False, "connected": False, "notice": NOT_PRESENT}

    # Behind the lab's edge the human is already signed in; report that
    # identity (verified by the shared secret) so the panel shows who is
    # signed in and never renders its own login form. Direct callers get none.
    secret = configured_secret(edge_secret)

    @app.get("/auth/config", **shared)
    def auth_config():
        return {"enabled": secret is not None}

    @app.get("/auth/me", **shared)
    def auth_me(request: Request):
        identity = edge_identity(request, secret)
        if identity is None:
            return {"authenticated": False, "identity": None}
        return {
            "authenticated": True,
            "identity": {"email": identity["email"], "role": identity["role"] or "user", "via": "edge"},
        }

    # /camera/* and /realsense/*: real routes, configured by the local
    # lab_camera / camera_service blocks (see cameras.py).
    cameras.install(app)

    @app.get("/assistant/status", **shared)
    def assistant_status():
        return {
            "enabled": False,
            "reason": NOT_PRESENT,
            "model": None,
            "graph_loaded": graph is not None,
            "places": [],
        }

    @app.get("/api/configurations", **shared)
    def configurations():
        # The panel's "Connect to" list: only the configured robot profile.
        # The value is the model id; main.js maps it to a display label.
        return [settings.model] if settings.model else []

    # With fast telemetry the push follows it, so the panel's joint and TCP
    # read-outs move live (fresh pushes also stand down its HTTP polling).
    push_interval = settings.telemetry_interval_s if fast_telemetry else settings.poll_interval_s

    @app.websocket("/ws")
    async def status_push(websocket: WebSocket):
        # Same cached envelope as /status, pushed at the push cadence. Browser
        # messages are drained only to notice the disconnect; none is acted on.
        await websocket.accept()

        async def push():
            try:
                while True:
                    # status() can wait on the control SDK lock: never on the loop.
                    envelope = await asyncio.to_thread(status)
                    await websocket.send_text(
                        json.dumps({"type": "status_update", "data": envelope.model_dump(mode="json")})
                    )
                    await asyncio.sleep(push_interval)
            except (WebSocketDisconnect, RuntimeError):
                return
            except Exception:
                log.exception("Status push stopped")
                return

        pusher = asyncio.create_task(push())
        try:
            while True:
                await websocket.receive_text()
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            pusher.cancel()

    model_label = {"ur3e": "UR3e", "ur5e": "UR5e", "ur5_cb3": "UR5 CB3", "mg400": "MG400"}
    page_title = f"{model_label.get(settings.model, 'Robot Motion')} Control"

    @app.get("/web", include_in_schema=False)
    @app.get("/web/", include_in_schema=False)
    @app.get("/web/{asset}", include_in_schema=False)
    def shared_ui(asset: str = "index.html"):
        if asset not in SHARED_UI_FILES:
            raise HTTPException(status_code=404, detail="Unknown shared UI asset")
        path = files("web").joinpath(asset)
        if asset == "index.html":
            # The shared page is the xArm panel; only its tab title names the
            # robot. Swap that one tag for the configured model, nothing else.
            html = path.read_text(encoding="utf-8").replace(
                "<title>xArm Control</title>", f"<title>{page_title}</title>", 1
            )
            return HTMLResponse(html, headers={"Cache-Control": "no-store"})
        return FileResponse(str(path), media_type=SHARED_UI_FILES[asset])

    return app
