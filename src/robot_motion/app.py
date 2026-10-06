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

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, PlainTextResponse
from sdl_lab_contract import EquipmentStatus, HealthResponse, ProbeResponse
from core.motion_graph import GraphError

from . import __version__
from .config import MODELS, Settings
from .drivers import inventory
from .drivers.ur import URObserver
from .graph import Graph, PreviewRequest

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
    "identity-checked single-joint steps under commissioning and is absent unless "
    "the local config enables it. MG400 support is planned. Existing xArm control "
    "uses the legacy application."
)
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

    async def poll():
        nonlocal observation, observed_at, observed_time, activity_since, previous_activity
        while True:
            try:
                read_task = asyncio.create_task(asyncio.to_thread(observer.read))
                try:
                    result = await asyncio.shield(read_task)
                except asyncio.CancelledError:
                    # Cancelling to_thread does not stop the native SDK call.
                    # Drain it before disconnecting the receiver at shutdown.
                    await asyncio.gather(read_task, return_exceptions=True)
                    raise
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

    @asynccontextmanager
    async def lifespan(_app):
        task = asyncio.create_task(poll()) if settings.observe else None
        try:
            yield
        finally:
            if task:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            if observer is not None and callable(getattr(observer, "close", None)):
                await asyncio.to_thread(observer.close)
            if control is not None:
                await asyncio.to_thread(control.shutdown)

    app = FastAPI(
        title="Robot Motion", version=__version__, lifespan=lifespan, description=NOTICE
    )
    app.state.settings = settings

    control = None
    if settings.control_enabled:
        from .control import URControl

        control = URControl(
            settings, session_factory=control_session_factory, edge_secret=edge_secret
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
                }
                if settings.observe
                else None
            ),
        }

    @app.get("/status", response_model=EquipmentStatus)
    def status():
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
            components=current["components"],
            last_error=current.get("last_error"),
            allowed_actions=control_state["allowed_actions"] if control_state else [],
            details={
                **current["details"],
                **measured(current["details"]),
                "monitoring_only": control is None,
                "control_enabled": control is not None,
                "control_implementation": "joint_step" if control is not None else "not_implemented",
                "claimed_by": control_state["claimed_by"] if control_state else None,
                "control_session": control_state["control_session"] if control_state else None,
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
        return "# Robot Motion\n\n- [Guide](/agent-docs)\n- [Reference](/agent-docs/api-reference)\n- [OpenAPI](/openapi.json)\n- [Status](/status)\n- [Drivers](/drivers)\n"

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

    @app.get("/auth/config", **shared)
    def auth_config():
        return {"enabled": False}

    @app.get("/auth/me", **shared)
    def auth_me():
        return {"authenticated": False, "identity": None}

    @app.get("/camera/config", **shared)
    def camera_config():
        return {"configured": False, "available": False, "connected": False}

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
        return []

    @app.websocket("/ws")
    async def status_push(websocket: WebSocket):
        # Same cached envelope as /status, pushed at the poll cadence. Browser
        # messages are drained only to notice the disconnect; none is acted on.
        await websocket.accept()

        async def push():
            try:
                while True:
                    await websocket.send_text(
                        json.dumps(
                            {"type": "status_update", "data": status().model_dump(mode="json")}
                        )
                    )
                    await asyncio.sleep(settings.poll_interval_s)
            except (WebSocketDisconnect, RuntimeError):
                return

        pusher = asyncio.create_task(push())
        try:
            while True:
                await websocket.receive_text()
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            pusher.cancel()

    @app.get("/web", include_in_schema=False)
    @app.get("/web/", include_in_schema=False)
    @app.get("/web/{asset}", include_in_schema=False)
    def shared_ui(asset: str = "index.html"):
        if asset not in SHARED_UI_FILES:
            raise HTTPException(status_code=404, detail="Unknown shared UI asset")
        return FileResponse(
            str(files("web").joinpath(asset)), media_type=SHARED_UI_FILES[asset]
        )

    return app
