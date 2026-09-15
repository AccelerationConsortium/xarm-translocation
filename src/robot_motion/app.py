"""Bundled prototyping UI and side-effect-free STATUS_SPEC observation service."""

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

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles
from sdl_lab_contract import EquipmentStatus, HealthResponse, ProbeResponse
from core.motion_graph import GraphError

from . import __version__
from .config import Settings
from .drivers import inventory
from .drivers.ur import URObserver
from .graph import Graph, PreviewRequest

log = logging.getLogger(__name__)
NOTICE = (
    "Prototype only. UR physical control is not implemented in this release. "
    "MG400 support is planned. Existing xArm control uses the legacy application."
)
SAFETY = (
    "Topology preview only: no collision, reachability, joint-limit, payload, "
    "TCP calibration, gripper, or physical clearance validation; not executable authorization."
)


def create_app(settings: Settings | None = None, *, observer=None):
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

    app = FastAPI(
        title="Robot Motion", version=__version__, lifespan=lifespan, description=NOTICE
    )
    app.state.settings = settings

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
            allowed_actions=[],
            details={
                **current["details"],
                "monitoring_only": True,
                "control_enabled": False,
                "control_implementation": "not_implemented",
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

    # No /control routes: claims are N/A in this observation/preview service.
    # The isolated legacy-xarm application retains its original hard claims.
    @app.get("/web/pyxarm/{asset}", include_in_schema=False)
    def shared_web_asset(asset: str):
        # Reuse the packaged assets byte-for-byte, not the legacy application
        # or its command-sending JavaScript. Never mount the whole web package:
        # that would also publish server.py and the xArm control pages.
        media_types = {
            "style.css": "text/css",
            "cytoscape.min.js": "text/javascript",
        }
        if asset not in media_types:
            raise HTTPException(status_code=404, detail="Unknown shared UI asset")
        return FileResponse(
            str(files("web").joinpath(asset)), media_type=media_types[asset]
        )

    app.mount(
        "/web",
        StaticFiles(directory=str(files("robot_motion").joinpath("web")), html=True),
        name="web",
    )
    return app
