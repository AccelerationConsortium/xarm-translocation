"""Tool-flange gripper: cached status for /status and claimed command routes.

Status (components.gripper, details.gripper, and the shared panel's
connection_details.gripper_type / gripper_config) comes from the driver's
read-only poll and never does I/O in the request. Commands are installed by
URControl only under control_enabled, behind the edge identity, the operator
allowlist and the hard claim. Every precondition refusal is a 412 decided by
gripper_block(), which also decides allowed_actions, so the two never
disagree (STATUS_SPEC section 6.2).

Paths are the ones the shared panel already calls for the xArm's gripper:
/gripper/open, /gripper/close, /control/freehand/gripper/stroke (alias
/gripper/move/stroke), /control/freehand/gripper/force (alias /gripper/force)
and /component/enable {"component": "gripper"}.
"""

from __future__ import annotations

import asyncio
from typing import Literal

from fastapi import HTTPException
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ConfigDict, Field
from sdl_lab_contract import ComponentStatus

from .drivers.robotiq import GripperBusy, GripperFailed, GripperRefused

MOVE_ACTIONS = ("gripper.open", "gripper.close", "gripper.move")


class GripperMotion(BaseModel):
    """Speed and force are percent of the gripper's range, capped by config."""

    model_config = ConfigDict(extra="forbid")
    speed: float | None = Field(default=None, ge=1, le=100, allow_inf_nan=False)
    force: float | None = Field(default=None, ge=0, le=100, allow_inf_nan=False)
    # Moves always wait for the fingers to stop; accepted for panel parity.
    wait: Literal[True] = True


class GripperStroke(GripperMotion):
    stroke: float = Field(ge=0, le=200, allow_inf_nan=False, description="Finger opening in mm")


class GripperForce(BaseModel):
    model_config = ConfigDict(extra="forbid")
    force: float = Field(ge=0, le=100, allow_inf_nan=False)


class ComponentRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    component: str = Field(min_length=1, max_length=32)


def component_status(gripper):
    summary = gripper.summary()
    state = summary["state"]
    message = None
    if state == "unknown":
        message = summary["error"] or "No fresh gripper status"
    elif state == "fault":
        message = summary["fault"]
    elif state == "disabled":
        message = "Not activated"
    return ComponentStatus(connected=summary["reachable"], state=state, message=message)


def connection_details(gripper, force_pct):
    s = gripper.settings
    return {
        "gripper_type": s.model,
        "gripper_config": {
            "name": s.name,
            "has_stroke_control": True,
            "has_force_control": True,
            "stroke_range": {"min": 0, "max": s.stroke_mm},
            "stroke_units": "mm opening",
            "force_range": {"min": 0, "max": s.max_force_pct},
            "force_units": "percent of maximum grip force",
            "force": force_pct,
            "speed_pct": s.default_speed_pct,
            "max_speed_pct": s.max_speed_pct,
        },
    }


class GripperControl:
    def __init__(self, control, gripper):
        self.control = control
        self.gripper = gripper
        self.settings = gripper.settings
        # Session force for moves that do not name one (the panel's force box).
        self.force_pct = self.settings.default_force_pct

    # ── the single precondition helper ──────────────────────────────
    def gripper_block(self, action):
        """412 body for why `action` cannot run now, or None."""
        summary = self.gripper.summary()
        if summary["state"] == "unknown":
            return {
                "detail": "Gripper status unavailable",
                "error": "gripper_unavailable",
                "reason": summary["error"],
            }
        _, problem = self.control.observed_robot()
        if problem is not None:
            return problem
        if action in MOVE_ACTIONS:
            if summary["fault_code"]:
                return {
                    "detail": f"Gripper fault: {summary['fault']}",
                    "error": "gripper_fault",
                    "fault_code": summary["fault_code"],
                    "hint": "POST /component/enable {\"component\": \"gripper\"} re-activates it",
                }
            if not summary["activated"]:
                return {
                    "detail": "Gripper is not activated",
                    "error": "gripper_not_activated",
                    "hint": "POST /component/enable {\"component\": \"gripper\"} (the fingers sweep to calibrate)",
                }
        return None

    def allowed_actions(self):
        if self.gripper.busy:
            return []
        actions = [a for a in (*MOVE_ACTIONS, "gripper.activate") if self.gripper_block(a) is None]
        if "gripper.activate" in actions and self.gripper.summary().get("activated"):
            actions.remove("gripper.activate")
        return actions

    # ── helpers ─────────────────────────────────────────────────────
    def _refuse(self, identity, action, body):
        self.control.event("gripper_refused", identity, action=action, reason=body.get("error"))
        raise HTTPException(status_code=412, detail=body)

    def _limits(self, identity, action, speed, force):
        speed = self.settings.default_speed_pct if speed is None else speed
        force = self.force_pct if force is None else force
        if speed > self.settings.max_speed_pct or force > self.settings.max_force_pct:
            self.control.event("gripper_refused", identity, action=action, reason="above_commissioned_limit")
            raise HTTPException(
                status_code=422,
                detail={
                    "error": "above_commissioned_limit",
                    "requested": {"speed_pct": speed, "force_pct": force},
                    "limits": {
                        "max_speed_pct": self.settings.max_speed_pct,
                        "max_force_pct": self.settings.max_force_pct,
                    },
                },
            )
        return speed, force

    async def _run(self, identity, action, call, **logged):
        block = self.gripper_block(action)
        if block is not None:
            self._refuse(identity, action, block)
        self.control.event(action, identity, **logged)
        try:
            result = await asyncio.to_thread(call)
        except GripperBusy as exc:
            raise HTTPException(status_code=409, detail={"error": "gripper_busy", "reason": str(exc)})
        except GripperRefused as exc:
            self.control.event("gripper_refused", identity, action=action, reason=str(exc))
            raise HTTPException(status_code=412, detail={"detail": str(exc), "error": "gripper_refused"})
        except GripperFailed as exc:
            self.control.event(
                "gripper_failed", identity, action=action, reason=str(exc), stop_error=exc.stop_error
            )
            return JSONResponse(
                status_code=500,
                content={
                    "error": "gripper_failed",
                    "reason": str(exc),
                    "stop_attempted": exc.stop_attempted,
                    "stop_error": exc.stop_error,
                    "hint": "Check the gripper and any held part before the next command",
                },
            )
        self.control.event(f"{action}_done", identity, **{k: v for k, v in result.items() if k != "requested_raw"})
        return {"ok": True, "action": action, **result}

    def _move(self, identity, action, position_raw, speed, force):
        speed, force = self._limits(identity, action, speed, force)
        g = self.gripper
        return self._run(
            identity,
            action,
            lambda: g.move(position_raw, g.pct_to_raw(speed), g.pct_to_raw(force)),
            position_raw=position_raw,
            speed_pct=speed,
            force_pct=force,
        )

    # ── routes ──────────────────────────────────────────────────────
    def install(self, app, login, claim):
        tags = ["gripper"]
        g, s = self.gripper, self.settings

        @app.get("/gripper/position", tags=tags)
        def gripper_position():
            """Cached read; no claim."""
            return g.summary()

        @app.post("/gripper/open", tags=tags, dependencies=[claim])
        async def gripper_open(body: GripperMotion | None = None, identity: dict = login):
            body = body or GripperMotion()
            return await self._move(identity, "gripper.open", s.open_raw, body.speed, body.force)

        @app.post("/gripper/close", tags=tags, dependencies=[claim])
        async def gripper_close(body: GripperMotion | None = None, identity: dict = login):
            body = body or GripperMotion()
            return await self._move(identity, "gripper.close", s.closed_raw, body.speed, body.force)

        @app.post("/control/freehand/gripper/stroke", tags=tags, dependencies=[claim])
        @app.post("/gripper/move/stroke", tags=tags, dependencies=[claim])
        async def gripper_stroke(body: GripperStroke, identity: dict = login):
            if body.stroke > s.stroke_mm:
                raise HTTPException(
                    status_code=422,
                    detail={"error": "stroke_out_of_range", "max_mm": s.stroke_mm},
                )
            position = min(max(g.mm_to_raw(body.stroke), s.open_raw), s.closed_raw)
            return await self._move(identity, "gripper.move", position, body.speed, body.force)

        @app.post("/control/freehand/gripper/force", tags=tags, dependencies=[claim])
        @app.post("/gripper/force", tags=tags, dependencies=[claim])
        def gripper_force(body: GripperForce, identity: dict = login):
            """Set the force used by later moves that name none; moves nothing."""
            if body.force > s.max_force_pct:
                raise HTTPException(
                    status_code=422,
                    detail={"error": "above_commissioned_limit", "max_force_pct": s.max_force_pct},
                )
            self.force_pct = body.force
            self.control.event("gripper_force", identity, force_pct=body.force)
            return {"ok": True, "force": body.force}

        @app.post("/component/enable", tags=tags, dependencies=[claim])
        async def component_enable(body: ComponentRequest, identity: dict = login):
            """Activate the gripper. If it is not already active, the fingers
            sweep through their full stroke to calibrate."""
            if body.component != "gripper":
                raise HTTPException(
                    status_code=404,
                    detail={"error": "unknown_component", "components": ["gripper"]},
                )
            return await self._run(identity, "gripper.activate", g.activate)

        return self
