"""Named-node motion and bench routines on top of the arm's move executor.

`POST /graph/move` walks the configured motion graph from the node the arm is
at to a target node, one edge at a time: a joint edge is a moveJ to the
target node's joints, a linear edge a moveL to its TCP pose. Every edge is an
ordinary arm move (`_run_motion`), so the claim, the commissioned envelope,
the speed caps, the force guard and the STOP latch apply to each one exactly
as they do to a panel move. Nothing here sends anything the panel could not.

Routines are fixed sequences of graph moves, rack moves and gripper moves
with the preconditions the bench needs (the Gibbie vial rack and XPR
balance: `routines_file`). A routine stops at the first step that is refused
or fails, and reports which steps already ran: the arm may have moved, and
nothing is rewound or retried. The person at the bench reconciles.

Not a safety function: the graph says which moves were taught, not that the
cell is clear. The controller's safety configuration, the pendant and the
e-stop remain the safety system.
"""

from __future__ import annotations

import json
import string
from typing import Annotated

import httpx
from fastapi import Header, HTTPException
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ConfigDict, Field, model_validator
from core.motion_graph import GraphError

from .drivers.ur_motion import JointMove, LinearMove
from .graph import Graph, Identifier

Number = Annotated[float, Field(allow_inf_nan=False)]
Six = tuple[Number, Number, Number, Number, Number, Number]
Raw = Annotated[int, Field(ge=0, le=255)]

GRAPH_ACTIONS = ("graph.move",)
ROUTINE_ACTIONS = ("routine.vial_rack_to_balance", "routine.vial_balance_to_rack")


# ── bench configuration (routines_file) ──────────────────────────────
class VialRack(BaseModel):
    """A grid of vial slots, columns A.. along -x and rows 1.. along -y from
    A1 (the sample-prep workflow's resource_handler layout). Every slot has
    A1's orientation."""

    model_config = ConfigDict(extra="forbid", frozen=True)
    approach_node: Identifier
    a1_tcp_mm_rpy_deg: Six
    columns: int = Field(ge=1, le=26)
    rows: int = Field(ge=1, le=26)
    column_pitch_mm: Number = Field(gt=0, le=200)
    row_pitch_mm: Number = Field(gt=0, le=200)
    # Straight down from the slot pose to grip a vial, and to release one.
    pick_depth_mm: Number = Field(gt=0, le=150)
    place_depth_mm: Number = Field(gt=0, le=150)

    def slot_pose(self, slot):
        letters = string.ascii_uppercase[: self.columns]
        if (
            len(slot) < 2
            or slot[0] not in letters
            or not slot[1:].isdigit()
            or not 1 <= int(slot[1:]) <= self.rows
        ):
            raise KeyError(slot)
        x, y, z, *rpy = self.a1_tcp_mm_rpy_deg
        column, row = letters.index(slot[0]), int(slot[1:]) - 1
        return [x - self.column_pitch_mm * column, y - self.row_pitch_mm * row, z, *rpy]

    def slots(self):
        return [f"{c}{r}" for c in string.ascii_uppercase[: self.columns] for r in range(1, self.rows + 1)]


class Grip(BaseModel):
    """Raw Robotiq positions for one part: fingers apart, and the close
    command (beyond the part, so the fingers stop on it)."""

    model_config = ConfigDict(extra="forbid", frozen=True)
    open_raw: Raw
    close_raw: Raw

    @model_validator(mode="after")
    def ordered(self):
        if self.close_raw <= self.open_raw:
            raise ValueError("close_raw must be above open_raw")
        return self


class BalancePort(BaseModel):
    """Where a vial sits in the balance, and the door the arm passes."""

    model_config = ConfigDict(extra="forbid", frozen=True)
    drop_node: Identifier
    pick_node: Identifier
    # The balance service's STATUS_SPEC envelope; the routine reads it and
    # never commands the balance. The door must read exactly "open".
    status_url: str = Field(min_length=1, max_length=500)
    door_component: str = Field(default="door_right", min_length=1, max_length=64)
    timeout_s: Number = Field(default=3.0, gt=0, le=10)


class RoutineSettings(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    home_node: Identifier
    # The arm is "at" a node when every joint is within this of it.
    node_tolerance_deg: Number = Field(default=1.0, gt=0, le=5)
    vial_rack: VialRack
    vial_grip: Grip
    balance: BalancePort
    # None: the gripper's configured defaults. Above its caps is refused.
    grip_speed_pct: Number | None = Field(default=None, gt=0, le=100)
    grip_force_pct: Number | None = Field(default=None, ge=0, le=100)

    def check_against(self, graph):
        nodes = {n.id for n in graph.nodes}
        named = {
            "home_node": self.home_node,
            "vial_rack.approach_node": self.vial_rack.approach_node,
            "balance.drop_node": self.balance.drop_node,
            "balance.pick_node": self.balance.pick_node,
        }
        missing = {k: v for k, v in named.items() if v not in nodes}
        if missing:
            raise ValueError(f"Routine nodes missing from the graph: {missing}")


# ── requests ─────────────────────────────────────────────────────────
class GraphMove(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    target: Identifier
    dry_run: bool = False


class VialTask(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    slot: str = Field(pattern=r"^[A-Z][0-9]{1,2}$")
    dry_run: bool = False


def _detail(exc):
    return exc.detail if isinstance(exc.detail, dict) else {"reason": exc.detail}


async def read_door(port: BalancePort):
    """The door component's state from the balance's /status, or raise."""
    async with httpx.AsyncClient(timeout=port.timeout_s) as client:
        response = await client.get(port.status_url)
        response.raise_for_status()
        component = (response.json().get("components") or {}).get(port.door_component)
    if not isinstance(component, dict) or "state" not in component:
        raise ValueError(f"balance status has no {port.door_component!r} component")
    return component["state"]


class Routines:
    def __init__(self, control, graph: Graph, settings: RoutineSettings | None, *, door_reader=read_door):
        if settings is not None:
            settings.check_against(graph)
            if control.gripper_control is None:
                raise ValueError("Routines that grip need a configured gripper")
        self.control = control
        self.graph = graph
        self.settings = settings
        self.nodes = {n.id: n for n in graph.nodes}
        self.modes = {(e.source, e.target): e.mode for e in graph.edges}
        self.read_door = door_reader
        self._running = None  # name of the routine in progress

    # ── where the arm is ────────────────────────────────────────────
    def locate(self):
        """(node id or None, nearest node id, its largest joint deviation)."""
        session = self.control.session
        feedback = getattr(session, "feedback", None) if session is not None else None
        sample = feedback.latest() if feedback is not None else None
        if sample is None:
            raise HTTPException(
                status_code=409,
                detail={"error": "no_control_session", "reason": "No arm session; Connect first"},
            )
        tolerance = self.settings.node_tolerance_deg if self.settings else 1.0
        best, best_dev = None, None
        for node in self.graph.nodes:
            dev = max(abs(a - b) for a, b in zip(sample["joints_deg"], node.joints_deg))
            if best_dev is None or dev < best_dev:
                best, best_dev = node.id, dev
        return (best if best_dev is not None and best_dev <= tolerance else None), best, best_dev

    def _at(self, required):
        node, nearest, deviation = self.locate()
        if node != required:
            raise HTTPException(
                status_code=412,
                detail={
                    "error": "not_at_node",
                    "reason": f"The arm must start at {required!r}",
                    "required": required,
                    "at": node,
                    "nearest": nearest,
                    "nearest_deviation_deg": round(deviation, 3),
                },
            )

    # ── step plans ──────────────────────────────────────────────────
    def path_steps(self, source, target):
        if target not in self.nodes:
            raise HTTPException(status_code=422, detail={"error": "unknown_node", "node": target})
        if source == target:
            return []
        path = [source, *self.graph.path(source, target)]
        steps = []
        for a, b in zip(path, path[1:]):
            if self.modes[(a, b)] == "joint":
                steps.append({"kind": "joint", "to": b, "angles": list(self.nodes[b].joints_deg)})
            else:
                steps.append({"kind": "linear", "to": b, "pose": list(self.nodes[b].tcp_mm_rpy_deg)})
        return steps

    def _grip(self, raw, label, expect_object):
        return {"kind": "gripper", "to": label, "raw": raw, "expect_object": expect_object}

    def rack_to_balance_steps(self, slot):
        s = self.settings
        rack, grip, port = s.vial_rack, s.vial_grip, s.balance
        above = rack.slot_pose(slot)
        down = [*above[:2], above[2] - rack.pick_depth_mm, *above[3:]]
        return [
            self._grip(grip.open_raw, "vial_open", False),
            *self.path_steps(s.home_node, rack.approach_node),
            {"kind": "linear", "to": f"slot_{slot}", "pose": above},
            {"kind": "linear", "to": f"slot_{slot}_grip", "pose": down},
            self._grip(grip.close_raw, "vial_close", True),
            {"kind": "linear", "to": f"slot_{slot}", "pose": above},
            {"kind": "linear", "to": rack.approach_node, "pose": list(self.nodes[rack.approach_node].tcp_mm_rpy_deg)},
            *self.path_steps(rack.approach_node, port.drop_node),
            self._grip(grip.open_raw, "vial_open", False),
            *self.path_steps(port.drop_node, s.home_node),
        ]

    def balance_to_rack_steps(self, slot):
        s = self.settings
        rack, grip, port = s.vial_rack, s.vial_grip, s.balance
        above = rack.slot_pose(slot)
        down = [*above[:2], above[2] - rack.place_depth_mm, *above[3:]]
        return [
            self._grip(grip.open_raw, "vial_open", False),
            *self.path_steps(s.home_node, port.pick_node),
            self._grip(grip.close_raw, "vial_close", True),
            *self.path_steps(port.pick_node, rack.approach_node),
            {"kind": "linear", "to": f"slot_{slot}", "pose": above},
            {"kind": "linear", "to": f"slot_{slot}_place", "pose": down},
            self._grip(grip.open_raw, "vial_open", False),
            # Straight up out of the slot before moving sideways.
            {"kind": "linear", "to": f"slot_{slot}", "pose": above},
            {"kind": "linear", "to": rack.approach_node, "pose": list(self.nodes[rack.approach_node].tcp_mm_rpy_deg)},
            *self.path_steps(rack.approach_node, s.home_node),
        ]

    # ── execution ───────────────────────────────────────────────────
    async def _step(self, identity, token, step):
        if step["kind"] == "joint":
            body = JointMove(angles=tuple(float(v) for v in step["angles"]))
            return await self.control._run_motion(identity, "arm.move_joints", body, token)
        if step["kind"] == "linear":
            x, y, z, roll, pitch, yaw = (float(v) for v in step["pose"])
            body = LinearMove(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)
            return await self.control._run_motion(identity, "arm.move_linear", body, token)
        speed = self.settings.grip_speed_pct if self.settings else None
        force = self.settings.grip_force_pct if self.settings else None
        return await self.control.gripper_control._move(identity, "gripper.move", step["raw"], speed, force)

    async def run(self, identity, token, name, steps, **logged):
        """Run steps in order; stop at the first refusal or failure."""
        if self._running is not None:
            raise HTTPException(
                status_code=409,
                detail={"error": "routine_in_progress", "running": self._running},
            )
        self._running = name
        done = []

        def where(index):
            return {
                "routine": name,
                "failed_step": index,
                "step": steps[index]["to"],
                "completed_steps": [s["to"] for s in done],
                "arm_may_have_moved": bool(done),
            }

        self.control.event(name, identity, steps=len(steps), **logged)
        try:
            for index, step in enumerate(steps):
                try:
                    result = await self._step(identity, token, step)
                except HTTPException as exc:
                    self.control.event(f"{name}_refused", identity, step=step["to"], status=exc.status_code)
                    raise HTTPException(status_code=exc.status_code, detail={**_detail(exc), **where(index)})
                if isinstance(result, JSONResponse):
                    self.control.event(f"{name}_failed", identity, step=step["to"])
                    return JSONResponse(
                        status_code=result.status_code,
                        content={**_json(result), **where(index), "error": "routine_failed",
                                 "cause": _json(result).get("error")},
                    )
                if step["kind"] == "gripper" and step["expect_object"] and not result.get("object_detected"):
                    self.control.event(f"{name}_failed", identity, step=step["to"], reason="no_object")
                    return JSONResponse(
                        status_code=500,
                        content={
                            "error": "routine_failed",
                            "cause": "no_object_gripped",
                            "reason": "The fingers closed without stopping on a part",
                            "gripper": result,
                            **where(index),
                            "hint": "The arm is stopped where it gripped; check the rack and the "
                            "gripper before anything else moves",
                        },
                    )
                done.append(step)
        finally:
            self._running = None
        self.control.event(f"{name}_done", identity, steps=len(steps), **logged)
        return {"ok": True, "routine": name, "steps": [s["to"] for s in steps], **logged}

    # ── preconditions ───────────────────────────────────────────────
    def _gripper_empty(self):
        holding = self.control.gripper_control.gripper.summary().get("object")
        if holding in ("contact_closing", "contact_opening"):
            raise HTTPException(
                status_code=412,
                detail={"error": "gripper_holding", "reason": "The gripper reports a part between its fingers"},
            )

    async def _door_open(self):
        port = self.settings.balance
        try:
            state = await self.read_door(port)
        except Exception as exc:  # noqa: BLE001 - any failure means unverified
            raise HTTPException(
                status_code=412,
                detail={"error": "balance_door_unknown", "reason": f"Could not read the balance door: {exc}"},
            )
        if state != "open":
            raise HTTPException(
                status_code=412,
                detail={
                    "error": "balance_door_not_open",
                    "reason": f"The balance's {port.door_component} reads {state!r}; open it first",
                    "door": port.door_component,
                    "state": state,
                },
            )

    def _slot_steps(self, build, slot):
        try:
            return build(slot)
        except KeyError:
            raise HTTPException(
                status_code=422,
                detail={"error": "unknown_slot", "slot": slot, "slots": self.settings.vial_rack.slots()},
            )

    async def _vial_task(self, identity, token, name, build, body):
        steps = self._slot_steps(build, body.slot)
        if body.dry_run:
            return {"ok": True, "dry_run": True, "routine": name, "slot": body.slot, "plan": steps}
        self._at(self.settings.home_node)
        self._gripper_empty()
        await self._door_open()
        return await self.run(identity, token, name, steps, slot=body.slot)

    # ── status and routes ───────────────────────────────────────────
    def advertised_actions(self, move_block):
        if move_block is not None or self._running is not None:
            return []
        return [*GRAPH_ACTIONS, *(ROUTINE_ACTIONS if self.settings else ())]

    def summary(self):
        s = self.settings
        return {
            "running": self._running,
            "nodes": list(self.nodes),
            "home_node": s.home_node if s else None,
            "vial_rack_slots": s.vial_rack.slots() if s else [],
        }

    def install(self, app, login, claim):
        tags = ["routines"]

        @app.post("/graph/move", tags=tags, dependencies=[claim])
        async def graph_move(body: GraphMove, identity: dict = login, x_claim_token: str | None = Header(default=None)):
            """Walk the motion graph from the node the arm is at to `target`,
            one ordinary arm move per edge. Refused unless the arm is at a node."""
            node, nearest, deviation = self.locate()
            if node is None:
                raise HTTPException(
                    status_code=412,
                    detail={
                        "error": "not_at_node",
                        "reason": "The arm is not at a graph node",
                        "nearest": nearest,
                        "nearest_deviation_deg": round(deviation, 3),
                    },
                )
            try:
                steps = self.path_steps(node, body.target)
            except GraphError as exc:
                raise HTTPException(status_code=422, detail={"error": "no_path", "reason": str(exc)})
            if body.dry_run:
                return {"ok": True, "dry_run": True, "from": node, "target": body.target, "plan": steps}
            return await self.run(identity, x_claim_token, "graph.move", steps, source=node, target=body.target)

        if self.settings is None:
            return

        @app.get("/routines", tags=tags)
        def routines():
            """The bench routines, rack slots and graph nodes. Read-only."""
            return {
                **self.summary(),
                "routines": {
                    "vial_rack_to_balance": "POST /routines/vial/rack_to_balance {slot}",
                    "vial_balance_to_rack": "POST /routines/vial/balance_to_rack {slot}",
                },
                "preconditions": [
                    f"arm session open and the arm at {self.settings.home_node!r}",
                    "gripper empty",
                    f"balance {self.settings.balance.door_component} open",
                ],
            }

        @app.post("/routines/vial/rack_to_balance", tags=tags, dependencies=[claim])
        async def rack_to_balance(body: VialTask, identity: dict = login, x_claim_token: str | None = Header(default=None)):
            """Take the vial in `slot` and leave it on the balance pan, back at home."""
            return await self._vial_task(
                identity, x_claim_token, "routine.vial_rack_to_balance", self.rack_to_balance_steps, body
            )

        @app.post("/routines/vial/balance_to_rack", tags=tags, dependencies=[claim])
        async def balance_to_rack(body: VialTask, identity: dict = login, x_claim_token: str | None = Header(default=None)):
            """Take the vial off the balance pan and put it in `slot`, back at home."""
            return await self._vial_task(
                identity, x_claim_token, "routine.vial_balance_to_rack", self.balance_to_rack_steps, body
            )


def _json(response: JSONResponse):
    return json.loads(response.body)


__all__ = ["GRAPH_ACTIONS", "ROUTINE_ACTIONS", "RoutineSettings", "Routines", "read_door"]
