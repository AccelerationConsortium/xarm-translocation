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
from typing import Annotated, Literal, NamedTuple

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

# The lab-skills catalog names (graph.move_to, graph.travel_to, graph.gripper,
# graph.recover_to) as well as this service's own graph.move: the SDK offers a
# skill only when its name is in allowed_actions.
GRAPH_ACTIONS = ("graph.move", "graph.move_to", "graph.travel_to", "graph.gripper", "graph.recover_to")
# The panel's Recover: a moveJ onto the nearest node, only this close.
RECOVER_MAX_DEG = 15.0
ROUTINE_ACTIONS = (
    "routine.vial_rack_to_balance",
    "routine.vial_balance_to_rack",
    "routine.vial_balance_to_node",
    "routine.vial_node_to_rack",
)


def _signed_fold_deg(delta: float) -> float:
    """The shortest signed angle for a joint difference, in [-180, 180)."""
    return (delta + 180.0) % 360.0 - 180.0


def _fold_deg(delta: float) -> float:
    """Shortest angular distance for a joint difference, in [0, 180]."""
    delta = abs(delta) % 360.0
    return min(delta, 360.0 - delta)


class Location(NamedTuple):
    node: str | None  # the node the arm is at, or None
    nearest: str  # the closest node by shortest angular distance
    deviation_deg: float  # its largest per-joint deviation with whole turns folded out
    turns: dict[str, int]  # joints a whole number of turns off that node, e.g. {"j6": 1}


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
    # mm/s for the moves above the rack (approach node <-> above a slot) and
    # straight down into / up out of a slot; None: the motion block's default.
    transit_speed_mm_s: Number | None = Field(default=None, gt=0)
    slot_speed_mm_s: Number | None = Field(default=None, gt=0)

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
    command. `close_check` says what a close must do: "object" (close_raw is
    beyond the part, so the fingers must stop on it) or "stroke" (close_raw
    is a target stroke: the fingers stop there or on the part, at no more
    than the grip force, and either is a grip; the Gibbie fingertips cage a
    vial, so its gripper may never sense one)."""

    model_config = ConfigDict(extra="forbid", frozen=True)
    open_raw: Raw
    close_raw: Raw
    close_check: Literal["object", "stroke"] = "object"

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


class VialToNode(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    target: Identifier
    dry_run: bool = False


# The shared panel's Graph Drive requests (the xArm's /control/graph/* names).
class PanelGraphMove(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    node_id: Identifier
    # Upper bound for every edge on the way (deg/s joint, mm/s linear); it
    # can only slow a move. None: each edge's own speed.
    speed: Number | None = Field(default=None, gt=0)


class PanelGraphMode(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    mode: str = Field(min_length=1, max_length=32)


class PanelGripper(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    state: Identifier


class PanelRecover(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    node_id: Identifier
    # Accepted for panel parity; neither widens the recover bound.
    force: bool = False
    gripper_state: str | None = Field(default=None, max_length=64)


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
    def __init__(
        self, control, graph: Graph, settings: RoutineSettings | None, *, door_reader=read_door,
        observed_joints=None,
    ):
        if settings is not None:
            settings.check_against(graph)
            if control.gripper_control is None:
                raise ValueError("Routines that grip need a configured gripper")
            caps = control.gripper_control.settings
            for name, value, cap in (
                ("grip_force_pct", settings.grip_force_pct, caps.max_force_pct),
                ("grip_speed_pct", settings.grip_speed_pct, caps.max_speed_pct),
            ):
                if value is not None and value > cap:
                    raise ValueError(f"routines {name} {value:g} is above the gripper's commissioned {cap:g}")
        limits = control.arm_limits
        if settings is not None:
            rack = settings.vial_rack
            for name in ("transit_speed_mm_s", "slot_speed_mm_s"):
                value = getattr(rack, name)
                if value is not None and value > limits.max_linear_speed_mm_s:
                    raise ValueError(
                        f"vial_rack {name} {value:g} mm/s is above the motion block's "
                        f"{limits.max_linear_speed_mm_s:g} mm/s"
                    )
        for e in graph.edges:
            if e.speed is None:
                continue
            cap, unit = (
                (limits.max_joint_speed_deg_s, "deg/s") if e.mode == "joint" else (limits.max_linear_speed_mm_s, "mm/s")
            )
            if e.speed > cap:
                raise ValueError(
                    f"Graph edge {e.source} -> {e.target}: speed {e.speed:g} {unit} is above the "
                    f"motion block's {cap:g} {unit}"
                )
        self.control = control
        self.graph = graph
        self.settings = settings
        self.nodes = {n.id: n for n in graph.nodes}
        self.edges = {(e.source, e.target): e for e in graph.edges}
        self.modes = {pair: e.mode for pair, e in self.edges.items()}
        self.read_door = door_reader
        # Joints from the observation poll (no arm session needed), or None.
        self.observed_joints = observed_joints
        self._running = None  # name of the routine in progress

    # ── where the arm is ────────────────────────────────────────────
    def locate(self) -> Location:
        """Which graph node the arm is at, by shortest angular distance per joint.

        A joint a whole number of turns from its taught angle (the UR3e's
        wrist 3 turns without limit, so it reads 540° as readily as 180°)
        folds to the same pose, but a moveJ to the taught angle would spin it
        the whole way round. Such a node counts as *nearest*, not *at*, and
        the turns are reported so the operator can unwind the joint first.
        """
        joints = self._session_joints()
        if joints is None:
            raise HTTPException(
                status_code=409,
                detail={"error": "no_control_session", "reason": "No arm session; Connect first"},
            )
        return self._locate(joints)

    def _session_joints(self):
        session = self.control.session
        feedback = getattr(session, "feedback", None) if session is not None else None
        sample = feedback.latest() if feedback is not None else None
        return list(sample["joints_deg"]) if sample is not None else None

    def _display_joints(self):
        """The freshest joints there are: the session's feedback, else the
        observation poll; None when neither has a reading."""
        joints = self._session_joints()
        if joints is None and self.observed_joints is not None:
            try:
                joints = self.observed_joints()
            except Exception:
                joints = None
        return list(joints) if joints else None

    def observed_location(self) -> Location | None:
        """Where the arm is from _display_joints(). For display (/status,
        /graph/nearest); moves use locate()."""
        joints = self._display_joints()
        return self._locate(joints) if joints else None

    def closest_nodes(self, joints, count=3):
        """The nodes with the smallest worst-joint deviation, each with the
        signed move per joint (node minus arm, shortest way, whole turns
        folded out) that would bring the arm onto it."""
        ranked = []
        for node in self.graph.nodes:
            move = [_signed_fold_deg(b - a) for a, b in zip(joints, node.joints_deg)]
            ranked.append((max(abs(m) for m in move), node.id, move))
        ranked.sort(key=lambda r: r[0])
        return [
            {"node": nid, "worst_joint_deg": round(worst, 2), "move_by_deg": [round(m, 1) for m in move]}
            for worst, nid, move in ranked[:count]
        ]

    def _locate(self, joints) -> Location:
        tolerance = self.settings.node_tolerance_deg if self.settings else 1.0
        best = None
        for node in self.graph.nodes:
            raw = [a - b for a, b in zip(joints, node.joints_deg)]
            folded = max(_fold_deg(d) for d in raw)
            if best is None or folded < best[1]:
                best = (node, folded, raw)
        node, folded, raw = best
        if folded > tolerance:
            return Location(None, node.id, folded, {})
        turns = {f"j{i + 1}": round(d / 360) for i, d in enumerate(raw) if abs(d) > tolerance}
        return Location(None if turns else node.id, node.id, folded, turns)

    def _not_at(self, where: Location, reason: str, **extra):
        """The 412 for an arm that is not at the node a move needs."""
        detail = {
            "error": "not_at_node",
            "reason": reason,
            **extra,
            "at": where.node,
            "nearest": where.nearest,
            "nearest_deviation_deg": round(where.deviation_deg, 3),
        }
        if where.turns:
            joints = ", ".join(f"{j} by {n:+d} turn{'s' if abs(n) != 1 else ''}" for j, n in where.turns.items())
            detail["error"] = "joint_turn_offset"
            detail["turns"] = where.turns
            detail["reason"] = (
                f"The arm is at {where.nearest!r} except that {joints} from the taught angle; "
                "unwind it before a graph move (a joint move to the taught angle would spin it the whole way)"
            )
        return HTTPException(status_code=412, detail=detail)

    def _at(self, required):
        where = self.locate()
        if where.node != required:
            raise self._not_at(where, f"The arm must start at {required!r}", required=required)

    # ── step plans ──────────────────────────────────────────────────
    def path_steps(self, source, target, speed_cap=None):
        """One step per edge from `source` to `target`, each at its edge's
        speed (None: the motion block's default). `speed_cap` can only slow
        them: deg/s on joint edges, mm/s on linear ones."""
        if target not in self.nodes:
            raise HTTPException(status_code=422, detail={"error": "unknown_node", "node": target})
        if source == target:
            return []
        path = [source, *self.graph.path(source, target)]
        steps = []
        for a, b in zip(path, path[1:]):
            edge = self.edges[(a, b)]
            if edge.mode == "joint":
                step = {"kind": "joint", "to": b, "angles": list(self.nodes[b].joints_deg)}
            else:
                step = {"kind": "linear", "to": b, "pose": list(self.nodes[b].tcp_mm_rpy_deg)}
            speed = self._edge_speed(edge, speed_cap)
            if speed is not None:
                step["speed"] = speed
            steps.append(step)
        return steps

    def _edge_speed(self, edge, cap):
        if cap is None:
            return edge.speed
        limits = self.control.arm_limits
        default = limits.joint_speed_default if edge.mode == "joint" else limits.linear_speed_default
        return min(edge.speed if edge.speed is not None else default, cap)

    def travel_targets(self, source):
        """Every node a graph move from `source` can reach."""
        seen, todo = {source}, [source]
        while todo:
            here = todo.pop()
            for a, b in self.edges:
                if a == here and b not in seen:
                    seen.add(b)
                    todo.append(b)
        return [n for n in self.nodes if n in seen and n != source]

    def _grip(self, raw, label, expect_object):
        return {"kind": "gripper", "to": label, "raw": raw, "expect_object": expect_object}

    def _close(self, grip):
        """The close step for `grip`: a part must stop the fingers only
        when close_check is "object"."""
        return self._grip(grip.close_raw, "vial_close", grip.close_check == "object")

    def _rack_move(self, to, pose, speed):
        step = {"kind": "linear", "to": to, "pose": list(pose)}
        if speed is not None:
            step["speed"] = speed
        return step

    def rack_to_balance_steps(self, slot):
        s = self.settings
        rack, grip, port = s.vial_rack, s.vial_grip, s.balance
        above = rack.slot_pose(slot)
        down = [*above[:2], above[2] - rack.pick_depth_mm, *above[3:]]
        transit, inside = rack.transit_speed_mm_s, rack.slot_speed_mm_s
        approach = self.nodes[rack.approach_node].tcp_mm_rpy_deg
        return [
            self._grip(grip.open_raw, "vial_open", False),
            *self.path_steps(s.home_node, rack.approach_node),
            self._rack_move(f"slot_{slot}", above, transit),
            self._rack_move(f"slot_{slot}_grip", down, inside),
            self._close(grip),
            self._rack_move(f"slot_{slot}", above, inside),
            self._rack_move(rack.approach_node, approach, transit),
            *self.path_steps(rack.approach_node, port.drop_node),
            self._grip(grip.open_raw, "vial_open", False),
            *self.path_steps(port.drop_node, s.home_node),
        ]

    def _pick_from_balance(self):
        """Home to the balance pick node, gripping the vial there."""
        s = self.settings
        return [
            self._grip(s.vial_grip.open_raw, "vial_open", False),
            *self.path_steps(s.home_node, s.balance.pick_node),
            self._close(s.vial_grip),
        ]

    def balance_to_rack_steps(self, slot):
        return [*self._pick_from_balance(), *self.node_to_rack_steps(self.settings.balance.pick_node, slot)]

    def balance_to_node_steps(self, target):
        """Pick the vial off the pan and carry it to `target`, still gripped."""
        if target == self.settings.balance.pick_node:
            raise HTTPException(
                status_code=422,
                detail={"error": "target_is_pick_node", "reason": "The target must be a node off the balance pan"},
            )
        return [*self._pick_from_balance(), *self.path_steps(self.settings.balance.pick_node, target)]

    def node_to_rack_steps(self, source, slot):
        """From `source`, holding a vial, put it in `slot` and go home."""
        s = self.settings
        rack, grip = s.vial_rack, s.vial_grip
        above = rack.slot_pose(slot)
        down = [*above[:2], above[2] - rack.place_depth_mm, *above[3:]]
        transit, inside = rack.transit_speed_mm_s, rack.slot_speed_mm_s
        approach = self.nodes[rack.approach_node].tcp_mm_rpy_deg
        return [
            *self.path_steps(source, rack.approach_node),
            self._rack_move(f"slot_{slot}", above, transit),
            self._rack_move(f"slot_{slot}_place", down, inside),
            self._grip(grip.open_raw, "vial_open", False),
            # Straight up out of the slot before moving sideways.
            self._rack_move(f"slot_{slot}", above, inside),
            self._rack_move(rack.approach_node, approach, transit),
            *self.path_steps(rack.approach_node, s.home_node),
        ]

    # ── execution ───────────────────────────────────────────────────
    async def _step(self, identity, token, step):
        if step["kind"] == "joint":
            body = JointMove(angles=tuple(float(v) for v in step["angles"]), speed=step.get("speed"))
            return await self.control._run_motion(identity, "arm.move_joints", body, token)
        if step["kind"] == "linear":
            x, y, z, roll, pitch, yaw = (float(v) for v in step["pose"])
            body = LinearMove(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, speed=step.get("speed"))
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
    def _holding(self, summary):
        """Whether the gripper holds a vial: stopped on contact, or (a
        stroke grip, which may never touch it) last sent to the grip stroke."""
        if summary.get("object") in ("contact_closing", "contact_opening"):
            return True
        grip = self.settings.vial_grip
        return grip.close_check == "stroke" and summary.get("requested_raw") == grip.close_raw

    def _gripper_empty(self):
        if self._holding(self.control.gripper_control.gripper.summary()):
            raise HTTPException(
                status_code=412,
                detail={"error": "gripper_holding", "reason": "The gripper reports a part between its fingers"},
            )

    def _gripper_holding(self):
        summary = self.control.gripper_control.gripper.summary()
        holding = summary.get("object")
        if not self._holding(summary):
            raise HTTPException(
                status_code=412,
                detail={
                    "error": "gripper_empty",
                    "reason": "The gripper reports no part between its fingers; there is no vial to place",
                    "object": holding,
                },
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
        except GraphError as exc:
            raise HTTPException(status_code=422, detail={"error": "no_path", "reason": str(exc)})

    def _target_steps(self, target):
        try:
            return self.balance_to_node_steps(target)
        except GraphError as exc:
            raise HTTPException(status_code=422, detail={"error": "no_path", "reason": str(exc)})

    async def _vial_task(self, identity, token, name, build, body):
        steps = self._slot_steps(build, body.slot)
        if body.dry_run:
            return {"ok": True, "dry_run": True, "routine": name, "slot": body.slot, "plan": steps}
        self._at(self.settings.home_node)
        self._gripper_empty()
        await self._door_open()
        return await self.run(identity, token, name, steps, slot=body.slot)

    async def _balance_to_node(self, identity, token, body):
        name = "routine.vial_balance_to_node"
        steps = self._target_steps(body.target)
        if body.dry_run:
            return {"ok": True, "dry_run": True, "routine": name, "target": body.target, "plan": steps}
        self._at(self.settings.home_node)
        self._gripper_empty()
        await self._door_open()
        return await self.run(identity, token, name, steps, target=body.target)

    async def _node_to_rack(self, identity, token, body):
        name = "routine.vial_node_to_rack"
        where = self.locate()
        if where.node is None:
            raise self._not_at(where, "The arm must be at a graph node")
        steps = self._slot_steps(lambda slot: self.node_to_rack_steps(where.node, slot), body.slot)
        if body.dry_run:
            return {"ok": True, "dry_run": True, "routine": name, "from": where.node, "slot": body.slot, "plan": steps}
        self._gripper_holding()
        # The way back to the rack may pass the balance door.
        await self._door_open()
        return await self.run(identity, token, name, steps, source=where.node, slot=body.slot)

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

    def gripper_state(self):
        """The catalog state the gripper was last sent to (its requested
        position register), or None: not activated, or a position no state
        names."""
        gripper_control = self.control.gripper_control
        if gripper_control is None or not self.graph.gripper_catalog:
            return None
        summary = gripper_control.gripper.summary()
        if not summary.get("activated"):
            return None
        requested = summary.get("requested_raw")
        for name, spec in self.graph.gripper_catalog.items():
            if spec.raw == requested:
                return name
        return None

    def _drops_part(self, node_id, state):
        """Whether setting `state` at `node_id` would let go of a held part
        where the node forbids it: the gripper holds one (stopped on contact,
        a routine's grip, or a grasp / expect_object catalog state) and
        `state` is neither."""
        if self.nodes[node_id].gripper_release:
            return False
        target = self.graph.gripper_catalog[state]
        if target.grasp or target.expect_object:
            return False
        summary = self.control.gripper_control.gripper.summary()
        if self.settings is not None and self._holding(summary):
            return True
        if summary.get("object") in ("contact_closing", "contact_opening"):
            return True
        current = self.graph.gripper_catalog.get(self.gripper_state() or "")
        return bool(current and (current.grasp or current.expect_object))

    def _settable(self, node_id, current_state):
        """The catalog states Graph Drive may set at `node_id` now."""
        return [
            g for g in self.nodes[node_id].gripper_states
            if g != current_state and not self._drops_part(node_id, g)
        ]

    def motion_graph_status(self):
        """details.motion_graph, the block the shared panel's Graph Drive reads
        (the xArm's field names). There is no graph mode to set here: graph
        moves always follow taught edges and freehand moves are not
        graph-gated, which is the panel's "off". The gripper is not tracked
        as graph state."""
        where = self.observed_location()
        current = where.node if where else None
        state = self.gripper_state()
        allowed = self._settable(current, state) if current and self.control.gripper_control else []
        return {
            "graph_mode": "off",
            "current_node": current,
            "nearest_node": where.nearest if where else None,
            "nearest_deviation_deg": round(where.deviation_deg, 3) if where else None,
            "turns": where.turns if where else {},
            "reachable_nodes": [b for a, b in self.edges if a == current] if current else [],
            "travel_targets": self.travel_targets(current) if current else [],
            "gripper_state": state,
            "allowed_gripper_targets": allowed,
            "gripper_catalog": {n: g.model_dump() for n, g in self.graph.gripper_catalog.items()},
            "mode_override": None,
            "running": self._running,
        }

    def _install_panel(self, app, login, claim, tags):
        """The shared panel's Graph Drive routes (the xArm's names), on top of
        the same walk as /graph/move."""

        def at_node():
            where = self.locate()
            if where.node is None:
                raise self._not_at(where, "The arm is not at a graph node; use Recover or move it onto one")
            return where.node

        def plan(source, target, speed):
            try:
                return self.path_steps(source, target, speed_cap=speed)
            except GraphError as exc:
                raise HTTPException(status_code=422, detail={"error": "no_path", "reason": str(exc)})

        @app.post("/control/graph/move_to", tags=tags, dependencies=[claim])
        async def panel_move_to(
            body: PanelGraphMove, identity: dict = login, x_claim_token: str | None = Header(default=None)
        ):
            """One taught edge from the node the arm is at to `node_id`."""
            node = at_node()
            if (node, body.node_id) not in self.edges:
                raise HTTPException(
                    status_code=422,
                    detail={
                        "error": "not_adjacent",
                        "reason": f"No taught edge from {node!r} to {body.node_id!r}; use Travel for a multi-edge path",
                        "from": node,
                        "reachable_nodes": [b for a, b in self.edges if a == node],
                    },
                )
            steps = plan(node, body.node_id, body.speed)
            result = await self.run(identity, x_claim_token, "graph.move", steps, source=node, target=body.node_id)
            return _with(result, path=[node, body.node_id])

        @app.post("/control/graph/travel_to", tags=tags, dependencies=[claim])
        async def panel_travel_to(
            body: PanelGraphMove, identity: dict = login, x_claim_token: str | None = Header(default=None)
        ):
            """The shortest taught path from the node the arm is at to `node_id`."""
            node = at_node()
            steps = plan(node, body.node_id, body.speed)
            result = await self.run(identity, x_claim_token, "graph.move", steps, source=node, target=body.node_id)
            return _with(result, path=[node, *(s["to"] for s in steps)])

        @app.post("/control/graph/gripper", tags=tags, dependencies=[claim])
        async def panel_gripper(
            body: PanelGripper, identity: dict = login, x_claim_token: str | None = Header(default=None)
        ):
            """Set the gripper to a catalog state, only at a node that allows it.
            A state that expects a part fails (no_object_gripped) when the
            fingers close on nothing."""
            if self.control.gripper_control is None:
                raise HTTPException(status_code=409, detail={"error": "no_gripper"})
            node = at_node()
            allowed = self.nodes[node].gripper_states
            if body.state not in allowed:
                raise HTTPException(
                    status_code=422,
                    detail={
                        "error": "gripper_change_not_allowed",
                        "reason": f"{body.state!r} may not be set at {node!r}",
                        "node": node,
                        "allowed_gripper_states": allowed,
                    },
                )
            if self._drops_part(node, body.state):
                raise HTTPException(
                    status_code=422,
                    detail={
                        "error": "gripper_release_not_allowed",
                        "reason": f"The gripper holds a part; {body.state!r} would drop it at {node!r}. "
                        "Release it at a node that allows it, or with a rack routine",
                        "node": node,
                        "allowed_gripper_states": self._settable(node, self.gripper_state()),
                    },
                )
            spec = self.graph.gripper_catalog[body.state]
            step = self._grip(spec.raw, body.state, spec.expect_object)
            result = await self.run(identity, x_claim_token, "graph.gripper", [step], node=node, state=body.state)
            return _with(result, gripper_state=body.state)

        @app.post("/control/graph/mode", tags=tags, dependencies=[claim])
        def panel_graph_mode(body: PanelGraphMode):
            """Nothing to set: graph moves always follow taught edges, and
            freehand moves are not graph-gated on this service."""
            return {
                "ok": True,
                "graph_mode": "off",
                "requested": body.mode,
                "changed": False,
                "message": "This arm has no graph mode: graph moves always follow taught edges; "
                "freehand moves are not graph-gated",
            }

        @app.get("/graph/nearest", tags=tags)
        def panel_nearest():
            """The node nearest the arm's joints. Read-only."""
            where = self.observed_location()
            base = {
                "rail_residual_mm": None,
                "recover_max_deg": RECOVER_MAX_DEG,
                "gripper_state": "not tracked",
                "allowed_gripper_states": [],
                "gripper_stroke": None,
            }
            joints = self._display_joints()
            if joints is None:
                return {**base, "suggested_node": None, "arm_residual_deg": None, "within_tolerance": False,
                        "recoverable": False, "reason": "No joint reading", "closest": []}
            where = self._locate(joints)
            recoverable = not where.turns and where.deviation_deg <= RECOVER_MAX_DEG
            answer = {
                **base,
                "arm_residual_deg": round(where.deviation_deg, 3),
                "within_tolerance": where.node is not None,
                "turns": where.turns,
                "recoverable": recoverable,
                # The worst-joint ranking for guiding the arm by hand; with
                # nothing in reach the first entry is only the least far.
                "closest": self.closest_nodes(joints),
            }
            if recoverable:
                return {**answer, "suggested_node": where.nearest, "reason": None}
            if where.turns:
                joints_off = ", ".join(f"{j} {n:+d} turn{'s' if abs(n) != 1 else ''}" for j, n in where.turns.items())
                reason = f"At {where.nearest!r} except {joints_off}; unwind by hand first"
            else:
                reason = (
                    f"No node within {RECOVER_MAX_DEG:g} deg (the least far, {where.nearest!r}, has a joint "
                    f"{where.deviation_deg:.0f} deg off); guide the arm onto a node by hand, then Recover"
                )
            return {**answer, "suggested_node": None, "reason": reason}

        @app.post("/control/graph/recover_to", tags=tags, dependencies=[claim])
        async def panel_recover_to(
            body: PanelRecover, identity: dict = login, x_claim_token: str | None = Header(default=None)
        ):
            """A moveJ onto the nearest node at the default speed, only when no
            joint is a whole turn off it and every joint is within
            RECOVER_MAX_DEG. `force` does not widen that."""
            where = self.locate()
            if body.node_id != where.nearest:
                raise HTTPException(
                    status_code=422,
                    detail={"error": "not_nearest", "reason": f"The nearest node is {where.nearest!r}",
                            "nearest": where.nearest},
                )
            if where.turns:
                raise self._not_at(where, "")
            if where.node == body.node_id:
                return {"ok": True, "recovered_to": body.node_id, "moved": False, "message": "Already at the node"}
            if where.deviation_deg > RECOVER_MAX_DEG:
                raise HTTPException(
                    status_code=412,
                    detail={
                        "error": "too_far_to_recover",
                        "reason": f"A joint is {where.deviation_deg:.1f} deg from {body.node_id!r}; "
                        f"Recover moves only within {RECOVER_MAX_DEG:g} deg",
                        "nearest": body.node_id,
                        "nearest_deviation_deg": round(where.deviation_deg, 3),
                        "recover_max_deg": RECOVER_MAX_DEG,
                        "hint": "Guide the arm closer by hand (Manual mode) or jog it, then Recover",
                    },
                )
            steps = [{"kind": "joint", "to": body.node_id, "angles": list(self.nodes[body.node_id].joints_deg)}]
            result = await self.run(identity, x_claim_token, "graph.recover", steps, target=body.node_id)
            return _with(result, recovered_to=body.node_id, moved=True)

    def install(self, app, login, claim):
        tags = ["routines"]

        @app.post("/control/graph/move", tags=tags, dependencies=[claim])
        @app.post("/graph/move", tags=tags, dependencies=[claim])
        async def graph_move(body: GraphMove, identity: dict = login, x_claim_token: str | None = Header(default=None)):
            """Walk the motion graph from the node the arm is at to `target`,
            one ordinary arm move per edge. Refused unless the arm is at a node."""
            where = self.locate()
            node = where.node
            if node is None:
                raise self._not_at(where, "The arm is not at a graph node")
            try:
                steps = self.path_steps(node, body.target)
            except GraphError as exc:
                raise HTTPException(status_code=422, detail={"error": "no_path", "reason": str(exc)})
            if body.dry_run:
                return {"ok": True, "dry_run": True, "from": node, "target": body.target, "plan": steps}
            return await self.run(identity, x_claim_token, "graph.move", steps, source=node, target=body.target)

        self._install_panel(app, login, claim, tags)

        if self.settings is None:
            return

        @app.get("/routines", tags=tags)
        def routines():
            """The bench routines, rack slots and graph nodes. Read-only."""
            return {
                **self.summary(),
                "routines": {
                    "vial_rack_to_balance": "POST /control/routines/vial/rack_to_balance {slot}",
                    "vial_balance_to_rack": "POST /control/routines/vial/balance_to_rack {slot}",
                    "vial_balance_to_node": "POST /control/routines/vial/balance_to_node {target}",
                    "vial_node_to_rack": "POST /control/routines/vial/node_to_rack {slot}",
                },
                "preconditions": [
                    f"arm session open and the arm at {self.settings.home_node!r}, gripper empty "
                    "(node_to_rack: at any graph node, holding a vial)",
                    f"balance {self.settings.balance.door_component} open",
                ],
            }

        @app.post("/control/routines/vial/rack_to_balance", tags=tags, dependencies=[claim])
        @app.post("/routines/vial/rack_to_balance", tags=tags, dependencies=[claim])
        async def rack_to_balance(body: VialTask, identity: dict = login, x_claim_token: str | None = Header(default=None)):
            """Take the vial in `slot` and leave it on the balance pan, back at home."""
            return await self._vial_task(
                identity, x_claim_token, "routine.vial_rack_to_balance", self.rack_to_balance_steps, body
            )

        @app.post("/control/routines/vial/balance_to_rack", tags=tags, dependencies=[claim])
        @app.post("/routines/vial/balance_to_rack", tags=tags, dependencies=[claim])
        async def balance_to_rack(body: VialTask, identity: dict = login, x_claim_token: str | None = Header(default=None)):
            """Take the vial off the balance pan and put it in `slot`, back at home."""
            return await self._vial_task(
                identity, x_claim_token, "routine.vial_balance_to_rack", self.balance_to_rack_steps, body
            )

        @app.post("/control/routines/vial/balance_to_node", tags=tags, dependencies=[claim])
        @app.post("/routines/vial/balance_to_node", tags=tags, dependencies=[claim])
        async def balance_to_node(body: VialToNode, identity: dict = login, x_claim_token: str | None = Header(default=None)):
            """Take the vial off the balance pan and carry it to graph node `target`.
            The arm stops there still holding it; /routines/vial/node_to_rack puts it away."""
            return await self._balance_to_node(identity, x_claim_token, body)

        @app.post("/control/routines/vial/node_to_rack", tags=tags, dependencies=[claim])
        @app.post("/routines/vial/node_to_rack", tags=tags, dependencies=[claim])
        async def node_to_rack(body: VialTask, identity: dict = login, x_claim_token: str | None = Header(default=None)):
            """From the graph node the arm is at, holding a vial, put it in `slot`, back at home."""
            return await self._node_to_rack(identity, x_claim_token, body)


def _with(result, **extra):
    """Add fields to a successful run() answer; a failure passes through."""
    return {**result, **extra} if isinstance(result, dict) else result


def _json(response: JSONResponse):
    return json.loads(response.body)


__all__ = ["GRAPH_ACTIONS", "ROUTINE_ACTIONS", "RoutineSettings", "Routines", "read_door"]
