"""Motion-graph interlock layer.

Loads the whitelist of allowed transitions from ``motion_graph.yaml`` and
answers "given my current node, what can I do next?". Pure data — no
hardware coupling. The controller is responsible for tracking the arm
pose name and rail location name (which resolve to a node) plus the
gripper state (which resolves to a leaf of that node).

Schema 0.2 model:

- A NODE is an arm position only: a named arm pose from
  ``joint_config.yaml`` combined with a rail location. One node per
  physical position — gripper state is NOT part of node identity.
- A global ``gripper_states`` catalog names the allowed gripper leaves
  (e.g. ``empty``, ``grip_120``, ``reach_90``, ``grip_80``), each with a
  commanded stroke and a verification intent.
- Each node lists the ``gripper_states`` you may OCCUPY there and the
  ``gripper_transitions`` (state changes) allowed WHILE PARKED there.
- EDGES never change the gripper. An edge is traversable with gripper
  state G iff G is allowed at both endpoints. The gripper may only
  change at a node, through that node's transition whitelist.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Iterable

import yaml


SCHEMA_VERSION = "0.2"

# Sentinel: controller cannot pin its state to a known node (fresh boot,
# after STOP, after a raw cartesian/joint/rail move). The graph reports
# zero outgoing edges; recovery is by explicit operator declaration.
UNKNOWN_NODE = "__unknown__"

# Bio Gen2 stroke limits (mm of jaw opening).
_STROKE_MIN = 71.0
_STROKE_MAX = 150.0

# Name of the mandatory no-payload catalog state.
EMPTY_STATE = "empty"


# ── Enums ────────────────────────────────────────────────────────────


class MoveMode(str, Enum):
    JOINT = "joint"
    LINEAR = "linear"


class GraphMode(str, Enum):
    OFF = "off"            # graph not consulted
    ADVISORY = "advisory"  # off-whitelist moves are warned, allowed
    STRICT = "strict"      # off-whitelist moves are rejected (Phase 2)


class GripIntent(str, Enum):
    GRASP = "grasp"        # object expected; actual > commanded = success
    POSITION = "position"  # free travel; actual ≈ commanded = success
    NONE = "none"          # no verification (open moves)


# ── Domain types ─────────────────────────────────────────────────────


@dataclass(frozen=True)
class GripperState:
    """A named gripper leaf from the ``gripper_states`` catalog.

    ``stroke`` is the commanded stroke (Bio Gen2: 150 = fully open,
    71 = fully closed). ``intent`` tells the controller what to verify
    after actuating:

    - GRASP: an object is expected between the jaws, so the actual
      stroke must settle ABOVE the commanded value; reaching the
      commanded value means nothing was gripped → error.
    - POSITION: free travel is expected, so the jaws must actually
      REACH the commanded value; stalling early means blocked → error.
    - NONE: no verification (used by ``empty``).
    """
    name: str
    stroke: float
    intent: GripIntent


@dataclass(frozen=True)
class Node:
    """An arm position: a named arm pose at a rail location.

    ``gripper_states`` are the catalog states you may occupy at this
    node (its leaves). ``gripper_transitions`` are the (from, to) state
    changes allowed while the arm is parked here — grip/release/narrow
    never happens during motion, only at a node that whitelists it.
    """
    id: str
    arm: str          # name in joint_config.yaml::positions
    rail: str         # name in linear_track_config.yaml::locations
    gripper_states: tuple[str, ...] = (EMPTY_STATE,)
    gripper_transitions: tuple[tuple[str, str], ...] = ()
    tags: tuple[str, ...] = ()

    def allows_gripper(self, state: str | None) -> bool:
        return state is not None and state in self.gripper_states


@dataclass(frozen=True)
class Edge:
    """A whitelisted directional transition between two nodes.

    Edges carry no gripper information: the gripper state is invariant
    along an edge, and the edge is traversable with any state allowed
    at both endpoints.
    """
    from_node: str
    to_node: str
    mode: MoveMode
    speed: float | None = None
    preconditions: tuple[str, ...] = ()
    comment: str = ""


@dataclass
class ControllerView:
    """Minimal read-only snapshot a guard / advisory check needs.

    Decouples the graph from ``XArmController`` so this module is unit-
    testable without any arm fixtures. The controller builds one of
    these per evaluation.
    """
    gripper_stroke: float | None = None   # last commanded stroke
    last_force_magnitude: float | None = None


# Precondition guards — registered by name; YAML references them by name.
PreconditionFn = Callable[["MotionGraph", Edge, ControllerView], bool]


# ── Exceptions ───────────────────────────────────────────────────────


class GraphError(Exception):
    """Topology or coherence violation in motion_graph.yaml."""


class UnknownNodeError(GraphError):
    """Caller asked about a node id that doesn't exist."""


class EdgeNotAllowedError(GraphError):
    """STRICT mode refused a transition."""

    def __init__(self, current: str | None, target: str, reason: str):
        self.current = current
        self.target = target
        self.reason = reason
        super().__init__(f"{current!r} -> {target!r}: {reason}")


class GripperTransitionError(GraphError):
    """A gripper state change was refused (not whitelisted at the
    current node, unknown state, or no node pinned)."""

    def __init__(self, node: str | None, from_state: str | None,
                 to_state: str, reason: str):
        self.node = node
        self.from_state = from_state
        self.to_state = to_state
        self.reason = reason
        super().__init__(
            f"gripper {from_state!r} -> {to_state!r} at node {node!r}: {reason}"
        )


class RecoveryMismatch(GraphError):
    """``recover_to(node_id, force=False)`` refused because the
    nearest-node detector disagrees with the requested node id. Carries
    the detector's suggestion + residuals so the operator can decide
    whether to retry with ``force=True``."""

    def __init__(
        self, requested: str, suggested: str | None,
        arm_residual: float | None, rail_residual: float | None,
    ):
        self.requested = requested
        self.suggested = suggested
        self.arm_residual = arm_residual
        self.rail_residual = rail_residual
        super().__init__(
            f"requested recovery to {requested!r}, but nearest match is "
            f"{suggested!r} (arm residual {arm_residual}, rail residual "
            f"{rail_residual}); retry with force=True to override"
        )


@dataclass(frozen=True)
class NodeMatch:
    """Result of ``find_nearest_node()``.

    ``node_id`` is the best arm+rail match found, or None when no node
    passes the rail predicate. ``gripper_state`` is the catalog state
    resolved from the commanded stroke (None if the stroke matches no
    catalog state). ``gripper_match`` is True when that resolved state
    is one of the matched node's leaves. ``within_tolerance`` is True
    only when the arm residual is also within the joint tolerance —
    callers can use it as a "confident snap" gate.
    """
    node_id: str | None
    arm_residual: float | None    # sum of |angle_diff| in degrees
    rail_residual: float | None   # |position_diff| in mm
    gripper_state: str | None
    gripper_match: bool
    within_tolerance: bool


# ── The graph ────────────────────────────────────────────────────────


class MotionGraph:
    def __init__(
        self,
        nodes: dict[str, Node],
        edges: list[Edge],
        gripper_states: dict[str, GripperState],
        preconditions: dict[str, PreconditionFn] | None = None,
    ):
        self._nodes = nodes
        self._edges = edges
        self._gripper_states = gripper_states
        self._preconditions = preconditions or {}
        # Outgoing adjacency: from_node id -> [Edge]
        self._out: dict[str, list[Edge]] = {}
        for e in edges:
            self._out.setdefault(e.from_node, []).append(e)
        self._validate_topology()

    # ── Loading ──────────────────────────────────────────────────

    @classmethod
    def from_yaml(
        cls,
        path: str | Path,
        preconditions: dict[str, PreconditionFn] | None = None,
    ) -> "MotionGraph":
        data = yaml.safe_load(Path(path).read_text()) or {}
        return cls.from_dict(data, preconditions=preconditions)

    @classmethod
    def from_dict(
        cls,
        data: dict[str, Any],
        preconditions: dict[str, PreconditionFn] | None = None,
    ) -> "MotionGraph":
        version = data.get("schema_version")
        if version != SCHEMA_VERSION:
            raise GraphError(
                f"unsupported schema_version {version!r}, expected {SCHEMA_VERSION!r}"
            )

        # ── Gripper-state catalog ─────────────────────────────────
        gripper_states: dict[str, GripperState] = {}
        for name, spec in (data.get("gripper_states") or {}).items():
            spec = spec or {}
            try:
                intent = GripIntent(spec.get("intent", "none"))
            except ValueError:
                raise GraphError(
                    f"gripper_state {name!r}: unknown intent {spec.get('intent')!r}; "
                    f"must be one of: grasp, position, none"
                )
            gripper_states[name] = GripperState(
                name=name,
                stroke=float(spec["stroke"]),
                intent=intent,
            )

        # ── Nodes ─────────────────────────────────────────────────
        nodes: dict[str, Node] = {}
        for n in data.get("nodes", []) or []:
            node_id: str = n["id"]
            raw_states = n.get("gripper_states")
            states = tuple(raw_states) if raw_states else (EMPTY_STATE,)

            transitions: list[tuple[str, str]] = []
            for t in n.get("gripper_transitions", []) or []:
                if not isinstance(t, (list, tuple)) or len(t) != 2:
                    raise GraphError(
                        f"node {node_id!r}: gripper_transitions entries must be "
                        f"[from_state, to_state] pairs, got {t!r}"
                    )
                transitions.append((str(t[0]), str(t[1])))

            node = Node(
                id=node_id,
                arm=n["arm"],
                rail=n["rail"],
                gripper_states=states,
                gripper_transitions=tuple(transitions),
                tags=tuple(n.get("tags", [])),
            )
            if node.id in nodes:
                raise GraphError(f"duplicate node id: {node.id!r}")
            nodes[node.id] = node

        # ── Edges ─────────────────────────────────────────────────
        edges: list[Edge] = []
        seen_pairs: set[tuple[str, str]] = set()
        for e in data.get("edges", []) or []:
            edge = Edge(
                from_node=e["from"],
                to_node=e["to"],
                mode=MoveMode(e["mode"]),
                speed=e.get("speed"),
                preconditions=tuple(e.get("preconditions", [])),
                comment=e.get("comment", ""),
            )
            pair = (edge.from_node, edge.to_node)
            if pair in seen_pairs:
                raise GraphError(
                    f"duplicate edge {edge.from_node!r} -> {edge.to_node!r} "
                    f"(speed differences belong on the move call, not as parallel edges)"
                )
            seen_pairs.add(pair)
            edges.append(edge)

        return cls(nodes, edges, gripper_states, preconditions)

    # ── Validation ───────────────────────────────────────────────

    def _validate_topology(self) -> None:
        # Catalog validation: the empty sentinel must exist, strokes in range.
        if EMPTY_STATE not in self._gripper_states:
            raise GraphError(
                f"gripper_states catalog must define {EMPTY_STATE!r} "
                f"(the no-payload sentinel)"
            )
        for gs in self._gripper_states.values():
            if not (_STROKE_MIN <= gs.stroke <= _STROKE_MAX):
                raise GraphError(
                    f"gripper_state {gs.name!r}: stroke {gs.stroke} is outside "
                    f"the valid range {_STROKE_MIN:g}-{_STROKE_MAX:g}"
                )

        # Per-node validation.
        seen_positions: dict[tuple[str, str], str] = {}
        for n in self._nodes.values():
            # One node per physical position — gripper state is a leaf,
            # not part of node identity, so (arm, rail) must be unique.
            pos = (n.arm, n.rail)
            if pos in seen_positions:
                raise GraphError(
                    f"nodes {seen_positions[pos]!r} and {n.id!r} share the same "
                    f"(arm={n.arm!r}, rail={n.rail!r}); collapse them into one "
                    f"node with multiple gripper_states"
                )
            seen_positions[pos] = n.id

            if not n.gripper_states:
                raise GraphError(f"node {n.id!r}: gripper_states must not be empty")
            for s in n.gripper_states:
                if s not in self._gripper_states:
                    raise GraphError(
                        f"node {n.id!r} references unknown gripper_state {s!r}"
                    )
            for (a, b) in n.gripper_transitions:
                if a not in n.gripper_states or b not in n.gripper_states:
                    raise GraphError(
                        f"node {n.id!r}: gripper_transition {a!r} -> {b!r} "
                        f"references a state outside the node's gripper_states "
                        f"{list(n.gripper_states)}"
                    )
                if a == b:
                    raise GraphError(
                        f"node {n.id!r}: gripper_transition {a!r} -> {b!r} "
                        f"is a no-op"
                    )

        # Per-edge validation.
        for e in self._edges:
            if e.from_node not in self._nodes:
                raise GraphError(f"edge from unknown node: {e.from_node!r}")
            if e.to_node not in self._nodes:
                raise GraphError(f"edge to unknown node: {e.to_node!r}")
            for p in e.preconditions:
                if p not in self._preconditions:
                    raise GraphError(
                        f"edge {e.from_node!r}->{e.to_node!r} references "
                        f"unregistered precondition: {p!r}"
                    )
            # The gripper state is invariant along an edge, so an edge whose
            # endpoints share no state can never be traversed.
            from_states = set(self._nodes[e.from_node].gripper_states)
            to_states = set(self._nodes[e.to_node].gripper_states)
            if not (from_states & to_states):
                raise GraphError(
                    f"edge {e.from_node!r}->{e.to_node!r} is untraversable: "
                    f"endpoints share no gripper state "
                    f"({sorted(from_states)} vs {sorted(to_states)})"
                )

    # ── Query API ────────────────────────────────────────────────

    @property
    def nodes(self) -> Iterable[Node]:
        return self._nodes.values()

    @property
    def edges(self) -> Iterable[Edge]:
        return iter(self._edges)

    @property
    def gripper_states(self) -> Iterable[GripperState]:
        return self._gripper_states.values()

    def gripper_state(self, name: str) -> GripperState:
        if name not in self._gripper_states:
            raise GraphError(f"unknown gripper_state: {name!r}")
        return self._gripper_states[name]

    def has_gripper_state(self, name: str) -> bool:
        return name in self._gripper_states

    def resolve_gripper_state(
        self, stroke: float | None, tolerance: float = 1.0,
    ) -> str | None:
        """Map a commanded stroke to a catalog state name, or None.

        Used by the controller to derive its current gripper state from
        ``last_gripper_position``.
        """
        if stroke is None:
            return None
        for gs in self._gripper_states.values():
            if abs(gs.stroke - float(stroke)) < tolerance:
                return gs.name
        return None

    def node(self, node_id: str) -> Node:
        if node_id not in self._nodes:
            raise UnknownNodeError(node_id)
        return self._nodes[node_id]

    def has_node(self, node_id: str) -> bool:
        return node_id in self._nodes

    def find_node(self, arm: str | None, rail: str | None) -> Node | None:
        """Resolve (arm pose name, rail location name) to a node, or None.

        Either None coordinate causes a None result — the controller
        must have both named before a node is pin-able.
        """
        if None in (arm, rail):
            return None
        for n in self._nodes.values():
            if n.arm == arm and n.rail == rail:
                return n
        return None

    def outgoing(self, node_id: str | None) -> list[Edge]:
        """Edges leaving node_id. Empty if node_id is None or UNKNOWN_NODE."""
        if node_id is None or node_id == UNKNOWN_NODE:
            return []
        return list(self._out.get(node_id, []))

    def edge_allows_gripper(self, edge: Edge, state: str | None) -> bool:
        """True iff ``state`` may ride along ``edge`` (allowed at both ends)."""
        if state is None:
            return False
        return (
            self._nodes[edge.from_node].allows_gripper(state)
            and self._nodes[edge.to_node].allows_gripper(state)
        )

    def outgoing_for_state(
        self, node_id: str | None, state: str | None,
    ) -> list[Edge]:
        """Edges leaving node_id that are traversable with gripper ``state``."""
        return [e for e in self.outgoing(node_id)
                if self.edge_allows_gripper(e, state)]

    def allowed_targets(self, node_id: str | None) -> list[str]:
        return [e.to_node for e in self.outgoing(node_id)]

    def allowed_targets_for_state(
        self, node_id: str | None, state: str | None,
    ) -> list[str]:
        return [e.to_node for e in self.outgoing_for_state(node_id, state)]

    def allowed_gripper_targets(
        self, node_id: str | None, current_state: str | None,
    ) -> list[str]:
        """Gripper states reachable from ``current_state`` while parked at
        ``node_id``, per the node's transition whitelist."""
        if node_id is None or node_id == UNKNOWN_NODE or current_state is None:
            return []
        if node_id not in self._nodes:
            return []
        node = self._nodes[node_id]
        return [b for (a, b) in node.gripper_transitions if a == current_state]

    def find_edge(self, from_id: str, to_id: str) -> Edge | None:
        for e in self._out.get(from_id, []):
            if e.to_node == to_id:
                return e
        return None

    # ── Introspection ────────────────────────────────────────────

    def unreachable_nodes(self, from_node: str) -> list[str]:
        """Nodes not reachable from ``from_node`` by any path. Useful for
        validating that 'home' can reach every operational node.
        """
        if from_node not in self._nodes:
            raise UnknownNodeError(from_node)
        reachable = {from_node}
        frontier = [from_node]
        while frontier:
            cur = frontier.pop()
            for e in self._out.get(cur, []):
                if e.to_node not in reachable:
                    reachable.add(e.to_node)
                    frontier.append(e.to_node)
        return sorted(n for n in self._nodes if n not in reachable)

    def adjacency_summary(self) -> dict[str, list[str]]:
        """Map node id -> sorted list of outgoing target ids. For logging."""
        return {
            n_id: sorted(e.to_node for e in self._out.get(n_id, []))
            for n_id in self._nodes
        }


# ── Built-in preconditions ───────────────────────────────────────────


def _gripper_empty(graph: MotionGraph, edge: Edge, view: ControllerView) -> bool:
    """Gripper is at the empty-state stroke (not holding anything)."""
    if view.gripper_stroke is None:
        return True
    try:
        empty_stroke = graph.gripper_state(EMPTY_STATE).stroke
    except GraphError:
        return True
    return abs(view.gripper_stroke - empty_stroke) < 1.0


DEFAULT_PRECONDITIONS: dict[str, PreconditionFn] = {
    "gripper_empty": _gripper_empty,
}


# ── Nearest-node detection ───────────────────────────────────────────


def _angle_distance_deg(a: float, b: float) -> float:
    """Modular shortest-path distance between two joint angles in degrees.

    Handles 360-degree wrap (e.g. J1 at 180 vs -180 is 0deg apart, not
    360). Other joints typically can't reach the wrap region, so this
    reduces to ``abs(a - b)`` for them.
    """
    diff = abs(float(a) - float(b)) % 360.0
    return min(diff, 360.0 - diff)


def find_nearest_node(
    graph: "MotionGraph",
    *,
    current_joints: list[float] | None,
    current_rail_mm: float | None,
    current_gripper_stroke: float | None,
    arm_pose_joints: dict[str, list[float]],
    rail_position_mm: dict[str, float],
    joint_tolerance_deg: float = 10.0,
    rail_tolerance_mm: float = 2.0,
    gripper_stroke_tolerance: float = 1.0,
) -> NodeMatch:
    """Find the graph node whose arm+rail position best matches the
    controller's physical state.

    Strategy: rail must be within ``rail_tolerance_mm``; arm pose is
    scored by summed per-joint angular distance, and we pick the
    candidate with the smallest score. The gripper is NOT a node
    filter — it is resolved separately against the catalog and
    reported so the caller can pin (node, state) together.

    Returns a NodeMatch. ``within_tolerance`` is set when the arm
    residual is also <= joint_tolerance_deg — callers should treat that
    as the "safe to snap" condition. ``gripper_match`` is True when the
    resolved state is one of the matched node's leaves.

    Returns NodeMatch(None, ...) if no candidate passes the rail
    predicate, or if the current state is incomplete (joints or rail
    unknown).
    """
    resolved_state = graph.resolve_gripper_state(
        current_gripper_stroke, tolerance=gripper_stroke_tolerance,
    )
    empty = NodeMatch(
        node_id=None,
        arm_residual=None,
        rail_residual=None,
        gripper_state=resolved_state,
        gripper_match=False,
        within_tolerance=False,
    )
    if current_joints is None or current_rail_mm is None:
        return empty

    best: NodeMatch | None = None
    for node in graph.nodes:
        # Rail must resolve and be within tolerance.
        rail_target = rail_position_mm.get(node.rail)
        if rail_target is None:
            continue
        rail_residual = abs(float(rail_target) - float(current_rail_mm))
        if rail_residual > rail_tolerance_mm:
            continue

        # Arm joints must be defined (joint-list preset) to compute distance.
        # Cartesian-dict presets are skipped here — the operator should
        # use a different recovery path (declare position manually with
        # force=True) for those.
        joints_target = arm_pose_joints.get(node.arm)
        if not isinstance(joints_target, list):
            continue
        # Pair element-wise as far as we have data; ignore trailing joints
        # if either side is shorter than the other (5-joint arms vs 6/7).
        pairs = list(zip(current_joints, joints_target))
        if not pairs:
            continue
        arm_residual = sum(_angle_distance_deg(c, t) for c, t in pairs)

        candidate = NodeMatch(
            node_id=node.id,
            arm_residual=arm_residual,
            rail_residual=rail_residual,
            gripper_state=resolved_state,
            gripper_match=node.allows_gripper(resolved_state),
            within_tolerance=(arm_residual <= joint_tolerance_deg),
        )
        # best.arm_residual is always a float when best is not None
        # (we set it explicitly above). Use explicit None check to avoid
        # the `0.0 or X` falsy-zero trap.
        if best is None or arm_residual < best.arm_residual:
            best = candidate

    return best or empty
