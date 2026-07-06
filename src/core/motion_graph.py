"""Motion-graph interlock layer.

Loads the whitelist of allowed transitions from ``motion_graph.yaml`` and
answers "given my current node, what can I do next?". Pure data — no
hardware coupling. The controller is responsible for tracking the three
dimensions (arm pose name, rail location name, gripper commanded stroke)
and asking ``MotionGraph.find_node(...)`` to resolve them to a node id.

Node gripper stroke and intent are encoded in the node id suffix:

    *_empty  (and bare pose names) → stroke 150 (fully open), intent NONE
    *_grip_<n>                     → stroke n, intent GRASP
                                     (object expected; actual must stay
                                     above n or the grasp is considered
                                     failed)
    *_open_<n>                     → stroke n, intent POSITION
                                     (free travel; jaws must reach n or
                                     the move is considered blocked)

Nodes whose id does not match any of the above suffixes (e.g. legacy
``*_press`` nodes) must carry explicit ``gripper_stroke`` + ``grip_intent``
fields in the YAML as a bridge until renamed.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Iterable

import yaml


SCHEMA_VERSION = "0.1"

# Sentinel: controller cannot pin its state to a known node (fresh boot,
# after STOP, after a raw cartesian/joint/rail move). The graph reports
# zero outgoing edges; recovery is by explicit operator declaration.
UNKNOWN_NODE = "__unknown__"

# Regex patterns for node id suffix parsing.
_GRIP_SUFFIX_RE = re.compile(r'_grip_(\d+(?:\.\d+)?)$')
_OPEN_SUFFIX_RE = re.compile(r'_open_(\d+(?:\.\d+)?)$')

# Bio Gen2 fully-open stroke (also the default for bare / _empty nodes).
_OPEN_STROKE = 150.0


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
    NONE = "none"          # no verification (open moves, bare poses)


# ── Helpers ──────────────────────────────────────────────────────────


def parse_node_gripper(node_id: str) -> tuple[float, GripIntent]:
    """Parse (gripper_stroke, grip_intent) from a node id suffix.

    Returns (150.0, GripIntent.NONE) for bare pose names and *_empty nodes.
    Returns (n, GripIntent.GRASP) for *_grip_<n> nodes.
    Returns (n, GripIntent.POSITION) for *_open_<n> nodes.
    Never returns None — explicit YAML fields take priority in the loader
    and override this function when present.
    """
    m = _GRIP_SUFFIX_RE.search(node_id)
    if m:
        return (float(m.group(1)), GripIntent.GRASP)
    m = _OPEN_SUFFIX_RE.search(node_id)
    if m:
        return (float(m.group(1)), GripIntent.POSITION)
    # Bare pose names (robot_home, deck_home, …) and *_empty nodes all
    # default to fully open / no verification.
    return (_OPEN_STROKE, GripIntent.NONE)


# ── Domain types ─────────────────────────────────────────────────────


@dataclass(frozen=True)
class Node:
    """A reachable state in the (arm, rail, gripper_stroke) product.

    ``gripper_stroke`` is the commanded stroke value (Bio Gen2: 71-150).
    ``grip_intent`` tells the controller what to verify after moving the
    gripper: GRASP (object expected), POSITION (free travel), NONE (skip).
    """
    id: str
    arm: str              # name in joint_config.yaml::positions
    rail: str             # name in linear_track_config.yaml::locations
    gripper_stroke: float # commanded stroke; 150 = fully open, 71 = fully closed
    grip_intent: GripIntent
    tags: tuple[str, ...] = ()


@dataclass(frozen=True)
class Edge:
    """A whitelisted directional transition between two nodes."""
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
    """STRICT mode refused a transition. Phase 2 will raise this from
    the controller; Phase 1 only constructs it for advisory logging."""

    def __init__(self, current: str | None, target: str, reason: str):
        self.current = current
        self.target = target
        self.reason = reason
        super().__init__(f"{current!r} -> {target!r}: {reason}")


class RecoveryMismatch(GraphError):
    """Phase 4: ``recover_to(node_id, force=False)`` refused because the
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

    ``node_id`` is the best match found, or None when no node satisfies
    the rail / gripper predicates. ``within_tolerance`` is True only
    when the arm residual is also within the joint tolerance — callers
    can use it as a "confident snap" gate.
    """
    node_id: str | None
    arm_residual: float | None    # sum of |angle_diff| in degrees
    rail_residual: float | None   # |position_diff| in mm
    gripper_match: bool
    within_tolerance: bool


# ── The graph ────────────────────────────────────────────────────────


class MotionGraph:
    def __init__(
        self,
        nodes: dict[str, Node],
        edges: list[Edge],
        preconditions: dict[str, PreconditionFn] | None = None,
    ):
        self._nodes = nodes
        self._edges = edges
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

        nodes: dict[str, Node] = {}
        for n in data.get("nodes", []) or []:
            node_id: str = n["id"]

            # Explicit fields (gripper_stroke + grip_intent) always win —
            # required for legacy nodes whose id is not suffix-parseable.
            # Optional for standard nodes as an override.
            gs = n.get("gripper_stroke")
            if gs is not None:
                gripper_stroke = float(gs)
                gi_raw = n.get("grip_intent", "none")
                try:
                    grip_intent = GripIntent(gi_raw)
                except ValueError:
                    raise GraphError(
                        f"node {node_id!r}: unknown grip_intent {gi_raw!r}; "
                        f"must be one of: grasp, position, none"
                    )
            else:
                # Fall back to id-suffix parsing.
                gripper_stroke, grip_intent = parse_node_gripper(node_id)

            node = Node(
                id=node_id,
                arm=n["arm"],
                rail=n["rail"],
                gripper_stroke=gripper_stroke,
                grip_intent=grip_intent,
                tags=tuple(n.get("tags", [])),
            )
            if node.id in nodes:
                raise GraphError(f"duplicate node id: {node.id!r}")
            nodes[node.id] = node

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

        return cls(nodes, edges, preconditions)

    # ── Validation ───────────────────────────────────────────────

    def _validate_topology(self) -> None:
        # Per-node validation.
        for n in self._nodes.values():
            # Stroke must be within Bio Gen2 range.
            if not (71.0 <= n.gripper_stroke <= 150.0):
                raise GraphError(
                    f"node {n.id!r}: gripper_stroke {n.gripper_stroke} is outside "
                    f"the valid range 71-150"
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

    # ── Query API ────────────────────────────────────────────────

    @property
    def nodes(self) -> Iterable[Node]:
        return self._nodes.values()

    @property
    def edges(self) -> Iterable[Edge]:
        return iter(self._edges)

    def node(self, node_id: str) -> Node:
        if node_id not in self._nodes:
            raise UnknownNodeError(node_id)
        return self._nodes[node_id]

    def has_node(self, node_id: str) -> bool:
        return node_id in self._nodes

    def find_node(
        self,
        arm: str | None,
        rail: str | None,
        gripper_stroke: float | None,
    ) -> Node | None:
        """Resolve a 3-tuple of named coordinates to a node, or None.

        Any None coordinate causes a None result — the controller must
        have all three before a node is pin-able. Gripper stroke is
        matched with a tolerance of 1.0 to absorb integer rounding.
        """
        if None in (arm, rail, gripper_stroke):
            return None
        for n in self._nodes.values():
            if (n.arm == arm and n.rail == rail
                    and abs(n.gripper_stroke - gripper_stroke) < 1.0):
                return n
        return None

    def outgoing(self, node_id: str | None) -> list[Edge]:
        """Edges leaving node_id. Empty if node_id is None or UNKNOWN_NODE."""
        if node_id is None or node_id == UNKNOWN_NODE:
            return []
        return list(self._out.get(node_id, []))

    def allowed_targets(self, node_id: str | None) -> list[str]:
        return [e.to_node for e in self.outgoing(node_id)]

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
    """Gripper is at the open stroke (not holding anything)."""
    return view.gripper_stroke is None or abs(view.gripper_stroke - _OPEN_STROKE) < 1.0


DEFAULT_PRECONDITIONS: dict[str, PreconditionFn] = {
    "gripper_empty": _gripper_empty,
}


# ── Nearest-node detection (Phase 4) ─────────────────────────────────


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
    """Find the graph node whose 3-tuple best matches the controller's
    physical state.

    Strategy: gripper stroke is an exact-match predicate (within
    ``gripper_stroke_tolerance``); rail must be within
    ``rail_tolerance_mm``; arm pose is scored by summed per-joint
    angular distance, and we pick the candidate with the smallest score.

    Returns a NodeMatch. ``within_tolerance`` is set when the arm
    residual is also <= joint_tolerance_deg — callers should treat that
    as the "safe to snap" condition.

    Returns NodeMatch(None, ...) if no candidate passes the gripper /
    rail predicates, or if the current state is incomplete (joints or
    rail unknown).
    """
    empty = NodeMatch(
        node_id=None,
        arm_residual=None,
        rail_residual=None,
        gripper_match=False,
        within_tolerance=False,
    )
    if current_joints is None or current_rail_mm is None:
        return empty

    best: NodeMatch | None = None
    for node in graph.nodes:
        # Gripper stroke is an exact-match predicate.
        gripper_match = (
            current_gripper_stroke is not None
            and abs(node.gripper_stroke - current_gripper_stroke) < gripper_stroke_tolerance
        )
        if not gripper_match:
            continue

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
            gripper_match=True,
            within_tolerance=(arm_residual <= joint_tolerance_deg),
        )
        # best.arm_residual is always a float when best is not None
        # (we set it explicitly above). Use explicit None check to avoid
        # the `0.0 or X` falsy-zero trap.
        if best is None or arm_residual < best.arm_residual:
            best = candidate

    return best or empty
