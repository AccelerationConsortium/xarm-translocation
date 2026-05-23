"""Motion-graph interlock layer (Phase 1: data + introspection only).

Loads the whitelist of allowed transitions from ``motion_graph.yaml`` and
answers "given my current node, what can I do next?". Pure data — no
hardware coupling. The controller is responsible for tracking the four
dimensions (arm pose name, rail location name, gripper state name,
payload name) and asking ``MotionGraph.find_node(...)`` to resolve them
to a node id.

This module exists to formalize the implicit sequencing that today lives
only in YAML comments in ``position_config.yaml`` (e.g., "happens when
linear rail is at 0", "You hear a click"). See
``ac-organic-lab/docs/INTERLOCKS.md`` for where this fits in the
four-layer interlock model (layers 1+2).

Phase 1 ships the loader, validation, and query API. No move call is
gated by the graph yet — the controller surfaces the current node and
reachable nodes through ``/status.details`` but otherwise behaves
identically to before.
"""

from __future__ import annotations

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


# ── Enums ────────────────────────────────────────────────────────────


class MoveMode(str, Enum):
    JOINT = "joint"
    LINEAR = "linear"


class GraphMode(str, Enum):
    OFF = "off"            # graph not consulted
    ADVISORY = "advisory"  # off-whitelist moves are warned, allowed
    STRICT = "strict"      # off-whitelist moves are rejected (Phase 2)


# ── Domain types ─────────────────────────────────────────────────────


@dataclass(frozen=True)
class GripperState:
    """A named commanded gripper mechanical position.

    The stroke is the SDK position value (bio_gen2 range: 71-150).
    Force is not a state attribute — it's an action parameter on the
    edge that performs a grip.
    """
    name: str
    stroke: float


@dataclass(frozen=True)
class Payload:
    """A named item identity that can be held in the gripper.

    ``empty`` is the no-payload sentinel; all other payloads represent
    a specific labware identity (plate type, vial, etc.).
    """
    name: str
    description: str = ""


@dataclass(frozen=True)
class Node:
    """A reachable state in the (arm, rail, gripper, payload) product."""
    id: str
    arm: str          # name in position_config.yaml::positions
    rail: str         # name in linear_track_config.yaml::locations
    gripper: str      # name in motion_graph.yaml::gripper_states
    payload: str      # name in motion_graph.yaml::payloads
    tags: tuple[str, ...] = ()


@dataclass(frozen=True)
class GripAction:
    """Edge sub-block: parameters for closing the gripper around a payload."""
    stroke: float
    force: float | None = None


@dataclass(frozen=True)
class ReleaseAction:
    """Edge sub-block: parameters for releasing a payload (always opens to
    a known stroke; force is irrelevant)."""
    stroke: float | None = None  # defaults to gripper_states[open].stroke at runtime


@dataclass(frozen=True)
class Edge:
    """A whitelisted directional transition between two nodes."""
    from_node: str
    to_node: str
    mode: MoveMode
    speed: float | None = None
    grip: GripAction | None = None
    release: ReleaseAction | None = None
    preconditions: tuple[str, ...] = ()
    comment: str = ""


@dataclass
class ControllerView:
    """Minimal read-only snapshot a guard / advisory check needs.

    Decouples the graph from ``XArmController`` so this module is unit-
    testable without any arm fixtures. The controller builds one of
    these per evaluation.
    """
    gripper_payload: str           # "empty" or a payload name
    gripper_stroke: float | None = None
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


# ── The graph ────────────────────────────────────────────────────────


class MotionGraph:
    def __init__(
        self,
        nodes: dict[str, Node],
        edges: list[Edge],
        gripper_states: dict[str, GripperState],
        payloads: dict[str, Payload],
        preconditions: dict[str, PreconditionFn] | None = None,
    ):
        self._nodes = nodes
        self._edges = edges
        self._gripper_states = gripper_states
        self._payloads = payloads
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

        gripper_states = {
            name: GripperState(name=name, stroke=float(spec["stroke"]))
            for name, spec in (data.get("gripper_states") or {}).items()
        }
        payloads = {
            name: Payload(name=name, description=(spec or {}).get("description", ""))
            for name, spec in (data.get("payloads") or {}).items()
        }
        nodes = {}
        for n in data.get("nodes", []) or []:
            node = Node(
                id=n["id"],
                arm=n["arm"],
                rail=n["rail"],
                gripper=n["gripper"],
                payload=n["payload"],
                tags=tuple(n.get("tags", [])),
            )
            if node.id in nodes:
                raise GraphError(f"duplicate node id: {node.id!r}")
            nodes[node.id] = node

        edges: list[Edge] = []
        seen_pairs: set[tuple[str, str]] = set()
        for e in data.get("edges", []) or []:
            action = e.get("action") or {}
            grip_spec = action.get("grip")
            release_spec = action.get("release")
            grip = (
                GripAction(stroke=float(grip_spec["stroke"]), force=grip_spec.get("force"))
                if grip_spec else None
            )
            release = (
                ReleaseAction(stroke=release_spec.get("stroke"))
                if release_spec is not None else None
            )
            edge = Edge(
                from_node=e["from"],
                to_node=e["to"],
                mode=MoveMode(e["mode"]),
                speed=e.get("speed"),
                grip=grip,
                release=release,
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

        return cls(nodes, edges, gripper_states, payloads, preconditions)

    # ── Validation ───────────────────────────────────────────────

    def _validate_topology(self) -> None:
        # Empty-payload sentinel must exist.
        if "empty" not in self._payloads:
            raise GraphError("payloads.empty is required as the no-payload sentinel")

        # Identify the fully-open gripper state(s) for coherence rule 1.
        # We treat the state with the maximum stroke as "open" for the
        # purpose of "non-empty payload implies not fully open".
        if not self._gripper_states:
            raise GraphError("at least one gripper_state must be defined")
        open_strokes = {
            s.stroke for s in self._gripper_states.values()
            if s.stroke == max(g.stroke for g in self._gripper_states.values())
        }

        # Per-node validation.
        for n in self._nodes.values():
            if n.gripper not in self._gripper_states:
                raise GraphError(
                    f"node {n.id!r} references unknown gripper_state {n.gripper!r}"
                )
            if n.payload not in self._payloads:
                raise GraphError(
                    f"node {n.id!r} references unknown payload {n.payload!r}"
                )
            # Coherence rule 1: non-empty payload cannot coexist with the
            # fully-open gripper. Holding a plate with a fully-open
            # gripper is physically impossible.
            if n.payload != "empty":
                stroke = self._gripper_states[n.gripper].stroke
                if stroke in open_strokes:
                    raise GraphError(
                        f"node {n.id!r}: payload {n.payload!r} cannot be held "
                        f"with fully-open gripper {n.gripper!r}"
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

            from_node = self._nodes[e.from_node]
            to_node = self._nodes[e.to_node]
            payload_changed = from_node.payload != to_node.payload
            grips = from_node.payload == "empty" and to_node.payload != "empty"
            releases = from_node.payload != "empty" and to_node.payload == "empty"

            # Coherence rule 2: payload-changing edges must carry an action.
            if grips and e.grip is None:
                raise GraphError(
                    f"edge {e.from_node!r}->{e.to_node!r} grips a payload "
                    f"({from_node.payload!r} -> {to_node.payload!r}) but has no action.grip"
                )
            if releases and e.release is None:
                raise GraphError(
                    f"edge {e.from_node!r}->{e.to_node!r} releases a payload "
                    f"({from_node.payload!r} -> {to_node.payload!r}) but has no action.release"
                )

            # Coherence rule 3: edges that DON'T change payload must NOT
            # carry grip/release blocks. Forces explicit modeling.
            if not payload_changed and (e.grip is not None or e.release is not None):
                raise GraphError(
                    f"edge {e.from_node!r}->{e.to_node!r} carries a grip/release "
                    f"action but payload does not change ({from_node.payload!r})"
                )

            # Payload identity swaps (plate_A -> plate_B without going
            # through empty) make no physical sense — you can't hand
            # off a plate without an intermediate empty state.
            if payload_changed and not grips and not releases:
                raise GraphError(
                    f"edge {e.from_node!r}->{e.to_node!r} swaps payload identity "
                    f"({from_node.payload!r} -> {to_node.payload!r}) without "
                    f"transiting 'empty'; insert intermediate release+grip edges"
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

    @property
    def payloads(self) -> Iterable[Payload]:
        return self._payloads.values()

    def node(self, node_id: str) -> Node:
        if node_id not in self._nodes:
            raise UnknownNodeError(node_id)
        return self._nodes[node_id]

    def has_node(self, node_id: str) -> bool:
        return node_id in self._nodes

    def find_node(
        self, arm: str | None, rail: str | None, gripper: str | None, payload: str | None,
    ) -> Node | None:
        """Resolve a 4-tuple of named coordinates to a node id, or None.

        Any None coordinate causes a None result — the controller must
        have all four named before a node is pin-able.
        """
        if None in (arm, rail, gripper, payload):
            return None
        for n in self._nodes.values():
            if (n.arm == arm and n.rail == rail
                    and n.gripper == gripper and n.payload == payload):
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
    return view.gripper_payload == "empty"


def _plate_held(graph: MotionGraph, edge: Edge, view: ControllerView) -> bool:
    return view.gripper_payload != "empty"


DEFAULT_PRECONDITIONS: dict[str, PreconditionFn] = {
    "gripper_empty": _gripper_empty,
    "plate_held": _plate_held,
}
