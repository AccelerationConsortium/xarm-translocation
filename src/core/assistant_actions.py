"""Lab-assistant action resolver.

Turns a small set of structured intents (``move_to``, ``pick``, ``place``,
``set_gripper``, ``go_home``) into a concrete, ordered list of motion-graph
steps the API layer can execute. This module never touches hardware and
never calls the LLM — it only reads ``MotionGraph`` (pure data) plus the
controller's current ``(node, gripper_state)`` snapshot, so a "plan" can be
computed and shown to the operator with zero side effects.

The place catalog below is derived entirely from the motion graph's node
ids and tags (see ``src/settings/motion_graph.yaml``) rather than being
hand-maintained, so it stays in sync as stations/slots are added.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Any

from .motion_graph import (
    EMPTY_STATE,
    GraphError,
    GripIntent,
    MotionGraph,
    NoPathError,
    UnknownNodeError,
)

# Canonical "go home" target. Every other safe/home pose (back/left/right)
# is a graph node too, but robot_home is the one operator-facing "home".
HOME_NODE = "robot_home"

# Tags that group nodes into stations. A node without one of these (and
# without a home/safe tag) isn't part of any place and is skipped.
_STATION_TAGS = {"deck", "opentrons", "uplc", "hood", "cytation", "plateloc"}
_HOME_TAGS = {"safe", "global_home", "home"}
# Tags describing *how* a node behaves (gateway/pass-through), not *where*
# it is — never part of a place's identity. ``transit_home`` marks a
# station's front-door pose (see motion_graph.yaml's wiring checklist).
_SKIP_TAGS = {"transit_home", "transit"}

# Node-id suffix -> role within its place. Checked as an exact trailing
# "_<suffix>" match, longest/most-specific first so e.g. "_low_press"
# doesn't get mis-read as "_low" (it wouldn't anyway since the match is a
# full-suffix compare, but keeping specificity first avoids surprises if
# more suffixes are added later).
_ROLE_SUFFIXES: tuple[tuple[str, str], ...] = (
    ("low_press", "press"),
    ("top_plate", "low"),
    ("open_max", "drawer_open"),
    ("open_min", "drawer_ajar"),
    ("open_close", "drawer_closed"),
    ("low", "low"),
    ("high", "high"),
    ("home", "home"),
    ("in", "low"),
)


def _classify_role(node_id: str) -> str:
    for suffix, role in _ROLE_SUFFIXES:
        if node_id.endswith(f"_{suffix}"):
            return role
    return "home"  # bare/unsuffixed node (e.g. robot_home) is its own arrival point


def _place_key(station: str | None, specific: tuple[str, ...]) -> str:
    if station is None:
        return "_".join(specific) or "home"
    if not specific:
        return station
    return f"{station}_{'_'.join(specific)}"


def _place_label(station: str | None, specific: tuple[str, ...]) -> str:
    def cap(word: str) -> str:
        return "UPLC" if word == "uplc" else word.capitalize()

    parts = [cap(station)] if station else []
    parts.extend(cap(p) for p in specific)
    return " ".join(parts) if parts else "Home"


_SLOT_RE = re.compile(r"^slot(\d+)$")


def _place_aliases(station: str | None, specific: tuple[str, ...]) -> tuple[str, ...]:
    aliases: set[str] = set()
    key = _place_key(station, specific)
    aliases.add(key)
    aliases.add(key.replace("_", " "))
    if station:
        aliases.add(station)
    for tag in specific:
        aliases.add(tag)
        aliases.add(tag.replace("_", " "))
        m = _SLOT_RE.match(tag)
        if m:
            aliases.add(f"slot {m.group(1)}")
            if station:
                aliases.add(f"{station} slot {m.group(1)}")
    return tuple(sorted(aliases))


@dataclass(frozen=True)
class Place:
    """A named location assembled from one or more motion-graph nodes.

    ``nodes_by_role`` maps a role (``home``/``high``/``low``/``press``/...)
    to the node id that plays it. ``arrival_node`` is where the assistant
    parks for a plain "move to X"; ``pick_node`` is where a grip/release
    transition is actually whitelisted (None if this place has none, e.g.
    a pure transit waypoint).
    """

    key: str
    label: str
    station: str | None
    tags: tuple[str, ...]
    nodes_by_role: dict[str, str] = field(default_factory=dict)
    aliases: tuple[str, ...] = ()

    @property
    def arrival_node(self) -> str | None:
        for role in ("high", "home", "low"):
            if role in self.nodes_by_role:
                return self.nodes_by_role[role]
        return next(iter(self.nodes_by_role.values()), None)

    @property
    def pick_node(self) -> str | None:
        for role in ("low", "press"):
            if role in self.nodes_by_role:
                return self.nodes_by_role[role]
        return None


def build_place_catalog(graph: MotionGraph) -> dict[str, Place]:
    """Derive the place catalog from the graph's node tags and ids.

    Nodes are grouped by (station tag, remaining tags) — e.g. all of
    ``deck_slot1_high/low/low_press`` share tags ``{deck, slot1}`` and
    collapse into one ``deck_slot1`` place with roles ``high``/``low``/
    ``press``. Nodes tagged only ``safe``/``home`` (no station) become a
    single ``home`` place.
    """
    groups: dict[tuple[str | None, tuple[str, ...]], dict[str, str]] = {}
    group_tags: dict[tuple[str | None, tuple[str, ...]], tuple[str, ...]] = {}

    for node in graph.nodes:
        tags = set(node.tags)
        station_matches = tags & _STATION_TAGS
        if station_matches:
            station = sorted(station_matches)[0]
            specific = tuple(sorted(tags - {station} - _SKIP_TAGS))
        elif tags & _HOME_TAGS:
            station = None
            specific = ("home",)
        else:
            continue  # untagged node; not exposed as a place

        group_key = (station, specific)
        role = _classify_role(node.id)
        groups.setdefault(group_key, {}).setdefault(role, node.id)
        group_tags[group_key] = tuple(sorted(tags))

    catalog: dict[str, Place] = {}
    for (station, specific), nodes_by_role in groups.items():
        key = _place_key(station, specific)
        catalog[key] = Place(
            key=key,
            label=_place_label(station, specific),
            station=station,
            tags=group_tags[(station, specific)],
            nodes_by_role=dict(nodes_by_role),
            aliases=_place_aliases(station, specific),
        )
    return catalog


def find_place_key(catalog: dict[str, Place], text: str) -> str | None:
    """Best-effort match of free text to a catalog key via aliases.

    The LLM layer constrains its tool arguments to the catalog's actual
    keys via a JSON-schema enum, so this is only a safety net for
    slightly-off matches (e.g. a stale enum after a graph edit).
    """
    needle = text.strip().lower().replace("-", " ")
    if needle in catalog:
        return needle
    for place in catalog.values():
        if needle == place.key.lower() or needle == place.label.lower():
            return place.key
        if needle in {a.lower() for a in place.aliases}:
            return place.key
    return None


# ── Plan result ──────────────────────────────────────────────────────


@dataclass
class PlanResult:
    """Outcome of resolving one intent into steps.

    ``steps`` is the ordered list find_path() returns (``[]`` is a valid
    *feasible* result: "you're already there"). ``interpretation`` is a
    short human-readable restatement of what was asked, shown in the
    chat preview regardless of feasibility.
    """

    feasible: bool
    steps: list[dict[str, Any]]
    interpretation: str
    reason: str | None = None
    place: str | None = None


def _off_grid_result(verb: str, target: str | None) -> PlanResult:
    suffix = f" {target}" if target else ""
    return PlanResult(
        feasible=False,
        steps=[],
        interpretation=f"{verb}{suffix}",
        reason=(
            "the robot's position or gripper state is not pinned to the "
            "motion graph; take control and recover to a known node first"
        ),
    )


def _unknown_place_result(verb: str, place_key: str, catalog: dict[str, Place]) -> PlanResult:
    known = ", ".join(sorted(catalog)) or "(none defined)"
    return PlanResult(
        feasible=False,
        steps=[],
        interpretation=f"{verb} {place_key}",
        reason=f"unknown place {place_key!r}; known places: {known}",
    )


def _grasp_target_from_empty(graph: MotionGraph, pick_node: str) -> str | None:
    """The catalog state ``pick_node`` transitions to from empty that is
    actually a grasp (as opposed to a narrowing/position move)."""
    for target in graph.allowed_gripper_targets(pick_node, EMPTY_STATE):
        try:
            if graph.gripper_state(target).intent == GripIntent.GRASP:
                return target
        except GraphError:
            continue
    return None


# ── Intent resolvers ─────────────────────────────────────────────────


def plan_move_to(
    catalog: dict[str, Place],
    graph: MotionGraph,
    current_node: str | None,
    current_gripper_state: str | None,
    place_key: str,
) -> PlanResult:
    place = catalog.get(place_key)
    if place is None:
        return _unknown_place_result("move to", place_key, catalog)
    if current_node is None:
        return _off_grid_result("move to", place.label)

    target = place.arrival_node
    if target is None:
        return PlanResult(
            False, [], f"move to {place.label}",
            reason=f"{place.label} has no reachable position defined", place=place.key,
        )
    try:
        steps = graph.find_path(
            current_node, current_gripper_state, target, current_gripper_state,
        )
    except (NoPathError, UnknownNodeError, GraphError) as exc:
        return PlanResult(False, [], f"move to {place.label}", reason=str(exc), place=place.key)
    return PlanResult(True, steps, f"Move to {place.label}", place=place.key)


def plan_pick(
    catalog: dict[str, Place],
    graph: MotionGraph,
    current_node: str | None,
    current_gripper_state: str | None,
    place_key: str,
) -> PlanResult:
    place = catalog.get(place_key)
    if place is None:
        return _unknown_place_result("pick up from", place_key, catalog)
    if current_node is None or current_gripper_state is None:
        return _off_grid_result("pick up from", place.label)
    if current_gripper_state != EMPTY_STATE:
        return PlanResult(
            False, [], f"pick up from {place.label}",
            reason=(
                f"gripper already holds state {current_gripper_state!r}; "
                f"place it down before picking up something else"
            ),
            place=place.key,
        )

    pick_node = place.pick_node
    if pick_node is None:
        return PlanResult(
            False, [], f"pick up from {place.label}",
            reason=f"{place.label} has no defined pick/place position", place=place.key,
        )
    grip_state = _grasp_target_from_empty(graph, pick_node)
    if grip_state is None:
        return PlanResult(
            False, [], f"pick up from {place.label}",
            reason=f"no grasp transition is whitelisted at {pick_node!r}", place=place.key,
        )
    arrival = place.arrival_node or pick_node
    try:
        steps = graph.find_path(current_node, current_gripper_state, arrival, grip_state)
    except (NoPathError, UnknownNodeError, GraphError) as exc:
        return PlanResult(False, [], f"pick up from {place.label}", reason=str(exc), place=place.key)
    return PlanResult(True, steps, f"Pick up the tray from {place.label}", place=place.key)


def plan_place(
    catalog: dict[str, Place],
    graph: MotionGraph,
    current_node: str | None,
    current_gripper_state: str | None,
    place_key: str,
) -> PlanResult:
    place = catalog.get(place_key)
    if place is None:
        return _unknown_place_result("place at", place_key, catalog)
    if current_node is None or current_gripper_state is None:
        return _off_grid_result("place at", place.label)
    if current_gripper_state == EMPTY_STATE:
        return PlanResult(
            False, [], f"place at {place.label}",
            reason="gripper is empty; there is nothing to place", place=place.key,
        )

    pick_node = place.pick_node
    if pick_node is None:
        return PlanResult(
            False, [], f"place at {place.label}",
            reason=f"{place.label} has no defined pick/place position", place=place.key,
        )
    arrival = place.arrival_node or pick_node
    try:
        steps = graph.find_path(current_node, current_gripper_state, arrival, EMPTY_STATE)
    except (NoPathError, UnknownNodeError, GraphError) as exc:
        return PlanResult(False, [], f"place at {place.label}", reason=str(exc), place=place.key)
    return PlanResult(True, steps, f"Place the tray at {place.label}", place=place.key)


def plan_set_gripper(
    graph: MotionGraph,
    current_node: str | None,
    current_gripper_state: str | None,
    state: str,
) -> PlanResult:
    if not graph.has_gripper_state(state):
        return PlanResult(False, [], f"set gripper to {state!r}",
                           reason=f"unknown gripper state {state!r}")
    if current_node is None:
        return _off_grid_result("set gripper to", state)
    if current_gripper_state == state:
        return PlanResult(True, [], f"Gripper is already at {state!r}")

    allowed = graph.allowed_gripper_targets(current_node, current_gripper_state)
    if state not in allowed:
        return PlanResult(
            False, [], f"set gripper to {state!r}",
            reason=(
                f"transition {current_gripper_state!r} -> {state!r} is not "
                f"whitelisted here (allowed: {allowed or 'none'})"
            ),
        )
    return PlanResult(True, [{"kind": "gripper", "state": state}], f"Set gripper to {state!r}")


def plan_go_home(
    graph: MotionGraph,
    current_node: str | None,
    current_gripper_state: str | None,
) -> PlanResult:
    if current_node is None:
        return _off_grid_result("go", "home")
    if not graph.has_node(HOME_NODE):
        return PlanResult(False, [], "go home", reason="home node not defined in the motion graph")
    try:
        steps = graph.find_path(
            current_node, current_gripper_state, HOME_NODE, current_gripper_state,
        )
    except (NoPathError, UnknownNodeError, GraphError) as exc:
        return PlanResult(False, [], "go home", reason=str(exc))
    return PlanResult(True, steps, "Move to the home position")


def resolve_action(
    catalog: dict[str, Place],
    graph: MotionGraph,
    current_node: str | None,
    current_gripper_state: str | None,
    action: str,
    place: str | None = None,
    gripper_state: str | None = None,
) -> PlanResult:
    """Single entry point the API layer calls with the LLM's chosen tool.

    ``action`` is one of ``move_to``, ``pick``, ``place``, ``set_gripper``,
    ``go_home``. Unknown actions/missing required args resolve to an
    infeasible ``PlanResult`` rather than raising, so the caller can
    always render a chat reply.
    """
    if action == "move_to":
        if not place:
            return PlanResult(False, [], "move", reason="no destination given")
        return plan_move_to(catalog, graph, current_node, current_gripper_state, place)
    if action == "pick":
        if not place:
            return PlanResult(False, [], "pick up", reason="no location given")
        return plan_pick(catalog, graph, current_node, current_gripper_state, place)
    if action == "place":
        if not place:
            return PlanResult(False, [], "place", reason="no location given")
        return plan_place(catalog, graph, current_node, current_gripper_state, place)
    if action == "set_gripper":
        if not gripper_state:
            return PlanResult(False, [], "set gripper", reason="no gripper state given")
        return plan_set_gripper(graph, current_node, current_gripper_state, gripper_state)
    if action == "go_home":
        return plan_go_home(graph, current_node, current_gripper_state)
    return PlanResult(False, [], action, reason=f"unrecognized action {action!r}")
