"""Unit tests for MotionGraph (schema 0.2 — gripper-leaf model).

Covers:
- the real motion_graph.yaml loads and is internally consistent
- the gripper-state catalog (loading, validation, resolve_gripper_state)
- the loader's query API (nodes, edges, outgoing, 2-tuple find_node, find_edge)
- per-node gripper_states / gripper_transitions validation
- state-aware queries (edge_allows_gripper, outgoing_for_state,
  allowed_gripper_targets)
- duplicate node ids, duplicate (arm, rail) positions, duplicate edges
- edges whose endpoints share no gripper state are rejected
- precondition registration
- unreachable_nodes / adjacency_summary behaviors
"""

from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core.motion_graph import (
    DEFAULT_PRECONDITIONS,
    UNKNOWN_NODE,
    Edge,
    GraphError,
    GraphMode,
    GripIntent,
    MoveMode,
    MotionGraph,
    Node,
    NoPathError,
    UnknownNodeError,
)


REAL_YAML = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'src', 'settings', 'motion_graph.yaml')
)


def _catalog() -> dict:
    return {
        "empty":    {"stroke": 150, "intent": "none"},
        "grip_120": {"stroke": 120, "intent": "grasp"},
        "reach_90": {"stroke": 90,  "intent": "position"},
    }


def _base_dict() -> dict:
    """Minimum valid schema-0.2 dict: two positions, one of which allows
    holding and can grip/release while parked."""
    return {
        "schema_version": "0.2",
        "gripper_states": _catalog(),
        "nodes": [
            {"id": "a", "arm": "robot_home", "rail": "Home"},
            {"id": "b", "arm": "uplc_plate_high", "rail": "Home",
             "gripper_states": ["empty", "grip_120"],
             "gripper_transitions": [["empty", "grip_120"], ["grip_120", "empty"]]},
        ],
        "edges": [
            {"from": "a", "to": "b", "mode": "joint",  "speed": 30},
            {"from": "b", "to": "a", "mode": "joint",  "speed": 30},
        ],
    }


# ── Catalog ──────────────────────────────────────────────────────────


def test_catalog_loads_states_with_stroke_and_intent():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    gs = graph.gripper_state("grip_120")
    assert gs.stroke == 120.0
    assert gs.intent == GripIntent.GRASP
    assert graph.gripper_state("reach_90").intent == GripIntent.POSITION
    assert graph.gripper_state("empty").intent == GripIntent.NONE


def test_catalog_missing_empty_is_rejected():
    data = _base_dict()
    del data["gripper_states"]["empty"]
    # 'a' defaults to [empty] so this also breaks node validation — but
    # the catalog check fires first.
    with pytest.raises(GraphError, match="must define 'empty'"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


def test_catalog_stroke_out_of_range_is_rejected():
    data = _base_dict()
    data["gripper_states"]["too_narrow"] = {"stroke": 50, "intent": "grasp"}
    with pytest.raises(GraphError, match="outside the valid range"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


def test_catalog_unknown_intent_is_rejected():
    data = _base_dict()
    data["gripper_states"]["weird"] = {"stroke": 100, "intent": "squeeze"}
    with pytest.raises(GraphError, match="unknown intent"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


def test_resolve_gripper_state_maps_stroke_to_name():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    assert graph.resolve_gripper_state(150.0) == "empty"
    assert graph.resolve_gripper_state(120.4) == "grip_120"
    assert graph.resolve_gripper_state(100.0) is None
    assert graph.resolve_gripper_state(None) is None


# ── Real YAML loads ──────────────────────────────────────────────────


def test_real_yaml_loads_and_validates():
    """The shipped motion_graph.yaml must load cleanly with default preconditions."""
    graph = MotionGraph.from_yaml(REAL_YAML, preconditions=DEFAULT_PRECONDITIONS)
    nodes = {n.id for n in graph.nodes}
    assert "robot_home" in nodes
    assert "uplc_draw_open_close" in nodes
    # Nodes are bare pose names now — no gripper-state stacking.
    assert "opentrons_2_high" in nodes
    assert "opentrons_2_high_empty" not in nodes
    assert "opentrons_2_high_grip_120" not in nodes
    # The drawer close pose is a leaf for direct return — must back out.
    assert graph.allowed_targets("uplc_draw_open_close") == ["uplc_draw_open_min"]


def test_real_yaml_has_four_state_catalog():
    graph = MotionGraph.from_yaml(REAL_YAML, preconditions=DEFAULT_PRECONDITIONS)
    names = {gs.name for gs in graph.gripper_states}
    assert names == {"empty", "grip_120", "reach_90", "grip_80"}
    assert graph.gripper_state("grip_120").intent == GripIntent.GRASP
    assert graph.gripper_state("reach_90").stroke == 90.0


def test_real_yaml_press_nodes_are_grip_only():
    """The *_press nodes may only be occupied while holding (grip_120)."""
    graph = MotionGraph.from_yaml(REAL_YAML, preconditions=DEFAULT_PRECONDITIONS)
    press_node = graph.node("deck_slot1_low_press")
    assert press_node.gripper_states == ("grip_120",)


def test_real_yaml_pick_nodes_have_transitions():
    graph = MotionGraph.from_yaml(REAL_YAML, preconditions=DEFAULT_PRECONDITIONS)
    n = graph.node("deck_slot1_low")
    assert ("empty", "grip_120") in n.gripper_transitions
    assert ("grip_120", "empty") in n.gripper_transitions
    # Transit nodes allow occupancy but no transitions.
    assert graph.node("deck_high").gripper_transitions == ()


def test_real_yaml_transit_nodes_allow_all_states():
    """Non-press nodes may be occupied in any catalog state (held pass-through)
    and still expose no grip/release transitions at transit poses."""
    graph = MotionGraph.from_yaml(REAL_YAML, preconditions=DEFAULT_PRECONDITIONS)
    all_states = {"empty", "grip_120", "reach_90", "grip_80"}
    for node_id in ("deck_high", "hood_home", "robot_home", "cytation_home"):
        n = graph.node(node_id)
        assert set(n.gripper_states) == all_states
        assert n.gripper_transitions == ()


def test_real_yaml_reachability_from_home():
    graph = MotionGraph.from_yaml(REAL_YAML, preconditions=DEFAULT_PRECONDITIONS)
    unreachable = set(graph.unreachable_nodes("robot_home"))
    drawer = {
        "uplc_draw_home", "uplc_draw_open_max",
        "uplc_draw_open_min", "uplc_draw_open_close",
    }
    assert drawer & unreachable == set()


# Station poses whose graph edges are not wired up yet (poses still being
# tuned on hardware — see the position_config TODOs in motion_graph.yaml).
# These are the ONLY nodes allowed to be unreachable from robot_home. When a
# station is wired up, drop it from this set; when a new orphan appears that
# is NOT in this set, the guard below fails so the regression is caught before
# STRICT mode can strand the arm.
WIP_UNREACHABLE_FROM_HOME = {
    "cytation_home", "cytation_high", "cytation_low",
    "plateloc_home", "plateloc_high", "plateloc_low",
    "opentrons_4_high", "opentrons_4_low", "opentrons_4_low_press",
    "opentrons_6_high", "opentrons_6_low", "opentrons_6_low_press",
    "opentrons_2_low_press",
    "deck_slot1_low_press", "deck_solid_low_press", "hood_shaker_low_press",
    "uplc_plate_high", "uplc_plate_in",
    "robot_home_back", "robot_home_left", "robot_home_right",
}


def test_wip_allowlist_entries_are_real_nodes():
    """The allowlist may not carry stale ids — every entry must still name a
    node in the graph, so deleting/renaming a node forces an allowlist edit."""
    graph = MotionGraph.from_yaml(REAL_YAML, preconditions=DEFAULT_PRECONDITIONS)
    node_ids = {n.id for n in graph.nodes}
    stale = WIP_UNREACHABLE_FROM_HOME - node_ids
    assert not stale, f"allowlist names nodes that no longer exist: {sorted(stale)}"


def test_no_unexpected_unreachable_nodes():
    """Connectivity guard: nothing outside the documented WIP allowlist may be
    unreachable from robot_home. Catches a previously-connected node silently
    losing its edges — which STRICT mode would turn into a stranded arm."""
    graph = MotionGraph.from_yaml(REAL_YAML, preconditions=DEFAULT_PRECONDITIONS)
    unreachable = set(graph.unreachable_nodes("robot_home"))
    unexpected = unreachable - WIP_UNREACHABLE_FROM_HOME
    assert not unexpected, (
        f"nodes unreachable from robot_home but not in the WIP allowlist: "
        f"{sorted(unexpected)} — wire up their edges, or if intentionally "
        f"deferred add them to WIP_UNREACHABLE_FROM_HOME with a reason"
    )


# ── Query API ────────────────────────────────────────────────────────


def test_find_node_resolves_arm_rail_pair():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    n = graph.find_node("robot_home", "Home")
    assert n is not None and n.id == "a"


def test_find_node_returns_none_for_partial_pair():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    assert graph.find_node(None, "Home") is None
    assert graph.find_node("robot_home", None) is None


def test_find_node_returns_none_for_unknown_position():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    assert graph.find_node("robot_home", "Deck") is None


def test_default_gripper_states_is_empty_only():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    assert graph.node("a").gripper_states == ("empty",)
    assert graph.node("a").gripper_transitions == ()


def test_outgoing_handles_sentinels():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    assert graph.outgoing(None) == []
    assert graph.outgoing(UNKNOWN_NODE) == []
    assert len(graph.outgoing("b")) == 1


def test_node_lookup_raises_on_missing():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    with pytest.raises(UnknownNodeError):
        graph.node("nonexistent")


def test_find_edge_returns_edge_or_none():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    e = graph.find_edge("a", "b")
    assert e is not None
    assert e.mode == MoveMode.JOINT
    assert graph.find_edge("a", "nonexistent") is None


def test_adjacency_summary_lists_every_node():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    summary = graph.adjacency_summary()
    assert set(summary.keys()) == {"a", "b"}
    assert summary["a"] == ["b"]
    assert summary["b"] == ["a"]


# ── State-aware queries ──────────────────────────────────────────────


def test_edge_allows_gripper_requires_state_at_both_ends():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    edge = graph.find_edge("a", "b")
    # empty is allowed at both a and b.
    assert graph.edge_allows_gripper(edge, "empty") is True
    # grip_120 is allowed at b but not at a.
    assert graph.edge_allows_gripper(edge, "grip_120") is False
    # unknown state never rides an edge.
    assert graph.edge_allows_gripper(edge, None) is False


def test_outgoing_for_state_filters_edges():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    # From b with empty: b->a allowed.
    assert [e.to_node for e in graph.outgoing_for_state("b", "empty")] == ["a"]
    # From b while holding: b->a would carry grip_120 into a (empty-only) — blocked.
    assert graph.outgoing_for_state("b", "grip_120") == []
    assert graph.allowed_targets_for_state("b", "grip_120") == []
    assert graph.allowed_targets_for_state("b", None) == []


def test_allowed_gripper_targets_uses_node_whitelist():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    assert graph.allowed_gripper_targets("b", "empty") == ["grip_120"]
    assert graph.allowed_gripper_targets("b", "grip_120") == ["empty"]
    # 'a' whitelists no transitions.
    assert graph.allowed_gripper_targets("a", "empty") == []
    # Sentinels / unknowns yield nothing.
    assert graph.allowed_gripper_targets(None, "empty") == []
    assert graph.allowed_gripper_targets(UNKNOWN_NODE, "empty") == []
    assert graph.allowed_gripper_targets("b", None) == []


# ── Per-node validation ──────────────────────────────────────────────


def test_node_with_unknown_gripper_state_is_rejected():
    data = _base_dict()
    data["nodes"][0]["gripper_states"] = ["empty", "grip_9000"]
    with pytest.raises(GraphError, match="unknown gripper_state"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


def test_transition_outside_node_states_is_rejected():
    data = _base_dict()
    # 'a' only allows empty; a transition to grip_120 is incoherent.
    data["nodes"][0]["gripper_transitions"] = [["empty", "grip_120"]]
    with pytest.raises(GraphError, match="outside the node's gripper_states"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


def test_noop_transition_is_rejected():
    data = _base_dict()
    data["nodes"][1]["gripper_transitions"] = [["empty", "empty"]]
    with pytest.raises(GraphError, match="no-op"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


def test_malformed_transition_entry_is_rejected():
    data = _base_dict()
    data["nodes"][1]["gripper_transitions"] = ["empty->grip_120"]
    with pytest.raises(GraphError, match="pairs"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


def test_duplicate_arm_rail_position_is_rejected():
    """One node per physical position — a second node at the same
    (arm, rail) is stacking, which schema 0.2 forbids."""
    data = _base_dict()
    data["nodes"].append({
        "id": "a_clone", "arm": "robot_home", "rail": "Home",
    })
    with pytest.raises(GraphError, match="share the same"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


# ── Cross-rail safety ────────────────────────────────────────────────


def _cross_rail_dict(*, to_tags: list[str]) -> dict:
    """Two nodes at different rails joined by an edge. ``to_tags`` controls
    whether the destination qualifies as a transit gateway."""
    return {
        "schema_version": "0.2",
        "gripper_states": _catalog(),
        "nodes": [
            {"id": "gateway", "arm": "robot_home", "rail": "Home",
             "tags": ["global_home"]},
            {"id": "station", "arm": "cytation_low", "rail": "Cytation",
             "tags": to_tags},
        ],
        "edges": [
            {"from": "gateway", "to": "station", "mode": "joint", "speed": 25},
        ],
    }


def test_cross_rail_edge_to_untagged_station_is_rejected():
    """A rail move that lands straight on a station (no home/transit tag)
    is a loader error — stations are reached by an arm move from local home."""
    data = _cross_rail_dict(to_tags=["cytation", "plate"])
    with pytest.raises(GraphError, match="cross-rail edge"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


def test_cross_rail_edge_between_transit_nodes_is_allowed():
    """The permitted shape: rail translates between two transit gateways."""
    data = _cross_rail_dict(to_tags=["cytation", "transit_home"])
    graph = MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)
    assert graph.find_edge("gateway", "station") is not None


def test_same_rail_edge_to_untagged_node_is_allowed():
    """The rule only governs cross-rail edges; a pure arm move to an
    untagged station pose at the same rail is fine."""
    data = _cross_rail_dict(to_tags=["cytation", "plate"])
    data["nodes"][1]["rail"] = "Home"  # same rail now → pure arm move
    graph = MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)
    assert graph.find_edge("gateway", "station") is not None


# ── Topology error cases ─────────────────────────────────────────────


def test_unknown_schema_version_is_rejected():
    data = _base_dict()
    data["schema_version"] = "0.1"
    with pytest.raises(GraphError, match="unsupported schema_version"):
        MotionGraph.from_dict(data)


def test_duplicate_node_id_is_rejected():
    data = _base_dict()
    data["nodes"].append({"id": "a", "arm": "other_pose", "rail": "Home"})
    with pytest.raises(GraphError, match="duplicate node id"):
        MotionGraph.from_dict(data)


def test_duplicate_edge_pair_is_rejected():
    data = _base_dict()
    data["edges"].append({"from": "a", "to": "b", "mode": "linear", "speed": 99})
    with pytest.raises(GraphError, match="duplicate edge"):
        MotionGraph.from_dict(data)


def test_edge_to_unknown_node_is_rejected():
    data = _base_dict()
    data["edges"].append({"from": "a", "to": "ghost", "mode": "joint"})
    with pytest.raises(GraphError, match="edge to unknown node"):
        MotionGraph.from_dict(data)


def test_edge_with_no_shared_gripper_state_is_rejected():
    """An edge between an empty-only node and a grip-only node can never
    be traversed (the state is invariant along an edge)."""
    data = _base_dict()
    data["nodes"].append({
        "id": "press", "arm": "press_pose", "rail": "Home",
        "gripper_states": ["grip_120"],
    })
    data["edges"].append({"from": "a", "to": "press", "mode": "linear"})
    with pytest.raises(GraphError, match="share no gripper state"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


def test_unregistered_precondition_is_rejected():
    data = _base_dict()
    data["edges"][0]["preconditions"] = ["mystery_guard"]
    with pytest.raises(GraphError, match="unregistered precondition"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


# ── Reachability ─────────────────────────────────────────────────────


def test_unreachable_nodes_detects_isolated_subgraphs():
    data = _base_dict()
    data["nodes"].append({"id": "isolated", "arm": "z", "rail": "Home"})
    graph = MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)
    assert "isolated" in graph.unreachable_nodes("a")


# ── Path planning (plan_path / reachable_set) ────────────────────────


def _path_dict() -> dict:
    """Three-node line a — b — c. 'a' and 'c' are empty-only, 'b' also
    allows grip_120 — so grip_120 can ride no edge at all."""
    data = _base_dict()
    data["nodes"].append({
        "id": "c", "arm": "uplc_plate_in", "rail": "Home",
        "gripper_states": ["empty"],
    })
    data["edges"].append({"from": "b", "to": "c", "mode": "joint", "speed": 30})
    data["edges"].append({"from": "c", "to": "b", "mode": "joint", "speed": 30})
    return data


def test_plan_path_multi_hop():
    graph = MotionGraph.from_dict(_path_dict(), preconditions=DEFAULT_PRECONDITIONS)
    assert graph.plan_path("a", "c", "empty") == ["b", "c"]


def test_plan_path_single_hop_and_same_node():
    graph = MotionGraph.from_dict(_path_dict(), preconditions=DEFAULT_PRECONDITIONS)
    assert graph.plan_path("a", "b", "empty") == ["b"]
    assert graph.plan_path("a", "a", "empty") == []


def test_plan_path_unknown_endpoint_raises():
    graph = MotionGraph.from_dict(_path_dict(), preconditions=DEFAULT_PRECONDITIONS)
    with pytest.raises(UnknownNodeError):
        graph.plan_path("a", "ghost", "empty")
    with pytest.raises(UnknownNodeError):
        graph.plan_path("ghost", "a", "empty")


def test_plan_path_no_path_for_blocked_gripper_state():
    """grip_120 is allowed only at 'b', so no edge can carry it — every
    journey in that state must raise NoPathError (with context attached)."""
    graph = MotionGraph.from_dict(_path_dict(), preconditions=DEFAULT_PRECONDITIONS)
    with pytest.raises(NoPathError) as exc_info:
        graph.plan_path("b", "a", "grip_120")
    assert exc_info.value.from_node == "b"
    assert exc_info.value.to_node == "a"
    assert exc_info.value.gripper_state == "grip_120"


def test_reachable_set_respects_gripper_state():
    graph = MotionGraph.from_dict(_path_dict(), preconditions=DEFAULT_PRECONDITIONS)
    assert graph.reachable_set("a", "empty") == ["b", "c"]
    # grip_120 rides no edge — nothing reachable from anywhere.
    assert graph.reachable_set("b", "grip_120") == []


def test_reachable_set_off_grid_is_empty():
    graph = MotionGraph.from_dict(_path_dict(), preconditions=DEFAULT_PRECONDITIONS)
    assert graph.reachable_set(None, "empty") == []
    assert graph.reachable_set(UNKNOWN_NODE, "empty") == []
    assert graph.reachable_set("ghost", "empty") == []


def test_real_yaml_plan_path_home_to_opentrons_pick():
    """The shipped graph must offer an empty-gripper corridor from
    robot_home to the OT-2 slot-2 pick pose, ending at the target."""
    graph = MotionGraph.from_yaml(REAL_YAML, preconditions=DEFAULT_PRECONDITIONS)
    path = graph.plan_path("robot_home", "opentrons_2_low", "empty")
    assert path, "expected a non-empty path"
    assert path[-1] == "opentrons_2_low"
    # Every hop must be a whitelisted edge traversable while empty.
    cur = "robot_home"
    for hop in path:
        assert hop in graph.allowed_targets_for_state(cur, "empty")
        cur = hop
    # And the travel-target set must agree the destination is reachable.
    assert "opentrons_2_low" in graph.reachable_set("robot_home", "empty")
