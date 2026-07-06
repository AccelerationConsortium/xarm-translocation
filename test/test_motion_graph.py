"""Unit tests for MotionGraph.

Covers:
- the real motion_graph.yaml loads and is internally consistent
- the loader's query API (nodes, edges, outgoing, find_node, find_edge)
- id-suffix parsing (gripper_stroke + grip_intent)
- explicit fallback for legacy _press nodes
- stroke range validation
- duplicate node ids and duplicate (from, to) edges are rejected
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
    UnknownNodeError,
    parse_node_gripper,
)


REAL_YAML = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'src', 'settings', 'motion_graph.yaml')
)


def _base_dict() -> dict:
    """Minimum valid dict using the new id-suffix convention."""
    return {
        "schema_version": "0.1",
        "nodes": [
            {"id": "a_empty",    "arm": "robot_home",    "rail": "Home"},
            {"id": "b_empty",    "arm": "uplc_plate_high", "rail": "Home"},
            {"id": "b_grip_120", "arm": "uplc_plate_high", "rail": "Home"},
        ],
        "edges": [
            {"from": "a_empty",    "to": "b_empty",    "mode": "joint",  "speed": 30},
            {"from": "b_empty",    "to": "a_empty",    "mode": "joint",  "speed": 30},
            {"from": "b_empty",    "to": "b_grip_120", "mode": "linear", "speed": 10},
            {"from": "b_grip_120", "to": "b_empty",    "mode": "linear", "speed": 10},
        ],
    }


# ── id-suffix parsing ────────────────────────────────────────────────


def test_parse_node_gripper_empty_suffix():
    stroke, intent = parse_node_gripper("deck_slot1_high_empty")
    assert stroke == 150.0
    assert intent == GripIntent.NONE


def test_parse_node_gripper_bare_name():
    stroke, intent = parse_node_gripper("robot_home")
    assert stroke == 150.0
    assert intent == GripIntent.NONE


def test_parse_node_gripper_grip_suffix():
    stroke, intent = parse_node_gripper("deck_slot1_low_grip_120")
    assert stroke == 120.0
    assert intent == GripIntent.GRASP


def test_parse_node_gripper_open_suffix():
    stroke, intent = parse_node_gripper("transit_open_130")
    assert stroke == 130.0
    assert intent == GripIntent.POSITION


def test_parse_node_gripper_fractional_stroke():
    stroke, intent = parse_node_gripper("some_pose_grip_85")
    assert stroke == 85.0
    assert intent == GripIntent.GRASP


# ── Real YAML loads ──────────────────────────────────────────────────


def test_real_yaml_loads_and_validates():
    """The shipped motion_graph.yaml must load cleanly with default preconditions."""
    graph = MotionGraph.from_yaml(REAL_YAML, preconditions=DEFAULT_PRECONDITIONS)
    nodes = {n.id for n in graph.nodes}
    assert "robot_home" in nodes
    assert "uplc_draw_open_close" in nodes
    # _held nodes must now be _grip_120
    assert "opentrons_2_high_grip_120" in nodes
    assert "opentrons_2_high_held" not in nodes
    # The drawer close pose is a leaf for direct return — must back out.
    assert graph.allowed_targets("uplc_draw_open_close") == ["uplc_draw_open_min"]


def test_real_yaml_press_nodes_have_explicit_fields():
    """Legacy _press nodes must still load and carry gripper_stroke=120, GRASP intent."""
    graph = MotionGraph.from_yaml(REAL_YAML, preconditions=DEFAULT_PRECONDITIONS)
    press_node = graph.node("deck_slot1_low_press")
    assert press_node.gripper_stroke == 120.0
    assert press_node.grip_intent == GripIntent.GRASP


def test_real_yaml_reachability_from_home():
    graph = MotionGraph.from_yaml(REAL_YAML, preconditions=DEFAULT_PRECONDITIONS)
    unreachable = set(graph.unreachable_nodes("robot_home"))
    drawer = {
        "uplc_draw_home", "uplc_draw_open_max",
        "uplc_draw_open_min", "uplc_draw_open_close",
    }
    assert drawer & unreachable == set()


# ── Query API ────────────────────────────────────────────────────────


def test_find_node_resolves_full_tuple():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    n = graph.find_node("robot_home", "Home", 150.0)
    assert n is not None and n.id == "a_empty"


def test_find_node_returns_none_for_partial_tuple():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    assert graph.find_node(None, "Home", 150.0) is None
    assert graph.find_node("robot_home", None, 150.0) is None
    assert graph.find_node("robot_home", "Home", None) is None


def test_find_node_returns_none_for_unmatched_stroke():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    # There is no node at robot_home with stroke 71.
    assert graph.find_node("robot_home", "Home", 71.0) is None


def test_find_node_resolves_grip_node():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    n = graph.find_node("uplc_plate_high", "Home", 120.0)
    assert n is not None and n.id == "b_grip_120"


def test_outgoing_handles_sentinels():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    assert graph.outgoing(None) == []
    assert graph.outgoing(UNKNOWN_NODE) == []
    assert len(graph.outgoing("b_empty")) == 2  # b_empty->a_empty and b_empty->b_grip_120


def test_node_lookup_raises_on_missing():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    with pytest.raises(UnknownNodeError):
        graph.node("nonexistent")


def test_find_edge_returns_edge_or_none():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    e = graph.find_edge("a_empty", "b_empty")
    assert e is not None
    assert e.mode == MoveMode.JOINT
    assert graph.find_edge("a_empty", "b_grip_120") is None


def test_adjacency_summary_lists_every_node():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    summary = graph.adjacency_summary()
    assert set(summary.keys()) == {"a_empty", "b_empty", "b_grip_120"}
    assert summary["a_empty"] == ["b_empty"]
    assert summary["b_grip_120"] == ["b_empty"]


# ── Stroke range validation ──────────────────────────────────────────


def test_stroke_below_range_is_rejected():
    data = _base_dict()
    data["nodes"].append({
        "id": "bad_node", "arm": "robot_home", "rail": "Home",
        "gripper_stroke": 50, "grip_intent": "none",
    })
    with pytest.raises(GraphError, match="outside the valid range"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


def test_stroke_above_range_is_rejected():
    data = _base_dict()
    data["nodes"].append({
        "id": "bad_node", "arm": "robot_home", "rail": "Home",
        "gripper_stroke": 200, "grip_intent": "none",
    })
    with pytest.raises(GraphError, match="outside the valid range"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


# ── Legacy node with explicit fields ────────────────────────────────


def test_legacy_node_with_explicit_fields_loads():
    data = _base_dict()
    data["nodes"].append({
        "id": "some_low_press", "arm": "robot_home", "rail": "Home",
        "gripper_stroke": 120, "grip_intent": "grasp",
    })
    graph = MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)
    n = graph.node("some_low_press")
    assert n.gripper_stroke == 120.0
    assert n.grip_intent == GripIntent.GRASP


def test_legacy_node_without_explicit_fields_raises():
    """A node with an explicit but invalid grip_intent value must error."""
    data = _base_dict()
    data["nodes"].append({
        "id": "mystery_press", "arm": "robot_home", "rail": "Home",
        "gripper_stroke": 120, "grip_intent": "unknown_intent",
    })
    with pytest.raises(GraphError, match="unknown grip_intent"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


# ── Node grip_intent from id suffix ─────────────────────────────────


def test_grip_node_has_grasp_intent():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    n = graph.node("b_grip_120")
    assert n.grip_intent == GripIntent.GRASP
    assert n.gripper_stroke == 120.0


def test_empty_node_has_none_intent():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    n = graph.node("a_empty")
    assert n.grip_intent == GripIntent.NONE
    assert n.gripper_stroke == 150.0


# ── Topology error cases ─────────────────────────────────────────────


def test_unknown_schema_version_is_rejected():
    data = _base_dict()
    data["schema_version"] = "9.9"
    with pytest.raises(GraphError, match="unsupported schema_version"):
        MotionGraph.from_dict(data)


def test_duplicate_node_id_is_rejected():
    data = _base_dict()
    data["nodes"].append({"id": "a_empty", "arm": "robot_home", "rail": "Home"})
    with pytest.raises(GraphError, match="duplicate node id"):
        MotionGraph.from_dict(data)


def test_duplicate_edge_pair_is_rejected():
    data = _base_dict()
    data["edges"].append({"from": "a_empty", "to": "b_empty", "mode": "linear", "speed": 99})
    with pytest.raises(GraphError, match="duplicate edge"):
        MotionGraph.from_dict(data)


def test_edge_to_unknown_node_is_rejected():
    data = _base_dict()
    data["edges"].append({"from": "a_empty", "to": "ghost", "mode": "joint"})
    with pytest.raises(GraphError, match="edge to unknown node"):
        MotionGraph.from_dict(data)


def test_unregistered_precondition_is_rejected():
    data = _base_dict()
    for e in data["edges"]:
        if e["from"] == "a_empty" and e["to"] == "b_empty":
            e["preconditions"] = ["mystery_guard"]
    with pytest.raises(GraphError, match="unregistered precondition"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


# ── Reachability ─────────────────────────────────────────────────────


def test_unreachable_nodes_detects_isolated_subgraphs():
    data = _base_dict()
    data["nodes"].append({"id": "isolated", "arm": "z", "rail": "Home"})
    graph = MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)
    assert "isolated" in graph.unreachable_nodes("a_empty")
