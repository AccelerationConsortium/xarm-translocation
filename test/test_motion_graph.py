"""Unit tests for MotionGraph (Phase 1).

Covers:
- the real motion_graph.yaml loads and is internally consistent
- the loader's query API (nodes, edges, outgoing, find_node, find_edge)
- each of the three coherence rules raises GraphError on bad input
- duplicate node ids and duplicate (from, to) edges are rejected
- unreachable_nodes / adjacency_summary behaviors
- registered vs unregistered preconditions
"""

from __future__ import annotations

import os
import sys
from copy import deepcopy

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core.motion_graph import (
    DEFAULT_PRECONDITIONS,
    UNKNOWN_NODE,
    Edge,
    GraphError,
    GraphMode,
    GripAction,
    MoveMode,
    MotionGraph,
    Node,
    ReleaseAction,
    UnknownNodeError,
)


REAL_YAML = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'src', 'settings', 'motion_graph.yaml')
)


# Minimum valid dict — small synthetic graph used as the base for negative tests.
def _base_dict() -> dict:
    return {
        "schema_version": "0.1",
        "gripper_states": {
            "open":   {"stroke": 150},
            "closed": {"stroke": 71},
            "stroke_88": {"stroke": 88},
        },
        "payloads": {
            "empty": {},
            "plate_a": {"description": "Test plate A"},
        },
        "nodes": [
            {"id": "a", "arm": "robot_home", "rail": "Home", "gripper": "open",       "payload": "empty"},
            {"id": "b", "arm": "uplc_plate_high", "rail": "Home", "gripper": "open",  "payload": "empty"},
            {"id": "c", "arm": "uplc_plate_low",  "rail": "Home", "gripper": "stroke_88", "payload": "plate_a"},
        ],
        "edges": [
            {"from": "a", "to": "b", "mode": "joint",  "speed": 30},
            {"from": "b", "to": "a", "mode": "joint",  "speed": 30},
            {"from": "b", "to": "c", "mode": "linear", "speed": 10,
             "action": {"grip": {"stroke": 88, "force": 50}}},
            {"from": "c", "to": "b", "mode": "linear", "speed": 10,
             "action": {"release": {}}},
        ],
    }


# ── Real YAML loads ──────────────────────────────────────────────────


def test_real_yaml_loads_and_validates():
    """The shipped motion_graph.yaml must load cleanly with default preconditions."""
    graph = MotionGraph.from_yaml(REAL_YAML, preconditions=DEFAULT_PRECONDITIONS)
    nodes = {n.id for n in graph.nodes}
    assert "home" in nodes
    assert "uplc_draw_close_click" in nodes
    # The drawer click is a leaf for direct return — must back out.
    assert graph.allowed_targets("uplc_draw_close_click") == ["uplc_draw_open_min"]


def test_real_yaml_reachability_from_home():
    graph = MotionGraph.from_yaml(REAL_YAML, preconditions=DEFAULT_PRECONDITIONS)
    # Every drawer node should be reachable from home.
    unreachable = graph.unreachable_nodes("home")
    assert unreachable == []


# ── Query API ────────────────────────────────────────────────────────


def test_find_node_resolves_full_tuple():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    n = graph.find_node("robot_home", "Home", "open", "empty")
    assert n is not None and n.id == "a"


def test_find_node_returns_none_for_partial_tuple():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    assert graph.find_node(None, "Home", "open", "empty") is None
    assert graph.find_node("robot_home", None, "open", "empty") is None


def test_find_node_returns_none_for_unmatched_tuple():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    assert graph.find_node("robot_home", "Home", "closed", "empty") is None


def test_outgoing_handles_sentinels():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    assert graph.outgoing(None) == []
    assert graph.outgoing(UNKNOWN_NODE) == []
    assert len(graph.outgoing("b")) == 2  # b->a (retreat) and b->c (grip)


def test_node_lookup_raises_on_missing():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    with pytest.raises(UnknownNodeError):
        graph.node("nonexistent")


def test_find_edge_returns_edge_or_none():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    e = graph.find_edge("a", "b")
    assert e is not None
    assert e.mode == MoveMode.JOINT
    assert graph.find_edge("a", "c") is None


def test_adjacency_summary_lists_every_node():
    graph = MotionGraph.from_dict(_base_dict(), preconditions=DEFAULT_PRECONDITIONS)
    summary = graph.adjacency_summary()
    assert set(summary.keys()) == {"a", "b", "c"}
    assert summary["a"] == ["b"]
    assert summary["c"] == ["b"]


# ── Coherence rule 1: non-empty payload + fully-open gripper is illegal ──


def test_payload_held_with_open_gripper_is_rejected():
    data = _base_dict()
    # Create an illegal node: payload=plate_a with the open gripper.
    data["nodes"].append({
        "id": "bad", "arm": "robot_home", "rail": "Home",
        "gripper": "open", "payload": "plate_a",
    })
    with pytest.raises(GraphError, match="cannot be held with fully-open gripper"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


# ── Coherence rule 2: payload-changing edges must carry the right action ──


def test_grip_edge_missing_action_is_rejected():
    data = _base_dict()
    # Remove the grip action from b->c.
    for e in data["edges"]:
        if e["from"] == "b" and e["to"] == "c":
            e.pop("action", None)
    with pytest.raises(GraphError, match="grips a payload .* but has no action.grip"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


def test_release_edge_missing_action_is_rejected():
    data = _base_dict()
    for e in data["edges"]:
        if e["from"] == "c" and e["to"] == "b":
            e.pop("action", None)
    with pytest.raises(GraphError, match="releases a payload .* but has no action.release"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


# ── Coherence rule 3: non-payload-changing edges must not carry actions ──


def test_action_on_non_payload_change_edge_is_rejected():
    data = _base_dict()
    # a->b doesn't change payload (both empty), but we attach a grip action.
    for e in data["edges"]:
        if e["from"] == "a" and e["to"] == "b":
            e["action"] = {"grip": {"stroke": 88, "force": 50}}
    with pytest.raises(GraphError, match="carries a grip/release action but payload does not change"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


# ── Payload identity-swap rule (subsumed in rule 3 but worth its own test) ──


def test_direct_payload_identity_swap_is_rejected():
    data = _base_dict()
    # Add a node holding a different payload.
    data["payloads"]["plate_b"] = {}
    data["nodes"].append({
        "id": "d", "arm": "uplc_plate_low", "rail": "Home",
        "gripper": "stroke_88", "payload": "plate_b",
    })
    # Edge c (plate_a) -> d (plate_b) without going through empty.
    data["edges"].append({"from": "c", "to": "d", "mode": "linear", "speed": 5})
    with pytest.raises(GraphError, match="swaps payload identity .* without transiting 'empty'"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


# ── Topology error cases ─────────────────────────────────────────────


def test_unknown_schema_version_is_rejected():
    data = _base_dict()
    data["schema_version"] = "9.9"
    with pytest.raises(GraphError, match="unsupported schema_version"):
        MotionGraph.from_dict(data)


def test_missing_empty_payload_is_rejected():
    data = _base_dict()
    del data["payloads"]["empty"]
    # Also drop any node that used empty so we test only the payload-table error.
    data["nodes"] = [n for n in data["nodes"] if n["payload"] != "empty"]
    data["edges"] = []
    with pytest.raises(GraphError, match="payloads.empty is required"):
        MotionGraph.from_dict(data)


def test_duplicate_node_id_is_rejected():
    data = _base_dict()
    data["nodes"].append({
        "id": "a", "arm": "robot_home", "rail": "Home",
        "gripper": "closed", "payload": "empty",
    })
    with pytest.raises(GraphError, match="duplicate node id"):
        MotionGraph.from_dict(data)


def test_duplicate_edge_pair_is_rejected():
    data = _base_dict()
    # Add another a->b edge (the design forbids parallel edges).
    data["edges"].append({"from": "a", "to": "b", "mode": "linear", "speed": 99})
    with pytest.raises(GraphError, match="duplicate edge"):
        MotionGraph.from_dict(data)


def test_edge_to_unknown_node_is_rejected():
    data = _base_dict()
    data["edges"].append({"from": "a", "to": "ghost", "mode": "joint"})
    with pytest.raises(GraphError, match="edge to unknown node"):
        MotionGraph.from_dict(data)


def test_node_with_unknown_gripper_state_is_rejected():
    data = _base_dict()
    data["nodes"].append({
        "id": "bad", "arm": "x", "rail": "Home",
        "gripper": "nonexistent", "payload": "empty",
    })
    with pytest.raises(GraphError, match="references unknown gripper_state"):
        MotionGraph.from_dict(data)


def test_node_with_unknown_payload_is_rejected():
    data = _base_dict()
    data["nodes"].append({
        "id": "bad", "arm": "x", "rail": "Home",
        "gripper": "open", "payload": "ghost_plate",
    })
    with pytest.raises(GraphError, match="references unknown payload"):
        MotionGraph.from_dict(data)


def test_unregistered_precondition_is_rejected():
    data = _base_dict()
    for e in data["edges"]:
        if e["from"] == "a" and e["to"] == "b":
            e["preconditions"] = ["mystery_guard"]
    with pytest.raises(GraphError, match="unregistered precondition"):
        MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)


# ── Reachability ─────────────────────────────────────────────────────


def test_unreachable_nodes_detects_isolated_subgraphs():
    data = _base_dict()
    data["nodes"].append({
        "id": "isolated", "arm": "z", "rail": "Home",
        "gripper": "open", "payload": "empty",
    })
    graph = MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)
    assert "isolated" in graph.unreachable_nodes("a")
