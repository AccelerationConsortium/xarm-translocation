"""Phase 2 tests: strict-mode dispatch, edge.mode override, speed cap.

The controller tests inject a small in-memory MotionGraph into the
fixture-built controller, then exercise move_to_named_location under
each enforcement mode. Position-config names match the mocked fixture
('home', 'pickup') so the graph's arm-pose names align.
"""

from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock, patch

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Match the controller's effective import path. Conftest loads
# `from src.core.xarm_controller import ...`, which triggers the
# controller's `from .motion_graph` relative import to resolve as
# `src.core.motion_graph`. Importing here from the same module gives
# us an identical class object — pytest.raises uses isinstance, which
# fails silently when src.core.motion_graph and core.motion_graph are
# loaded as separate modules.
from src.core.motion_graph import (
    DEFAULT_PRECONDITIONS,
    EdgeNotAllowedError,
    GraphMode,
    MotionGraph,
)


# ── Fixtures ─────────────────────────────────────────────────────────


def _test_graph_dict():
    """A graph aligned with conftest.mock_config_files' position_config:
       home / pickup (both Cartesian dicts) on rail=Home. Both nodes
       allow the empty and grip_120 leaves so occupancy gating passes by
       default; the occupancy tests below override this. The edge
       home->pickup is LINEAR with speed 25; the reverse is JOINT with
       speed 40 — gives us coverage of both modes.
    """
    return {
        "schema_version": "0.2",
        "gripper_states": {
            "empty":    {"stroke": 150, "intent": "none"},
            "grip_120": {"stroke": 120, "intent": "grasp"},
        },
        "nodes": [
            {"id": "n_home",   "arm": "home",   "rail": "Home",
             "gripper_states": ["empty", "grip_120"]},
            {"id": "n_pickup", "arm": "pickup", "rail": "Home",
             "gripper_states": ["empty", "grip_120"]},
        ],
        "edges": [
            {"from": "n_home",   "to": "n_pickup", "mode": "linear", "speed": 25},
            {"from": "n_pickup", "to": "n_home",   "mode": "joint",  "speed": 40},
        ],
    }


@pytest.fixture
def graph_controller(initialized_controller):
    """Controller with a small in-memory graph attached + Home rail pinned.

    Pinning rail=Home is necessary because the mocked position_config
    doesn't carry the linear track and the graph's nodes all require
    rail=Home for find_node() to resolve them. We also pin the gripper
    stroke to 150 to match the test graph's "open" gripper_state
    (conftest's bio gripper config doesn't carry open_position).
    """
    c = initialized_controller
    c.motion_graph = MotionGraph.from_dict(
        _test_graph_dict(), preconditions=DEFAULT_PRECONDITIONS,
    )
    c.last_rail_location_name = "Home"
    c.last_gripper_position = 150  # resolves to the 'empty' catalog state
    return c


# ── Mode setter ──────────────────────────────────────────────────────


def test_set_graph_mode_changes_mode(graph_controller):
    c = graph_controller
    c.set_graph_mode(GraphMode.STRICT)
    assert c.graph_mode == GraphMode.STRICT
    c.set_graph_mode(GraphMode.ADVISORY)
    assert c.graph_mode == GraphMode.ADVISORY


def test_set_graph_mode_refuses_enable_without_graph(initialized_controller):
    c = initialized_controller
    c.motion_graph = None
    c.graph_mode = GraphMode.OFF
    with pytest.raises(RuntimeError, match="not loaded"):
        c.set_graph_mode(GraphMode.STRICT)
    # OFF is always allowed
    c.set_graph_mode(GraphMode.OFF)
    assert c.graph_mode == GraphMode.OFF


# ── ADVISORY mode does not change dispatch ───────────────────────────


def test_advisory_mode_preserves_preset_dispatch(graph_controller):
    """In ADVISORY mode, edge.mode is metadata only — the preset's
    format (dict in this fixture) still drives dispatch (Cartesian
    move_to_position), and the caller's speed is honored."""
    c = graph_controller
    c.set_graph_mode(GraphMode.ADVISORY)
    c.last_arm_pose_name = "home"  # pin starting node
    assert c.current_node == "n_home"

    # Edge home->pickup is mode=linear in the graph; preset is also dict
    # so this naturally goes through move_to_position. Use a patch to
    # confirm which method is called and with what speed.
    with patch.object(c, "move_to_position", return_value=True) as mtp, \
         patch.object(c, "move_joints", return_value=True) as mj:
        # Note: caller speed 99 is GREATER than edge.speed=25, but
        # ADVISORY does NOT clamp.
        assert c.move_to_named_location("pickup", speed=99) is True
        mtp.assert_called_once()
        mj.assert_not_called()
        kwargs = mtp.call_args.kwargs
        assert kwargs["speed"] == 99  # not clamped


# ── STRICT mode: rejection ───────────────────────────────────────────


def test_strict_mode_rejects_when_current_off_grid(graph_controller):
    c = graph_controller
    c.set_graph_mode(GraphMode.STRICT)
    c.last_arm_pose_name = None  # off-grid
    with pytest.raises(EdgeNotAllowedError, match="off-grid"):
        c.move_to_named_location("pickup")


def test_strict_mode_rejects_when_target_not_a_node(graph_controller, monkeypatch):
    """If the target location is in position_config but no graph node
    matches its 4-tuple, STRICT mode refuses."""
    c = graph_controller
    c.set_graph_mode(GraphMode.STRICT)
    c.last_arm_pose_name = "home"
    # Add a position to the mocked config that has no corresponding node.
    c.position_config["positions"]["orphan"] = {
        "x": 100, "y": 0, "z": 100, "roll": 180, "pitch": 0, "yaw": 0,
    }
    with pytest.raises(EdgeNotAllowedError, match="does not resolve to a graph node"):
        c.move_to_named_location("orphan")


def test_strict_mode_rejects_no_whitelisted_edge(graph_controller):
    """home and pickup are both nodes and home->pickup is whitelisted,
    so we need a third node to test "no edge" rejection. Add one
    in-memory by mutating position_config + the graph."""
    c = graph_controller
    c.set_graph_mode(GraphMode.STRICT)
    c.last_arm_pose_name = "home"
    # Add a target with a node but NO edge from home.
    c.position_config["positions"]["other"] = {
        "x": 200, "y": 0, "z": 200, "roll": 180, "pitch": 0, "yaw": 0,
    }
    # Inject a node for it but no incoming edge.
    data = _test_graph_dict()
    data["nodes"].append({"id": "n_other", "arm": "other", "rail": "Home"})
    c.motion_graph = MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)
    with pytest.raises(EdgeNotAllowedError, match="no whitelisted edge"):
        c.move_to_named_location("other")


# ── STRICT mode: gripper occupancy gating ────────────────────────────


def test_strict_mode_blocks_edge_target_disallows_current_state(graph_controller):
    """Holding grip_120 while the target node only allows empty: the
    state would ride the edge into a node that can't occupy it, so
    STRICT refuses."""
    c = graph_controller
    data = _test_graph_dict()
    data["nodes"][1]["gripper_states"] = ["empty"]  # n_pickup: empty-only
    c.motion_graph = MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)
    c.set_graph_mode(GraphMode.STRICT)
    c.last_arm_pose_name = "home"
    c.last_gripper_position = 120  # grip_120
    assert c.current_gripper_state == "grip_120"
    with pytest.raises(EdgeNotAllowedError, match="not traversable"):
        c.move_to_named_location("pickup")


def test_strict_mode_blocks_when_gripper_state_unknown(graph_controller):
    """An off-catalog stroke (e.g. after a manual stroke command) means
    the occupancy invariant can't be checked — STRICT refuses."""
    c = graph_controller
    c.set_graph_mode(GraphMode.STRICT)
    c.last_arm_pose_name = "home"
    c.last_gripper_position = 100  # matches no catalog state
    assert c.current_gripper_state is None
    with pytest.raises(EdgeNotAllowedError, match="matches no catalog state"):
        c.move_to_named_location("pickup")


def test_strict_mode_allows_edge_when_state_allowed_at_both_ends(graph_controller):
    c = graph_controller
    c.set_graph_mode(GraphMode.STRICT)
    c.last_arm_pose_name = "home"
    c.last_gripper_position = 120  # grip_120 — allowed at both nodes
    with patch.object(c, "move_to_position", return_value=True) as mtp:
        assert c.move_to_named_location("pickup") is True
        mtp.assert_called_once()


def test_reachable_node_ids_filters_by_gripper_state(graph_controller):
    c = graph_controller
    data = _test_graph_dict()
    data["nodes"][1]["gripper_states"] = ["empty"]  # n_pickup: empty-only
    c.motion_graph = MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)
    c.last_arm_pose_name = "home"
    c.last_gripper_position = 150
    assert c.reachable_node_ids() == ["n_pickup"]
    c.last_gripper_position = 120  # holding — can't enter empty-only node
    assert c.reachable_node_ids() == []
    c.last_gripper_position = 100  # off-catalog — nothing is reachable
    assert c.reachable_node_ids() == []


# ── STRICT mode: edge.mode override ──────────────────────────────────


def test_strict_mode_forces_joint_dispatch_via_edge(graph_controller):
    """Preset 'home' is a Cartesian dict (would normally dispatch
    cartesian), but the n_pickup -> n_home edge has mode=joint. The
    fixture's preset is a dict, so this combination triggers the
    "edge requires joint but preset is cartesian" guardrail — verify
    it refuses cleanly rather than silently doing the wrong thing."""
    c = graph_controller
    c.set_graph_mode(GraphMode.STRICT)
    c.last_arm_pose_name = "pickup"  # at n_pickup; edge to n_home is joint
    assert c.current_node == "n_pickup"
    # Returns False with a printed error because preset is cartesian.
    assert c.move_to_named_location("home") is False


def test_strict_mode_forces_linear_dispatch_when_edge_says_linear(graph_controller):
    """home -> pickup edge has mode=linear, and preset is already
    cartesian, so this is the natural-fit case. Confirm move_to_position
    is called (not move_joints) and the call goes through cleanly."""
    c = graph_controller
    c.set_graph_mode(GraphMode.STRICT)
    c.last_arm_pose_name = "home"
    with patch.object(c, "move_to_position", return_value=True) as mtp, \
         patch.object(c, "move_joints", return_value=True) as mj:
        assert c.move_to_named_location("pickup") is True
        mtp.assert_called_once()
        mj.assert_not_called()


# ── STRICT mode: speed cap ───────────────────────────────────────────


def test_strict_mode_clamps_speed_to_edge_cap(graph_controller):
    c = graph_controller
    c.set_graph_mode(GraphMode.STRICT)
    c.last_arm_pose_name = "home"
    with patch.object(c, "move_to_position", return_value=True) as mtp:
        # Caller asks for 100; edge.speed cap is 25.
        c.move_to_named_location("pickup", speed=100)
        kwargs = mtp.call_args.kwargs
        assert kwargs["speed"] == 25


def test_strict_mode_honors_slower_speed(graph_controller):
    c = graph_controller
    c.set_graph_mode(GraphMode.STRICT)
    c.last_arm_pose_name = "home"
    with patch.object(c, "move_to_position", return_value=True) as mtp:
        # Caller asks for 10; below the 25 cap.
        c.move_to_named_location("pickup", speed=10)
        kwargs = mtp.call_args.kwargs
        assert kwargs["speed"] == 10


# ── Transition recording ─────────────────────────────────────────────


def test_successful_named_move_records_transition(graph_controller):
    c = graph_controller
    c.set_graph_mode(GraphMode.ADVISORY)
    c.last_arm_pose_name = "home"
    assert c.last_transition is None
    assert c.move_to_named_location("pickup") is True
    assert c.last_transition is not None
    assert c.last_transition["from_node"] == "n_home"
    assert c.last_transition["to_node"] == "n_pickup"


def test_off_grid_named_move_does_not_record(graph_controller):
    c = graph_controller
    c.set_graph_mode(GraphMode.ADVISORY)
    c.last_arm_pose_name = None  # off-grid start
    assert c.last_transition is None
    # Move succeeds in ADVISORY mode but starting node was None, so no
    # transition is recorded.
    assert c.move_to_named_location("pickup") is True
    assert c.last_transition is None


# ── OFF mode is fully permissive and pre-Phase-1 behavior ────────────


def test_off_mode_is_fully_permissive(graph_controller):
    c = graph_controller
    c.set_graph_mode(GraphMode.OFF)
    c.last_arm_pose_name = None  # off-grid would refuse in STRICT
    # Add an orphan target that has no node.
    c.position_config["positions"]["orphan"] = {
        "x": 100, "y": 0, "z": 100, "roll": 180, "pitch": 0, "yaw": 0,
    }
    assert c.move_to_named_location("orphan") is True


# ── travel_to_node (multi-hop) ───────────────────────────────────────


def _three_node_graph_dict():
    """n_home -> n_pickup -> n_far chain (plus reverses) and an
    edge-less n_island for the no-path case. Arm names beyond the
    conftest presets are fine: travel tests mock move_to_node, so only
    the planner consults the topology."""
    data = _test_graph_dict()
    data["nodes"].append({"id": "n_far", "arm": "far", "rail": "Home",
                          "gripper_states": ["empty", "grip_120"]})
    data["nodes"].append({"id": "n_island", "arm": "island", "rail": "Home",
                          "gripper_states": ["empty", "grip_120"]})
    data["edges"].append({"from": "n_pickup", "to": "n_far", "mode": "joint"})
    data["edges"].append({"from": "n_far", "to": "n_pickup", "mode": "joint"})
    return data


@pytest.fixture
def travel_controller(graph_controller):
    c = graph_controller
    c.motion_graph = MotionGraph.from_dict(
        _three_node_graph_dict(), preconditions=DEFAULT_PRECONDITIONS,
    )
    c.last_arm_pose_name = "home"  # pins current_node to n_home
    return c


def test_travel_to_node_multi_hop_success(travel_controller):
    c = travel_controller
    with patch.object(c, "move_to_node", return_value=True) as mock_move:
        result = c.travel_to_node("n_far", speed=20)
    assert result == {"success": True, "path": ["n_pickup", "n_far"],
                      "completed": ["n_pickup", "n_far"], "failed_hop": None}
    assert [call.args[0] for call in mock_move.call_args_list] == \
        ["n_pickup", "n_far"]
    assert all(call.kwargs == {"speed": 20} for call in mock_move.call_args_list)


def test_travel_to_node_already_at_target(travel_controller):
    c = travel_controller
    with patch.object(c, "move_to_node") as mock_move:
        result = c.travel_to_node("n_home")
    assert result["success"] is True
    assert result["path"] == []
    mock_move.assert_not_called()


def test_travel_to_node_fail_fast_reports_progress(travel_controller):
    c = travel_controller
    with patch.object(c, "move_to_node", side_effect=[True, False]):
        result = c.travel_to_node("n_far")
    assert result == {"success": False, "path": ["n_pickup", "n_far"],
                      "completed": ["n_pickup"], "failed_hop": "n_far"}


def test_travel_to_node_off_grid_raises(travel_controller):
    c = travel_controller
    c.last_arm_pose_name = None  # current_node -> None
    with pytest.raises(EdgeNotAllowedError, match="not pinned"):
        c.travel_to_node("n_far")


def test_travel_to_node_no_path_raises(travel_controller):
    from src.core.motion_graph import NoPathError
    c = travel_controller
    with pytest.raises(NoPathError):
        c.travel_to_node("n_island")


def test_travel_to_node_unknown_target_raises(travel_controller):
    from src.core.motion_graph import UnknownNodeError
    c = travel_controller
    with pytest.raises(UnknownNodeError):
        c.travel_to_node("n_ghost")
