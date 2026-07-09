"""Tests for _verify_gripper, the arm-only move_to_node, and
set_gripper_state (the at-node gripper transition).

Covers:
- NONE intent always passes (no hardware read needed)
- GRASP intent passes when actual - commanded >= grasp_min_offset
- GRASP intent fails when actual == commanded (nothing held)
- GRASP intent fails when gap < grasp_min_offset
- GRASP intent fails when gap > grasp_max_offset (when configured)
- POSITION intent passes when |actual - commanded| <= tolerance
- POSITION intent fails when jaws are blocked (actual >> commanded)
- move_to_node is a pure arm move (gripper never touched)
- set_gripper_state: transition whitelist (STRICT rejects, ADVISORY
  warns), moving interlock, off-grid rejection, grasp/reach
  verification, empty uses open_gripper, no-op when already there
"""

from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock, patch, call

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core.motion_graph import (
    DEFAULT_PRECONDITIONS,
    GraphMode,
    GripIntent,
    GripperTransitionError,
    MotionGraph,
)


# ── _verify_gripper unit tests ────────────────────────────────────────


@pytest.fixture
def ctrl(initialized_controller):
    """Controller with a minimal gripper config and a mocked
    get_gripper_position return value."""
    c = initialized_controller
    c.current_gripper_config = {
        'grasp_min_offset': 3,
        'grasp_max_offset': None,
        'position_tolerance': 3,
    }
    return c


def set_actual(ctrl, actual):
    ctrl.get_gripper_position = MagicMock(return_value=actual)


# NONE intent ────────────────────────────────────────────────────────

def test_verify_none_always_passes(ctrl):
    """NONE intent should pass without reading hardware."""
    ctrl.get_gripper_position = MagicMock(side_effect=AssertionError("should not be called"))
    assert ctrl._verify_gripper(150.0, GripIntent.NONE) is True


# GRASP intent ───────────────────────────────────────────────────────

def test_verify_grasp_passes_when_gap_meets_min_offset(ctrl):
    """actual=124, commanded=120, gap=4 >= min_offset=3 → pass."""
    set_actual(ctrl, 124)
    assert ctrl._verify_gripper(120.0, GripIntent.GRASP) is True


def test_verify_grasp_passes_exactly_at_min_offset(ctrl):
    """gap == min_offset exactly is a pass."""
    set_actual(ctrl, 123)  # 123-120=3 == min_offset=3
    assert ctrl._verify_gripper(120.0, GripIntent.GRASP) is True


def test_verify_grasp_fails_when_jaws_reached_commanded(ctrl):
    """actual == commanded means nothing was held → fail."""
    set_actual(ctrl, 120)
    assert ctrl._verify_gripper(120.0, GripIntent.GRASP) is False


def test_verify_grasp_fails_when_gap_below_min_offset(ctrl):
    """gap=1 < min_offset=3 → fail."""
    set_actual(ctrl, 121)  # 121-120=1
    assert ctrl._verify_gripper(120.0, GripIntent.GRASP) is False


def test_verify_grasp_fails_when_gap_exceeds_max_offset(ctrl):
    """With max_offset configured, a too-large gap means pre-contact block."""
    ctrl.current_gripper_config['grasp_max_offset'] = 10
    set_actual(ctrl, 135)  # 135-120=15 > max_offset=10
    assert ctrl._verify_gripper(120.0, GripIntent.GRASP) is False


def test_verify_grasp_passes_when_gap_within_max_offset(ctrl):
    ctrl.current_gripper_config['grasp_max_offset'] = 10
    set_actual(ctrl, 128)  # 128-120=8, 3<=8<=10 → pass
    assert ctrl._verify_gripper(120.0, GripIntent.GRASP) is True


# POSITION intent ────────────────────────────────────────────────────

def test_verify_position_passes_when_reached_commanded(ctrl):
    """actual == commanded means free travel → pass."""
    set_actual(ctrl, 120)
    assert ctrl._verify_gripper(120.0, GripIntent.POSITION) is True


def test_verify_position_passes_within_tolerance(ctrl):
    """delta=2 <= tolerance=3 → pass."""
    set_actual(ctrl, 122)  # |122-120|=2 <= 3
    assert ctrl._verify_gripper(120.0, GripIntent.POSITION) is True


def test_verify_position_fails_when_blocked(ctrl):
    """actual far above commanded means jaws blocked before reaching target."""
    set_actual(ctrl, 130)  # |130-120|=10 > tolerance=3
    assert ctrl._verify_gripper(120.0, GripIntent.POSITION) is False


def test_verify_position_fails_at_boundary(ctrl):
    """delta == tolerance+1 is a fail."""
    set_actual(ctrl, 124)  # |124-120|=4 > tolerance=3
    assert ctrl._verify_gripper(120.0, GripIntent.POSITION) is False


# ── Shared fixture: a small gripper-leaf graph ────────────────────────


def _mini_graph_dict():
    """'start' (arm home) can grip/release/narrow while parked; 'target'
    (arm pickup) allows occupancy but no transitions."""
    return {
        "schema_version": "0.2",
        "gripper_states": {
            "empty":    {"stroke": 150, "intent": "none"},
            "grip_120": {"stroke": 120, "intent": "grasp"},
            "reach_90": {"stroke": 90,  "intent": "position"},
        },
        "nodes": [
            {"id": "start", "arm": "home", "rail": "Home",
             "gripper_states": ["empty", "grip_120", "reach_90"],
             "gripper_transitions": [
                 ["empty", "grip_120"], ["grip_120", "empty"],
                 ["empty", "reach_90"], ["reach_90", "empty"],
             ]},
            {"id": "target", "arm": "pickup", "rail": "Home",
             "gripper_states": ["empty", "grip_120"]},
        ],
        "edges": [
            {"from": "start", "to": "target", "mode": "linear", "speed": 20},
        ],
    }


@pytest.fixture
def move_ctrl(initialized_controller):
    c = initialized_controller
    c.motion_graph = MotionGraph.from_dict(
        _mini_graph_dict(), preconditions=DEFAULT_PRECONDITIONS,
    )
    c.graph_mode = GraphMode.STRICT
    c.last_arm_pose_name = "home"
    c.last_rail_location_name = "Home"
    c.last_gripper_position = 150   # start empty
    c.current_gripper_config = {
        'grasp_min_offset': 3,
        'grasp_max_offset': None,
        'position_tolerance': 3,
        'force': 50,
    }
    return c


# ── move_to_node: pure arm move ───────────────────────────────────────


def test_move_to_node_moves_arm_only(move_ctrl):
    """move_to_node delegates to move_to_named_location and never
    touches the gripper — the stroke is invariant along edges."""
    c = move_ctrl
    with patch.object(c, 'move_to_named_location', return_value=True) as m_arm, \
         patch.object(c, 'move_gripper_to_stroke') as m_grip, \
         patch.object(c, 'open_gripper') as m_open:
        result = c.move_to_node('target')
    assert result is True
    m_arm.assert_called_once_with('pickup', speed=None)
    m_grip.assert_not_called()
    m_open.assert_not_called()


def test_move_to_node_returns_false_when_arm_fails(move_ctrl):
    c = move_ctrl
    with patch.object(c, 'move_to_named_location', return_value=False):
        assert c.move_to_node('target') is False


# ── set_gripper_state: gating ─────────────────────────────────────────


def test_set_gripper_state_grasp_success(move_ctrl):
    """empty -> grip_120 at 'start': whitelisted, actuates to 120 with
    the configured force, and the grasp verifies (jaws settle above)."""
    c = move_ctrl
    with patch.object(c, 'move_gripper_to_stroke', return_value=True) as m_grip, \
         patch.object(c, 'get_gripper_position', return_value=124):
        assert c.set_gripper_state('grip_120') is True
    m_grip.assert_called_once_with(120.0, force=50, wait=True)


def test_set_gripper_state_grasp_fails_at_commanded_stroke(move_ctrl):
    """Jaws reaching 120 exactly means the tray was missed → failure."""
    c = move_ctrl
    with patch.object(c, 'move_gripper_to_stroke', return_value=True), \
         patch.object(c, 'get_gripper_position', return_value=120):
        assert c.set_gripper_state('grip_120') is False


def test_set_gripper_state_reach_success(move_ctrl):
    """empty -> reach_90: position intent — jaws must reach 90."""
    c = move_ctrl
    with patch.object(c, 'move_gripper_to_stroke', return_value=True) as m_grip, \
         patch.object(c, 'get_gripper_position', return_value=90):
        assert c.set_gripper_state('reach_90') is True
    m_grip.assert_called_once_with(90.0, force=50, wait=True)


def test_set_gripper_state_reach_fails_when_blocked(move_ctrl):
    """Jaws stalling before 90 (blocked) → failure."""
    c = move_ctrl
    with patch.object(c, 'move_gripper_to_stroke', return_value=True), \
         patch.object(c, 'get_gripper_position', return_value=110):
        assert c.set_gripper_state('reach_90') is False


def test_set_gripper_state_empty_uses_open_gripper(move_ctrl):
    """Releasing to empty opens fully (no verification)."""
    c = move_ctrl
    c.last_gripper_position = 120  # currently grip_120
    with patch.object(c, 'open_gripper', return_value=True) as m_open, \
         patch.object(c, 'move_gripper_to_stroke') as m_stroke, \
         patch.object(c, 'get_gripper_position',
                      side_effect=AssertionError("empty skips verification")):
        assert c.set_gripper_state('empty') is True
    m_open.assert_called_once_with(wait=True)
    m_stroke.assert_not_called()


def test_set_gripper_state_noop_when_already_there(move_ctrl):
    c = move_ctrl
    with patch.object(c, 'open_gripper') as m_open, \
         patch.object(c, 'move_gripper_to_stroke') as m_grip:
        assert c.set_gripper_state('empty') is True
    m_open.assert_not_called()
    m_grip.assert_not_called()


def test_set_gripper_state_rejects_non_whitelisted_transition(move_ctrl):
    """'target' allows occupancy of grip_120 but whitelists no
    transitions — STRICT refuses the change while parked there."""
    c = move_ctrl
    c.last_arm_pose_name = "pickup"  # parked at 'target'
    with pytest.raises(GripperTransitionError, match="not.*whitelisted"), \
         patch.object(c, 'move_gripper_to_stroke') as m_grip:
        c.set_gripper_state('grip_120')
    m_grip.assert_not_called()


def test_set_gripper_state_advisory_warns_but_proceeds(move_ctrl):
    """ADVISORY logs the violation and actuates anyway."""
    c = move_ctrl
    c.graph_mode = GraphMode.ADVISORY
    c.last_arm_pose_name = "pickup"  # no transitions whitelisted here
    with patch.object(c, 'move_gripper_to_stroke', return_value=True) as m_grip, \
         patch.object(c, 'get_gripper_position', return_value=124):
        assert c.set_gripper_state('grip_120') is True
    m_grip.assert_called_once()


def test_set_gripper_state_rejects_while_arm_moving(move_ctrl):
    """The moving interlock: the gripper must never change mid-motion."""
    c = move_ctrl
    c.arm.state = 1  # SDK: in motion
    with pytest.raises(GripperTransitionError, match="arm is moving"), \
         patch.object(c, 'move_gripper_to_stroke') as m_grip:
        c.set_gripper_state('grip_120')
    m_grip.assert_not_called()


def test_set_gripper_state_rejects_when_off_grid(move_ctrl):
    c = move_ctrl
    c.last_arm_pose_name = None  # off-grid
    with pytest.raises(GripperTransitionError, match="off-grid"):
        c.set_gripper_state('grip_120')


def test_set_gripper_state_rejects_unknown_current_state(move_ctrl):
    """Off-catalog commanded stroke → the transition can't be validated."""
    c = move_ctrl
    c.last_gripper_position = 100  # matches no catalog state
    with pytest.raises(GripperTransitionError, match="matches no catalog"):
        c.set_gripper_state('grip_120')


def test_set_gripper_state_unknown_target_raises(move_ctrl):
    from src.core.motion_graph import GraphError
    with pytest.raises(GraphError, match="unknown gripper_state"):
        move_ctrl.set_gripper_state('grip_9000')


def test_set_gripper_state_returns_false_on_actuation_failure(move_ctrl):
    c = move_ctrl
    with patch.object(c, 'move_gripper_to_stroke', return_value=False):
        assert c.set_gripper_state('grip_120') is False
