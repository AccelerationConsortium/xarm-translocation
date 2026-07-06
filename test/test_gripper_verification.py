"""Tests for move_to_node orchestrator and _verify_gripper.

Covers:
- NONE intent always passes (no hardware read needed)
- GRASP intent passes when actual - commanded >= grasp_min_offset
- GRASP intent fails when actual == commanded (nothing held)
- GRASP intent fails when gap < grasp_min_offset
- GRASP intent fails when gap > grasp_max_offset (when configured)
- POSITION intent passes when |actual - commanded| <= tolerance
- POSITION intent fails when jaws are blocked (actual >> commanded)
- move_to_node calls arm move + gripper actuation + verification
- move_to_node skips gripper when stroke unchanged
"""

from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock, patch, call

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core.motion_graph import GripIntent


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


# ── move_to_node integration tests ───────────────────────────────────


def _mini_graph_dict():
    return {
        "schema_version": "0.1",
        "nodes": [
            {"id": "start_empty",    "arm": "home",   "rail": "Home"},
            {"id": "target_grip_120","arm": "pickup",  "rail": "Home"},
        ],
        "edges": [
            {"from": "start_empty", "to": "target_grip_120", "mode": "linear", "speed": 20},
        ],
    }


@pytest.fixture
def move_ctrl(initialized_controller):
    from src.core.motion_graph import MotionGraph, DEFAULT_PRECONDITIONS, GraphMode
    c = initialized_controller
    c.motion_graph = MotionGraph.from_dict(
        _mini_graph_dict(), preconditions=DEFAULT_PRECONDITIONS,
    )
    c.graph_mode = GraphMode.ADVISORY
    c.last_rail_location_name = "Home"
    c.last_gripper_position = 150   # start open
    c.current_gripper_config = {
        'grasp_min_offset': 3,
        'grasp_max_offset': None,
        'position_tolerance': 3,
    }
    return c


def test_move_to_node_calls_arm_move_and_gripper(move_ctrl):
    """move_to_node should call move_to_named_location and then gripper."""
    c = move_ctrl
    with patch.object(c, 'move_to_named_location', return_value=True) as m_arm, \
         patch.object(c, 'move_gripper_to_stroke', return_value=True) as m_grip, \
         patch.object(c, 'get_gripper_position', return_value=124):
        result = c.move_to_node('target_grip_120')
    assert result is True
    m_arm.assert_called_once_with('pickup', speed=None)
    m_grip.assert_called_once_with(120.0, force=None, wait=True)


def test_move_to_node_skips_gripper_when_stroke_unchanged(move_ctrl):
    """When already at the target stroke, no gripper command is issued."""
    c = move_ctrl
    c.last_gripper_position = 120  # already at grip stroke
    with patch.object(c, 'move_to_named_location', return_value=True), \
         patch.object(c, 'move_gripper_to_stroke') as m_grip, \
         patch.object(c, 'get_gripper_position', return_value=124):
        result = c.move_to_node('target_grip_120')
    m_grip.assert_not_called()
    assert result is True


def test_move_to_node_returns_false_when_arm_fails(move_ctrl):
    """Failed arm move must abort before gripper actuation."""
    c = move_ctrl
    with patch.object(c, 'move_to_named_location', return_value=False) as m_arm, \
         patch.object(c, 'move_gripper_to_stroke') as m_grip:
        result = c.move_to_node('target_grip_120')
    assert result is False
    m_grip.assert_not_called()


def test_move_to_node_returns_false_when_grasp_fails(move_ctrl):
    """Grasp verification failure → move_to_node returns False."""
    c = move_ctrl
    with patch.object(c, 'move_to_named_location', return_value=True), \
         patch.object(c, 'move_gripper_to_stroke', return_value=True), \
         patch.object(c, 'get_gripper_position', return_value=120):
        # actual==commanded means nothing held → fail
        result = c.move_to_node('target_grip_120')
    assert result is False


def test_move_to_node_open_node_uses_open_gripper(move_ctrl):
    """Going to an _empty node should call open_gripper, not move_gripper_to_stroke."""
    c = move_ctrl
    c.last_gripper_position = 120  # currently gripping
    with patch.object(c, 'move_to_named_location', return_value=True), \
         patch.object(c, 'open_gripper', return_value=True) as m_open, \
         patch.object(c, 'move_gripper_to_stroke') as m_stroke:
        result = c.move_to_node('start_empty')
    m_open.assert_called_once_with(wait=True)
    m_stroke.assert_not_called()
    assert result is True
