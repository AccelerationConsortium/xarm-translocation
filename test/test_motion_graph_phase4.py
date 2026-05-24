"""Phase 4 tests: nearest-node detection + recover_to.

Covers the find_nearest_node() pure helper, the controller's
suggest_current_node() / recover_to(), and the two new HTTP endpoints
(/graph/nearest + /control/graph/recover_to).
"""

from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock

import pytest
from fastapi.testclient import TestClient

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Match the api_server's effective import path so exception types line
# up across the test/server boundary.
from src.core.motion_graph import (
    DEFAULT_PRECONDITIONS,
    GraphMode,
    MotionGraph,
    NodeMatch,
    RecoveryMismatch,
    UnknownNodeError,
    find_nearest_node,
)


def _graph_dict():
    return {
        "schema_version": "0.1",
        "gripper_states": {"open": {"stroke": 150}, "closed": {"stroke": 71}},
        "payloads": {"empty": {}},
        "nodes": [
            {"id": "n_home",   "arm": "robot_home",   "rail": "Home",    "gripper": "open", "payload": "empty"},
            {"id": "n_drawer", "arm": "uplc_draw_home", "rail": "Home",  "gripper": "open", "payload": "empty"},
            {"id": "n_local1", "arm": "robot_home",   "rail": "Local_1", "gripper": "open", "payload": "empty"},
        ],
        "edges": [
            {"from": "n_home", "to": "n_drawer", "mode": "linear", "speed": 20},
        ],
    }


def _arm_poses():
    return {
        "robot_home":     [180.0, -45.0, 0.0, 45.0, 90.0],
        "uplc_draw_home": [176.5, 8.1, -70.1, 62.1, 90.0],
    }


def _rail_positions():
    return {"Home": 0.0, "Local_1": 62.0, "Local_2": 237.0}


# ── Pure function: find_nearest_node ────────────────────────────────


def test_exact_match_returns_within_tolerance():
    graph = MotionGraph.from_dict(_graph_dict(), preconditions=DEFAULT_PRECONDITIONS)
    match = find_nearest_node(
        graph,
        current_joints=[180.0, -45.0, 0.0, 45.0, 90.0],
        current_rail_mm=0.0,
        current_gripper_state="open",
        declared_payload="empty",
        arm_pose_joints=_arm_poses(),
        rail_position_mm=_rail_positions(),
    )
    assert match.node_id == "n_home"
    assert match.arm_residual == pytest.approx(0.0)
    assert match.rail_residual == pytest.approx(0.0)
    assert match.within_tolerance is True


def test_near_match_within_tolerance_suggests_node():
    """Joint angles 2deg off should still match within the 10deg default."""
    graph = MotionGraph.from_dict(_graph_dict(), preconditions=DEFAULT_PRECONDITIONS)
    match = find_nearest_node(
        graph,
        current_joints=[182.0, -45.0, 0.0, 45.0, 90.0],  # +2 on J1
        current_rail_mm=0.0,
        current_gripper_state="open",
        declared_payload="empty",
        arm_pose_joints=_arm_poses(),
        rail_position_mm=_rail_positions(),
    )
    assert match.node_id == "n_home"
    assert match.arm_residual == pytest.approx(2.0)
    assert match.within_tolerance is True


def test_far_from_anywhere_returns_node_but_not_within_tolerance():
    """A pose that's >>10deg off the nearest still gets reported as the
    best match — but ``within_tolerance`` is False so callers know not
    to snap."""
    graph = MotionGraph.from_dict(_graph_dict(), preconditions=DEFAULT_PRECONDITIONS)
    match = find_nearest_node(
        graph,
        current_joints=[0.0, 0.0, 0.0, 0.0, 0.0],
        current_rail_mm=0.0,
        current_gripper_state="open",
        declared_payload="empty",
        arm_pose_joints=_arm_poses(),
        rail_position_mm=_rail_positions(),
    )
    # robot_home at J1=180 vs current J1=0 is 180deg.
    assert match.node_id == "n_home"
    assert match.arm_residual > 100
    assert match.within_tolerance is False


def test_rail_mismatch_disqualifies_node():
    """Even if joints exactly match robot_home, if the rail is at Local_1
    the only valid candidate is n_local1, not n_home."""
    graph = MotionGraph.from_dict(_graph_dict(), preconditions=DEFAULT_PRECONDITIONS)
    match = find_nearest_node(
        graph,
        current_joints=[180.0, -45.0, 0.0, 45.0, 90.0],
        current_rail_mm=62.0,   # Local_1
        current_gripper_state="open",
        declared_payload="empty",
        arm_pose_joints=_arm_poses(),
        rail_position_mm=_rail_positions(),
    )
    assert match.node_id == "n_local1"
    assert match.rail_residual == pytest.approx(0.0)


def test_gripper_mismatch_returns_no_match():
    """No node has gripper=closed in our test graph; should return None."""
    graph = MotionGraph.from_dict(_graph_dict(), preconditions=DEFAULT_PRECONDITIONS)
    match = find_nearest_node(
        graph,
        current_joints=[180.0, -45.0, 0.0, 45.0, 90.0],
        current_rail_mm=0.0,
        current_gripper_state="closed",  # no node has this
        declared_payload="empty",
        arm_pose_joints=_arm_poses(),
        rail_position_mm=_rail_positions(),
    )
    assert match.node_id is None


def test_missing_current_state_returns_empty():
    graph = MotionGraph.from_dict(_graph_dict(), preconditions=DEFAULT_PRECONDITIONS)
    match = find_nearest_node(
        graph,
        current_joints=None,
        current_rail_mm=0.0,
        current_gripper_state="open",
        declared_payload="empty",
        arm_pose_joints=_arm_poses(),
        rail_position_mm=_rail_positions(),
    )
    assert match.node_id is None


def test_angle_wrap_treats_180_and_minus180_as_equal():
    """J1=180 and J1=-180 are physically the same orientation; the
    modular distance should report 0deg, not 360."""
    graph = MotionGraph.from_dict(_graph_dict(), preconditions=DEFAULT_PRECONDITIONS)
    match = find_nearest_node(
        graph,
        current_joints=[-180.0, -45.0, 0.0, 45.0, 90.0],
        current_rail_mm=0.0,
        current_gripper_state="open",
        declared_payload="empty",
        arm_pose_joints=_arm_poses(),
        rail_position_mm=_rail_positions(),
    )
    assert match.node_id == "n_home"
    assert match.arm_residual == pytest.approx(0.0)


# ── Controller: suggest_current_node + recover_to ────────────────────


@pytest.fixture
def graph_controller(initialized_controller):
    """Controller with motion graph, mocked position/track configs aligned
    with the test graph node references."""
    c = initialized_controller
    c.motion_graph = MotionGraph.from_dict(_graph_dict(), preconditions=DEFAULT_PRECONDITIONS)
    c.graph_mode = GraphMode.ADVISORY
    c.position_config = {"positions": _arm_poses()}
    c.track_config = {"locations": _rail_positions()}
    c.last_gripper_position = 150  # matches gripper_states.open.stroke
    c.last_joints = [180.0, -45.0, 0.0, 45.0, 90.0]
    c.last_track_position = 0.0
    return c


def test_suggest_current_node_returns_node_when_at_pose(graph_controller):
    match = graph_controller.suggest_current_node()
    assert match.node_id == "n_home"
    assert match.within_tolerance is True


def test_suggest_current_node_no_graph_returns_empty(initialized_controller):
    c = initialized_controller
    c.motion_graph = None
    match = c.suggest_current_node()
    assert match.node_id is None


def test_recover_to_succeeds_when_pose_matches(graph_controller):
    # Start off-grid (raw move cleared the tracker).
    graph_controller.last_arm_pose_name = None
    assert graph_controller.current_node is None

    result = graph_controller.recover_to("n_home")
    assert result["current_node"] == "n_home"
    assert graph_controller.last_arm_pose_name == "robot_home"
    assert graph_controller.last_rail_location_name == "Home"


def test_recover_to_raises_mismatch_when_pose_disagrees(graph_controller):
    """Controller is at robot_home but operator declares n_drawer —
    must refuse without force."""
    with pytest.raises(RecoveryMismatch) as info:
        graph_controller.recover_to("n_drawer")
    assert info.value.requested == "n_drawer"
    # Suggestion should be n_home (where we actually are).
    assert info.value.suggested == "n_home"


def test_recover_to_with_force_bypasses_check(graph_controller):
    """force=True trusts the operator's declaration."""
    result = graph_controller.recover_to("n_drawer", force=True)
    assert result["current_node"] == "n_drawer"


def test_recover_to_unknown_node_raises(graph_controller):
    with pytest.raises(UnknownNodeError):
        graph_controller.recover_to("ghost_node")


# ── HTTP: /graph/nearest + /control/graph/recover_to ────────────────


@pytest.fixture
def api_mock_controller():
    """Mock for the FastAPI test client. recover_to delegates to the
    real method on a separate real-ish controller pair so we exercise
    the API plumbing without dragging the full controller fixture."""
    mc = MagicMock()
    mc.motion_graph = MotionGraph.from_dict(_graph_dict(), preconditions=DEFAULT_PRECONDITIONS)
    mc.graph_mode = GraphMode.ADVISORY
    mc.current_node = None
    mc.declared_payload = "empty"
    mc.last_arm_pose_name = None
    mc.last_rail_location_name = None
    mc.last_transition = None
    mc.last_joints = [180.0, -45.0, 0.0, 45.0, 90.0]
    mc.last_track_position = 0.0
    mc.last_gripper_position = 150
    mc.reachable_node_ids.return_value = []
    mc.is_connected.return_value = True
    mc.is_alive = True
    mc.host = "127.0.0.1"
    mc.xarm_config = {"port": 18333}

    # Have suggest_current_node return an n_home match (matching the
    # pose we set in last_joints/last_track_position above).
    mc.suggest_current_node.return_value = NodeMatch(
        node_id="n_home",
        arm_residual=0.0,
        rail_residual=0.0,
        gripper_match=True,
        payload_match=True,
        within_tolerance=True,
    )

    def _recover(node_id, force=False):
        if node_id not in {"n_home", "n_drawer", "n_local1"}:
            raise UnknownNodeError(node_id)
        if not force:
            suggestion = mc.suggest_current_node.return_value
            if suggestion.node_id != node_id:
                raise RecoveryMismatch(
                    requested=node_id, suggested=suggestion.node_id,
                    arm_residual=suggestion.arm_residual,
                    rail_residual=suggestion.rail_residual,
                )
        return {"recovered_to": node_id, "current_node": node_id}
    mc.recover_to.side_effect = _recover

    return mc


@pytest.fixture
def client(monkeypatch, api_mock_controller):
    from src.core.xarm_api_server import app
    monkeypatch.setattr("src.core.xarm_api_server.controller", api_mock_controller)
    with TestClient(app) as c:
        yield c


def test_get_nearest_returns_suggestion(client):
    resp = client.get("/graph/nearest")
    assert resp.status_code == 200
    body = resp.json()
    assert body["suggested_node"] == "n_home"
    assert body["within_tolerance"] is True


def test_get_nearest_returns_404_when_no_graph(client, api_mock_controller):
    api_mock_controller.motion_graph = None
    resp = client.get("/graph/nearest")
    assert resp.status_code == 404


def test_recover_to_accepts_matching_node(client):
    resp = client.post("/control/graph/recover_to", json={"node_id": "n_home"})
    assert resp.status_code == 200
    assert resp.json()["recovered_to"] == "n_home"


def test_recover_to_rejects_mismatch_without_force(client):
    resp = client.post("/control/graph/recover_to", json={"node_id": "n_drawer"})
    assert resp.status_code == 422
    detail = resp.json()["detail"]
    assert detail["error"] == "recovery_mismatch"
    assert detail["requested"] == "n_drawer"
    assert detail["suggested"] == "n_home"


def test_recover_to_force_bypasses_check(client):
    resp = client.post(
        "/control/graph/recover_to",
        json={"node_id": "n_drawer", "force": True},
    )
    assert resp.status_code == 200


def test_recover_to_unknown_node_returns_409(client):
    resp = client.post("/control/graph/recover_to", json={"node_id": "ghost"})
    assert resp.status_code == 409
