"""Refusal of actuating commands while the real control box is in Sim mode.

UFACTORY Studio carries a Real/Sim toggle on the control box itself. In
Sim the SDK short-circuits the non-joint hardware to *success*: every
``set_linear_track_*`` call (an alias for ``set_linear_motor_*``, all
decorated ``@xarm_is_not_simulation_mode(ret=(0, []))``) and every BIO
gripper move return code 0 without moving anything. The device would then
cache that as truth — the commanded gripper stroke reads as "holding a
plate", the graph pins read as "parked at the destination" — and both
survive the flip back to Real, beside a real arm and a real plate.

So the box is refused. The Docker simulator is NOT: it reports the same
report bit (verified live against the container) but is the supported
dry-run path, and everything a workflow exercises — graph, interlocks,
claims, STRICT gating — lives in this service and behaves identically
there. That asymmetry is the point of these tests.
"""

from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock

import pytest
from fastapi.testclient import TestClient

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core.claims import ClaimManager
from src.core.motion_graph import (
    DEFAULT_PRECONDITIONS, GraphMode, MotionGraph,
)
from src.core.status_builder import build_status


def _graph_dict():
    return {
        "schema_version": "0.2",
        "gripper_states": {
            "empty": {"stroke": 150, "intent": "none"},
            "closed": {"stroke": 100, "intent": "position"},
        },
        "nodes": [
            {
                "id": "n_home", "arm": "home", "rail": "Home",
                "gripper_states": ["empty", "closed"],
                "gripper_transitions": [["empty", "closed"], ["closed", "empty"]],
            },
            {"id": "n_pickup", "arm": "pickup", "rail": "Home"},
        ],
        "edges": [{"from": "n_home", "to": "n_pickup", "mode": "linear", "speed": 25}],
    }


@pytest.fixture
def mock_controller():
    """A controller whose real box is in Studio-Sim."""
    mc = MagicMock()
    mc.is_simulated = True
    mc.is_docker_target = False
    mc.is_controller_simulating = True
    mc.is_real_box_simulating = True
    mc.claim_manager = ClaimManager(default_ttl_s=30.0)
    mc.motion_graph = MotionGraph.from_dict(
        _graph_dict(), preconditions=DEFAULT_PRECONDITIONS
    )
    mc.graph_mode = GraphMode.ADVISORY   # so STRICT is not what refuses
    mc.current_node = "n_home"
    mc.current_gripper_state = "empty"
    mc.last_arm_pose_name = "home"
    mc.last_rail_location_name = "Home"
    mc._motion_in_progress = False
    mc.is_connected.return_value = True
    mc.is_alive = True
    return mc


@pytest.fixture
def client(monkeypatch, mock_controller):
    from src.core.xarm_api_server import app
    monkeypatch.setattr("src.core.xarm_api_server.controller", mock_controller)
    with TestClient(app) as c:
        yield c


@pytest.fixture
def claim_headers(client):
    resp = client.post("/control/claim", json={"owner": "t", "session_id": "s1"})
    return {"X-Claim-Token": resp.json()["claim_token"]}


# Every actuating surface: motion goes through reserve_motion(), gripper
# endpoints guard themselves (they do not reserve the motion slot).
ACTUATING_CALLS = [
    ("/move/position", {"x": 100, "y": 0, "z": 200}),
    ("/move/joints", {"angles": [0, 0, 0, 0, 0]}),
    ("/move/relative", {"dx": 5}),
    ("/track/move", {"position": 100}),
    ("/gripper/open", None),
    ("/gripper/close", None),
    ("/gripper/move/stroke", {"stroke": 42}),
    ("/gripper/force", {"force": 30}),
]


@pytest.mark.parametrize("path,body", ACTUATING_CALLS)
def test_box_sim_refuses_actuating_calls(client, claim_headers, path, body):
    resp = client.post(path, json=body, headers=claim_headers)
    assert resp.status_code == 412, resp.text
    detail = resp.json()["detail"]
    assert detail["error"] == "controller_simulating"
    # Operator-driven recovery (flip the panel back), so no elapsed time
    # clears it -- spec §6.1 wants retry_after_s null in exactly this case.
    assert detail["retry_after_s"] is None


def test_graph_move_refused(client, claim_headers):
    resp = client.post(
        "/control/graph/move_to", json={"node_id": "n_pickup"}, headers=claim_headers
    )
    assert resp.status_code == 412, resp.text
    assert resp.json()["detail"]["error"] == "controller_simulating"


def test_stop_is_never_gated(client, claim_headers, mock_controller):
    """The safety floor: an abort must stay reachable in every mode."""
    mock_controller.stop_motion.return_value = True
    resp = client.post("/move/stop", headers=claim_headers)
    assert resp.status_code == 200, resp.text


# ── Docker must stay fully actuable (the dry-run path) ───────────────


@pytest.fixture
def docker_controller(mock_controller):
    """Same simulating bit, but it is the container, not the real box."""
    mock_controller.is_docker_target = True
    mock_controller.is_real_box_simulating = False
    return mock_controller


@pytest.mark.parametrize("path,body", ACTUATING_CALLS)
def test_docker_is_not_refused(monkeypatch, docker_controller, path, body):
    """The regression that would silently kill the dry-run path: the
    container reports is_simulation_robot True exactly like the box, so a
    guard keyed on the raw bit would refuse Docker too."""
    from src.core.xarm_api_server import app
    monkeypatch.setattr("src.core.xarm_api_server.controller", docker_controller)
    with TestClient(app) as c:
        headers = {
            "X-Claim-Token": c.post(
                "/control/claim", json={"owner": "t", "session_id": "s1"}
            ).json()["claim_token"]
        }
        resp = c.post(path, json=body, headers=headers)
        assert resp.status_code != 412, f"{path} refused the docker dry-run path"


# ── graph.record refuses BOTH simulators ─────────────────────────────


def test_graph_record_refused_in_box_sim(client, claim_headers):
    resp = client.post("/control/graph/record", json={}, headers=claim_headers)
    assert resp.status_code == 412, resp.text
    assert resp.json()["detail"]["error"] == "simulated_transition"


def test_graph_record_refused_in_docker_too(monkeypatch, docker_controller):
    """Unlike motion, recording is refused for the container as well: the
    edge would enter motion_graph.yaml -- the safety model -- validating no
    geometry, clearance or reach, and the write persists across restarts."""
    from src.core.xarm_api_server import app
    monkeypatch.setattr("src.core.xarm_api_server.controller", docker_controller)
    with TestClient(app) as c:
        headers = {
            "X-Claim-Token": c.post(
                "/control/claim", json={"owner": "t", "session_id": "s1"}
            ).json()["claim_token"]
        }
        resp = c.post("/control/graph/record", json={}, headers=headers)
        assert resp.status_code == 412, resp.text
        assert resp.json()["detail"]["error"] == "simulated_transition"


# ── §6.2: allowed_actions mirrors the refusal ────────────────────────


def _envelope_controller(**overrides):
    from src.core.xarm_controller import ComponentState

    mc = MagicMock()
    mc.alive = True
    mc.is_simulated = True
    mc.is_docker_target = False
    mc.is_controller_simulating = True
    mc.is_real_box_simulating = True
    mc._motion_in_progress = False
    mc.last_error_code = 0
    mc.last_error = None
    mc.states = {
        'connection': ComponentState.ENABLED,
        'arm': ComponentState.ENABLED,
        'gripper': ComponentState.ENABLED,
        'track': ComponentState.ENABLED,
        'force_torque': ComponentState.DISABLED,
    }
    mc.has_gripper.return_value = True
    mc.has_track.return_value = True
    mc.has_force_torque_sensor.return_value = False
    mc.last_position = [300, 0, 300, 180, 0, 0]
    mc.last_joints = [0, 0, 0, 0, 0]
    mc.last_track_position = 0.0
    mc.graph_mode = MagicMock(value='strict')
    mc.reachable_node_ids.return_value = ['n_pickup']
    for key, value in overrides.items():
        setattr(mc, key, value)
    return mc


def test_allowed_actions_withholds_moves_in_box_sim():
    """§6.2: if POSTing it would 412, /status must not advertise it."""
    envelope = build_status(_envelope_controller())
    assert envelope.equipment_status == 'dry_run'
    assert 'stop' in envelope.allowed_actions          # safety floor stays
    assert not [a for a in envelope.allowed_actions if a.startswith('move.')]


def test_allowed_actions_keeps_moves_for_docker():
    """The mirror must not over-withhold: Docker is actuable, so its move
    targets stay advertised or the dry-run path becomes undriveable."""
    envelope = build_status(
        _envelope_controller(is_docker_target=True, is_real_box_simulating=False)
    )
    assert envelope.equipment_status == 'dry_run'
    assert 'move.n_pickup' in envelope.allowed_actions


# ── Mid-flight flip clears the pins ──────────────────────────────────


def test_pin_arm_pose_clears_when_box_flipped_mid_move():
    """box_sim_guard refuses before a move starts, so the only way a false
    pin can be written is a flip while a move was already in flight. Then
    neither coordinate is trustworthy -- same response as stop_motion()."""
    from src.core.xarm_controller import XArmController

    c = XArmController.__new__(XArmController)
    c.profile_name = 'robot'
    c.arm = MagicMock(connected=True, is_simulation_robot=True)
    c.last_arm_pose_name = 'home'
    c.last_rail_location_name = 'Home'

    assert c._pin_arm_pose('pickup') is False
    assert c.last_arm_pose_name is None
    assert c.last_rail_location_name is None


def test_pin_arm_pose_pins_normally_on_real_hardware():
    from src.core.xarm_controller import XArmController

    c = XArmController.__new__(XArmController)
    c.profile_name = 'robot'
    c.arm = MagicMock(connected=True, is_simulation_robot=False)
    c.last_arm_pose_name = 'home'
    c.last_rail_location_name = 'Home'

    assert c._pin_arm_pose('pickup') is True
    assert c.last_arm_pose_name == 'pickup'
    assert c.last_rail_location_name == 'Home'
