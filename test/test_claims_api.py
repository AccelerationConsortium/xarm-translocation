"""Phase 3 API + status_builder tests.

Covers:
- /control/claim returns ClaimResponse on success, 409 + ClaimRejection
  on conflict, idempotent for same session_id
- /control/heartbeat returns 204 / 401
- /control/release returns 204 always (idempotent)
- /status surfaces details.claimed_by when a claim is held
- /status.allowed_actions is populated by status_builder based on
  equipment_status, has_error, and graph_mode (STRICT only adds
  graph-derived moves)
"""

from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock

import pytest
from fastapi.testclient import TestClient

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Match the api_server's effective import path.
from src.core.claims import ClaimManager
from src.core.motion_graph import (
    DEFAULT_PRECONDITIONS, GraphMode, MotionGraph,
)
from src.core.status_builder import build_status


def _graph_dict():
    return {
        "schema_version": "0.2",
        "gripper_states": {"empty": {"stroke": 150, "intent": "none"}},
        "nodes": [
            {"id": "n_home",   "arm": "home",   "rail": "Home"},
            {"id": "n_pickup", "arm": "pickup", "rail": "Home"},
        ],
        "edges": [{"from": "n_home", "to": "n_pickup", "mode": "linear", "speed": 25}],
    }


@pytest.fixture
def mock_controller():
    """Controller mock with a real ClaimManager and motion graph attached."""
    mc = MagicMock()
    mc.is_simulated = False
    mc.claim_manager = ClaimManager(default_ttl_s=30.0)
    mc.motion_graph = MotionGraph.from_dict(_graph_dict(), preconditions=DEFAULT_PRECONDITIONS)
    mc.graph_mode = GraphMode.STRICT
    mc.current_node = "n_home"
    mc.current_gripper_state = "empty"
    mc.last_arm_pose_name = "home"
    mc.last_rail_location_name = "Home"
    mc.last_transition = None
    mc.reachable_node_ids.return_value = ["n_pickup"]
    mc.allowed_gripper_targets.return_value = []
    mc.is_connected.return_value = True
    mc.is_alive = True
    mc.host = "127.0.0.1"
    mc.xarm_config = {"port": 18333}
    return mc


@pytest.fixture
def client(monkeypatch, mock_controller):
    from src.core.xarm_api_server import app
    monkeypatch.setattr("src.core.xarm_api_server.controller", mock_controller)
    with TestClient(app) as c:
        yield c


# ── /control/claim ───────────────────────────────────────────────────


def test_acquire_returns_claim_response(client):
    resp = client.post(
        "/control/claim",
        json={"owner": "alice", "session_id": "s1", "ttl_s": 30.0},
    )
    assert resp.status_code == 200
    body = resp.json()
    assert "claim_token" in body
    assert body["heartbeat_interval_s"] > 0
    assert "expires_at" in body


def test_acquire_conflict_returns_409(client, mock_controller):
    # First session takes the claim
    client.post(
        "/control/claim",
        json={"owner": "alice", "session_id": "s1"},
    )
    # Second session tries to take it
    resp = client.post(
        "/control/claim",
        json={"owner": "bob", "session_id": "s2"},
    )
    assert resp.status_code == 409
    body = resp.json()
    assert body["claimed_by"]["session_id"] == "s1"
    assert body["claimed_by"]["owner"] == "alice"
    assert body["retry_after_s"] is not None
    # Retry-After header per spec
    assert "Retry-After" in resp.headers


def test_acquire_same_session_is_idempotent(client):
    r1 = client.post("/control/claim", json={"owner": "a", "session_id": "s1"})
    r2 = client.post("/control/claim", json={"owner": "a", "session_id": "s1"})
    assert r1.status_code == 200
    assert r2.status_code == 200
    # Token rotates (we chose to rotate on re-acquire).
    assert r1.json()["claim_token"] != r2.json()["claim_token"]


# ── /control/heartbeat ───────────────────────────────────────────────


def test_heartbeat_with_valid_token_returns_204(client):
    acquire = client.post(
        "/control/claim", json={"owner": "a", "session_id": "s1"},
    )
    token = acquire.json()["claim_token"]
    resp = client.post("/control/heartbeat", headers={"X-Claim-Token": token})
    assert resp.status_code == 204


def test_heartbeat_with_wrong_token_returns_401(client):
    client.post("/control/claim", json={"owner": "a", "session_id": "s1"})
    resp = client.post("/control/heartbeat", headers={"X-Claim-Token": "garbage"})
    assert resp.status_code == 401


def test_heartbeat_missing_header_returns_422(client):
    # FastAPI surfaces missing required header as 422 (validation error)
    resp = client.post("/control/heartbeat")
    assert resp.status_code == 422


# ── /control/release ─────────────────────────────────────────────────


def test_release_with_valid_token_returns_204(client, mock_controller):
    acquire = client.post(
        "/control/claim", json={"owner": "a", "session_id": "s1"},
    )
    token = acquire.json()["claim_token"]
    resp = client.post("/control/release", headers={"X-Claim-Token": token})
    assert resp.status_code == 204
    # Claim is gone.
    assert mock_controller.claim_manager.claimed_by() is None


def test_release_is_idempotent_on_unknown_token(client):
    """Per spec: 'releasing an unknown / already-released token also
    returns 204'."""
    resp = client.post("/control/release", headers={"X-Claim-Token": "ghost"})
    assert resp.status_code == 204


# ── /status surfaces claimed_by ──────────────────────────────────────


def test_status_details_carries_claimed_by_when_claim_active(client):
    client.post("/control/claim", json={"owner": "alice", "session_id": "s1"})
    resp = client.get("/status")
    assert resp.status_code == 200
    body = resp.json()
    claimed = body["details"]["claimed_by"]
    assert claimed is not None
    assert claimed["owner"] == "alice"
    assert claimed["session_id"] == "s1"


def test_status_details_claimed_by_is_null_when_no_claim(client):
    """Spec example 9.3 shows ``claimed_by: null`` rather than a
    missing field. We preserve that convention so v1.1 readers always
    see the field."""
    resp = client.get("/status")
    body = resp.json()
    assert "claimed_by" in body["details"]
    assert body["details"]["claimed_by"] is None


# ── allowed_actions (status_builder logic, not via HTTP) ────────────


def test_allowed_actions_in_strict_mode_includes_move_targets(mock_controller):
    """Direct status_builder test — bypasses HTTP. STRICT mode emits
    'move.<node_id>' entries from reachable_node_ids()."""
    # Make the controller look 'ready' to the builder.
    from src.core.xarm_controller import ComponentState
    mock_controller.states = {
        "arm": ComponentState.ENABLED,
        "gripper": ComponentState.DISABLED,
        "track": ComponentState.DISABLED,
        "force_torque": ComponentState.DISABLED,
        "connection": ComponentState.ENABLED,
    }
    mock_controller.alive = True
    mock_controller.last_error_code = 0
    mock_controller.last_error = None
    mock_controller._motion_in_progress = False
    mock_controller.has_gripper.return_value = False
    mock_controller.has_track.return_value = False
    mock_controller.has_force_torque_sensor.return_value = False
    mock_controller.last_position = [0, 0, 0, 0, 0, 0]
    mock_controller.last_joints = [0] * 6
    mock_controller.last_track_position = 0

    env = build_status(mock_controller)
    assert env.equipment_status == "ready"
    assert "stop" in env.allowed_actions
    assert "move.n_pickup" in env.allowed_actions


def test_allowed_actions_in_advisory_mode_omits_graph_moves(mock_controller):
    """ADVISORY/OFF modes don't constrain moves, so allowed_actions
    deliberately doesn't list move.<node_id> entries."""
    mock_controller.graph_mode = GraphMode.ADVISORY
    from src.core.xarm_controller import ComponentState
    mock_controller.states = {
        "arm": ComponentState.ENABLED, "gripper": ComponentState.DISABLED,
        "track": ComponentState.DISABLED, "force_torque": ComponentState.DISABLED,
        "connection": ComponentState.ENABLED,
    }
    mock_controller.alive = True
    mock_controller.last_error_code = 0
    mock_controller.last_error = None
    mock_controller._motion_in_progress = False
    mock_controller.has_gripper.return_value = False
    mock_controller.has_track.return_value = False
    mock_controller.has_force_torque_sensor.return_value = False
    mock_controller.last_position = [0, 0, 0, 0, 0, 0]
    mock_controller.last_joints = [0] * 6
    mock_controller.last_track_position = 0

    env = build_status(mock_controller)
    assert "stop" in env.allowed_actions
    assert not any(a.startswith("move.") for a in env.allowed_actions)


def test_allowed_actions_in_error_state_is_recovery_only(mock_controller):
    from src.core.xarm_controller import ComponentState
    mock_controller.states = {
        "arm": ComponentState.ENABLED, "gripper": ComponentState.DISABLED,
        "track": ComponentState.DISABLED, "force_torque": ComponentState.DISABLED,
        "connection": ComponentState.ENABLED,
    }
    mock_controller.alive = True
    mock_controller.last_error_code = 42
    mock_controller.last_error = "something bad"
    mock_controller._motion_in_progress = False
    mock_controller.has_gripper.return_value = False
    mock_controller.has_track.return_value = False
    mock_controller.has_force_torque_sensor.return_value = False
    mock_controller.last_position = [0, 0, 0, 0, 0, 0]
    mock_controller.last_joints = [0] * 6
    mock_controller.last_track_position = 0

    env = build_status(mock_controller)
    assert env.equipment_status == "error"
    assert set(env.allowed_actions) == {"clear_errors", "stop"}


# ── PROTOCOL_VERSION bump ────────────────────────────────────────────


def test_probe_reports_protocol_version_1_2(client):
    resp = client.get("/")
    assert resp.status_code == 200
    assert resp.json()["protocol_version"] == "1.2"


def test_status_reports_protocol_version_1_2(client):
    resp = client.get("/status")
    assert resp.status_code == 200
    assert resp.json()["protocol_version"] == "1.2"
