"""STRICT-mode gating of the legacy freehand control surface.

The hybrid enforcement model:

- Freehand endpoints that bypass the motion graph entirely — raw
  Cartesian/joint/relative moves, velocity streaming, raw track position,
  gripper stroke, gripper force — are refused wholesale with
  409 ``graph_mode_strict`` while ``graph_mode == STRICT``. They keep
  their legacy behavior in ADVISORY/OFF (the deliberate escape hatch).
- ``/gripper/open`` and ``/gripper/close`` stay usable in STRICT, but are
  routed through the graph-sanctioned ``set_gripper_state`` so they
  inherit the parked-at-node / arm-not-moving / whitelist interlocks.
  Open maps to the ``empty`` catalog state; close resolves the gripper's
  configured close stroke against the catalog (409 when nothing matches).
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
    DEFAULT_PRECONDITIONS, GraphMode, GripperTransitionError, MotionGraph,
)


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
    mc = MagicMock()
    mc.is_simulated = False
    # Real box, not UFACTORY Studio's Sim mode. Must be pinned: unset
    # MagicMock attributes are truthy, and box_sim_guard would then 412
    # every motion and gripper call in this fixture.
    mc.is_real_box_simulating = False
    mc.claim_manager = ClaimManager(default_ttl_s=30.0)
    mc.motion_graph = MotionGraph.from_dict(_graph_dict(), preconditions=DEFAULT_PRECONDITIONS)
    mc.graph_mode = GraphMode.STRICT
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


FREEHAND_CALLS = [
    ("/move/position", {"x": 100, "y": 0, "z": 200}, "move.position"),
    ("/move/joints", {"angles": [0, 0, 0, 0, 0]}, "move.joints"),
    ("/move/relative", {"dx": 5}, "move.relative"),
    ("/move/plate_linear", {"target_location": "pickup"}, "move.plate_linear"),
    ("/velocity/cartesian", {"vx": 10}, "velocity.cartesian"),
    ("/track/move", {"position": 100}, "track.move"),
    ("/gripper/move/stroke", {"stroke": 42}, "gripper.move_stroke"),
    ("/gripper/force", {"force": 30}, "gripper.force"),
]


# ── STRICT refuses every freehand endpoint ───────────────────────────


@pytest.mark.parametrize("path,body,action", FREEHAND_CALLS)
def test_strict_refuses_freehand(client, claim_headers, path, body, action):
    resp = client.post(path, json=body, headers=claim_headers)
    assert resp.status_code == 409, resp.text
    detail = resp.json()["detail"]
    assert detail["error"] == "graph_mode_strict"
    assert detail["action"] == action


@pytest.mark.parametrize("path,body,action", FREEHAND_CALLS)
def test_advisory_allows_freehand(client, claim_headers, mock_controller, path, body, action):
    mock_controller.graph_mode = GraphMode.ADVISORY
    resp = client.post(path, json=body, headers=claim_headers)
    assert resp.status_code == 200, resp.text


def test_strict_freehand_refusal_does_not_hold_motion_slot(client, claim_headers, mock_controller):
    """The guard runs before reserve_motion, so a refusal must not leave
    the motion slot latched (enter_motion never called)."""
    client.post("/move/position", json={"x": 1, "y": 2, "z": 3}, headers=claim_headers)
    mock_controller.enter_motion.assert_not_called()


# ── STRICT routes open/close through the graph transition ───────────


def test_strict_open_routes_through_set_gripper_state(client, claim_headers, mock_controller):
    mock_controller.set_gripper_state.return_value = True
    resp = client.post("/gripper/open", headers=claim_headers)
    assert resp.status_code == 200, resp.text
    mock_controller.set_gripper_state.assert_called_once_with("empty")
    mock_controller.open_gripper.assert_not_called()
    assert "graph state 'empty'" in resp.json()["message"]


def test_strict_close_resolves_stroke_to_catalog_state(client, claim_headers, mock_controller):
    mock_controller.default_close_stroke.return_value = 100.0  # matches 'closed'
    mock_controller.set_gripper_state.return_value = True
    resp = client.post("/gripper/close", headers=claim_headers)
    assert resp.status_code == 200, resp.text
    mock_controller.set_gripper_state.assert_called_once_with("closed")
    mock_controller.close_gripper.assert_not_called()


def test_strict_close_with_off_catalog_stroke_refused(client, claim_headers, mock_controller):
    mock_controller.default_close_stroke.return_value = 120.0  # matches nothing
    resp = client.post("/gripper/close", headers=claim_headers)
    assert resp.status_code == 409
    detail = resp.json()["detail"]
    assert detail["error"] == "graph_mode_strict"
    assert detail["action"] == "gripper.close"
    mock_controller.set_gripper_state.assert_not_called()


def test_strict_open_transition_violation_is_409(client, claim_headers, mock_controller):
    mock_controller.set_gripper_state.side_effect = GripperTransitionError(
        "n_home", "empty", "empty", "arm is moving",
    )
    resp = client.post("/gripper/open", headers=claim_headers)
    assert resp.status_code == 409
    assert resp.json()["detail"]["error"] == "gripper_transition_not_allowed"


def test_advisory_open_uses_legacy_path(client, claim_headers, mock_controller):
    mock_controller.graph_mode = GraphMode.ADVISORY
    mock_controller.open_gripper.return_value = True
    resp = client.post("/gripper/open", headers=claim_headers)
    assert resp.status_code == 200
    mock_controller.open_gripper.assert_called_once()
    mock_controller.set_gripper_state.assert_not_called()


# ── Claim gate still runs first ──────────────────────────────────────


def test_tokenless_freehand_still_423_in_strict(client, mock_controller):
    """With hard claim enforcement on, the claim gate (a dependency)
    precedes the strict gate (endpoint body): no token → 423, not 409."""
    mock_controller.claim_manager.enable_enforcement()
    resp = client.post("/gripper/move/stroke", json={"stroke": 42})
    assert resp.status_code == 423
