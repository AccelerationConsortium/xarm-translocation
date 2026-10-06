"""The pose-authoring endpoint, ``POST /control/graph/pose``.

It writes a named pose into joint_config.yaml. Until it existed,
``POST /control/graph/node`` could only reference poses a human had
hand-edited into that file, so no agent could grow the graph.

(The node-anchored ``/control/freehand/nudge`` that used to share this file
was removed; one test pins that it stays gone.)
"""

from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock

import pytest
from fastapi.testclient import TestClient

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core.claims import ClaimManager
from src.core.motion_graph import DEFAULT_PRECONDITIONS, GraphMode, MotionGraph


def _graph_dict():
    return {
        "schema_version": "0.2",
        "gripper_states": {"empty": {"stroke": 150, "intent": "none"}},
        "nodes": [
            {"id": "n_home", "arm": "home", "rail": "Home"},
            {"id": "n_pickup", "arm": "pickup", "rail": "Home"},
        ],
        "edges": [{"from": "n_home", "to": "n_pickup", "mode": "linear", "speed": 25}],
    }


@pytest.fixture
def mock_controller():
    mc = MagicMock()
    mc.is_simulated = False
    mc.is_real_box_simulating = False
    mc.claim_manager = ClaimManager(default_ttl_s=30.0)
    mc.motion_graph = MotionGraph.from_dict(_graph_dict(), preconditions=DEFAULT_PRECONDITIONS)
    mc.graph_mode = GraphMode.STRICT
    mc.current_node = "n_home"
    mc.last_arm_pose_name = "home"
    mc.last_rail_location_name = "Home"
    mc.freehand_offset = None
    mc._motion_in_progress = False
    mc.sash_interlock = None          # interlock unconfigured in this fixture
    mc.move_relative.return_value = True
    mc.num_joints = 5
    mc.position_config = {"positions": {"home": [1.0, 2.0, 3.0, 4.0, 5.0]}}
    mc._validate_joint_angles.return_value = True
    return mc


@pytest.fixture(autouse=True)
def isolated_pose_file(tmp_path, monkeypatch):
    """Point the pose writer at a tmp copy. Autouse and unconditional: an
    earlier revision of these tests wrote a junk pose into the repo's real
    joint_config.yaml, which is the live cell's configuration."""
    f = tmp_path / "joint_config.yaml"
    f.write_text("positions:\n  home: [1.0, 2.0, 3.0, 4.0, 5.0]\n")
    monkeypatch.setenv("XARM_JOINT_CONFIG_PATH", str(f))
    return f


@pytest.fixture
def enforcing_controller(mock_controller):
    """Claims hard-enforced, as on the deployed device (tokenless -> 423).
    ClaimManager defaults to enforce=False, so this must be explicit."""
    mock_controller.claim_manager = ClaimManager(default_ttl_s=30.0, enforce=True)
    return mock_controller


@pytest.fixture
def client(monkeypatch, mock_controller):
    from src.core.xarm_api_server import app
    monkeypatch.setattr("src.core.xarm_api_server.controller", mock_controller)
    with TestClient(app) as c:
        yield c


@pytest.fixture
def claim_headers(client):
    r = client.post("/control/claim", json={"owner": "t", "session_id": "s1"})
    return {"X-Claim-Token": r.json()["claim_token"]}


# ── nudge: legal in STRICT, unlike every other freehand route ────────


def test_nudge_route_is_gone(client, claim_headers):
    """Nudge was removed (freehand moves cover it); the route must not linger."""
    resp = client.post("/control/freehand/nudge", headers=claim_headers, json={"dz": 1})
    assert resp.status_code in (404, 405)


def test_pose_save_refuses_to_clobber_without_overwrite(client, claim_headers):
    r = client.post(
        "/control/graph/pose",
        json={"name": "home", "angles": [0, 0, 0, 0, 0]},
        headers=claim_headers,
    )
    assert r.status_code == 409
    assert r.json()["detail"]["error"] == "pose_exists"
    assert r.json()["detail"]["current"] == [1.0, 2.0, 3.0, 4.0, 5.0]


def test_pose_save_rejects_wrong_joint_count(client, claim_headers):
    r = client.post(
        "/control/graph/pose",
        json={"name": "newpose", "angles": [0, 0, 0]},
        headers=claim_headers,
    )
    assert r.status_code == 422
    assert r.json()["detail"]["error"] == "wrong_joint_count"


def test_pose_save_rejects_out_of_limit_angles(client, claim_headers, mock_controller):
    """A pose outside joint limits could never be moved to, so writing it
    would create a node that is unreachable by construction."""
    mock_controller._validate_joint_angles.return_value = False

    r = client.post(
        "/control/graph/pose",
        json={"name": "newpose", "angles": [999, 0, 0, 0, 0]},
        headers=claim_headers,
    )

    assert r.status_code == 422
    assert r.json()["detail"]["error"] == "joint_limits"


def test_pose_save_requires_a_claim(monkeypatch, enforcing_controller):
    from src.core.xarm_api_server import app
    monkeypatch.setattr("src.core.xarm_api_server.controller", enforcing_controller)
    with TestClient(app) as c:
        r = c.post("/control/graph/pose", json={"name": "x", "angles": [0, 0, 0, 0, 0]})
    assert r.status_code == 423


def test_pose_save_writes_and_hot_reloads(client, claim_headers, mock_controller,
                                          isolated_pose_file):
    """The happy path: the pose lands in the file AND in position_config, so
    POST /control/graph/node can reference it without a service restart."""
    r = client.post(
        "/control/graph/pose",
        json={"name": "deck_slot3_high", "angles": [1.234, 2.0, 3.0, 4.0, 5.0],
              "comment": "taught by nudge"},
        headers=claim_headers,
    )

    assert r.status_code == 200, r.text
    assert r.json()["saved"]["angles"] == [1.23, 2.0, 3.0, 4.0, 5.0]
    assert r.json()["replaced"] is None

    text = isolated_pose_file.read_text()
    assert "deck_slot3_high: [1.23, 2.0, 3.0, 4.0, 5.0]  # taught by nudge" in text
    # hot-reloaded, no restart needed
    assert mock_controller.position_config["positions"]["deck_slot3_high"] == [
        1.23, 2.0, 3.0, 4.0, 5.0
    ]


def test_pose_overwrite_replaces_in_place_and_reports_the_old_value(
    client, claim_headers, isolated_pose_file
):
    """Recalibration. The old value comes back so a bad calibration is
    revertible, and the edit stays on one line so git shows what moved."""
    r = client.post(
        "/control/graph/pose",
        json={"name": "home", "angles": [9.0, 8.0, 7.0, 6.0, 5.0], "overwrite": True},
        headers=claim_headers,
    )

    assert r.status_code == 200, r.text
    assert r.json()["replaced"] == [1.0, 2.0, 3.0, 4.0, 5.0]
    text = isolated_pose_file.read_text()
    assert "home: [9.0, 8.0, 7.0, 6.0, 5.0]" in text
    assert "[1.0, 2.0, 3.0, 4.0, 5.0]" not in text
    assert len([l for l in text.splitlines() if l.strip().startswith("home:")]) == 1


def test_pose_captures_current_joints_when_angles_omitted(
    client, claim_headers, mock_controller
):
    """Teach-by-demonstration: jog there, save it, no numbers typed."""
    mock_controller.arm.connected = True
    mock_controller.get_current_joints.return_value = [11.0, 22.0, 33.0, 44.0, 55.0]

    r = client.post(
        "/control/graph/pose", json={"name": "taught"}, headers=claim_headers
    )

    assert r.status_code == 200, r.text
    assert r.json()["captured_from_arm"] is True
    assert r.json()["saved"]["angles"] == [11.0, 22.0, 33.0, 44.0, 55.0]
