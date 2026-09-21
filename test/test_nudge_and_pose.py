"""Node-anchored nudge, and the pose-authoring endpoint.

Two capabilities that together close the "grow the graph" loop:

- ``POST /control/freehand/nudge`` is the ONLY freehand route legal in
  STRICT. It earns that by staying inside a bounded envelope around the
  node the arm is pinned at, and by restoring the pin afterwards -- which
  is what lets the sash interlock gate it by node membership the way it
  gates a named move. A raw freehand move cannot be gated on entry because
  it has no target node; a nudge has one.
- ``POST /control/graph/pose`` writes a named pose into joint_config.yaml.
  Until it existed, ``POST /control/graph/node`` could only reference poses
  a human had hand-edited into that file, so no agent could grow the graph.
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


def test_nudge_is_allowed_in_strict(client, claim_headers, mock_controller):
    """The whole point: STRICT refuses the other eight freehand routes."""
    r = client.post("/control/freehand/nudge", json={"dz": 1.0}, headers=claim_headers)
    assert r.status_code == 200, r.text
    assert r.json()["anchor"] == "n_home"
    assert mock_controller.move_relative.called


def test_nudge_retains_the_pin(client, claim_headers, mock_controller):
    """move_relative clears last_arm_pose_name (every raw move does).
    The nudge must put it back, or the arm goes off-grid and the interlock
    loses the node it gates by -- which is the entire bargain."""
    def _clear(*a, **k):
        mock_controller.last_arm_pose_name = None
        return True
    mock_controller.move_relative.side_effect = _clear

    r = client.post("/control/freehand/nudge", json={"dx": 1.0}, headers=claim_headers)

    assert r.status_code == 200, r.text
    assert mock_controller.last_arm_pose_name == "home"


def test_nudge_refused_off_grid(client, claim_headers, mock_controller):
    """No anchor means no envelope origin AND no node for the sash
    interlock to gate by. Refuse rather than fall back to unbounded."""
    mock_controller.current_node = None

    r = client.post("/control/freehand/nudge", json={"dz": 1.0}, headers=claim_headers)

    assert r.status_code == 409
    assert r.json()["detail"]["error"] == "no_anchor_node"
    assert not mock_controller.move_relative.called


def test_nudge_rejects_an_oversized_single_step(client, claim_headers, mock_controller):
    r = client.post("/control/freehand/nudge", json={"dz": 50.0}, headers=claim_headers)
    assert r.status_code == 422
    assert r.json()["detail"]["error"] == "step_too_large"
    assert not mock_controller.move_relative.called


def test_nudge_bounds_the_CUMULATIVE_offset(client, claim_headers, mock_controller):
    """A sequence of individually-legal steps must not walk the arm out of
    the envelope. This is the failure a per-step cap alone would miss."""
    for _ in range(2):
        assert client.post(
            "/control/freehand/nudge", json={"dx": 1.5}, headers=claim_headers
        ).status_code == 200

    r = client.post("/control/freehand/nudge", json={"dx": 1.5}, headers=claim_headers)

    assert r.status_code == 422, r.text
    assert r.json()["detail"]["error"] == "offset_exceeded"
    assert r.json()["detail"]["current_offset_mm"] == [3.0, 0.0, 0.0]


def test_nudge_reports_remaining_envelope(client, claim_headers):
    r = client.post("/control/freehand/nudge", json={"dx": 1.0}, headers=claim_headers)
    body = r.json()
    assert body["offset_mm"] == [1.0, 0.0, 0.0]
    assert body["remaining_mm"][0] == pytest.approx(2.0)


def test_nudge_requires_a_claim(monkeypatch, enforcing_controller):
    from src.core.xarm_api_server import app
    monkeypatch.setattr("src.core.xarm_api_server.controller", enforcing_controller)
    with TestClient(app) as c:
        r = c.post("/control/freehand/nudge", json={"dz": 1.0})
    assert r.status_code == 423
    assert r.json()["detail"]["error"] == "claim_required"


# ── pose authoring ───────────────────────────────────────────────────


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
