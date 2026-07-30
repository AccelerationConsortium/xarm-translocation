"""Phase 2 API-level tests for the motion-graph endpoints.

Covers:
- GET /graph returns the snapshot when a graph is loaded; 404 when not
- POST /control/graph/mode validates input and switches modes
- /move/location returns HTTP 409 when STRICT-mode refuses (raises
  EdgeNotAllowedError) and HTTP 200 when ADVISORY/OFF allows
- POST /control/graph/record appends to YAML, reloads, and surfaces
  validation errors as HTTP 400
"""

from __future__ import annotations

import os
import shutil
import sys
import tempfile
from unittest.mock import MagicMock

import pytest
from fastapi.testclient import TestClient

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Test loads `from src.core.xarm_api_server`, which resolves relative
# imports inside api_server.py as `src.core.motion_graph`. Match that
# path so EdgeNotAllowedError raised in the mock and caught in the
# handler is the same class object. (See dual-import note in
# controller's motion_graph import.)
from src.core.motion_graph import (
    DEFAULT_PRECONDITIONS,
    EdgeNotAllowedError,
    GraphMode,
    GripperTransitionError,
    MotionGraph,
    NoPathError,
)


def _test_graph_dict():
    return {
        "schema_version": "0.2",
        "gripper_states": {
            "empty":    {"stroke": 150, "intent": "none"},
            "grip_120": {"stroke": 120, "intent": "grasp"},
        },
        "nodes": [
            {"id": "n_home",   "arm": "home",   "rail": "Home",
             "gripper_states": ["empty", "grip_120"],
             "gripper_transitions": [["empty", "grip_120"], ["grip_120", "empty"]]},
            {"id": "n_pickup", "arm": "pickup", "rail": "Home",
             "gripper_states": ["empty", "grip_120"]},
        ],
        "edges": [
            {"from": "n_home", "to": "n_pickup", "mode": "linear", "speed": 25},
        ],
    }


@pytest.fixture
def mock_controller_with_graph():
    """A MagicMock that mirrors the parts of XArmController the graph
    endpoints touch. Has a real MotionGraph attached so the snapshot
    endpoint returns plausible data."""
    mc = MagicMock()
    # Idle: a MagicMock attribute is truthy by default, which would make
    # every motion endpoint refuse with 409 (motion_in_progress).
    mc._motion_in_progress = False
    mc.motion_graph = MotionGraph.from_dict(
        _test_graph_dict(), preconditions=DEFAULT_PRECONDITIONS,
    )
    mc.graph_mode = GraphMode.ADVISORY
    mc.current_node = "n_home"
    mc.current_gripper_state = "empty"
    mc.last_arm_pose_name = "home"
    mc.last_rail_location_name = "Home"
    mc.last_gripper_position = 150
    mc.last_transition = None
    mc.reachable_node_ids.return_value = ["n_pickup"]
    mc.allowed_gripper_targets.return_value = ["grip_120"]
    mc.is_connected.return_value = True
    mc.is_alive = True
    mc.host = "127.0.0.1"
    mc.xarm_config = {"port": 18333}

    # set_graph_mode mutates graph_mode like the real method.
    def _set_mode(mode):
        if mc.motion_graph is None and mode != GraphMode.OFF:
            raise RuntimeError("not loaded")
        mc.graph_mode = mode
    mc.set_graph_mode.side_effect = _set_mode

    return mc


@pytest.fixture
def graph_client(monkeypatch, mock_controller_with_graph):
    from src.core.xarm_api_server import app
    monkeypatch.setattr(
        "src.core.xarm_api_server.controller", mock_controller_with_graph,
    )
    with TestClient(app) as c:
        yield c


# ── GET /graph ───────────────────────────────────────────────────────


def test_get_graph_returns_snapshot(graph_client, mock_controller_with_graph):
    resp = graph_client.get("/graph")
    assert resp.status_code == 200
    body = resp.json()
    assert body["graph_mode"] == "advisory"
    assert body["current_node"] == "n_home"
    assert body["reachable_nodes"] == ["n_pickup"]
    assert "adjacency" in body
    # Schema 0.2 serialization: catalog + per-node leaves + live state.
    assert body["gripper_state"] == "empty"
    assert body["allowed_gripper_targets"] == ["grip_120"]
    catalog = body["gripper_state_catalog"]
    assert catalog["grip_120"] == {"stroke": 120.0, "intent": "grasp"}
    nodes = {n["id"]: n for n in body["nodes"]}
    assert nodes["n_home"]["gripper_states"] == ["empty", "grip_120"]
    assert nodes["n_home"]["gripper_transitions"] == [
        ["empty", "grip_120"], ["grip_120", "empty"],
    ]
    assert "gripper_stroke" not in nodes["n_home"]


def test_get_graph_returns_404_when_no_graph(graph_client, mock_controller_with_graph):
    mock_controller_with_graph.motion_graph = None
    resp = graph_client.get("/graph")
    assert resp.status_code == 404


# ── POST /control/graph/mode ─────────────────────────────────────────


def test_post_graph_mode_switches_mode(graph_client, mock_controller_with_graph):
    resp = graph_client.post("/control/graph/mode", json={"mode": "strict"})
    assert resp.status_code == 200
    assert resp.json()["graph_mode"] == "strict"
    assert mock_controller_with_graph.graph_mode == GraphMode.STRICT


def test_post_graph_mode_invalid_value_returns_422(graph_client):
    resp = graph_client.post("/control/graph/mode", json={"mode": "panic"})
    assert resp.status_code == 422


def test_post_graph_mode_off_works_without_graph(graph_client, mock_controller_with_graph):
    mock_controller_with_graph.motion_graph = None
    resp = graph_client.post("/control/graph/mode", json={"mode": "off"})
    assert resp.status_code == 200


# ── /move/location returns 409 in STRICT on off-whitelist ────────────


def test_named_move_returns_409_on_edge_not_allowed(
    graph_client, mock_controller_with_graph,
):
    # The mocked controller's move_to_named_location raises
    # EdgeNotAllowedError to simulate STRICT-mode rejection.
    mock_controller_with_graph.move_to_named_location.side_effect = EdgeNotAllowedError(
        current="n_home", target="n_unrelated",
        reason="no whitelisted edge",
    )
    resp = graph_client.post(
        "/move/location", json={"location_name": "unrelated"},
    )
    assert resp.status_code == 409
    detail = resp.json()["detail"]
    assert detail["error"] == "edge_not_allowed"
    assert detail["current_node"] == "n_home"
    assert detail["target"] == "n_unrelated"


def test_named_move_returns_200_on_success(graph_client, mock_controller_with_graph):
    mock_controller_with_graph.move_to_named_location.return_value = True
    resp = graph_client.post(
        "/move/location", json={"location_name": "pickup"},
    )
    assert resp.status_code == 200


def test_named_move_returns_500_on_unrelated_failure(
    graph_client, mock_controller_with_graph,
):
    mock_controller_with_graph.move_to_named_location.return_value = False
    resp = graph_client.post(
        "/move/location", json={"location_name": "pickup"},
    )
    assert resp.status_code == 500


# ── /control/graph/record ────────────────────────────────────────────


def test_record_returns_409_when_no_transition(
    graph_client, mock_controller_with_graph,
):
    mock_controller_with_graph.last_transition = None
    resp = graph_client.post("/control/graph/record", json={})
    assert resp.status_code == 409


def test_record_appends_edge_to_yaml_file(
    graph_client, mock_controller_with_graph, tmp_path, monkeypatch,
):
    """End-to-end: record a transition, verify YAML grew, in-memory
    graph reloaded with the new edge present."""
    # Hand-formatted block-style YAML with a single seed edge — text
    # append assumes block style with column-0 list items (matches the
    # real motion_graph.yaml).
    initial_yaml = """\
schema_version: "0.2"

gripper_states:
  empty: {stroke: 150, intent: none}

nodes:
- id: a
  arm: home
  rail: Home
- id: b
  arm: pickup
  rail: Home

edges:
- from: a
  to: b
  mode: joint
  speed: 30
"""
    target_dir = tmp_path / "src" / "settings"
    target_dir.mkdir(parents=True)
    (target_dir / "motion_graph.yaml").write_text(initial_yaml)
    monkeypatch.chdir(tmp_path)

    # Initial YAML has a->b. Record the reverse direction (new edge).
    mock_controller_with_graph.last_transition = {
        "from_node": "b",
        "to_node": "a",
        "mode": "linear",
        "speed": 30,
        "timestamp": 0,
    }

    resp = graph_client.post(
        "/control/graph/record",
        json={"comment": "test edge", "speed": 20},
    )
    assert resp.status_code == 200, resp.json()
    body = resp.json()["recorded"]
    assert body["from"] == "b" and body["to"] == "a"
    assert body["speed"] == 20  # override applied

    # Verify YAML file got the new edge appended.
    import yaml as _yaml
    reloaded = _yaml.safe_load((target_dir / "motion_graph.yaml").read_text())
    assert len(reloaded["edges"]) == 2
    last_edge = reloaded["edges"][-1]
    assert last_edge["from"] == "b" and last_edge["to"] == "a"

    # Verify in-memory graph was reloaded (the mock's motion_graph was
    # replaced by the endpoint).
    assert mock_controller_with_graph.motion_graph.find_edge("b", "a") is not None


def test_record_returns_400_on_validation_failure(
    graph_client, mock_controller_with_graph, tmp_path, monkeypatch,
):
    """Recording an edge that violates a coherence rule must NOT touch
    the YAML and must return 400."""
    import yaml as _yaml
    initial = {
        "schema_version": "0.2",
        "gripper_states": {"empty": {"stroke": 150, "intent": "none"}},
        "nodes": [
            {"id": "a", "arm": "home", "rail": "Home"},
        ],
        "edges": [],
    }
    target_dir = tmp_path / "src" / "settings"
    target_dir.mkdir(parents=True)
    (target_dir / "motion_graph.yaml").write_text(_yaml.safe_dump(initial))
    monkeypatch.chdir(tmp_path)

    # Record a transition whose `to_node` doesn't exist in the YAML.
    mock_controller_with_graph.last_transition = {
        "from_node": "a",
        "to_node": "ghost",
        "mode": "linear",
        "speed": 30,
        "timestamp": 0,
    }
    resp = graph_client.post("/control/graph/record", json={})
    assert resp.status_code == 400
    assert "validation" in resp.json()["detail"].lower()

    # YAML must be unchanged.
    reloaded = _yaml.safe_load((target_dir / "motion_graph.yaml").read_text())
    assert reloaded["edges"] == []


# ── POST /control/graph/gripper ──────────────────────────────────────


def test_gripper_endpoint_success(graph_client, mock_controller_with_graph):
    mock_controller_with_graph.set_gripper_state.return_value = True
    mock_controller_with_graph.current_gripper_state = "grip_120"
    mock_controller_with_graph.allowed_gripper_targets.return_value = ["empty"]
    resp = graph_client.post("/control/graph/gripper", json={"state": "grip_120"})
    assert resp.status_code == 200
    body = resp.json()
    assert body["gripper_state"] == "grip_120"
    assert body["allowed_gripper_targets"] == ["empty"]
    mock_controller_with_graph.set_gripper_state.assert_called_once_with("grip_120")


def test_gripper_endpoint_unknown_state_returns_422(graph_client):
    resp = graph_client.post("/control/graph/gripper", json={"state": "grip_9000"})
    assert resp.status_code == 422


def test_gripper_endpoint_transition_not_allowed_returns_409(
    graph_client, mock_controller_with_graph,
):
    mock_controller_with_graph.set_gripper_state.side_effect = GripperTransitionError(
        "n_pickup", "empty", "grip_120",
        "transition 'empty' -> 'grip_120' is not whitelisted at node 'n_pickup'",
    )
    resp = graph_client.post("/control/graph/gripper", json={"state": "grip_120"})
    assert resp.status_code == 409
    detail = resp.json()["detail"]
    assert detail["error"] == "gripper_transition_not_allowed"
    assert detail["node"] == "n_pickup"
    assert detail["from_state"] == "empty"
    assert detail["to_state"] == "grip_120"


def test_gripper_endpoint_verification_failure_returns_500(
    graph_client, mock_controller_with_graph,
):
    mock_controller_with_graph.set_gripper_state.return_value = False
    resp = graph_client.post("/control/graph/gripper", json={"state": "grip_120"})
    assert resp.status_code == 500


def test_gripper_endpoint_404_when_no_graph(graph_client, mock_controller_with_graph):
    mock_controller_with_graph.motion_graph = None
    resp = graph_client.post("/control/graph/gripper", json={"state": "grip_120"})
    assert resp.status_code == 404


# ── /control/graph/travel_to ─────────────────────────────────────────


def test_travel_to_returns_200_with_path(graph_client, mock_controller_with_graph):
    mock_controller_with_graph.travel_to_node.return_value = {
        "success": True, "path": ["n_pickup"],
        "completed": ["n_pickup"], "failed_hop": None,
    }
    resp = graph_client.post(
        "/control/graph/travel_to", json={"node_id": "n_pickup", "speed": 20},
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body["path"] == ["n_pickup"]
    assert body["current_node"] == "n_home"
    mock_controller_with_graph.travel_to_node.assert_called_once_with(
        node_id="n_pickup", speed=20,
    )


def test_travel_to_returns_409_on_unknown_node(graph_client):
    resp = graph_client.post(
        "/control/graph/travel_to", json={"node_id": "n_ghost"},
    )
    assert resp.status_code == 409
    assert "unknown node" in resp.json()["detail"]


def test_travel_to_returns_409_on_no_path(graph_client, mock_controller_with_graph):
    mock_controller_with_graph.travel_to_node.side_effect = NoPathError(
        "n_home", "n_pickup", "grip_120",
        "target unreachable with this gripper state",
    )
    resp = graph_client.post(
        "/control/graph/travel_to", json={"node_id": "n_pickup"},
    )
    assert resp.status_code == 409
    detail = resp.json()["detail"]
    assert detail["error"] == "no_path"
    assert detail["from"] == "n_home"
    assert detail["to"] == "n_pickup"
    assert detail["gripper_state"] == "grip_120"


def test_travel_to_returns_409_when_off_grid(graph_client, mock_controller_with_graph):
    mock_controller_with_graph.travel_to_node.side_effect = EdgeNotAllowedError(
        current=None, target="n_pickup",
        reason="not pinned to a graph node — recover to a node first",
    )
    resp = graph_client.post(
        "/control/graph/travel_to", json={"node_id": "n_pickup"},
    )
    assert resp.status_code == 409
    detail = resp.json()["detail"]
    assert detail["error"] == "edge_not_allowed"
    assert detail["current_node"] is None


def test_travel_to_returns_500_on_mid_journey_failure(
    graph_client, mock_controller_with_graph,
):
    mock_controller_with_graph.travel_to_node.return_value = {
        "success": False, "path": ["n_pickup"],
        "completed": [], "failed_hop": "n_pickup",
    }
    resp = graph_client.post(
        "/control/graph/travel_to", json={"node_id": "n_pickup"},
    )
    assert resp.status_code == 500
    detail = resp.json()["detail"]
    assert detail["error"] == "travel_failed"
    assert detail["failed_hop"] == "n_pickup"
    assert detail["completed"] == []


def test_travel_to_returns_404_without_graph(
    graph_client, mock_controller_with_graph,
):
    mock_controller_with_graph.motion_graph = None
    resp = graph_client.post(
        "/control/graph/travel_to", json={"node_id": "n_pickup"},
    )
    assert resp.status_code == 404


def test_get_graph_includes_travel_targets(graph_client):
    resp = graph_client.get("/graph")
    assert resp.status_code == 200
    assert resp.json()["travel_targets"] == ["n_pickup"]


# ── concurrent-motion refusal (STATUS_SPEC v1.2 §2.3 / §6.2) ──────────


@pytest.mark.parametrize("endpoint,body", [
    ("/control/graph/move_to",   {"node_id": "n_pickup"}),
    ("/control/graph/travel_to", {"node_id": "n_pickup"}),
    ("/move/location",           {"location_name": "pickup"}),
    ("/move/position",           {"x": 300, "y": 0, "z": 300}),
    ("/move/joints",             {"angles": [0, 0, 0, 0, 0]}),
    ("/track/move",              {"position": 100}),
    ("/move/home",               None),
])
def test_motion_endpoints_refuse_a_second_concurrent_motion(
    graph_client, mock_controller_with_graph, endpoint, body,
):
    """A move arriving while one is in flight is refused with 409.

    Two overlapping commands to the same arm are a collision risk, and
    spec §2.3 requires refusing a second concurrent run. The body is
    distinguishable by shape (``error: motion_in_progress``) so a client
    can branch on it rather than string-matching the detail text.
    """
    mock_controller_with_graph._motion_in_progress = True

    resp = graph_client.post(endpoint, json=body) if body is not None \
        else graph_client.post(endpoint)

    assert resp.status_code == 409, endpoint
    assert resp.json()["detail"]["error"] == "motion_in_progress", endpoint
    # Refused before dispatch: the arm was never commanded.
    mock_controller_with_graph.move_to_node.assert_not_called()
    mock_controller_with_graph.travel_to_node.assert_not_called()
    mock_controller_with_graph.move_to_named_location.assert_not_called()
    mock_controller_with_graph.move_to_position.assert_not_called()
    mock_controller_with_graph.go_home.assert_not_called()


def test_allowed_actions_agrees_with_the_409(graph_client, mock_controller_with_graph):
    """§6.2: the advisory list and the authoritative refusal must not drift.

    For both values of the motion state, ``move.n_pickup`` appears in
    ``allowed_actions`` if and only if POSTing it would not be refused.
    """
    from src.core.xarm_controller import ComponentState

    mock_controller_with_graph.graph_mode = GraphMode.STRICT
    mock_controller_with_graph.move_to_node.return_value = True
    # A ready arm. The graph fixture leaves `states` and the error fields as
    # bare mocks, which /status reads as error / requires_init — no move
    # target would be listed, for reasons unrelated to the motion gate this
    # test is about.
    mock_controller_with_graph.last_error_code = 0
    mock_controller_with_graph.last_error = None
    mock_controller_with_graph.states = {
        'connection': ComponentState.ENABLED,
        'arm': ComponentState.ENABLED,
        'gripper': ComponentState.ENABLED,
        'track': ComponentState.ENABLED,
        'force_torque': ComponentState.DISABLED,
    }

    for in_flight in (False, True):
        mock_controller_with_graph._motion_in_progress = in_flight

        listed = "move.n_pickup" in graph_client.get("/status").json()["allowed_actions"]
        refused = graph_client.post(
            "/control/graph/move_to", json={"node_id": "n_pickup"},
        ).status_code == 409

        assert listed is not refused, (
            f"drift with _motion_in_progress={in_flight}: "
            f"listed={listed} refused={refused}"
        )


def test_reservation_is_released_after_the_move(graph_client, mock_controller_with_graph):
    """The slot must not leak: a second move after the first completes is
    accepted. The real controller's enter/exit are mocked out here, so this
    pins the endpoint's own release path."""
    mock_controller_with_graph._motion_in_progress = False
    mock_controller_with_graph.move_to_node.return_value = True

    first = graph_client.post("/control/graph/move_to", json={"node_id": "n_pickup"})
    second = graph_client.post("/control/graph/move_to", json={"node_id": "n_pickup"})

    assert first.status_code == 200
    assert second.status_code == 200
    assert mock_controller_with_graph.exit_motion.call_count == 2
