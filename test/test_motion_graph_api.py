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
    MotionGraph,
)


def _test_graph_dict():
    return {
        "schema_version": "0.1",
        "nodes": [
            {"id": "n_home",   "arm": "home",   "rail": "Home"},
            {"id": "n_pickup", "arm": "pickup", "rail": "Home"},
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
    mc.motion_graph = MotionGraph.from_dict(
        _test_graph_dict(), preconditions=DEFAULT_PRECONDITIONS,
    )
    mc.graph_mode = GraphMode.ADVISORY
    mc.current_node = "n_home"
    mc.last_arm_pose_name = "home"
    mc.last_rail_location_name = "Home"
    mc.last_gripper_position = 150
    mc.last_transition = None
    mc.reachable_node_ids.return_value = ["n_pickup"]
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
    # New fields
    assert "nodes" in body
    assert body["nodes"][0]["gripper_stroke"] is not None


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
    # append assumes block style (matches the real motion_graph.yaml).
    initial_yaml = """\
schema_version: "0.1"

nodes:
  - id: a
    arm: home
    rail: Home
  - id: b
    arm: pickup
    rail: Home

edges:
  - from: a
    to:   b
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
        "schema_version": "0.1",
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
