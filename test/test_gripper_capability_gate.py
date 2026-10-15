"""Stroke/force gripper commands are refused with 409 when the active
gripper config turns the feature off (has_stroke_control / has_force_control
false), instead of failing deep in the controller as a generic 500.

The lab BioGripper Gen2 ran open/close only from 2026-10-11 to 2026-10-15;
while a feature is off, every agent-facing route that would command a stroke
or force must say so.
"""

from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock

import pytest
from fastapi.testclient import TestClient

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.core.claims import ClaimManager
from src.core.motion_graph import DEFAULT_PRECONDITIONS, GraphMode, MotionGraph


def _graph_dict():
    return {
        "schema_version": "0.2",
        "gripper_states": {
            "empty": {"stroke": 150, "intent": "none"},
            "grip_120": {"stroke": 120, "intent": "grasp"},
        },
        "nodes": [
            {
                "id": "n_home", "arm": "home", "rail": "Home",
                "gripper_states": ["empty", "grip_120"],
                "gripper_transitions": [["empty", "grip_120"], ["grip_120", "empty"]],
            },
        ],
        "edges": [],
    }


def _controller(stroke: bool, force: bool) -> MagicMock:
    mc = MagicMock()
    mc.is_simulated = False
    mc.is_real_box_simulating = False
    mc.claim_manager = ClaimManager(default_ttl_s=30.0)
    mc.motion_graph = MotionGraph.from_dict(_graph_dict(), preconditions=DEFAULT_PRECONDITIONS)
    mc.graph_mode = GraphMode.OFF
    mc.current_node = "n_home"
    mc.current_gripper_state = "empty"
    mc._motion_in_progress = False
    mc.is_connected.return_value = True
    mc.is_alive = True
    mc.current_gripper_config = {
        "has_stroke_control": stroke,
        "has_force_control": force,
        "stroke_range": {"min": 71, "max": 150},
        "force_range": {"min": 1, "max": 100},
    }
    mc.move_gripper_to_stroke.return_value = True
    mc.set_gripper_force.return_value = True
    mc.set_gripper_state.return_value = True
    mc.allowed_gripper_targets.return_value = ["grip_120"]
    return mc


@pytest.fixture
def make_client(monkeypatch):
    from src.core.xarm_api_server import app

    def _make(stroke: bool, force: bool):
        mc = _controller(stroke, force)
        monkeypatch.setattr("src.core.xarm_api_server.controller", mc)
        client = TestClient(app)
        client.__enter__()
        resp = client.post("/control/claim", json={"owner": "t", "session_id": "s1"})
        headers = {"X-Claim-Token": resp.json()["claim_token"]}
        return client, mc, headers

    return _make


@pytest.mark.parametrize("path", ["/control/freehand/gripper/stroke", "/gripper/move/stroke"])
def test_stroke_refused_when_stroke_control_off(make_client, path):
    client, mc, headers = make_client(stroke=False, force=False)
    resp = client.post(path, json={"stroke": 120}, headers=headers)
    assert resp.status_code == 409, resp.text
    detail = resp.json()["detail"]
    assert detail["error"] == "gripper_stroke_control_disabled"
    assert detail["action"] == "gripper.move_stroke"
    mc.move_gripper_to_stroke.assert_not_called()


@pytest.mark.parametrize("path", ["/control/freehand/gripper/force", "/gripper/force"])
def test_force_refused_when_force_control_off(make_client, path):
    client, mc, headers = make_client(stroke=False, force=False)
    resp = client.post(path, json={"force": 40}, headers=headers)
    assert resp.status_code == 409, resp.text
    assert resp.json()["detail"]["error"] == "gripper_force_control_disabled"
    mc.set_gripper_force.assert_not_called()


def test_graph_stroke_state_refused_when_stroke_control_off(make_client):
    client, mc, headers = make_client(stroke=False, force=False)
    resp = client.post("/control/graph/gripper", json={"state": "grip_120"}, headers=headers)
    assert resp.status_code == 409, resp.text
    detail = resp.json()["detail"]
    assert detail["error"] == "gripper_stroke_control_disabled"
    assert detail["action"] == "graph.gripper"
    mc.set_gripper_state.assert_not_called()


def test_graph_empty_state_still_allowed_when_stroke_control_off(make_client):
    client, mc, headers = make_client(stroke=False, force=False)
    resp = client.post("/control/graph/gripper", json={"state": "empty"}, headers=headers)
    assert resp.status_code == 200, resp.text
    mc.set_gripper_state.assert_called_once_with("empty")


def test_stroke_and_force_pass_when_enabled(make_client):
    client, mc, headers = make_client(stroke=True, force=True)
    assert client.post("/gripper/move/stroke", json={"stroke": 120}, headers=headers).status_code == 200
    assert client.post("/gripper/force", json={"force": 40}, headers=headers).status_code == 200
    assert client.post("/control/graph/gripper", json={"state": "grip_120"}, headers=headers).status_code == 200


def test_open_close_unaffected_when_stroke_control_off(make_client):
    client, mc, headers = make_client(stroke=False, force=False)
    mc.open_gripper.return_value = True
    mc.close_gripper.return_value = True
    assert client.post("/gripper/open", json={}, headers=headers).status_code == 200
    assert client.post("/gripper/close", json={}, headers=headers).status_code == 200
