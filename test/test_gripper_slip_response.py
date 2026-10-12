"""/gripper/open and /gripper/close answer 200 when the move ended in a
tolerated slip, and say so in the body (agents clear and retry for now)."""

from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock

import pytest
from fastapi.testclient import TestClient

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.core.claims import ClaimManager
from src.core.motion_graph import GraphMode


@pytest.fixture
def client(monkeypatch):
    from src.core.xarm_api_server import app

    mc = MagicMock()
    mc.is_simulated = False
    mc.is_real_box_simulating = False
    mc.claim_manager = ClaimManager(default_ttl_s=30.0)
    mc.graph_mode = GraphMode.OFF
    mc.motion_graph = None
    mc._motion_in_progress = False
    mc.is_connected.return_value = True
    mc.is_alive = True
    mc.open_gripper.return_value = True
    mc.close_gripper.return_value = True
    monkeypatch.setattr("src.core.xarm_api_server.controller", mc)
    with TestClient(app) as tc:
        token = tc.post("/control/claim", json={"owner": "t", "session_id": "s1"}).json()["claim_token"]
        yield tc, mc, {"X-Claim-Token": token}


@pytest.mark.parametrize("verb", ["open", "close"])
def test_slip_is_reported_in_a_200(client, verb):
    tc, mc, headers = client
    mc.last_gripper_move_fault = {"error_code": 12, "error_text": "object slipped"}
    resp = tc.post(f"/gripper/{verb}", headers=headers)
    assert resp.status_code == 200
    body = resp.json()
    assert body["gripper_fault"] == {"error_code": 12, "error_text": "object slipped"}
    assert "object slipped (code 12)" in body["message"]


@pytest.mark.parametrize("verb", ["open", "close"])
def test_clean_move_body_is_unchanged(client, verb):
    tc, mc, headers = client
    mc.last_gripper_move_fault = None
    resp = tc.post(f"/gripper/{verb}", headers=headers)
    assert resp.status_code == 200
    assert resp.json() == {"message": f"{verb.capitalize()} gripper command completed."}
