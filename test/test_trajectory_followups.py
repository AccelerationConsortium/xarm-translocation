"""Regressions for the five issues found on the arm on 2026-10-09 (stage 3)."""

from __future__ import annotations

import asyncio
import os
import sys
from datetime import datetime, timezone
from unittest.mock import MagicMock

import pytest
from fastapi.testclient import TestClient

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import src.core.xarm_api_server as srv
from src.core.claims import ClaimManager
from src.core.motion_graph import GraphMode
from src.core.status_builder import build_status
from src.core.trajectory_executor import EXPIRED, TrajectoryError, TrajectoryManager, TrajectorySettings
from src.core.xarm_controller import ComponentState, XArmController


# 1. Sign-in codes are requested from the shared service's POST /auth/login.


def test_request_code_uses_auth_login_and_falls_back_on_404(monkeypatch):
    monkeypatch.setattr(srv, "AUTH_SIDECAR_URL", "http://auth-sidecar:8009")
    calls = []

    def sidecar(method, path, body=None, cookie_token=None):
        calls.append(path)
        return (202, {"ok": True}, None) if path == "/auth/login" else (200, {"ok": True}, None)

    monkeypatch.setattr(srv, "_auth_sidecar_call", sidecar)
    with TestClient(srv.app) as client:
        assert client.post("/auth/request-code", json={"email": "op@lab"}).status_code == 202
    assert calls == ["/auth/login"]

    calls.clear()

    def old_sidecar(method, path, body=None, cookie_token=None):
        calls.append(path)
        return (404, {"detail": "Not Found"}, None) if path == "/auth/login" else (200, {"ok": True}, None)

    monkeypatch.setattr(srv, "_auth_sidecar_call", old_sidecar)
    with TestClient(srv.app) as client:
        assert client.post("/auth/request-code", json={"email": "op@lab"}).status_code == 200
    assert calls == ["/auth/login", "/auth/request-code"]


# 2a. Connect re-asserts state 0 until the arm leaves state 4/5.


def test_ensure_ready_state_resends_until_the_arm_is_ready(monkeypatch):
    monkeypatch.setattr("src.core.xarm_controller.time.sleep", lambda s: None)
    me = MagicMock()
    states = iter([5, 5, 2])
    type(me.arm).state = property(lambda self: next(states))
    assert XArmController._ensure_ready_state(me) is True
    assert me.arm.set_state.call_count == 2


def test_ensure_ready_state_gives_up_after_its_attempts(monkeypatch):
    monkeypatch.setattr("src.core.xarm_controller.time.sleep", lambda s: None)
    me = MagicMock()
    me.arm.state = 5
    assert XArmController._ensure_ready_state(me, attempts=3) is False
    assert me.arm.set_state.call_count == 3


# 2b. /status does not say "ready" for an arm in state 4 or 5.


def _healthy_controller():
    c = MagicMock()
    c.is_simulated = False
    c.last_error_code = 0
    c.last_error = None
    c.alive = True
    c._recovering = False
    c.health_failure = None
    c._motion_in_progress = False
    c._activity_since = datetime.now(timezone.utc)
    c.states = {"connection": ComponentState.ENABLED, "arm": ComponentState.ENABLED}
    c.has_gripper.return_value = False
    c.has_track.return_value = False
    c.has_force_torque_sensor.return_value = False
    c.claim_manager = ClaimManager()
    c.graph_mode = GraphMode.OFF
    c.arm.mode = 0
    return c


@pytest.mark.parametrize("state,expected", [(2, "ready"), (1, "ready"), (4, "degraded"), (5, "degraded")])
def test_status_reflects_a_halted_arm(state, expected):
    c = _healthy_controller()
    c.arm.state = state
    status = build_status(c)
    assert status.equipment_status == expected
    if expected == "degraded":
        assert f"controller state {state}" in status.message
        assert status.required_actions == ["clear_errors"]


# 3. A never-started session whose claim ended no longer blocks.


def test_unstarted_session_of_an_ended_claim_is_released():
    mgr = TrajectoryManager(TrajectorySettings.from_mapping({"enabled": True}))
    old = mgr.create(key="claim-A", owner="a", num_joints=5)
    with pytest.raises(TrajectoryError):
        mgr.create(key="claim-A", owner="a", num_joints=5)      # same owner: still one open session
    new = mgr.create(key="claim-B", owner="b", num_joints=5)    # A's claim has ended
    assert old.state == EXPIRED and old.reason == "abandoned: its claim ended"
    assert new.id != old.id and new.state == "created"


# 4. Start waits out the 5 Hz cached state after a move, and nothing else.


class _Arm:
    def __init__(self, states):
        self._states = list(states)

    @property
    def state(self):
        return self._states.pop(0) if len(self._states) > 1 else self._states[0]


def _ctrl(states, moving=False):
    c = MagicMock()
    c.arm = _Arm(states)
    c._motion_in_progress = moving
    return c


def test_wait_arm_idle():
    assert asyncio.run(srv._wait_arm_idle(_ctrl([1, 1, 1, 2]), poll_s=0.001)) is True
    assert asyncio.run(srv._wait_arm_idle(_ctrl([5]), poll_s=0.001)) is False          # never waits on 5
    assert asyncio.run(srv._wait_arm_idle(_ctrl([1, 2], moving=True), poll_s=0.001)) is False
    assert asyncio.run(srv._wait_arm_idle(_ctrl([1]), timeout_s=0.02, poll_s=0.001)) is False
