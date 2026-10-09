"""HTTP surface of joint-trajectory execution: gates, session flow, the
traffic guard, and the STOP / disconnect hooks (mock controller, fake SDK
backend, fake clock)."""

from __future__ import annotations

import os
import sys
import time
from unittest.mock import MagicMock

import pytest
from fastapi.testclient import TestClient

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import src.core.xarm_api_server as api
from src.core.claims import ClaimManager
from src.core.motion_graph import GraphMode
from src.core.trajectory_executor import TrajectoryManager, TrajectorySettings
from src.core.xarm_controller import XArmController
from test.test_trajectory_executor import JOINT_LIMITS_5, FakeBackend, FakeClock, points_j1

ZERO = [0.0] * 5


@pytest.fixture
def fake():
    return FakeClock()


@pytest.fixture
def settings(tmp_path):
    return TrajectorySettings.from_mapping(
        {"enabled": True, "realtime_report": False, "log_dir": str(tmp_path / "logs")})


@pytest.fixture
def mock_controller(fake, settings):
    mc = MagicMock()
    mc.is_simulated = False
    mc.is_real_box_simulating = False
    mc.claim_manager = ClaimManager(default_ttl_s=30.0, enforce=True)
    mc.graph_mode = GraphMode.ADVISORY
    mc.graph_mode_override_snapshot.return_value = None
    mc.sash_interlock = None
    mc._motion_in_progress = False
    mc.is_alive = True
    mc.arm = MagicMock()
    mc.arm.mode = 0
    mc.arm.state = 2
    mc.arm.error_code = 0
    mc.arm.warn_code = 0
    mc.num_joints = 5
    mc.joint_limits = JOINT_LIMITS_5
    mc.max_joint_speed = 60.0
    mc.get_current_joints.return_value = list(ZERO)
    mc.host = "127.0.0.1"
    mc._trajectory_manager = TrajectoryManager(settings, clock=fake.clock, sleep=fake.sleep)
    return mc


@pytest.fixture
def backend(fake):
    return FakeBackend(fake)


@pytest.fixture
def client(monkeypatch, mock_controller, settings, backend):
    monkeypatch.setattr("src.core.xarm_api_server.controller", mock_controller)
    monkeypatch.setattr("src.core.xarm_api_server._trajectory_settings_cache", settings)
    monkeypatch.setattr("src.core.xarm_api_server.make_backend", lambda name, arm, timeout: backend)
    with TestClient(api.app) as c:
        yield c


@pytest.fixture
def headers(client):
    resp = client.post("/control/claim", json={"owner": "t", "session_id": "s1"})
    return {"X-Claim-Token": resp.json()["claim_token"]}


def upload(client, headers, pts=None):
    resp = client.post("/control/freehand/trajectory", json={}, headers=headers)
    assert resp.status_code == 201, resp.text
    sid = resp.json()["session_id"]
    resp = client.put(f"/control/freehand/trajectory/{sid}/chunks/0",
                      json={"points": pts or points_j1(), "final": True}, headers=headers)
    assert resp.status_code == 200, resp.text
    return sid


def wait_state(client, sid, states, timeout=10):
    deadline = time.time() + timeout
    while time.time() < deadline:
        body = client.get(f"/control/freehand/trajectory/{sid}").json()
        # The terminal state comes after the slot is released; the log
        # follows it, so wait for both.
        if body["state"] in states and body["log_available"]:
            return body
        time.sleep(0.01)
    raise AssertionError(f"session never reached {states}")


# ── Disabled by default; validate always works ───────────────────────


def test_disabled_by_default(monkeypatch, client, headers):
    monkeypatch.setattr("src.core.xarm_api_server._trajectory_settings_cache", TrajectorySettings())
    resp = client.post("/control/freehand/trajectory", json={}, headers=headers)
    assert resp.status_code == 412 and resp.json()["detail"]["error"] == "trajectory_disabled"


def test_shipped_settings_file_is_disabled():
    assert TrajectorySettings.load(os.path.join("src", "settings", "trajectory.yaml")).enabled is False


def test_validate_route_reports_without_moving(monkeypatch, client, mock_controller, backend):
    monkeypatch.setattr("src.core.xarm_api_server._trajectory_settings_cache", TrajectorySettings())
    resp = client.post("/control/freehand/trajectory/validate", json={"points": points_j1()})
    assert resp.status_code == 200, resp.text
    assert resp.json()["valid"] is True and resp.json()["summary"]["interpolation"] == "quintic"
    bad = client.post("/control/freehand/trajectory/validate",
                      json={"points": [{"t": 0, "joints_deg": ZERO}, {"t": 0.1, "joints_deg": [90, 0, 0, 0, 0]}]})
    assert bad.status_code == 422 and bad.json()["detail"]["error"] == "trajectory_invalid"
    assert backend.sent == [] and not mock_controller.enter_motion.called


# ── The full flow ────────────────────────────────────────────────────


def test_complete_flow_holds_and_releases_the_motion_slot(client, headers, mock_controller, backend):
    sid = upload(client, headers)
    resp = client.post(f"/control/freehand/trajectory/{sid}/start", headers=headers)
    assert resp.status_code == 200, resp.text
    body = wait_state(client, sid, {"completed", "failed", "stopped"})
    assert body["state"] == "completed", body
    assert mock_controller.enter_motion.call_count == 1
    assert mock_controller.exit_motion.call_count == 1
    assert body["started_at_utc"] and body["executed"]["fraction"] == 1.0
    assert body["log_available"] is True
    log = client.get(f"/control/freehand/trajectory/{sid}/log").json()
    assert len(log["ticks"]["sent_s"]) == len(backend.sent) == 100
    assert mock_controller.last_arm_pose_name is None
    assert api.get_trajectory_manager(mock_controller).summary()["session"]["state"] == "completed"


def test_duplicate_and_conflicting_chunks_over_http(client, headers):
    resp = client.post("/control/freehand/trajectory", json={}, headers=headers)
    sid = resp.json()["session_id"]
    pts = points_j1()
    url = f"/control/freehand/trajectory/{sid}/chunks/0"
    assert client.put(url, json={"points": pts[:5]}, headers=headers).json()["duplicate"] is False
    assert client.put(url, json={"points": pts[:5]}, headers=headers).json()["duplicate"] is True
    resp = client.put(url, json={"points": pts[:4]}, headers=headers)
    assert resp.status_code == 409 and resp.json()["detail"]["error"] == "chunk_conflict"
    resp = client.put(f"/control/freehand/trajectory/{sid}/chunks/2", json={"points": pts[5:]}, headers=headers)
    assert resp.status_code == 409 and resp.json()["detail"]["expected_seq"] == 1
    resp = client.post(f"/control/freehand/trajectory/{sid}/start", headers=headers)
    assert resp.status_code == 409 and resp.json()["detail"]["error"] == "trajectory_incomplete"


# ── Gates ────────────────────────────────────────────────────────────


def test_strict_refuses_create_chunk_and_start_but_not_cancel(client, headers, mock_controller):
    sid = upload(client, headers)
    mock_controller.graph_mode = GraphMode.STRICT
    for method, path in (("post", "/control/freehand/trajectory"),
                         ("put", f"/control/freehand/trajectory/{sid}/chunks/1"),
                         ("post", f"/control/freehand/trajectory/{sid}/start")):
        resp = getattr(client, method)(path, json={"points": points_j1()} if method == "put" else {},
                                       headers=headers)
        assert resp.status_code == 409 and resp.json()["detail"]["error"] == "graph_mode_strict", path
    assert client.get(f"/control/freehand/trajectory/{sid}").status_code == 200
    resp = client.post(f"/control/freehand/trajectory/{sid}/cancel", headers=headers)
    assert resp.status_code == 200 and resp.json()["state"] == "cancelled"


def test_start_gates(client, headers, mock_controller):
    sid = upload(client, headers)
    other = {"X-Claim-Token": "not-the-creator"}
    assert client.post(f"/control/freehand/trajectory/{sid}/start", headers=other).status_code == 423
    mock_controller.graph_mode_override_snapshot.return_value = {
        "active": True, "persistent": False, "remaining_seconds": 5.0}
    resp = client.post(f"/control/freehand/trajectory/{sid}/start", headers=headers)
    assert resp.status_code == 409 and resp.json()["detail"]["error"] == "graph_mode_override_expiring"
    mock_controller.graph_mode_override_snapshot.return_value = None
    mock_controller.arm.mode = 2
    resp = client.post(f"/control/freehand/trajectory/{sid}/start", headers=headers)
    assert resp.status_code == 412 and resp.json()["detail"]["error"] == "manual_mode"
    mock_controller.arm.mode = 0
    mock_controller._motion_in_progress = True
    resp = client.post(f"/control/freehand/trajectory/{sid}/start", headers=headers)
    assert resp.status_code == 409 and resp.json()["detail"]["error"] == "motion_in_progress"
    assert not mock_controller.enter_motion.called


def test_start_state_mismatch_releases_the_slot(client, headers, mock_controller):
    sid = upload(client, headers)
    mock_controller.get_current_joints.return_value = [3.0, 0, 0, 0, 0]
    resp = client.post(f"/control/freehand/trajectory/{sid}/start", headers=headers)
    assert resp.status_code == 422
    assert [e["code"] for e in resp.json()["detail"]["report"]["errors"]] == ["start_joint_mismatch"]
    assert mock_controller.enter_motion.call_count == mock_controller.exit_motion.call_count == 1


# ── While running ────────────────────────────────────────────────────


def test_traffic_guard_and_stop_during_a_run(client, headers, mock_controller, backend):
    backend.gate_at = 20
    mgr = api.get_trajectory_manager(mock_controller)
    mock_controller.stop_motion.side_effect = lambda: (mgr.notify_hard_stop("stop"), True)[1]
    sid = upload(client, headers)
    assert client.post(f"/control/freehand/trajectory/{sid}/start", headers=headers).status_code == 200
    assert backend.reached_gate.wait(5)
    assert api.trajectory_running(mock_controller)
    for method, path, body in (("get", "/positions", None), ("post", "/gripper/open", {}),
                               ("post", "/robot/manual", {"enable": True}),
                               ("post", "/clear/errors", None), ("get", "/force-torque/status", None),
                               ("post", "/kinematics/fk", {"joints": ZERO})):
        kwargs = {"headers": headers}
        if body is not None:
            kwargs["json"] = body
        resp = getattr(client, method)(path, **kwargs)
        assert resp.status_code == 409 and resp.json()["detail"]["error"] == "trajectory_running", path
    resp = client.put(f"/control/freehand/trajectory/{sid}/chunks/1", json={"points": points_j1()}, headers=headers)
    assert resp.status_code == 409
    assert client.post("/control/stop").status_code == 200
    backend.gate.set()
    body = wait_state(client, sid, {"stopped", "failed", "completed"})
    assert body["state"] == "stopped" and body["reason"] == "stop"
    assert len(backend.sent) == 21
    assert mock_controller.exit_motion.call_count == 1
    assert not api.trajectory_running(mock_controller)
    assert client.get("/positions").status_code != 409


def test_cancel_during_a_run(client, headers, mock_controller, backend):
    backend.gate_at = 20
    sid = upload(client, headers, pts=points_j1(end=20.0, duration=2.0, n=21))
    client.post(f"/control/freehand/trajectory/{sid}/start", headers=headers)
    assert backend.reached_gate.wait(5)
    assert client.post(f"/control/freehand/trajectory/{sid}/cancel", headers=headers).status_code == 200
    backend.gate.set()
    body = wait_state(client, sid, {"cancelled", "failed", "completed"})
    assert body["state"] == "cancelled" and "finish" in backend.calls


def test_claim_release_mid_run_is_a_constrained_stop(client, headers, mock_controller, backend):
    backend.gate_at = 20
    sid = upload(client, headers, pts=points_j1(end=20.0, duration=2.0, n=21))
    client.post(f"/control/freehand/trajectory/{sid}/start", headers=headers)
    assert backend.reached_gate.wait(5)
    mock_controller.claim_manager.release(headers["X-Claim-Token"])
    backend.gate.set()
    body = wait_state(client, sid, {"failed", "cancelled", "completed"})
    assert body["state"] == "failed" and body["reason"] == "claim_lost"


# ── Controller hooks ─────────────────────────────────────────────────


def test_controller_stop_and_disconnect_notify_the_trajectory():
    me = MagicMock()
    me.arm.emergency_stop.return_value = 0
    me._stop_track.return_value = True
    XArmController.stop_motion(me)
    me._trajectory_manager.notify_hard_stop.assert_called_with("stop")
    assert me._trajectory_manager.notify_hard_stop.call_count == 1

    me = MagicMock()
    XArmController.disconnect(me)
    me._trajectory_manager.notify_hard_stop.assert_called_once_with("disconnect")
    me._trajectory_manager.wait_idle.assert_called_once()


# ── Review regressions ───────────────────────────────────────────────


def test_stop_in_the_start_window_refuses_and_releases(client, headers, mock_controller, backend):
    sid = upload(client, headers)
    mgr = api.get_trajectory_manager(mock_controller)

    def read_joints_while_someone_presses_stop():
        mgr.notify_hard_stop("stop")
        return list(ZERO)

    mock_controller.get_current_joints.side_effect = read_joints_while_someone_presses_stop
    resp = client.post(f"/control/freehand/trajectory/{sid}/start", headers=headers)
    assert resp.status_code == 409 and resp.json()["detail"]["error"] == "stopped_during_start"
    assert backend.sent == [] and backend.calls == []
    assert mock_controller.enter_motion.call_count == mock_controller.exit_motion.call_count == 1
    mock_controller.get_current_joints.side_effect = None
    # The session is still startable once the operator chooses to.
    resp = client.post(f"/control/freehand/trajectory/{sid}/start", headers=headers)
    assert resp.status_code == 200, resp.text


def test_trajectories_need_claim_enforcement(client, headers, mock_controller):
    mock_controller.claim_manager.disable_enforcement()
    resp = client.post("/control/freehand/trajectory", json={}, headers=headers)
    assert resp.status_code == 412 and resp.json()["detail"]["error"] == "claim_enforcement_off"


def test_session_survives_a_claim_token_rotation(client, headers):
    sid = upload(client, headers)
    rotated = client.post("/control/claim", json={"owner": "t", "session_id": "s1"}).json()["claim_token"]
    assert rotated != headers["X-Claim-Token"]
    resp = client.post(f"/control/freehand/trajectory/{sid}/start", headers={"X-Claim-Token": rotated})
    assert resp.status_code == 200, resp.text


def test_start_refuses_an_arm_that_is_not_idle_or_has_warnings(client, headers, mock_controller):
    sid = upload(client, headers)
    mock_controller.arm.state = 1
    resp = client.post(f"/control/freehand/trajectory/{sid}/start", headers=headers)
    assert resp.status_code == 412 and resp.json()["detail"]["error"] == "arm_not_idle"
    mock_controller.arm.state = 2
    mock_controller.arm.warn_code = 7
    resp = client.post(f"/control/freehand/trajectory/{sid}/start", headers=headers)
    assert resp.status_code == 412 and resp.json()["detail"]["error"] == "arm_not_ready"
    assert not mock_controller.enter_motion.called


def test_start_waits_for_a_force_torque_tare(client, headers, mock_controller):
    sid = upload(client, headers)
    api._ft_tare_busy.set()
    try:
        resp = client.post(f"/control/freehand/trajectory/{sid}/start", headers=headers)
    finally:
        api._ft_tare_busy.clear()
    assert resp.status_code == 409 and resp.json()["detail"]["error"] == "force_torque_tare_running"


def test_graph_reads_connect_and_log_are_refused_during_a_run(client, headers, mock_controller, backend):
    backend.gate_at = 20
    sid = upload(client, headers)
    client.post(f"/control/freehand/trajectory/{sid}/start", headers=headers)
    assert backend.reached_gate.wait(5)
    for method, path, body in (("get", "/graph/nearest", None),
                               ("post", "/control/graph/recover_to", {"node_id": "n_home"}),
                               ("post", "/control/graph/gripper", {"state": "empty"}),
                               ("post", "/control/graph/pose", {}),
                               ("post", "/connect", {}),
                               ("get", f"/control/freehand/trajectory/{sid}/log", None)):
        kwargs = {"headers": headers}
        if body is not None:
            kwargs["json"] = body
        resp = getattr(client, method)(path, **kwargs)
        assert resp.status_code == 409 and resp.json()["detail"]["error"] == "trajectory_running", path
    client.post(f"/control/freehand/trajectory/{sid}/cancel", headers=headers)
    backend.gate.set()
    wait_state(client, sid, {"cancelled", "failed", "completed"})


def test_chunk_upload_also_answers_post_for_the_dashboard_proxy(client, headers):
    sid = client.post("/control/freehand/trajectory", json={}, headers=headers).json()["session_id"]
    resp = client.post(f"/control/freehand/trajectory/{sid}/chunks/0",
                       json={"points": points_j1(), "final": True}, headers=headers)
    assert resp.status_code == 200 and resp.json()["accepted"]
