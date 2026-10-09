"""End-to-end check of tools/trajectory_http_check.py against the real API
app served on a local port, with a mock controller, the fake SDK backend
and a fake clock (no arm, nothing moves)."""

from __future__ import annotations

import importlib.util
import json
import os
import socket
import sys
import threading
import time

import pytest
import uvicorn

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, ROOT)

import src.core.xarm_api_server as api
from test.test_trajectory_api import mock_controller, settings, fake, backend  # noqa: F401  fixtures


@pytest.fixture
def server(monkeypatch, mock_controller, settings, backend):
    mock_controller.get_current_position.return_value = [0.0] * 6
    mock_controller.has_track.return_value = False
    mock_controller.get_gripper_position.return_value = None
    # Real time here: the script polls at 10 Hz and must see each run running.
    from src.core.trajectory_executor import TrajectoryManager
    mock_controller._trajectory_manager = TrajectoryManager(settings)
    mgr = api.get_trajectory_manager(mock_controller)
    mock_controller.stop_motion.side_effect = lambda: (mgr.notify_hard_stop("stop"), True)[1]
    def move_joints(angles, *args, **kwargs):
        backend.q = list(angles[:5])          # the fake arm really goes home
        return True

    mock_controller.move_joints.side_effect = move_joints
    # Enough of a healthy, connected controller for /status to say "ready",
    # and a motion slot that /status can observe.
    from datetime import datetime, timezone
    from src.core.xarm_controller import ComponentState
    mock_controller.last_error_code = 0
    mock_controller.last_error = None
    mock_controller.alive = True
    mock_controller._recovering = False
    mock_controller.health_failure = None
    mock_controller._activity_since = datetime.now(timezone.utc)
    mock_controller.states = {name: ComponentState.ENABLED for name in ("connection", "arm")}
    mock_controller.has_gripper.return_value = False
    mock_controller.has_force_torque_sensor.return_value = False

    def enter():
        mock_controller._motion_in_progress = True

    def leave():
        mock_controller._motion_in_progress = False

    mock_controller.enter_motion.side_effect = enter
    mock_controller.exit_motion.side_effect = leave
    monkeypatch.setattr("src.core.xarm_api_server.controller", mock_controller)
    monkeypatch.setattr("src.core.xarm_api_server._trajectory_settings_cache", settings)
    monkeypatch.setattr("src.core.xarm_api_server.make_backend", lambda name, arm, timeout: backend)
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    srv = uvicorn.Server(uvicorn.Config(api.app, host="127.0.0.1", port=port, log_level="warning"))
    thread = threading.Thread(target=srv.run, daemon=True)
    thread.start()
    deadline = time.time() + 10
    while not srv.started and time.time() < deadline:
        time.sleep(0.05)
    yield f"http://127.0.0.1:{port}"
    srv.should_exit = True
    thread.join(5)


def test_the_stage3_check_runs_full_cancel_and_stop(server, monkeypatch, tmp_path, mock_controller):
    spec = importlib.util.spec_from_file_location("trajectory_http_check",
                                                  os.path.join(ROOT, "tools", "trajectory_http_check.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr("builtins.input", lambda prompt="": "yes")
    monkeypatch.setattr(sys, "argv", ["x", "--email", "t@example.org", "--no-login", "--base", server, "--cycles", "1"])
    real_stdout = sys.stdout
    try:
        module.main()
    finally:
        sys.stdout = real_stdout
    out = next((tmp_path / "logs" / "trajectory_http_check").glob("2*"))
    results = json.loads((out / "summary.json").read_text())
    assert [r["scenario"] for r in results] == ["full", "cancel", "stop"]
    assert [r["state"] for r in results] == ["completed", "cancelled", "stopped"]
    full = results[0]
    assert full["observed"]["positions_during_run"] == [409, "trajectory_running"]
    assert full["observed"]["status_trajectory_state"] in ("running", "stopping")
    assert full["observed"]["status_activity"] == "running"
    assert all(r["positions_after_run_http"] == 200 for r in results)
    assert results[1]["observed"]["cancel_http"] == 200 and results[2]["observed"]["stop_http"] == 200
    assert mock_controller.move_joints.call_count == 2          # home after cancel and after stop
    assert mock_controller.claim_manager.claimed_by() is None    # released
