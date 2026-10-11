"""Offline tests for the UR joint-trajectory routes (trajectory/routes.py).

Reuses the motion fixture robot (test_motion.py) with servoJ added: a
servoJ target is reached at once, so the measured joints follow the stream.
Limits are the synthetic fixture values. No SDK, socket or robot is touched.
"""

import math
import time

import pytest
from fastapi.testclient import TestClient

from robot_motion.app import create_app
from robot_motion.config import Settings
from tests_robot_motion.test_motion import (
    HEADERS,
    MOTION,
    OPERATOR,
    SECRET,
    Control,
    Rig,
    connected,
    wait_for,
)

BASE = "/control/freehand/trajectory"
TERMINAL = ("completed", "cancelled", "stopped", "failed", "expired")


class ServoControl(Control):
    def __init__(self, world):
        super().__init__(world)
        self.servo = []
        self.servo_stops = []
        self.servo_delay_s = 0.0

    def servoJ(self, q, speed, acceleration, time_s, lookahead_time, gain):
        self.servo.append((list(q), time_s, lookahead_time, gain))
        with self.world.lock:
            self.world.q = list(q)
        if self.servo_delay_s:
            time.sleep(self.servo_delay_s)
        return True

    def servoStop(self, deceleration):
        self.servo_stops.append(deceleration)
        return True


def config(tmp_path, **trajectory):
    return Settings(
        driver="ur",
        model="ur5e",
        observe=True,
        robot_host="robot.invalid",
        ur_transport="rtde",
        control_enabled=True,
        control={
            "authorized_operators": [OPERATOR],
            "motion": MOTION,
            # A STOP while servoing brakes under the SDK lock: 20 deg/s at
            # 90 deg/s^2 needs a watchdog below 2.25 Hz (config rule).
            "watchdog_hz": 2,
            "trajectory": {"enabled": True, "log_dir": str(tmp_path), **trajectory},
        },
    )


class TrajectoryRig(Rig):
    def __init__(self, settings):
        super().__init__(settings)
        self.control = ServoControl(self.world)

    def client(self):
        return TestClient(
            create_app(self.config, observer=self.observer, control_session_factory=self.session,
                       edge_secret=SECRET)
        )


def ramp(j1_end=3.0, duration=1.0, n=3):
    return [
        {"t": duration * i / (n - 1), "joints_deg": [j1_end * i / (n - 1), 0, 0, 0, 0, 0]}
        for i in range(n)
    ]


def run(client, held, points, wait=True):
    created = client.post(BASE, json={}, headers=held)
    assert created.status_code == 201, created.text
    sid = created.json()["session_id"]
    chunk = client.put(f"{BASE}/{sid}/chunks/0", json={"points": points, "final": True}, headers=held)
    assert chunk.status_code == 200, chunk.text
    started = client.post(f"{BASE}/{sid}/start", headers=held)
    if wait and started.status_code == 200:
        wait_for(lambda: state(client, sid) in TERMINAL, timeout=10, what="trajectory end")
    return sid, started


def state(client, sid):
    return client.get(f"{BASE}/{sid}", headers=HEADERS).json()["state"]


@pytest.fixture
def rig(tmp_path):
    return TrajectoryRig(config(tmp_path))


def test_limits_come_from_the_motion_block(rig):
    with rig.client() as client:
        body = client.get(f"{BASE}/limits", headers=HEADERS).json()
    assert body["enabled"] is True and body["backend"] == "ur_servoj"
    assert body["num_joints"] == 6
    assert body["joint_limits_deg"] == [[-90.0, 90.0]] * 6
    assert body["max_joint_speed_deg_s"] == 20 and body["max_joint_acc_deg_s2"] == 40
    assert body["workspace_mm"]["x"] == [150, 450]
    assert body["rate_hz"] == {"default": 125.0, "min": 50.0, "max": 250.0}


def test_disabled_block_refuses_create(tmp_path):
    rig = TrajectoryRig(config(tmp_path, enabled=False))
    with rig.client() as client:
        held = connected(client)
        response = client.post(BASE, json={}, headers=held)
    assert response.status_code == 412
    assert response.json()["detail"]["error"] == "trajectory_disabled"


def test_create_needs_the_claim(rig):
    with rig.client() as client:
        response = client.post(BASE, json={}, headers=HEADERS)
    assert response.status_code == 423


def test_validate_needs_a_session_then_checks_the_workspace(rig):
    with rig.client() as client:
        refused = client.post(f"{BASE}/validate", json={"points": ramp()}, headers=HEADERS)
        assert refused.status_code == 409
        held = connected(client)
        ok = client.post(f"{BASE}/validate", json={"points": ramp()}, headers=held)
        assert ok.status_code == 200, ok.text
        # J1 60 deg puts the toy TCP at x = 614 mm, outside the 450 mm box,
        # though every joint and the speed are within limits.
        far = client.post(f"{BASE}/validate", json={"points": ramp(60.0, 12.0)}, headers=held)
        assert far.status_code == 422
        assert far.json()["detail"]["report"]["errors"][0]["code"] == "path_leaves_workspace"
        fast = client.post(f"{BASE}/validate", json={"points": ramp(30.0, 1.0)}, headers=held)
        assert fast.status_code == 422
        codes = {e["code"] for e in fast.json()["detail"]["report"]["errors"]}
        assert "joint_speed_too_high" in codes


def test_a_run_streams_servoj_and_frees_the_arm(rig):
    with rig.client() as client:
        held = connected(client)
        sid, started = run(client, held, ramp())
        assert started.status_code == 200, started.text
        body = client.get(f"{BASE}/{sid}", headers=HEADERS).json()
        assert body["state"] == "completed", body
        assert body["log_available"] is True
        assert body["measured"]["joints_deg"][0] == pytest.approx(3.0, abs=0.05)
        assert body["tracking"]["samples"] > 10
        status = client.get("/status").json()
        assert status["details"]["trajectory"]["session"]["state"] == "completed"
        assert "arm.move_joints" in status["allowed_actions"]
        log = client.get(f"{BASE}/{sid}/log", headers=HEADERS)
        assert log.status_code == 200
        assert not rig.control.stops     # no stopJ on a clean finish (shutdown sends one)
    servo = rig.control.servo
    assert 120 <= len(servo) <= 140   # 1 s at 125 Hz plus the end
    q, dt, lookahead, gain = servo[-1]
    assert dt == pytest.approx(1 / 125) and lookahead == 0.03 and gain == 1000
    assert math.degrees(q[0]) == pytest.approx(3.0, abs=1e-6)
    assert rig.control.servo_stops   # left servo mode at the end


def test_moves_and_disconnect_are_refused_while_running_and_stop_halts_it(rig):
    rig.control.servo_delay_s = 0.002
    with rig.client() as client:
        held = connected(client)
        sid, started = run(client, held, ramp(3.0, 4.0), wait=False)
        assert started.status_code == 200, started.text
        wait_for(lambda: len(rig.control.servo) > 20, what="streaming")
        move = client.post("/control/freehand/joints", json={"angles": [1, 0, 0, 0, 0, 0]}, headers=held)
        assert move.status_code == 409 and move.json()["detail"]["error"] == "motion_busy"
        assert client.post("/disconnect", headers=held).status_code == 409
        again = client.post(BASE, json={}, headers=held)
        assert again.status_code == 409   # one session per arm
        stop = client.post("/control/stop", headers=HEADERS).json()
        assert stop["stop_requested"] is True
        wait_for(lambda: state(client, sid) in TERMINAL, what="stopped")
        sent = len(rig.control.servo)
        time.sleep(0.1)
        assert len(rig.control.servo) == sent   # nothing after STOP
        assert state(client, sid) == "stopped"
        # servoStop, never stopJ: a stopJ while servoing aborts the script.
        assert rig.control.servo_stops and not rig.control.stops
        # STOP latches like any other: moves wait for Clear errors.
        status = client.get("/status").json()
        assert status["details"]["control_session"]["latched"] == "Stop requested"
        assert status["details"]["control_session"]["busy"] is False


def test_start_is_refused_in_manual_mode_and_outside_the_box(rig):
    with rig.client() as client:
        held = connected(client)
        on = client.post("/robot/manual", json={"enable": True}, headers=held)
        assert on.status_code == 200, on.text
        _, started = run(client, held, ramp())
        assert started.status_code == 412
        assert started.json()["detail"]["error"] == "manual_mode"
        client.post("/robot/manual", json={"enable": False}, headers=held)
        # The refused session is still open (created): cancel it first.
        sid = client.get("/status").json()["details"]["trajectory"]["session"]["session_id"]
        client.post(f"{BASE}/{sid}/cancel", headers=held)
        _, far = run(client, held, ramp(60.0, 12.0))
        assert far.status_code == 422
        assert far.json()["detail"]["report"]["errors"][0]["code"] == "path_leaves_workspace"
    assert not rig.control.servo


def test_force_guard_stops_the_run(tmp_path):
    motion = {**MOTION, "force_guard_n": 10}
    settings = config(tmp_path)
    control = settings.control.model_copy(update={"motion": type(settings.control.motion).model_validate(motion)})
    rig = TrajectoryRig(settings.model_copy(update={"control": control}))
    rig.control.servo_delay_s = 0.002

    def push(world):
        if len(rig.control.servo) > 30:
            world.wrench = [30.0, 2.0, 3.0, 0.0, 0.0, 0.0]

    rig.world.on_tick = push
    with rig.client() as client:
        held = connected(client)
        sid, started = run(client, held, ramp(3.0, 2.0))
        assert started.status_code == 200, started.text
        body = client.get(f"{BASE}/{sid}", headers=HEADERS).json()
        assert rig.control.servo_stops and not rig.control.stops   # emergency stop while servoing
    assert body["state"] == "failed", body
    assert "force guard" in body["reason"]


def test_trajectories_are_refused_when_kinematics_cannot_be_verified(rig):
    rig.control.getTCPOffset = lambda: [0.0] * 6
    with rig.client() as client:
        held = connected(client)
        check = client.post(f"{BASE}/validate", json={"points": ramp()}, headers=held)
        assert check.status_code == 412
        assert check.json()["detail"]["error"] == "kinematics_unverified"
        _, started = run(client, held, ramp())
        assert started.status_code == 412
        assert started.json()["detail"]["error"] == "kinematics_unverified"
    assert not rig.control.servo
