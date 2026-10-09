"""Offline checks of tools/servo_mode_probe.py (no arm, nothing moves)."""

from __future__ import annotations

import argparse
import importlib.util
import os
import sys

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, ROOT)


@pytest.fixture(scope="module")
def probe():
    spec = importlib.util.spec_from_file_location("servo_mode_probe", os.path.join(ROOT, "tools", "servo_mode_probe.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


Q0 = [10.0, -20.0, -30.0, 40.0, 5.0]


def test_wiggle_returns_to_start_and_stays_within_amplitude(probe):
    pts = probe.wiggle_points(Q0, 5.0, 10.0, 20.0, 2)
    assert pts[0]["joints_deg"] == Q0 and pts[-1]["joints_deg"] == Q0
    j1 = [p["joints_deg"][0] - Q0[0] for p in pts]
    j5 = [p["joints_deg"][4] - Q0[4] for p in pts]
    assert max(j1) == pytest.approx(5.0, abs=0.05) and min(j1) == pytest.approx(-5.0, abs=0.05)
    assert max(abs(v) for v in j5) <= 10.0 + 1e-9
    assert all(p["joints_deg"][1:4] == Q0[1:4] for p in pts)
    times = [p["t"] for p in pts]
    assert all(b - a >= 0.01 - 1e-9 for a, b in zip(times, times[1:]))
    peak = max(abs(v) for p in pts for v in p["velocities_deg_s"])
    assert peak == pytest.approx(20.0, rel=0.01)


@pytest.mark.parametrize("backend,rate", [("servoj", 100.0), ("servoj", 200.0),
                                          ("online_planning", 20.0), ("online_planning", 50.0)])
def test_every_planned_run_validates(probe, backend, rate):
    pts = probe.wiggle_points(Q0, 5.0, 10.0, 20.0, 2)
    settings = probe.TrajectorySettings.from_mapping({
        "enabled": True, "backend": backend,
        "servo_rate_hz": rate if backend == "servoj" else 100.0,
        "online_rate_hz": rate if backend == "online_planning" else 50.0})
    mgr = probe.TrajectoryManager(settings)
    session = mgr.create(key=None, owner="t", num_joints=5, rate_hz=rate)
    result = mgr.add_chunk(session.id, 0, None, pts, final=True,
                           limits=mgr.validation_limits(session, probe.XARM5_LIMITS, 60.0))
    assert result["report"]["valid"]


def test_analysis_recovers_lag_and_tracking_from_bursty_reports(probe):
    pts = probe.wiggle_points(Q0, 5.0, 10.0, 20.0, 1)
    plan = probe.JointTrajectory([probe.JointPoint.from_mapping(p) for p in pts])
    # Commands every 10 ms; the arm follows exactly 20 ms behind. Reports
    # are sampled every 10 ms but delivered five at a time, 50 ms apart.
    sent = [k * 0.01 for k in range(int(plan.duration / 0.01) + 1)]
    commanded = list(zip(*[plan.sample(t)[0] for t in sent]))
    frames, received = [], []
    for i in range(-20, len(sent) + 40):
        t = i * 0.01
        frames.append(plan.sample(t - 0.02)[0])
        received.append((i // 5 + 1) * 0.05 + 0.003)
    record = {"ticks": {"sent_s": sent, "commanded_deg": [list(c) for c in commanded]},
              "reported": {"received_s": received, "joints_deg": frames}}
    result = probe.analyse(pts, record, 5)
    # 20 ms of arm lag plus the freshest frame's 13 ms delivery delay.
    assert 30 <= result["lag_ms"] <= 40
    assert result["tracking_deg"]["max"] < 0.1
    assert result["single_frame_holds"] == 0
    assert result["smoothness_50ms"]["peak_speed_deg_s"] == pytest.approx(20.0, rel=0.1)


class FakeArm:
    """Enough of XArmAPI for one probe run."""

    def __init__(self):
        self.q = list(Q0) + [0.0, 0.0]
        self.mode, self.state = 0, 2
        self.error_code = self.warn_code = 0
        self.connected = True
        self.version = "fake"
        self.calls = []
        self._timeout = 2.0

    def get_servo_angle(self, is_radian=False):
        return 0, list(self.q)

    def set_timeout(self, value):
        if value:
            self._timeout = value
        return self._timeout

    def set_mode(self, mode):
        self.calls.append(("set_mode", mode))
        self.mode = mode
        return 0

    def set_state(self, state):
        self.calls.append(("set_state", state))
        return 0

    def set_servo_angle_j(self, angles, is_radian=False):
        self.q[:5] = list(angles)
        return 0

    def set_servo_angle(self, angle, speed, mvacc, wait, is_radian=False):
        self.q[:5] = list(angle)
        return 0

    def emergency_stop(self):
        self.calls.append(("emergency_stop",))
        self.state = 4


def test_one_short_run_against_a_fake_arm(probe, tmp_path):
    arm = FakeArm()
    args = argparse.Namespace(max_acc=500.0, max_speed=60.0, stop_at=0.25)
    pts = probe.wiggle_points(Q0, 1.0, 2.0, 10.0, 1)   # 1.5 s
    result = probe.one_run(arm, "127.0.0.1", args, "servoj", 100.0, pts, "full", tmp_path, 1)
    assert result["state"] == "completed", result
    assert ("set_mode", 1) in arm.calls and arm.calls[-2:] == [("set_mode", 0), ("set_state", 0)]
    assert arm.q[:5] == pytest.approx(Q0)
    assert os.path.exists(result["log"])
