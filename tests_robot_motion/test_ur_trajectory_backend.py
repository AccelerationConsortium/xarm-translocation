"""UR servoJ backend for streamed joint trajectories, against a fake ur_rtde
control interface and feedback sampler (no hardware)."""

from __future__ import annotations

import math

import pytest

from robot_motion.trajectory.executor import (
    COMPLETED, STOPPED, BackendError, StopRequested, TrajectoryManager, TrajectorySettings,
)
from robot_motion.trajectory.ur_backend import URServoJBackend

UR5E_LIMITS = [(-360, 360)] * 6
ZERO6 = [0.0] * 6


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def sleep(self, s):
        self.now += max(0.0, s)


class FakeControl:
    def __init__(self, feedback):
        self.feedback = feedback
        self.calls = []
        self.connected = True
        self.running = True

    def isConnected(self):
        return self.connected

    def isProgramRunning(self):
        return self.running

    def servoJ(self, q, speed, acc, time_s, lookahead, gain):
        self.calls.append(("servoJ", list(q), speed, acc, time_s, lookahead, gain))
        self.feedback.sample["joints_deg"] = tuple(math.degrees(v) for v in q)
        self.feedback.sample["received_monotonic_s"] = self.feedback.clock()
        return True

    def servoStop(self, decel):
        self.calls.append(("servoStop", decel))
        return True

    def stopJ(self, decel):
        self.calls.append(("stopJ", decel))
        return True


class FakeFeedback:
    def __init__(self, clock, joints=ZERO6):
        self.clock = clock
        self.sample = {"joints_deg": tuple(joints), "velocities_deg_s": ZERO6, "controller_connected": True,
                       "robot_mode": "RUNNING", "safety_mode": "NORMAL", "speed_fraction": 1.0,
                       "received_monotonic_s": clock()}

    def latest(self):
        self.sample["received_monotonic_s"] = max(self.sample["received_monotonic_s"], self.clock() - 0.01)
        return dict(self.sample)


def make(clock=None, **kw):
    clock = clock or Clock()
    fb = FakeFeedback(clock)
    control = FakeControl(fb)
    return URServoJBackend(control, fb, period_s=0.008, clock=clock, **kw), control, fb, clock


@pytest.mark.parametrize("change,needle", [
    (lambda fb, c: fb.sample.update(robot_mode="IDLE"), "robot mode IDLE"),
    (lambda fb, c: fb.sample.update(safety_mode="PROTECTIVE_STOP"), "safety mode PROTECTIVE_STOP"),
    (lambda fb, c: fb.sample.update(speed_fraction=0.5), "speed slider"),
    (lambda fb, c: fb.sample.update(velocities_deg_s=(1, 0, 0, 0, 0, 0)), "moving"),
    (lambda fb, c: setattr(c, "running", False), "control script"),
    (lambda fb, c: setattr(c, "connected", False), "control link down"),
])
def test_prepare_refuses_an_arm_that_is_not_ready(change, needle):
    backend, control, fb, _ = make()
    change(fb, control)
    with pytest.raises(BackendError, match=needle):
        backend.prepare()
    assert not [c for c in control.calls if c[0] == "servoJ"]


def test_prepare_refuses_stale_feedback():
    backend, control, fb, clock = make()
    fb.latest = lambda: dict(fb.sample, received_monotonic_s=clock() - 1.0)
    with pytest.raises(BackendError, match="old"):
        backend.prepare()


def test_prepare_honours_a_stop():
    backend, *_ = make()
    with pytest.raises(StopRequested):
        backend.prepare(abort=lambda: True)


def test_send_converts_to_radians_and_passes_servo_parameters():
    backend, control, _, _ = make(lookahead_s=0.1, gain=300)
    assert backend.send([90, 0, 0, 0, 0, -45], 10, 100) == 0
    name, q, speed, acc, t, lookahead, gain = control.calls[-1]
    assert name == "servoJ" and q[0] == pytest.approx(math.pi / 2) and q[5] == pytest.approx(-math.pi / 4)
    assert (speed, acc, t, lookahead, gain) == (0.0, 0.0, 0.008, 0.1, 300)


def test_finish_and_emergency_stop():
    backend, control, _, _ = make(stop_decel_deg_s2=90, emergency_decel_deg_s2=720)
    backend.finish()
    assert control.calls == []                 # nothing streamed: nothing to stop
    backend.send(ZERO6, 1, 1)
    backend.finish()
    assert control.calls[-1] == ("servoStop", pytest.approx(math.radians(90)))
    n = len(control.calls)
    backend.emergency_stop()
    # stopJ only (the wrapper sends it without waiting); a blocking
    # servoStop while braking would starve the watchdog.
    assert [c[0] for c in control.calls[n:]] == ["stopJ"]


def test_fault_reports_a_protective_stop_and_a_dead_script():
    backend, control, fb, _ = make()
    assert backend.fault(None, False) == (None, False)
    fb.sample["safety_mode"] = "PROTECTIVE_STOP"
    assert backend.fault(None, False)[0] == "safety mode PROTECTIVE_STOP"
    fb.sample["safety_mode"] = "NORMAL"
    control.running = False
    assert backend.fault(None, False)[0] == "control script stopped"


def test_parameter_bounds():
    with pytest.raises(ValueError):
        make(lookahead_s=0.5)
    with pytest.raises(ValueError):
        make(gain=50)


def _points(end=5.0, duration=1.0, n=11):
    pts = []
    for i in range(n):
        s = i / (n - 1)
        q = end * (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5)
        v = end * (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4) / duration
        a = end * (60 * s - 180 * s ** 2 + 120 * s ** 3) / duration ** 2
        pts.append({"t": duration * s, "joints_deg": [q, 0, 0, 0, 0, 0],
                    "velocities_deg_s": [v, 0, 0, 0, 0, 0], "accelerations_deg_s2": [a, 0, 0, 0, 0, 0]})
    return pts


def _run(tmp_path, backend, clock, stop_after=None):
    mgr = TrajectoryManager(TrajectorySettings.from_mapping({"enabled": True, "servo_rate_hz": 125.0,
                                                             "realtime_report": False,
                                                             "log_dir": str(tmp_path)}),
                            clock=clock, sleep=clock.sleep)
    session = mgr.create(key="k", owner="t", num_joints=6)
    limits = mgr.validation_limits(session, UR5E_LIMITS, 10.0)
    limits.max_joint_acc_deg_s2 = 30.0
    mgr.add_chunk(session.id, 0, "k", _points(), final=True, limits=limits)
    if stop_after is not None:
        original = backend.control.servoJ

        def servo(*args):
            if len([c for c in backend.control.calls if c[0] == "servoJ"]) == stop_after:
                mgr.notify_hard_stop("stop")
            return original(*args)

        backend.control.servoJ = servo
    mgr.start(session.id, "k", limits=limits, start_joints=ZERO6, backend=backend,
              watch=lambda: None, on_finish=lambda s: None)
    assert mgr.wait_idle(10)
    return session


def test_full_run_on_the_ur_backend(tmp_path):
    clock = Clock()
    backend, control, fb, _ = make(clock)
    # 5 deg in 1 s peaks at 9.4 deg/s and 29 deg/s^2: inside the UR placeholder limits.
    session = _run(tmp_path, backend, clock)
    assert session.state == COMPLETED, session.reason
    servo = [c for c in control.calls if c[0] == "servoJ"]
    assert len(servo) == 125 and servo[-1][1][0] == pytest.approx(math.radians(5.0), abs=1e-9)
    assert control.calls[-1][0] == "servoStop"
    assert session.final["mode_restored"] is True


def test_stop_during_a_ur_run_sends_nothing_more(tmp_path):
    clock = Clock()
    backend, control, fb, _ = make(clock)
    session = _run(tmp_path, backend, clock, stop_after=30)
    assert session.state == STOPPED
    assert len([c for c in control.calls if c[0] == "servoJ"]) == 31
    assert "servoStop" not in [c[0] for c in control.calls]   # the service's STOP stops the arm
