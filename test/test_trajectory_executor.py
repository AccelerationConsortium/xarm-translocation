"""Joint-trajectory sessions and the fixed-rate executor, against a fake
clock and a fake SDK backend (no hardware, no real time)."""

from __future__ import annotations

import math
import os
import socket
import struct
import sys
import threading
import time

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import src.core.trajectory_executor as executor_module
from src.core.joint_trajectory import JointLimits
from src.core.trajectory_executor import (
    CANCELLED, COMPLETED, CREATED, EXPIRED, FAILED, STOPPED, BackendError, RealtimeJointMonitor,
    StopRequested, TrajectoryError, TrajectoryManager, TrajectorySettings, XArmOnlinePlanningBackend,
    XArmServoJBackend, _health_fault, _sample_fault,
)

JOINT_LIMITS_5 = [(-360, 360), (-118, 120), (-225, 11), (-97, 180), (-360, 360)]
ZERO = [0.0] * 5
VMAX = 60.0
AMAX = 500.0


class FakeClock:
    def __init__(self):
        self.now = 1000.0
        self.lock = threading.Lock()

    def clock(self):
        with self.lock:
            return self.now

    def sleep(self, seconds):
        with self.lock:
            self.now += max(0.0, seconds)

    def advance(self, seconds):
        self.sleep(seconds)


class FakeBackend:
    name = "servoj"
    mode = 1

    def __init__(self, clock, start=ZERO, round_trip=0.0003, stalls=None, fail_at=None,
                 fault_at=None, gate_at=None, prepare_error=None, on_prepare=None, on_read=None,
                 on_finish=None, fault_state=22):
        self.clock = clock
        self.q = list(start)
        self.round_trip = round_trip
        self.stalls = stalls or {}
        self.fail_at = fail_at
        self.fault_at = fault_at
        self.gate_at = gate_at
        self.gate = threading.Event()
        self.reached_gate = threading.Event()
        self.prepare_error = prepare_error
        self.on_prepare = on_prepare
        self.on_read = on_read
        self.on_finish_hook = on_finish
        self.fault_state = fault_state
        self.reads = 0
        self.mode_changed = False
        self.sent = []
        self.speeds = []
        self.calls = []
        self.state = 2
        self.arm_mode = 0
        self.error_code = 0

    def prepare(self, abort=lambda: False):
        self.calls.append("prepare")
        if self.prepare_error:
            self.mode_changed = True
            raise RuntimeError(self.prepare_error)
        self.mode_changed = True
        if self.on_prepare:
            self.on_prepare(self)
        if abort():
            raise StopRequested()
        self.arm_mode = self.mode

    def send(self, q, speed, acc):
        if self.gate_at is not None and len(self.sent) == self.gate_at:
            self.reached_gate.set()
            self.gate.wait(5)
        self.clock.advance(self.round_trip + self.stalls.get(len(self.sent), 0.0))
        self.sent.append(list(q))
        self.speeds.append(speed)
        self.q = list(q)
        if self.fault_at is not None and len(self.sent) == self.fault_at:
            if self.fault_state == 22:
                self.error_code = 22
            else:
                self.state = self.fault_state
        if self.fail_at is not None and len(self.sent) == self.fail_at:
            return 9
        return 0

    def finish(self):
        self.calls.append("finish")
        self.arm_mode = 0
        if self.on_finish_hook:
            self.on_finish_hook(self)

    def restore(self):
        self.calls.append("restore")

    def emergency_stop(self):
        self.calls.append("estop")
        self.state = 4

    def read_joints(self, n):
        self.reads += 1
        if self.on_read:
            self.on_read(self)
        return list(self.q)

    def health(self):
        return {"connected": True, "state": self.state, "mode": self.arm_mode,
                "error_code": self.error_code, "warn_code": 0}


def manager(fake, tmp_path, **overrides):
    kw = dict(enabled=True, log_dir=str(tmp_path / "logs"))
    kw.update(overrides)
    return TrajectoryManager(TrajectorySettings.from_mapping(kw), clock=fake.clock, sleep=fake.sleep)


def points_j1(end=5.0, duration=1.0, n=11):
    """Quintic-friendly J1 move with velocities and accelerations (min-jerk)."""
    pts = []
    for i in range(n):
        s = i / (n - 1)
        t = duration * s
        q = end * (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5)
        v = end * (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4) / duration
        a = end * (60 * s - 180 * s ** 2 + 120 * s ** 3) / duration ** 2
        pts.append({"t": round(t, 9), "joints_deg": [q, 0, 0, 0, 0],
                    "velocities_deg_s": [v, 0, 0, 0, 0], "accelerations_deg_s2": [a, 0, 0, 0, 0]})
    return pts


def run(mgr, backend, pts=None, chunks=1, start=ZERO, watch=lambda: None, token="tok", wait=True,
        generation=None):
    pts = pts if pts is not None else points_j1()
    session = mgr.create(key=token, owner="t", num_joints=5)
    limits = mgr.validation_limits(session, JOINT_LIMITS_5, VMAX)
    size = math.ceil(len(pts) / chunks)
    parts = [pts[i:i + size] for i in range(0, len(pts), size)]
    for seq, part in enumerate(parts):
        mgr.add_chunk(session.id, seq, token, part, final=(seq == len(parts) - 1), limits=limits)
    finished = []
    mgr.start(session.id, token, limits=limits, start_joints=start, backend=backend, watch=watch,
              on_finish=finished.append, stop_generation=generation)
    if wait:
        assert mgr.wait_idle(10)
    return session, finished


def max_step(sent):
    return max(max(abs(a - b) for a, b in zip(x, y)) for x, y in zip(sent, sent[1:]))


# ── Normal runs ──────────────────────────────────────────────────────


def test_complete_run_streams_restores_mode_and_releases(tmp_path):
    fake = FakeClock()
    backend = FakeBackend(fake)
    mgr = manager(fake, tmp_path)
    session, finished = run(mgr, backend)
    assert session.state == COMPLETED and session.reason is None
    assert finished == [session]
    assert backend.calls[0] == "prepare" and backend.calls[-2:] == ["finish", "restore"]
    assert "estop" not in backend.calls
    assert backend.sent[-1][0] == pytest.approx(5.0, abs=1e-9)
    # 1 s at 100 Hz, no lead-in.
    assert len(backend.sent) == 100
    assert max_step(backend.sent) <= VMAX * 0.01
    assert session.started_at is not None
    assert session.final["mode_restored"] is True and session.final["error_to_last_target_deg"] == 0
    assert session.stats["servo"]["ticks"] == 100 and session.stats["servo"]["sdk_errors"] == 0
    assert os.path.exists(session.log_path)
    assert not mgr.running


def test_whole_and_chunked_uploads_send_identical_streams(tmp_path):
    streams = []
    for chunks in (1, 3, 7):
        fake = FakeClock()
        backend = FakeBackend(fake)
        run(manager(fake, tmp_path), backend, chunks=chunks)
        streams.append(backend.sent)
    assert streams[0] == streams[1] == streams[2]


def test_lead_in_moves_smoothly_from_the_measured_pose(tmp_path):
    fake = FakeClock()
    measured = [0.3, 0, -0.2, 0, 0]
    backend = FakeBackend(fake, start=measured)
    session, _ = run(manager(fake, tmp_path), backend, start=measured)
    assert session.state == COMPLETED
    assert session.lead_in_s > 0
    # One tick into the lead-in: close to the measured pose, within one step.
    assert abs(backend.sent[0][0] - 0.3) <= VMAX * 0.01
    assert backend.sent[0][0] < 0.3
    assert max_step(backend.sent) <= VMAX * 0.01 + 1e-9
    assert session.started_at is not None and session.started_at >= session.began_at


def test_online_planning_samples_ahead_and_sends_speed(tmp_path):
    fake = FakeClock()
    backend = FakeBackend(fake)
    backend.name, backend.mode = "online_planning", 6
    mgr = manager(fake, tmp_path, backend="online_planning", online_rate_hz=50.0)
    session, _ = run(mgr, backend)
    assert session.state == COMPLETED and session.rate_hz == 50.0
    assert len(backend.sent) == 50
    # Two command periods (40 ms) of lookahead: the first target is the
    # trajectory at 60 ms, not 20 ms.
    first_tau = 0.02 + 0.04
    s = first_tau
    expected = 5.0 * (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5)
    assert backend.sent[0][0] == pytest.approx(expected, rel=1e-6)
    assert all(1.0 <= v <= VMAX for v in backend.speeds)


# ── Uploads ──────────────────────────────────────────────────────────


def test_chunk_rules(tmp_path):
    fake = FakeClock()
    mgr = manager(fake, tmp_path)
    pts = points_j1()
    s = mgr.create(key="tok", owner="t", num_joints=5)
    limits = mgr.validation_limits(s, JOINT_LIMITS_5, VMAX)
    first = mgr.add_chunk(s.id, 0, "tok", pts[:4], final=False, limits=limits)
    assert first["accepted"] and not first["duplicate"]
    again = mgr.add_chunk(s.id, 0, "tok", pts[:4], final=False, limits=limits)
    assert again["duplicate"] is True and len(s.raw_points) == 4
    with pytest.raises(TrajectoryError) as e:
        mgr.add_chunk(s.id, 0, "tok", pts[:3], final=False, limits=limits)
    assert e.value.detail["error"] == "chunk_conflict"
    with pytest.raises(TrajectoryError) as e:
        mgr.add_chunk(s.id, 2, "tok", pts[4:], final=True, limits=limits)
    assert e.value.detail["error"] == "chunk_out_of_order" and e.value.detail["expected_seq"] == 1
    with pytest.raises(TrajectoryError) as e:
        mgr.add_chunk(s.id, 1, "other", pts[4:], final=True, limits=limits)
    assert e.value.status_code == 423
    with pytest.raises(TrajectoryError) as e:
        mgr.start(s.id, "tok", limits=limits, start_joints=ZERO, backend=None,
                  watch=lambda: None, on_finish=lambda _: None)
    assert e.value.detail["error"] == "trajectory_incomplete"
    mgr.add_chunk(s.id, 1, "tok", pts[4:], final=True, limits=limits)
    with pytest.raises(TrajectoryError) as e:
        mgr.add_chunk(s.id, 2, "tok", pts[-1:], final=True, limits=limits)
    assert e.value.detail["error"] == "session_closed"


def test_invalid_chunk_is_rejected_with_the_report(tmp_path):
    fake = FakeClock()
    mgr = manager(fake, tmp_path)
    s = mgr.create(key="tok", owner="t", num_joints=5)
    limits = mgr.validation_limits(s, JOINT_LIMITS_5, VMAX)
    with pytest.raises(TrajectoryError) as e:
        mgr.add_chunk(s.id, 0, "tok", [{"t": 0, "joints_deg": ZERO}, {"t": 0.1, "joints_deg": [90, 0, 0, 0, 0]}],
                      final=True, limits=limits)
    assert e.value.status_code == 422
    assert "joint_speed_too_high" in [x["code"] for x in e.value.detail["report"]["errors"]]
    assert s.next_seq == 0


def test_start_checks_the_measured_start(tmp_path):
    fake = FakeClock()
    mgr = manager(fake, tmp_path)
    s = mgr.create(key="tok", owner="t", num_joints=5)
    limits = mgr.validation_limits(s, JOINT_LIMITS_5, VMAX)
    mgr.add_chunk(s.id, 0, "tok", points_j1(), final=True, limits=limits)
    with pytest.raises(TrajectoryError) as e:
        mgr.start(s.id, "tok", limits=limits, start_joints=[2.0, 0, 0, 0, 0], backend=FakeBackend(fake),
                  watch=lambda: None, on_finish=lambda _: None)
    assert e.value.status_code == 422
    assert [x["code"] for x in e.value.detail["report"]["errors"]] == ["start_joint_mismatch"]


def test_one_open_session_and_expiry(tmp_path):
    fake = FakeClock()
    mgr = manager(fake, tmp_path, unstarted_ttl_s=60)
    s = mgr.create(key="tok", owner="t", num_joints=5)
    with pytest.raises(TrajectoryError) as e:
        mgr.create(key="tok", owner="t", num_joints=5)
    assert e.value.detail["error"] == "session_open"
    fake.advance(61)
    assert mgr.get(s.id).state == EXPIRED
    assert mgr.create(key="tok", owner="t", num_joints=5).id != s.id


def test_rate_and_tolerance_bounds(tmp_path):
    fake = FakeClock()
    mgr = manager(fake, tmp_path)
    for bad in (10.0, 201.0, math.inf):
        with pytest.raises(TrajectoryError) as e:
            mgr.create(key="t", owner="t", num_joints=5, rate_hz=bad)
        assert e.value.detail["error"] == "rate_out_of_range"
    with pytest.raises(TrajectoryError):
        mgr.create(key="t", owner="t", num_joints=5, start_tolerance_deg=5.0)


# ── Stops and faults ─────────────────────────────────────────────────


def test_cancel_is_a_constrained_stop_along_the_path(tmp_path):
    fake = FakeClock()
    backend = FakeBackend(fake, gate_at=40)
    mgr = manager(fake, tmp_path)
    session, finished = run(mgr, backend, pts=points_j1(end=20.0, duration=2.0, n=21), wait=False)
    assert backend.reached_gate.wait(5)
    mgr.cancel(session.id, "tok")
    backend.gate.set()
    assert mgr.wait_idle(10)
    assert session.state == CANCELLED and finished == [session]
    assert "finish" in backend.calls and "estop" not in backend.calls
    assert len(backend.sent) < 200   # stopped well before the 2 s end
    j1 = [q[0] for q in backend.sent]
    # Along the path: J1 never reverses, and it ends short of the target.
    assert all(b >= a - 1e-12 for a, b in zip(j1, j1[1:])) and j1[-1] < 20.0
    # The commanded speed falls to zero and acceleration stays within the limit.
    dt = 0.01
    v = [(b - a) / dt for a, b in zip(j1, j1[1:])]
    acc = [(b - a) / dt for a, b in zip(v, v[1:])]
    assert v[-1] == pytest.approx(0.0, abs=1e-6)
    assert max(abs(x) for x in acc[40:]) <= AMAX * 1.05


def test_stop_sends_nothing_more_and_keeps_the_stopped_state(tmp_path):
    fake = FakeClock()
    backend = FakeBackend(fake, gate_at=20)
    mgr = manager(fake, tmp_path)
    session, _ = run(mgr, backend, wait=False)
    assert backend.reached_gate.wait(5)
    mgr.notify_hard_stop("stop")
    backend.gate.set()
    assert mgr.wait_idle(10)
    assert session.state == STOPPED and session.reason == "stop"
    assert len(backend.sent) == 21          # the one in flight completes, nothing after
    assert "finish" not in backend.calls    # Clear errors restores the mode
    assert "estop" not in backend.calls     # the STOP route issues its own
    assert backend.calls[-1] == "restore"


def test_disconnect_emergency_stops(tmp_path):
    fake = FakeClock()
    backend = FakeBackend(fake, gate_at=20)
    mgr = manager(fake, tmp_path)
    session, _ = run(mgr, backend, wait=False)
    assert backend.reached_gate.wait(5)
    mgr.notify_hard_stop("disconnect")
    backend.gate.set()
    assert mgr.wait_idle(10)
    assert session.state == STOPPED and "estop" in backend.calls


def test_sdk_error_is_a_hard_failure(tmp_path):
    fake = FakeClock()
    backend = FakeBackend(fake, fail_at=30)
    session, finished = run(manager(fake, tmp_path), backend)
    assert session.state == FAILED and session.reason == "sdk_error: code 9"
    assert len(backend.sent) == 30 and "estop" in backend.calls and "finish" not in backend.calls
    assert finished == [session]


def test_controller_fault_is_a_hard_failure(tmp_path):
    fake = FakeClock()
    backend = FakeBackend(fake, fault_at=25)
    session, _ = run(manager(fake, tmp_path), backend)
    assert session.state == FAILED and session.reason.startswith("controller_fault: error code 22")
    assert len(backend.sent) == 25 and "estop" in backend.calls


def test_claim_loss_is_a_constrained_stop(tmp_path):
    fake = FakeClock()
    backend = FakeBackend(fake)
    calls = {"n": 0}

    def watch():
        calls["n"] += 1
        return "claim_lost" if calls["n"] >= 3 else None

    session, _ = run(manager(fake, tmp_path), backend, pts=points_j1(end=20.0, duration=2.0, n=21),
                     watch=watch)
    assert session.state == FAILED and session.reason == "claim_lost"
    assert "finish" in backend.calls and "estop" not in backend.calls
    assert len(backend.sent) < 200


def test_short_stall_delays_without_a_burst(tmp_path):
    fake = FakeClock()
    backend = FakeBackend(fake, stalls={30: 0.025})   # 25 ms at 100 Hz
    session, _ = run(manager(fake, tmp_path), backend)
    assert session.state == COMPLETED
    servo = session.stats["servo"]
    # A 25 ms send leaves the next tick 15 ms late (one period was due anyway).
    assert servo["late_ticks"] == 1 and servo["cumulative_delay_ms"] == pytest.approx(15.3, abs=0.5)
    # No catch-up burst: no step is larger than the validated speed allows.
    assert max_step(backend.sent) <= VMAX * 0.01 + 1e-9
    assert len(backend.sent) == 100


def test_long_stall_is_a_timing_fault(tmp_path):
    fake = FakeClock()
    backend = FakeBackend(fake, stalls={30: 0.08})    # 8 periods
    session, _ = run(manager(fake, tmp_path), backend)
    assert session.state == FAILED and session.reason == "timing"
    assert "finish" in backend.calls


def test_failed_prepare_restores_mode_and_sends_nothing(tmp_path):
    fake = FakeClock()
    backend = FakeBackend(fake, prepare_error="set_mode(1) returned code 1")
    session, finished = run(manager(fake, tmp_path), backend)
    assert session.state == FAILED and session.reason.startswith("prepare_failed")
    assert backend.sent == [] and "finish" in backend.calls and finished == [session]


# ── Settings ─────────────────────────────────────────────────────────


def test_settings_validation():
    assert TrajectorySettings().enabled is False
    with pytest.raises(ValueError):
        TrajectorySettings.from_mapping({"bogus": 1})
    with pytest.raises(ValueError):
        TrajectorySettings.from_mapping({"backend": "teleport"})
    with pytest.raises(ValueError):
        TrajectorySettings.from_mapping({"max_servo_rate_hz": 400})
    with pytest.raises(ValueError):
        TrajectorySettings.from_mapping({"command_timeout_s": 2.0})
    assert TrajectorySettings.load("/nonexistent/trajectory.yaml").enabled is False


# ── xArm backends and the real-time monitor ──────────────────────────


class FakeArm:
    def __init__(self):
        self.calls = []
        self.mode = 0
        self.state = 2
        self.error_code = 0
        self.warn_code = 0
        self._timeout = 2.0

    def set_timeout(self, value):
        self.calls.append(("set_timeout", value))
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

    def set_servo_angle_j(self, **kw):
        self.calls.append(("servo_j", kw))
        return 0

    def set_servo_angle(self, **kw):
        self.calls.append(("servo", kw))
        return 0


def test_servoj_backend_enters_mode_1_with_a_short_timeout_and_restores_it():
    arm = FakeArm()
    backend = XArmServoJBackend(arm, command_timeout_s=0.1)
    backend.prepare(sleep=lambda s: None)
    assert arm.calls[:4] == [("set_timeout", 0), ("set_timeout", 0.1), ("set_mode", 1), ("set_state", 0)]
    assert backend.send([1, 2, 3, 4, 5], 10, 500) == 0
    assert arm.calls[-1] == ("servo_j", {"angles": [1, 2, 3, 4, 5], "is_radian": False})
    backend.finish()
    backend.restore()
    assert arm.calls[-3:] == [("set_mode", 0), ("set_state", 0), ("set_timeout", 2.0)]


def test_online_planning_backend_uses_mode_6_and_no_wait():
    arm = FakeArm()
    backend = XArmOnlinePlanningBackend(arm)
    backend.prepare(sleep=lambda s: None)
    assert ("set_mode", 6) in arm.calls
    backend.send([1, 2, 3, 4, 5], 12.5, 500)
    assert arm.calls[-1] == ("servo", {"angle": [1, 2, 3, 4, 5], "speed": 12.5, "mvacc": 500,
                                       "wait": False, "is_radian": False})


def test_prepare_fails_when_the_mode_never_changes():
    arm = FakeArm()
    arm.set_mode = lambda mode: 0      # accepted but never takes effect
    now = {"t": 0.0}

    def clock():
        return now["t"]

    def sleep(s):
        now["t"] += s

    with pytest.raises(Exception, match="did not report mode 1"):
        XArmServoJBackend(arm).prepare(sleep=sleep, clock=clock)


def test_realtime_monitor_parses_frames():
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    port = server.getsockname()[1]
    angles = [0.1, -0.2, 0.3, 0.0, 1.0, 0.0, 0.0]

    def serve():
        conn, _ = server.accept()
        body = bytes([1 | (1 << 4)]) + struct.pack(">H", 7) + struct.pack("<7f", *angles) + bytes(60)
        frame = struct.pack(">I", 4 + len(body)) + body
        conn.sendall(frame * 3)
        time.sleep(0.3)
        conn.close()

    threading.Thread(target=serve, daemon=True).start()
    monitor = RealtimeJointMonitor("127.0.0.1", 5, port=port)
    monitor.start()
    deadline = time.time() + 3
    while time.time() < deadline and len(monitor.samples) < 3:
        time.sleep(0.02)
    monitor.stop()
    server.close()
    assert len(monitor.samples) == 3
    _, joints, state, mode = monitor.latest
    assert joints == pytest.approx([math.degrees(a) for a in angles[:5]], rel=1e-6)
    assert (state, mode) == (1, 1)


# ── Review regressions: stops and cancels that race the start ─────────


def test_stop_before_start_refuses_and_nothing_moves(tmp_path):
    fake = FakeClock()
    backend = FakeBackend(fake)
    mgr = manager(fake, tmp_path)
    generation = mgr.stop_generation()
    mgr.notify_hard_stop("stop")          # e.g. the sash watchdog during the route's gates
    with pytest.raises(TrajectoryError) as e:
        run(mgr, backend, generation=generation)
    assert e.value.detail["error"] == "stopped_during_start"
    assert backend.sent == [] and backend.calls == []
    assert mgr.get(mgr._current).state == CREATED


def test_stop_during_prepare_is_honoured(tmp_path):
    fake = FakeClock()
    mgr = manager(fake, tmp_path)
    backend = FakeBackend(fake, on_prepare=lambda b: mgr.notify_hard_stop("stop"))
    session, finished = run(mgr, backend)
    assert session.state == STOPPED and session.reason == "stop"
    assert backend.sent == [] and "finish" not in backend.calls and finished == [session]


def test_stop_during_settle_leaves_the_arm_stopped(tmp_path):
    fake = FakeClock()
    mgr = manager(fake, tmp_path)

    def on_read(b):
        if b.reads == 2:                  # the first settle read (1 is the re-measure)
            mgr.notify_hard_stop("stop")

    backend = FakeBackend(fake, on_read=on_read)
    session, _ = run(mgr, backend)
    assert session.state == STOPPED and "finish" not in backend.calls


def test_stop_during_mode_restore_is_reissued(tmp_path):
    fake = FakeClock()
    mgr = manager(fake, tmp_path)
    backend = FakeBackend(fake, on_finish=lambda b: mgr.notify_hard_stop("stop"))
    session, _ = run(mgr, backend)
    assert session.state == STOPPED
    assert backend.calls[-3:] == ["finish", "estop", "restore"]
    assert session.final["mode_restored"] is False


def test_cancel_during_start_is_not_overwritten(tmp_path, monkeypatch):
    fake = FakeClock()
    backend = FakeBackend(fake)
    mgr = manager(fake, tmp_path)
    real = executor_module.validate_joint_trajectory
    holder = {}

    def validate_then_cancel(*args, **kwargs):
        if kwargs.get("start_joints") is not None:
            mgr.cancel(holder["sid"], "tok")      # arrives while start validates
        return real(*args, **kwargs)

    session = mgr.create(key="tok", owner="t", num_joints=5)
    holder["sid"] = session.id
    limits = mgr.validation_limits(session, JOINT_LIMITS_5, VMAX)
    mgr.add_chunk(session.id, 0, "tok", points_j1(), final=True, limits=limits)
    monkeypatch.setattr(executor_module, "validate_joint_trajectory", validate_then_cancel)
    with pytest.raises(TrajectoryError) as e:
        mgr.start(session.id, "tok", limits=limits, start_joints=ZERO, backend=backend,
                  watch=lambda: None, on_finish=lambda s: None)
    assert e.value.detail["error"] == "session_cancelled"
    assert session.state == CANCELLED and backend.sent == [] and backend.calls == []


def test_arm_moved_after_prepare_fails_before_streaming(tmp_path):
    fake = FakeClock()

    def moved(b):
        b.q = [1.0, 0, 0, 0, 0]           # someone nudged the arm after start validated

    backend = FakeBackend(fake, on_prepare=moved)
    session, _ = run(manager(fake, tmp_path), backend)
    assert session.state == FAILED and session.reason.startswith("start_moved")
    assert backend.sent == [] and "finish" in backend.calls and "estop" not in backend.calls


def test_paused_arm_is_a_hard_fault(tmp_path):
    fake = FakeClock()
    backend = FakeBackend(fake, fault_at=25, fault_state=3)
    session, _ = run(manager(fake, tmp_path), backend)
    assert session.state == FAILED and session.reason == "controller_fault: arm state 3"
    assert len(backend.sent) == 25 and "estop" in backend.calls


def test_prepare_refused_before_any_mode_change_does_not_touch_the_arm(tmp_path):
    fake = FakeClock()
    backend = FakeBackend(fake)

    def refuse(abort=lambda: False):
        backend.calls.append("prepare")
        raise BackendError("arm is not idle (state 4)")

    backend.prepare = refuse
    session, finished = run(manager(fake, tmp_path), backend)
    assert session.state == FAILED and "not idle" in session.reason
    assert backend.calls == ["prepare", "restore"] and finished == [session]


def test_an_exception_in_finalise_still_ends_the_session(tmp_path):
    fake = FakeClock()

    def boom(b):
        if b.reads >= 2:
            raise RuntimeError("socket closed")

    backend = FakeBackend(fake, on_read=boom)
    session, finished = run(manager(fake, tmp_path), backend)
    assert session.state == FAILED and session.reason.startswith("finalise_error")
    assert finished == [session] and "estop" in backend.calls


def test_servoj_backend_refuses_a_stopped_or_faulted_arm_without_commands():
    for attrs in ({"state": 4}, {"state": 3}, {"error_code": 31}, {"warn_code": 1}):
        arm = FakeArm()
        for key, value in attrs.items():
            setattr(arm, key, value)
        with pytest.raises(BackendError):
            XArmServoJBackend(arm).prepare(sleep=lambda s: None)
        assert arm.calls == []


def test_servoj_backend_checks_for_stop_before_set_state():
    arm = FakeArm()
    seen = []

    def abort():
        seen.append(len(arm.calls))
        return len(arm.calls) >= 3      # after set_timeout x2 and set_mode

    with pytest.raises(StopRequested):
        XArmServoJBackend(arm).prepare(abort=abort, sleep=lambda s: None)
    assert ("set_state", 0) not in arm.calls and ("set_mode", 1) in arm.calls


class _Monitor:
    def __init__(self, state, mode):
        self.latest = (0.0, ZERO, state, mode)


def test_monitor_only_counts_after_it_has_seen_the_streaming_mode():
    ok = {"connected": True, "state": 1, "mode": 1, "error_code": 0, "warn_code": 0}
    fault, armed = _health_fault(ok, 1, _Monitor(2, 0), False)   # a frame from before the switch
    assert fault is None and armed is False
    fault, armed = _health_fault(ok, 1, _Monitor(1, 1), armed)
    assert fault is None and armed is True
    fault, _ = _health_fault(ok, 1, _Monitor(4, 1), armed)
    assert fault == "arm state 4 (real-time report)"
    fault, _ = _health_fault(dict(ok, state=3), 1, None, False)
    assert fault == "arm state 3"


def test_sample_guard_bounds_speed_and_acceleration():
    limits = JointLimits(JOINT_LIMITS_5, max_joint_speed_deg_s=60, max_joint_acc_deg_s2=500)
    dt = 0.01
    q0 = [0.0] * 5
    assert _sample_fault([0.004, 0, 0, 0, 0], q0, q0, dt, dt, limits, 60, 900) is None
    # 0.8 deg in one tick at 100 Hz = 80 deg/s > 1.2 x 60.
    assert "speed bound" in _sample_fault([0.8, 0, 0, 0, 0], q0, q0, dt, dt, limits, 60, 900)
    # 0.5 deg from rest in one tick = 5000 deg/s^2.
    assert "acceleration" in _sample_fault([0.5, 0, 0, 0, 0], q0, q0, dt, dt, limits, 60, 900)
    assert "outside" in _sample_fault([0, 0, 50, 0, 0], [0, 0, 49.99, 0, 0], None, dt, dt, limits, 1e9, 1e9)


# ── On-arm findings (2026-10-09) ─────────────────────────────────────


def test_lead_in_is_gentle_and_a_later_cancel_uses_the_trajectory_stop_rate(tmp_path):
    fake = FakeClock()
    measured = [0.15, 0, 0, 0, 0]               # a small gap, as after a mode 6 run
    backend = FakeBackend(fake, start=measured, gate_at=80)
    mgr = manager(fake, tmp_path)
    session, _ = run(mgr, backend, pts=points_j1(end=20.0, duration=2.0, n=21), start=measured, wait=False)
    assert backend.reached_gate.wait(5)
    mgr.cancel(session.id, "tok")
    backend.gate.set()
    assert mgr.wait_idle(10)
    assert session.state == CANCELLED and session.lead_in_s > 0
    # Lead-in sized at half the limits: 0.15 deg needs sqrt(5.77*0.15/250) s.
    assert session.lead_in_s == pytest.approx(math.sqrt((10 / math.sqrt(3)) * 0.15 / 250.0), rel=1e-6)
    # The stop happened in the main trajectory, so it used its fast rate,
    # not the 0.5 /s^2 floor that a full-limit lead-in used to force.
    assert session.stats["servo"]["stop_rate_per_s2"] > 5.0


def test_gc_is_paused_while_streaming_and_restored_after(tmp_path):
    import gc
    fake = FakeClock()
    seen = []
    backend = FakeBackend(fake)
    original = backend.send

    def send(q, speed, acc):
        seen.append(gc.isenabled())
        return original(q, speed, acc)

    backend.send = send
    assert gc.isenabled()
    session, _ = run(manager(fake, tmp_path), backend)
    assert session.state == COMPLETED
    assert seen and not any(seen)
    assert gc.isenabled()
    assert session.stats["servo"]["gc_paused_while_streaming"] is True
