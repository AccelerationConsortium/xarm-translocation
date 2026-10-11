"""UR e-Series backend for streamed joint trajectories (ur_rtde ``servoJ``).

Implements the backend interface the executor (``executor.TrajectoryManager``)
uses, on top of the UR service's existing control wrapper (one caller in the
SDK at a time) and its receive-side feedback sampler. No mode switch is
needed on a UR: servoJ runs inside the ur_rtde control script, and
servoStop leaves it. See src/docs/UR5E_TRAJECTORY_PLAN.md.

Health comes from the feedback sampler (RTDE receive, no command traffic):
the controller must stay RUNNING with safety NORMAL, the control link and
script must stay up, and the sample must be fresh. With the commissioned
motion limits (``guard``) every tick also checks, like a moveJ does, the
joint envelope, the workspace box, joint overspeed and the force guard;
any of them is a hard stop. ``path_errors`` checks the planned path against
the workspace box with the controller's forward kinematics before a start.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Sequence

from .executor import BackendError, StopRequested


class FeedbackMonitor:
    """The executor's real-time monitor for a UR, fed by the service's own
    feedback sampler (no extra connection). Each new controller packet is
    kept as (perf_counter receipt time, joints_deg, robot_mode,
    safety_mode), the same shape as the xArm's port-30003 monitor, so status
    ``measured``, tracking statistics and the session log work unchanged."""

    def __init__(self, feedback, num_joints: int = 6, poll_s: float = 0.002, keep: int = 200000):
        self.feedback = feedback
        self.num_joints = num_joints
        self.poll_s = poll_s
        self._keep = keep
        self.latest: Optional[tuple] = None
        self.samples: List[tuple] = []
        self.dropped = 0
        self.error: Optional[str] = None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="ur-trajectory-monitor", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        last = None
        try:
            while not self._stop.wait(self.poll_s):
                sample = self.feedback.latest()
                if not isinstance(sample, dict):
                    continue
                stamp = sample.get("controller_timestamp_s", sample.get("received_monotonic_s"))
                if stamp == last:
                    continue
                last = stamp
                age = time.monotonic() - float(sample["received_monotonic_s"])
                joints = [float(v) for v in sample["joints_deg"]][: self.num_joints]
                entry = (time.perf_counter() - max(0.0, age), joints,
                         sample.get("robot_mode"), sample.get("safety_mode"))
                self.latest = entry
                if len(self.samples) < self._keep:
                    self.samples.append(entry)
                else:
                    self.dropped += 1
        except Exception as exc:  # noqa: BLE001 - monitoring only; fault() still guards
            self.error = str(exc)


class URServoJBackend:
    name = "ur_servoj"
    mode = None   # no controller mode switch on a UR

    def __init__(self, control, feedback, *, period_s: float, num_joints: int = 6,
                 lookahead_s: float = 0.1, gain: float = 300.0, stop_decel_deg_s2: float = 90.0,
                 emergency_decel_deg_s2: float = 720.0, max_feedback_age_s: float = 0.2,
                 guard=None, clock: Callable[[], float] = time.monotonic):
        if not 0.03 <= lookahead_s <= 0.2:
            raise ValueError("lookahead_s must be within 0.03-0.2 s (ur_rtde servoJ)")
        if not 100 <= gain <= 2000:
            raise ValueError("gain must be within 100-2000 (ur_rtde servoJ)")
        self.control = control
        self.feedback = feedback
        self.period_s = period_s
        self.num_joints = num_joints
        self.lookahead_s = lookahead_s
        self.gain = gain
        self.stop_decel = math.radians(stop_decel_deg_s2)
        self.emergency_decel = math.radians(emergency_decel_deg_s2)
        self.max_feedback_age_s = max_feedback_age_s
        self.guard = guard
        self.force_baseline = None
        self.peak_force_change_n = 0.0
        self.clock = clock
        self.streaming = False
        self.mode_changed = False   # nothing to restore on a failed prepare

    # ── helpers ──────────────────────────────────────────────────────

    def _sample(self) -> Optional[Dict[str, Any]]:
        sample = self.feedback.latest()
        return sample if isinstance(sample, dict) else None

    def _problem(self, sample) -> Optional[str]:
        if sample is None:
            return "no control feedback"
        age = self.clock() - float(sample.get("received_monotonic_s", -1e9))
        if age > self.max_feedback_age_s:
            return f"control feedback is {age * 1000:.0f} ms old"
        if not sample.get("controller_connected", False) or not self.control.isConnected():
            return "control link down"
        if sample.get("robot_mode") != "RUNNING":
            return f"robot mode {sample.get('robot_mode')}"
        if sample.get("safety_mode") != "NORMAL":
            return f"safety mode {sample.get('safety_mode')}"
        return None

    # ── backend interface ────────────────────────────────────────────

    def prepare(self, abort: Callable[[], bool] = lambda: False, **_ignored) -> None:
        sample = self._sample()
        problem = self._problem(sample)
        if problem:
            raise BackendError(f"arm not ready: {problem}")
        if not self.control.isProgramRunning():
            raise BackendError("the ur_rtde control script is not running")
        fraction = sample.get("speed_fraction")
        if fraction is not None and fraction < 0.999:
            raise BackendError(f"the pendant speed slider is at {fraction:.0%}; set it to 100% "
                               "(it scales servoJ and would stretch the trajectory)")
        moving = max((abs(v) for v in sample.get("velocities_deg_s") or ()), default=0.0)
        if moving > 0.05:
            raise BackendError(f"arm is moving ({moving:.2f} deg/s); it must be still")
        if self.guard is not None:
            problem = self._guard_problem(sample, at_rest=True)
            if problem:
                raise BackendError(f"arm not ready: {problem}")
            self.force_baseline = list(sample["tcp_force"][:3])
        if abort():
            raise StopRequested()

    def send(self, q: Sequence[float], speed_deg_s: float, acc_deg_s2: float) -> int:
        self.streaming = True
        ok = self.control.servoJ([math.radians(v) for v in q], 0.0, 0.0, self.period_s,
                                 self.lookahead_s, self.gain)
        return 0 if ok in (True, None) else 1

    def finish(self) -> None:
        if self.streaming:
            self.control.servoStop(self.stop_decel)
            self.streaming = False

    def restore(self) -> None:
        pass

    def emergency_stop(self) -> None:
        for call in (lambda: self.control.servoStop(self.emergency_decel),
                     lambda: self.control.stopJ(self.emergency_decel)):
            try:
                call()
            except Exception:  # noqa: BLE001 - the stop path must not raise
                pass
        self.streaming = False

    def read_joints(self, n: int) -> Optional[List[float]]:
        sample = self._sample()
        if sample is None or self._problem(sample):
            return None
        joints = [float(v) for v in (sample.get("joints_deg") or ())[:n]]
        return joints if len(joints) == n and all(math.isfinite(v) for v in joints) else None

    def health(self) -> Dict[str, Any]:
        sample = self._sample() or {}
        return {"connected": self.control.isConnected(), "robot_mode": sample.get("robot_mode"),
                "safety_mode": sample.get("safety_mode")}

    def fault(self, monitor, armed):
        sample = self._sample()
        problem = self._problem(sample)
        if problem is None and not self.control.isProgramRunning():
            problem = "control script stopped"
        if problem is None and self.guard is not None:
            problem = self._guard_problem(sample)
        return problem, armed

    def _guard_problem(self, sample, at_rest=False) -> Optional[str]:
        """The commissioned limits a moveJ is held to, on one packet."""
        lim = self.guard
        slack = 0.0 if at_rest else lim.path_tolerance_deg
        try:
            joints = [float(v) for v in sample["joints_deg"]]
            speeds = [abs(float(v)) for v in sample["velocities_deg_s"]]
            point = [float(v) * 1000 for v in sample["tcp_pose"][:3]]
            force = [float(v) for v in sample["tcp_force"][:3]]
        except (KeyError, TypeError, ValueError):
            return "control feedback is incomplete"
        outside = [i + 1 for i, (q, lo, hi) in enumerate(zip(joints, lim.joint_lower_deg, lim.joint_upper_deg))
                   if not lo - slack <= q <= hi + slack]
        if outside:
            return f"joints {outside} outside the commissioned envelope"
        axes = lim.workspace.outside(point, margin_mm=0.0 if at_rest else lim.path_tolerance_mm)
        if axes:
            return f"TCP outside the workspace box on {''.join(axes)}"
        overspeed = 1.25 * lim.max_joint_speed_deg_s + 1.0
        if max(speeds, default=0.0) > overspeed:
            return f"a joint turned at {max(speeds):.0f} deg/s, over the {lim.max_joint_speed_deg_s:g} deg/s cap"
        if self.force_baseline is not None and not at_rest:
            change = math.dist(force, self.force_baseline)
            self.peak_force_change_n = max(self.peak_force_change_n, change)
            if lim.force_guard_n is not None and change > lim.force_guard_n:
                return f"force guard: TCP force changed by {change:.1f} N (limit {lim.force_guard_n:g} N)"
        return None


def path_errors(trajectory, control, guard, *, step_deg: float = 2.0, max_checks: int = 3000,
                rate_hz: float = 125.0) -> List[Dict[str, Any]]:
    """Workspace-box and controller-limit errors along an interpolated
    trajectory (``JointTrajectory``), in the validation report's format.

    Samples the path at the command rate and asks the controller for forward
    kinematics wherever any joint has moved ``step_deg`` since the last
    check, plus the first and last samples. A path that needs more than
    ``max_checks`` queries is refused rather than checked partly."""
    checks: List[tuple] = []
    last = None
    duration = trajectory.duration
    count = max(1, int(math.ceil(duration * rate_hz)))
    for i in range(count + 1):
        t = min(duration, i / rate_hz)
        q = list(trajectory.sample(t)[0])
        if last is None or i == count or max(abs(a - b) for a, b in zip(q, last)) >= step_deg:
            checks.append((t, q))
            last = q
            if len(checks) > max_checks:
                return [{"code": "path_check_too_long",
                         "message": f"the path needs more than {max_checks} workspace checks; "
                                    "split it or move less"}]
    for t, q in checks:
        radians = [math.radians(v) for v in q]
        if control.isJointsWithinSafetyLimits(radians) is not True:
            return [{"code": "outside_controller_limits", "t": round(t, 4),
                     "message": f"the controller's safety limits reject the joints at t={t:.3f} s"}]
        pose = control.getForwardKinematics(radians)
        point = [float(v) * 1000 for v in list(pose)[:3]]
        axes = guard.workspace.outside(point)
        if axes:
            return [{"code": "path_leaves_workspace", "t": round(t, 4),
                     "tcp_mm": [round(v, 1) for v in point],
                     "message": f"the TCP would leave the workspace box on {''.join(axes)} at t={t:.3f} s"}]
    return []
