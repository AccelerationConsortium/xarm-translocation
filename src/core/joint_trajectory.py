"""Joint trajectories for streamed execution (no hardware I/O).

A *joint trajectory* is a list of time-stamped points carrying the arm's
joint angles and, optionally, joint velocities and accelerations. This
module builds the interpolant the executor samples at a fixed rate,
validates a trajectory against the cell's limits and the measured start,
and plans the lead-in and the constrained stop. It never commands
hardware, so it is safe to call anywhere.

Units and conventions (fixed, not negotiable per request):

- ``t``                    seconds from the trajectory start; the first
                           point is at 0 and times strictly increase.
- ``joints_deg``           degrees, base to wrist (J1..J5 on the xArm5),
                           exactly ``num_joints`` entries.
- ``velocities_deg_s``     optional, same shape; deg/s.
- ``accelerations_deg_s2`` optional, same shape, only with velocities;
                           deg/s^2.

Interpolation is a *local* Hermite spline: each segment depends only on
the state at its two end points. That is what makes a chunk boundary the
same as any other point, and what lets the executor sample without
looking ahead more than one point. See ``src/docs/SERVOJ_TRAJECTORY_PLAN.md``.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

DEFAULT_SERVO_RATE_HZ = 100.0
# Mode 6 streams at 20-50 Hz, mode 1 at 100-250 Hz (UFACTORY: 250 Hz max).
MIN_SERVO_RATE_HZ = 20.0
MAX_SERVO_RATE_HZ = 250.0
DEFAULT_MAX_JOINT_ACC_DEG_S2 = 500.0
DEFAULT_MAX_DURATION_S = 600.0
DEFAULT_MAX_POINTS = 60000
DEFAULT_START_TOLERANCE_DEG = 0.5
DEFAULT_MAX_STOP_S = 2.0

# "At rest" allows for float noise from the planner, nothing more.
REST_VELOCITY_TOL_DEG_S = 1e-3
REST_ACC_TOL_DEG_S2 = 1e-2

# Error lists are capped so one bad 60 000-point upload cannot produce a
# multi-megabyte report.
MAX_REPORTED_ERRORS = 100

INTERPOLATIONS = ("quintic", "cubic", "cubic_estimated")

# Returned verbatim in every report so the planner sees the contract next
# to its verdict.
JOINT_EXECUTION_MODEL: Dict[str, Any] = {
    "timing": (
        "Device-clocked. The service samples the interpolant at a fixed rate "
        "on its own clock; when HTTP requests arrive does not affect execution."
    ),
    "interpolation": {
        "quintic": "positions, velocities and accelerations given: C2 everywhere",
        "cubic": "positions and velocities given: C1; acceleration can step at points",
        "cubic_estimated": (
            "positions only: velocities from weighted central differences, "
            "zero at both ends; C1"
        ),
    },
    "start": (
        "The arm must be within start_tolerance of the first point. A rest-to-rest "
        "lead-in from the measured pose to the first point is inserted before t = 0."
    ),
    "rest": "The first and last points are at rest (zero velocity and acceleration).",
    "collision_checking": (
        "None. Validation covers joint limits and the interpolant's joint speed "
        "and acceleration only. Passing validation is not a collision-safety "
        "proof; collision checking is the planner's responsibility."
    ),
    "stop": (
        "Cancel and soft faults slow the trajectory clock to zero along the "
        "planned path within the acceleration limit. STOP and controller faults "
        "halt the arm immediately (emergency stop)."
    ),
}


@dataclass(frozen=True)
class JointPoint:
    """One time-stamped point. ``qd``/``qdd`` are None when not given."""
    t: float
    q: Tuple[float, ...]
    qd: Optional[Tuple[float, ...]] = None
    qdd: Optional[Tuple[float, ...]] = None

    @classmethod
    def from_mapping(cls, data: Dict[str, Any]) -> "JointPoint":
        def vec(key):
            value = data.get(key)
            return None if value is None else tuple(float(v) for v in value)

        return cls(
            t=float(data["t"]),
            q=tuple(float(v) for v in data["joints_deg"]),
            qd=vec("velocities_deg_s"),
            qdd=vec("accelerations_deg_s2"),
        )


@dataclass
class JointLimits:
    joint_limits_deg: Sequence[Tuple[float, float]]
    max_joint_speed_deg_s: float
    max_joint_acc_deg_s2: float = DEFAULT_MAX_JOINT_ACC_DEG_S2
    servo_rate_hz: float = DEFAULT_SERVO_RATE_HZ
    max_duration_s: float = DEFAULT_MAX_DURATION_S
    max_points: int = DEFAULT_MAX_POINTS

    @property
    def num_joints(self) -> int:
        return len(self.joint_limits_deg)


# ── Interpolant ──────────────────────────────────────────────────────


def _quintic(p0, v0, a0, p1, v1, a1, h):
    """Coefficients c0..c5 of the quintic Hermite on local time [0, h]."""
    d = p1 - p0
    h2, h3, h4, h5 = h * h, h ** 3, h ** 4, h ** 5
    return (
        p0,
        v0,
        a0 / 2.0,
        (20 * d - (8 * v1 + 12 * v0) * h - (3 * a0 - a1) * h2) / (2 * h3),
        (-30 * d + (14 * v1 + 16 * v0) * h + (3 * a0 - 2 * a1) * h2) / (2 * h4),
        (12 * d - 6 * (v1 + v0) * h - (a0 - a1) * h2) / (2 * h5),
    )


def _cubic(p0, v0, p1, v1, h):
    """Coefficients c0..c3 of the cubic Hermite on local time [0, h]."""
    d = p1 - p0
    return (
        p0,
        v0,
        (3 * d - (2 * v0 + v1) * h) / (h * h),
        (-2 * d + (v0 + v1) * h) / (h ** 3),
    )


def _eval_poly(c, u):
    """Value, first and second derivative of sum(c[i] u^i) at u."""
    q = qd = qdd = 0.0
    n = len(c)
    for i in range(n - 1, -1, -1):
        q = q * u + c[i]
    for i in range(n - 1, 0, -1):
        qd = qd * u + i * c[i]
    for i in range(n - 1, 1, -1):
        qdd = qdd * u + i * (i - 1) * c[i]
    return q, qd, qdd


def interpolation_kind(points: Sequence[JointPoint]) -> str:
    """Which interpolant the given derivatives select (assumes consistency)."""
    if points and all(p.qd is not None for p in points):
        if all(p.qdd is not None for p in points):
            return "quintic"
        return "cubic"
    return "cubic_estimated"


def estimated_velocities(points: Sequence[JointPoint]) -> List[Tuple[float, ...]]:
    """Weighted central differences, zero at both ends."""
    n = len(points)
    joints = len(points[0].q)
    out: List[Tuple[float, ...]] = [tuple(0.0 for _ in range(joints))]
    for i in range(1, n - 1):
        h0 = points[i].t - points[i - 1].t
        h1 = points[i + 1].t - points[i].t
        vel = []
        for j in range(joints):
            s0 = (points[i].q[j] - points[i - 1].q[j]) / h0
            s1 = (points[i + 1].q[j] - points[i].q[j]) / h1
            vel.append((h1 * s0 + h0 * s1) / (h0 + h1))
        out.append(tuple(vel))
    if n > 1:
        out.append(tuple(0.0 for _ in range(joints)))
    return out


class JointTrajectory:
    """Piecewise Hermite interpolant over validated points.

    Build it only from points that passed :func:`validate_joint_trajectory`
    (or from :func:`lead_in`); the constructor does not re-validate.
    """

    def __init__(self, points: Sequence[JointPoint]):
        if len(points) < 2:
            raise ValueError("a trajectory needs at least 2 points")
        self.points = list(points)
        self.kind = interpolation_kind(self.points)
        self.num_joints = len(self.points[0].q)
        self._times = [p.t for p in self.points]
        joints = self.num_joints
        zero = tuple(0.0 for _ in range(joints))
        if self.kind == "cubic_estimated":
            velocities = estimated_velocities(self.points)
        else:
            velocities = [p.qd for p in self.points]
        self._segments: List[List[Tuple[float, ...]]] = []
        for i in range(len(self.points) - 1):
            a, b = self.points[i], self.points[i + 1]
            h = b.t - a.t
            seg = []
            for j in range(joints):
                if self.kind == "quintic":
                    seg.append(_quintic(a.q[j], a.qd[j], a.qdd[j], b.q[j], b.qd[j], b.qdd[j], h))
                else:
                    seg.append(_cubic(a.q[j], velocities[i][j], b.q[j], velocities[i + 1][j], h))
            self._segments.append(seg)
        self._zero = zero

    @property
    def duration(self) -> float:
        return self._times[-1] - self._times[0]

    @property
    def start_q(self) -> Tuple[float, ...]:
        return self.points[0].q

    @property
    def final_q(self) -> Tuple[float, ...]:
        return self.points[-1].q

    def _segment_index(self, t: float) -> int:
        i = bisect.bisect_right(self._times, t) - 1
        return min(max(i, 0), len(self._segments) - 1)

    def sample(self, t: float):
        """(q, qd, qdd) at trajectory time t, clamped to [0, duration].

        Outside the trajectory the arm is at rest at the nearest end."""
        if t <= self._times[0]:
            return list(self.points[0].q), list(self._zero), list(self._zero)
        if t >= self._times[-1]:
            return list(self.points[-1].q), list(self._zero), list(self._zero)
        i = self._segment_index(t)
        return self._eval_segment(i, t - self._times[i])

    def _eval_segment(self, i: int, u: float):
        q, qd, qdd = [], [], []
        for coeffs in self._segments[i]:
            a, b, c = _eval_poly(coeffs, u)
            q.append(a)
            qd.append(b)
            qdd.append(c)
        return q, qd, qdd

    def segment_samples(self, rate_hz: float):
        """Yield (segment index, q, qd, qdd) on a grid of at most 1/rate_hz
        inside every segment, including both of its ends (so an acceleration
        step at a point is seen from both sides)."""
        step = 1.0 / rate_hz
        for i in range(len(self._segments)):
            h = self._times[i + 1] - self._times[i]
            n = max(1, int(math.ceil(h / step - 1e-9)))
            for k in range(n + 1):
                q, qd, qdd = self._eval_segment(i, h * k / n)
                yield i, q, qd, qdd


# ── Validation ───────────────────────────────────────────────────────


class _Errors:
    """Collects errors but stores only the first MAX_REPORTED_ERRORS, so a
    huge invalid upload cannot build a huge report. ``codes`` keeps every
    code seen, for the checks that decide whether to continue."""

    def __init__(self, cap: int = MAX_REPORTED_ERRORS):
        self.items: List[Dict[str, Any]] = []
        self.codes = set()
        self.total = 0
        self.cap = cap

    def append(self, err: Dict[str, Any]) -> None:
        self.total += 1
        self.codes.add(err["code"])
        if len(self.items) < self.cap:
            self.items.append(err)

    def __bool__(self) -> bool:
        return self.total > 0

    def __len__(self) -> int:
        return self.total


def _error(code: str, message: str, index: Optional[int] = None) -> Dict[str, Any]:
    err: Dict[str, Any] = {"code": code, "message": message}
    if index is not None:
        err["index"] = index
    return err


def _finite(values) -> bool:
    return all(math.isfinite(v) for v in values)


def validate_joint_trajectory(
    points: Sequence[JointPoint],
    limits: JointLimits,
    start_joints: Optional[Sequence[float]] = None,
    start_tolerance_deg: float = DEFAULT_START_TOLERANCE_DEG,
    check_start: bool = True,
    partial: bool = False,
) -> Dict[str, Any]:
    """Validate a joint trajectory and return a report.

    The report always has ``valid``, ``errors``, ``summary``,
    ``start_state`` and ``execution_model``. ``valid`` is True iff
    ``errors`` is empty. Every violation is collected (up to
    ``MAX_REPORTED_ERRORS``) rather than stopping at the first.

    Speed and acceleration are checked on the *interpolant*, sampled at
    the servo rate, not just between points: that is what the executor
    will actually command.

    ``partial=True`` checks a prefix of a trajectory still being uploaded:
    one point is enough, the last point need not be at rest, and when
    velocities are estimated the last segment is skipped, because its end
    velocity depends on the next point.
    """
    errors = _Errors()
    n = limits.num_joints
    period = 1.0 / limits.servo_rate_hz

    def done(trajectory=None, stats=None, start_report=None):
        return _report(errors, points, limits, trajectory, stats, start_report,
                       start_joints, start_tolerance_deg)

    # ── Shape and finiteness ──────────────────────────────────────────
    if len(points) < (1 if partial else 2):
        errors.append(_error("too_few_points", f"A trajectory needs at least 2 points, got {len(points)}"))
    if len(points) > limits.max_points:
        errors.append(_error("too_many_points", f"{len(points)} points; limit is {limits.max_points}"))
    with_qd = sum(1 for p in points if p.qd is not None)
    with_qdd = sum(1 for p in points if p.qdd is not None)
    if with_qd not in (0, len(points)) or with_qdd not in (0, len(points)):
        errors.append(_error(
            "inconsistent_derivatives",
            "Give velocities (and accelerations) on every point or on none",
        ))
    if with_qdd and not with_qd:
        errors.append(_error("inconsistent_derivatives", "Accelerations need velocities too"))
    for i, p in enumerate(points):
        vectors = [p.q] + [v for v in (p.qd, p.qdd) if v is not None]
        if not math.isfinite(p.t) or not all(_finite(v) for v in vectors):
            errors.append(_error("non_finite", "Point contains NaN or infinity", i))
        if len(p.q) != n:
            errors.append(_error(
                "joint_count", f"Expected exactly {n} joint angles (base to wrist), got {len(p.q)}", i,
            ))
        for name, vec in (("velocities", p.qd), ("accelerations", p.qdd)):
            if vec is not None and len(vec) != n:
                errors.append(_error("joint_count", f"Expected {n} {name}, got {len(vec)}", i))
    if errors or len(points) < 2:
        return done()

    # ── Timing ───────────────────────────────────────────────────────
    if points[0].t != 0:
        errors.append(_error("time_not_from_zero", f"First point must be at t=0 s, got {points[0].t}", 0))
    for i in range(1, len(points)):
        dt = points[i].t - points[i - 1].t
        if dt <= 0:
            errors.append(_error(
                "time_not_increasing",
                f"t must strictly increase; point {i} at {points[i].t} s follows {points[i - 1].t} s",
                i,
            ))
        elif dt < period - 1e-9:
            errors.append(_error(
                "segment_too_short",
                f"Point {i} is {dt * 1000:.2f} ms after the previous one; the minimum is one "
                f"servo period ({period * 1000:.2f} ms at {limits.servo_rate_hz:g} Hz)",
                i,
            ))
    duration = points[-1].t - points[0].t
    if duration > limits.max_duration_s:
        errors.append(_error(
            "duration_exceeded", f"Trajectory lasts {duration:.1f} s; limit is {limits.max_duration_s} s",
        ))

    # ── Rest at both ends ────────────────────────────────────────────
    for idx, label in ((0, "first"),) + (() if partial else ((len(points) - 1, "last"),)):
        p = points[idx]
        if p.qd is not None and any(abs(v) > REST_VELOCITY_TOL_DEG_S for v in p.qd):
            errors.append(_error("not_at_rest", f"The {label} point must have zero velocity", idx))
        if p.qdd is not None and any(abs(a) > REST_ACC_TOL_DEG_S2 for a in p.qdd):
            errors.append(_error("not_at_rest", f"The {label} point must have zero acceleration", idx))

    # ── Joint limits at the points ───────────────────────────────────
    for i, p in enumerate(points):
        for j, (angle, (lo, hi)) in enumerate(zip(p.q, limits.joint_limits_deg)):
            if angle < lo or angle > hi:
                errors.append(_error("joint_out_of_range", f"J{j + 1} = {angle} deg outside [{lo}, {hi}]", i))

    # The interpolant cannot be built over non-increasing times, and sampling
    # an over-long or over-dense trajectory would be a denial of service.
    if errors.codes & {"time_not_increasing", "duration_exceeded", "too_many_points"}:
        return done()

    # ── The interpolant: limits, speed and acceleration ──────────────
    trajectory = JointTrajectory(points)
    stats = {"samples": 0, "max_speed": 0.0, "max_acc": 0.0}
    seg_flags = set()
    skip_last = partial and trajectory.kind == "cubic_estimated"
    for i, q, qd, qdd in trajectory.segment_samples(limits.servo_rate_hz):
        if skip_last and i == len(points) - 2:
            continue
        stats["samples"] += 1
        speed = max(abs(v) for v in qd)
        acc = max(abs(a) for a in qdd)
        stats["max_speed"] = max(stats["max_speed"], speed)
        stats["max_acc"] = max(stats["max_acc"], acc)
        if speed > limits.max_joint_speed_deg_s and ("speed", i) not in seg_flags:
            seg_flags.add(("speed", i))
            j = [abs(v) for v in qd].index(speed) + 1
            errors.append(_error(
                "joint_speed_too_high",
                f"Segment into point {i + 1} reaches {speed:.1f} deg/s on J{j}; limit at the "
                f"current safety level is {limits.max_joint_speed_deg_s} deg/s",
                i + 1,
            ))
        if acc > limits.max_joint_acc_deg_s2 and ("acc", i) not in seg_flags:
            seg_flags.add(("acc", i))
            j = [abs(a) for a in qdd].index(acc) + 1
            errors.append(_error(
                "joint_acc_too_high",
                f"Segment into point {i + 1} reaches {acc:.0f} deg/s^2 on J{j}; limit is "
                f"{limits.max_joint_acc_deg_s2} deg/s^2",
                i + 1,
            ))
        if ("range", i) not in seg_flags:
            for j, (angle, (lo, hi)) in enumerate(zip(q, limits.joint_limits_deg)):
                if angle < lo - 1e-9 or angle > hi + 1e-9:
                    seg_flags.add(("range", i))
                    errors.append(_error(
                        "joint_out_of_range",
                        f"Between points {i} and {i + 1} the interpolant overshoots to "
                        f"J{j + 1} = {angle:.2f} deg, outside [{lo}, {hi}]",
                        i + 1,
                    ))
                    break

    # ── Start state ──────────────────────────────────────────────────
    start_report: Dict[str, Any] = {
        "joints_deg": list(start_joints) if start_joints is not None else None,
        "tolerance_deg": start_tolerance_deg,
    }
    if check_start:
        if start_joints is None or len(list(start_joints)) < n:
            errors.append(_error(
                "start_state_unavailable",
                "Current joint angles could not be read; refusing to validate the start state",
            ))
        else:
            measured = list(start_joints)[:n]
            deltas = [abs(a - b) for a, b in zip(points[0].q, measured)]
            worst = max(deltas)
            start_report["joint_error_deg"] = round(worst, 4)
            if worst > start_tolerance_deg:
                j = deltas.index(worst) + 1
                errors.append(_error(
                    "start_joint_mismatch",
                    f"First point is {worst:.3f} deg from the measured J{j}; tolerance is "
                    f"{start_tolerance_deg} deg",
                    0,
                ))
    return done(trajectory, stats, start_report)


def _report(collected, points, limits, trajectory, stats, start_report, start_joints, tolerance):
    errors = list(collected.items)
    if collected.total > len(errors):
        errors.append(_error(
            "too_many_errors", f"{collected.total - len(errors)} further errors not listed",
        ))
    duration = (points[-1].t - points[0].t) if len(points) >= 2 else 0.0
    summary = {
        "points": len(points),
        "num_joints": limits.num_joints,
        "duration_s": round(duration, 6),
        "interpolation": trajectory.kind if trajectory else interpolation_kind(points),
        "servo_rate_hz": limits.servo_rate_hz,
        "samples": stats["samples"] if stats else 0,
        "max_joint_speed_deg_s": round(stats["max_speed"], 3) if stats else None,
        "max_joint_acc_deg_s2": round(stats["max_acc"], 3) if stats else None,
        "limits": {
            "joint_limits_deg": [list(l) for l in limits.joint_limits_deg],
            "max_joint_speed_deg_s": limits.max_joint_speed_deg_s,
            "max_joint_acc_deg_s2": limits.max_joint_acc_deg_s2,
            "max_duration_s": limits.max_duration_s,
            "max_points": limits.max_points,
        },
    }
    if start_report is None:
        start_report = {
            "joints_deg": list(start_joints) if start_joints is not None else None,
            "tolerance_deg": tolerance,
        }
    return {
        "valid": not errors,
        "errors": errors,
        "summary": summary,
        "start_state": start_report,
        "execution_model": JOINT_EXECUTION_MODEL,
    }


# ── Lead-in and constrained stop ─────────────────────────────────────


def lead_in_duration(q_from: Sequence[float], q_to: Sequence[float],
                     max_speed_deg_s: float, max_acc_deg_s2: float) -> float:
    """Shortest rest-to-rest quintic duration within the limits.

    A minimum-jerk quintic over distance D and time T peaks at
    1.875 D/T in speed and 10/sqrt(3) D/T^2 in acceleration."""
    distance = max((abs(a - b) for a, b in zip(q_from, q_to)), default=0.0)
    if distance < 1e-9:
        return 0.0
    by_speed = 1.875 * distance / max_speed_deg_s
    by_acc = math.sqrt((10.0 / math.sqrt(3.0)) * distance / max_acc_deg_s2)
    return max(by_speed, by_acc)


def lead_in(q_from: Sequence[float], q_to: Sequence[float],
            max_speed_deg_s: float, max_acc_deg_s2: float) -> Optional[JointTrajectory]:
    """Rest-to-rest segment from the measured pose to the first point, or
    None when they already coincide."""
    duration = lead_in_duration(q_from, q_to, max_speed_deg_s, max_acc_deg_s2)
    if duration <= 0:
        return None
    zero = tuple(0.0 for _ in q_from)
    return JointTrajectory([
        JointPoint(0.0, tuple(float(v) for v in q_from), zero, zero),
        JointPoint(duration, tuple(float(v) for v in q_to), zero, zero),
    ])


def stop_deceleration(path_max_speed_deg_s: float, path_max_acc_deg_s2: float,
                      max_acc_deg_s2: float, max_stop_s: float = DEFAULT_MAX_STOP_S) -> float:
    """Rate alpha (1/s^2) at which the trajectory clock may slow from 1 to 0.

    While the clock rate r falls at alpha, the commanded joint acceleration
    is qdd * r^2 - alpha * qd, bounded by A + alpha * V for the path's peak
    acceleration A and speed V. The largest alpha that keeps that within
    the limit is (limit - A) / V. It is floored so a stop never takes longer
    than ``max_stop_s``; only a path that already runs at its acceleration
    limit can then exceed it briefly.
    """
    floor = 1.0 / max_stop_s
    if path_max_speed_deg_s <= 1e-9:
        return float("inf")
    budget = max(0.0, max_acc_deg_s2 - path_max_acc_deg_s2)
    return max(budget / path_max_speed_deg_s, floor)
