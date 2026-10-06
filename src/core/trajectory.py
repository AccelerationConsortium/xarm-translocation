"""Coordinated rail + arm trajectory validation (no hardware I/O).

A *trajectory* is a list of time-stamped waypoints, each carrying the
absolute linear-rail position and every arm joint angle. This module
checks such a request against the cell's limits and the arm's measured
start state and returns a full report. It never commands hardware, so it
is safe to call in any graph mode and without a claim.

Units and conventions (fixed, not negotiable per request):

- ``t``          seconds from trajectory start; the first waypoint is at 0
                 and times strictly increase.
- ``rail_mm``    absolute rail position in mm from the rail's homed origin
                 (0 mm = Home, 700 mm = Cytation on this cell).
- ``joints_deg`` arm joint angles in degrees, base to wrist (J1..J5 on
                 the xArm5), exactly ``num_joints`` entries.

Why validation collects *every* error instead of stopping at the first:
the caller is a planner, and one round trip that lists all violations
beats a fix-one-resubmit loop against a shared robot.

Execution model this validates against (see
``src/docs/TRAJECTORY_PLAN.md``): the rail is a Modbus-RTU servo behind
the control box, moved one segment at a time at a per-segment constant
speed, while the arm is streamed in servo mode against the rail's
measured position. That is why segments have a minimum duration (one
rail status poll) and why a moving rail needs at least the servo's
minimum commandable speed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

try:
    from .xarm_utils import validate_track_position
except ImportError:  # pragma: no cover - flat ``core.*`` import path
    from core.xarm_utils import validate_track_position


# Mirrors XArmController._validate_track_position / _validate_track_speed.
DEFAULT_RAIL_LIMITS_MM: Tuple[float, float] = (0.0, 700.0)
DEFAULT_RAIL_SPEED_LIMITS_MM_S: Tuple[float, float] = (1.0, 1000.0)

# One rail status poll in the SDK is 100 ms; a segment shorter than that
# cannot be observed, let alone tracked.
DEFAULT_MIN_SEGMENT_S = 0.1
DEFAULT_MAX_DURATION_S = 600.0

# What the executor can and cannot promise. Returned verbatim in every
# report so the planner sees the contract next to its verdict.
EXECUTION_MODEL: Dict[str, Any] = {
    "synchronization": "rail-indexed",
    "description": (
        "The rail is not a planner axis: it is a Modbus-RTU servo moved "
        "one segment at a time at a constant per-segment speed. The arm "
        "is streamed in servo mode and interpolated against the rail's "
        "measured position, so the rail defines the timeline and path "
        "shape is preserved when rail timing drifts."
    ),
    "arm_joint_timing_ms": 10,
    "rail_readback_latency_ms": [10, 30],
    "rail_speed_resolution_mm_s": 0.15,
    "rail_acceleration": "fixed in the servo; not commandable per segment",
    "rail_segment_timing_error_ms": [20, 200],
    "rail_velocity_profile": "piecewise-constant per segment",
    "stop_skew_ms": [10, 50],
    "simulator": (
        "The Docker simulator has no rail; track commands return success "
        "without moving. Rail timing can only be measured on hardware."
    ),
}


@dataclass(frozen=True)
class Waypoint:
    t: float
    rail_mm: float
    joints_deg: Tuple[float, ...]

    @classmethod
    def from_mapping(cls, data: Dict[str, Any]) -> "Waypoint":
        return cls(
            t=float(data["t"]),
            rail_mm=float(data["rail_mm"]),
            joints_deg=tuple(float(j) for j in data["joints_deg"]),
        )


@dataclass
class TrajectoryLimits:
    joint_limits_deg: Sequence[Tuple[float, float]]
    max_joint_speed_deg_s: float
    rail_limits_mm: Tuple[float, float] = DEFAULT_RAIL_LIMITS_MM
    rail_speed_limits_mm_s: Tuple[float, float] = DEFAULT_RAIL_SPEED_LIMITS_MM_S
    rail_danger_zones: List[Dict[str, Any]] = field(default_factory=list)
    min_segment_s: float = DEFAULT_MIN_SEGMENT_S
    max_duration_s: float = DEFAULT_MAX_DURATION_S

    @property
    def num_joints(self) -> int:
        return len(self.joint_limits_deg)


@dataclass
class StartState:
    """Measured state at validation time. ``None`` means unreadable."""
    joints_deg: Optional[Sequence[float]]
    rail_mm: Optional[float]


@dataclass
class StartTolerance:
    joint_deg: float = 1.0
    rail_mm: float = 2.0


def _error(code: str, message: str, index: Optional[int] = None) -> Dict[str, Any]:
    err: Dict[str, Any] = {"code": code, "message": message}
    if index is not None:
        err["index"] = index
    return err


def _is_finite(*values: float) -> bool:
    return all(math.isfinite(v) for v in values)


def validate_trajectory(
    waypoints: Sequence[Waypoint],
    limits: TrajectoryLimits,
    start: StartState,
    tolerance: Optional[StartTolerance] = None,
) -> Dict[str, Any]:
    """Validate a trajectory and return a report.

    The report always has ``valid``, ``errors``, ``summary``,
    ``start_state`` and ``execution_model``. ``valid`` is True iff
    ``errors`` is empty. Per-segment speeds are the implied constant
    speeds between consecutive waypoints; the executor uses exactly
    these for the rail, so they are what the planner must stay under.
    """
    tolerance = tolerance or StartTolerance()
    errors: List[Dict[str, Any]] = []
    n = limits.num_joints

    # ── Shape and finiteness ──────────────────────────────────────────
    if len(waypoints) < 2:
        errors.append(_error(
            "too_few_waypoints",
            f"A trajectory needs at least 2 waypoints, got {len(waypoints)}",
        ))

    for i, wp in enumerate(waypoints):
        if not _is_finite(wp.t, wp.rail_mm, *wp.joints_deg):
            errors.append(_error("non_finite", "Waypoint contains NaN or infinity", i))
        if len(wp.joints_deg) != n:
            errors.append(_error(
                "joint_count",
                f"Expected exactly {n} joint angles (base to wrist), got {len(wp.joints_deg)}",
                i,
            ))

    if errors:
        # Later checks assume well-formed waypoints.
        return _report(False, errors, waypoints, limits, start, None, [])

    # ── Timing ───────────────────────────────────────────────────────
    if waypoints[0].t != 0:
        errors.append(_error(
            "time_not_from_zero",
            f"First waypoint must be at t=0 s, got {waypoints[0].t}",
            0,
        ))
    for i in range(1, len(waypoints)):
        dt = waypoints[i].t - waypoints[i - 1].t
        if dt <= 0:
            errors.append(_error(
                "time_not_increasing",
                f"t must strictly increase; waypoint {i} at {waypoints[i].t} s "
                f"follows {waypoints[i - 1].t} s",
                i,
            ))
        elif dt < limits.min_segment_s:
            errors.append(_error(
                "segment_too_short",
                f"Segment into waypoint {i} lasts {dt:.3f} s; the rail cannot be "
                f"tracked below {limits.min_segment_s} s (one status poll)",
                i,
            ))
    duration = waypoints[-1].t - waypoints[0].t
    if duration > limits.max_duration_s:
        errors.append(_error(
            "duration_exceeded",
            f"Trajectory lasts {duration:.1f} s; limit is {limits.max_duration_s} s",
        ))

    # ── Bounds ───────────────────────────────────────────────────────
    lo_rail, hi_rail = limits.rail_limits_mm
    for i, wp in enumerate(waypoints):
        for j, (angle, (lo, hi)) in enumerate(zip(wp.joints_deg, limits.joint_limits_deg)):
            if angle < lo or angle > hi:
                errors.append(_error(
                    "joint_out_of_range",
                    f"J{j + 1} = {angle} deg outside [{lo}, {hi}]",
                    i,
                ))
        ok, msg = validate_track_position(wp.rail_mm, (lo_rail, hi_rail), limits.rail_danger_zones)
        if not ok:
            code = "rail_danger_zone" if "danger zone" in (msg or "") else "rail_out_of_range"
            errors.append(_error(code, msg or "Rail position rejected", i))

    # ── Implied segment speeds ───────────────────────────────────────
    segments: List[Dict[str, Any]] = []
    lo_speed, hi_speed = limits.rail_speed_limits_mm_s
    for i in range(1, len(waypoints)):
        prev, cur = waypoints[i - 1], waypoints[i]
        dt = cur.t - prev.t
        if dt <= 0:
            continue  # already reported
        rail_speed = abs(cur.rail_mm - prev.rail_mm) / dt
        joint_speeds = [abs(a - b) / dt for a, b in zip(cur.joints_deg, prev.joints_deg)]
        joint_speed = max(joint_speeds) if joint_speeds else 0.0
        segments.append({
            "index": i,
            "dt_s": round(dt, 6),
            "rail_speed_mm_s": round(rail_speed, 3),
            "joint_speed_deg_s": round(joint_speed, 3),
        })
        if rail_speed > hi_speed:
            errors.append(_error(
                "rail_speed_too_high",
                f"Segment into waypoint {i} needs {rail_speed:.1f} mm/s on the rail; "
                f"limit is {hi_speed} mm/s",
                i,
            ))
        elif 0 < rail_speed < lo_speed:
            errors.append(_error(
                "rail_speed_too_low",
                f"Segment into waypoint {i} needs {rail_speed:.3f} mm/s on the rail; "
                f"the servo cannot be commanded below {lo_speed} mm/s. Hold the rail "
                f"still or move it faster",
                i,
            ))
        if joint_speed > limits.max_joint_speed_deg_s:
            errors.append(_error(
                "joint_speed_too_high",
                f"Segment into waypoint {i} needs {joint_speed:.1f} deg/s on a joint; "
                f"limit at the current safety level is {limits.max_joint_speed_deg_s} deg/s",
                i,
            ))

    # ── Start state ──────────────────────────────────────────────────
    first = waypoints[0]
    start_report: Dict[str, Any] = {
        "joints_deg": list(start.joints_deg) if start.joints_deg is not None else None,
        "rail_mm": start.rail_mm,
        "tolerance": {"joint_deg": tolerance.joint_deg, "rail_mm": tolerance.rail_mm},
    }
    if start.joints_deg is None:
        errors.append(_error(
            "start_state_unavailable",
            "Current joint angles could not be read; refusing to validate the start state",
        ))
    else:
        measured = list(start.joints_deg)[:n]
        if len(measured) < n:
            errors.append(_error(
                "start_state_unavailable",
                f"Read {len(measured)} joint angles, expected {n}",
            ))
        else:
            deltas = [abs(a - b) for a, b in zip(first.joints_deg, measured)]
            worst = max(deltas)
            start_report["joint_error_deg"] = round(worst, 3)
            if worst > tolerance.joint_deg:
                j = deltas.index(worst) + 1
                errors.append(_error(
                    "start_joint_mismatch",
                    f"First waypoint is {worst:.2f} deg from the measured J{j}; "
                    f"tolerance is {tolerance.joint_deg} deg",
                    0,
                ))
    if start.rail_mm is None:
        errors.append(_error(
            "start_state_unavailable",
            "Current rail position could not be read; refusing to validate the start state",
        ))
    else:
        rail_err = abs(first.rail_mm - start.rail_mm)
        start_report["rail_error_mm"] = round(rail_err, 3)
        if rail_err > tolerance.rail_mm:
            errors.append(_error(
                "start_rail_mismatch",
                f"First waypoint is {rail_err:.2f} mm from the measured rail position; "
                f"tolerance is {tolerance.rail_mm} mm",
                0,
            ))

    return _report(not errors, errors, waypoints, limits, start, start_report, segments)


def _report(
    valid: bool,
    errors: List[Dict[str, Any]],
    waypoints: Sequence[Waypoint],
    limits: TrajectoryLimits,
    start: StartState,
    start_report: Optional[Dict[str, Any]],
    segments: List[Dict[str, Any]],
) -> Dict[str, Any]:
    duration = (waypoints[-1].t - waypoints[0].t) if len(waypoints) >= 2 else 0.0
    rail_travel = sum(
        abs(waypoints[i].rail_mm - waypoints[i - 1].rail_mm) for i in range(1, len(waypoints))
    )
    summary = {
        "waypoints": len(waypoints),
        "num_joints": limits.num_joints,
        "duration_s": round(duration, 6),
        "rail_travel_mm": round(rail_travel, 3),
        "max_rail_speed_mm_s": max((s["rail_speed_mm_s"] for s in segments), default=0.0),
        "max_joint_speed_deg_s": max((s["joint_speed_deg_s"] for s in segments), default=0.0),
        "segments": segments,
        "limits": {
            "joint_limits_deg": [list(l) for l in limits.joint_limits_deg],
            "max_joint_speed_deg_s": limits.max_joint_speed_deg_s,
            "rail_limits_mm": list(limits.rail_limits_mm),
            "rail_speed_limits_mm_s": list(limits.rail_speed_limits_mm_s),
            "min_segment_s": limits.min_segment_s,
            "max_duration_s": limits.max_duration_s,
        },
    }
    if start_report is None:
        start_report = {
            "joints_deg": list(start.joints_deg) if start.joints_deg is not None else None,
            "rail_mm": start.rail_mm,
        }
    return {
        "valid": valid,
        "errors": errors,
        "summary": summary,
        "start_state": start_report,
        "execution_model": EXECUTION_MODEL,
    }
