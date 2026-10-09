"""Pure joint-trajectory interpolation and validation (no hardware)."""

from __future__ import annotations

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core.joint_trajectory import (
    JOINT_EXECUTION_MODEL, MAX_REPORTED_ERRORS, JointLimits, JointPoint, JointTrajectory,
    estimated_velocities, lead_in, lead_in_duration, stop_deceleration,
    validate_joint_trajectory,
)

JOINT_LIMITS_5 = [(-360, 360), (-118, 120), (-225, 11), (-97, 180), (-360, 360)]
ZERO = (0.0,) * 5


def limits(**overrides):
    kw = dict(joint_limits_deg=JOINT_LIMITS_5, max_joint_speed_deg_s=90.0,
              max_joint_acc_deg_s2=500.0, servo_rate_hz=100.0)
    kw.update(overrides)
    return JointLimits(**kw)


def p(t, q, qd=None, qdd=None):
    return JointPoint(t, tuple(q), None if qd is None else tuple(qd),
                      None if qdd is None else tuple(qdd))


def codes(report):
    return [e["code"] for e in report["errors"]]


def ramp(j1_end=10.0, duration=2.0):
    """Positions-only J1 move, three points."""
    return [p(0, ZERO), p(duration / 2, (j1_end / 2, 0, 0, 0, 0)), p(duration, (j1_end, 0, 0, 0, 0))]


# ── Interpolants ──────────────────────────────────────────────────────


def test_quintic_meets_both_end_states():
    a = p(0.0, (1, 2, 3, 4, 5), (0.5, 0, -1, 0, 2), (0.1, 0, 0, -3, 0))
    b = p(0.8, (2, 2, 1, 4, 6), (0, 1, 0, 0, -1), (0, 0, 2, 0, 0))
    traj = JointTrajectory([a, b])
    for point, u in ((a, 0.0), (b, 0.8)):
        i = 0
        q, qd, qdd = traj._eval_segment(i, u)
        assert q == pytest.approx(point.q, abs=1e-9)
        assert qd == pytest.approx(point.qd, abs=1e-9)
        assert qdd == pytest.approx(point.qdd, abs=1e-9)
    assert traj.kind == "quintic"


def test_cubic_meets_positions_and_velocities():
    a = p(0.0, ZERO, (0, 1, 0, 0, 0))
    b = p(0.5, (1, 1, 1, 1, 1), (2, 0, 0, 0, 0))
    traj = JointTrajectory([a, b])
    q0, qd0, _ = traj._eval_segment(0, 0.0)
    q1, qd1, _ = traj._eval_segment(0, 0.5)
    assert q0 == pytest.approx(a.q) and qd0 == pytest.approx(a.qd)
    assert q1 == pytest.approx(b.q) and qd1 == pytest.approx(b.qd)
    assert traj.kind == "cubic"


def test_estimated_velocities_zero_at_ends_and_central_inside():
    pts = [p(0, ZERO), p(1, (10, 0, 0, 0, 0)), p(3, (30, 0, 0, 0, 0)), p(4, (30, 0, 0, 0, 0))]
    v = estimated_velocities(pts)
    assert v[0] == ZERO and v[-1] == ZERO
    # Point 1: slopes 10 (h=1) and 10 (h=2) -> 10.
    assert v[1][0] == pytest.approx(10.0)
    # Point 2: slopes 10 (h=2) and 0 (h=1): (1*10 + 2*0)/3.
    assert v[2][0] == pytest.approx(10.0 / 3.0)


def test_sample_clamps_to_rest_outside_the_trajectory():
    traj = JointTrajectory(ramp())
    q, qd, qdd = traj.sample(-1.0)
    assert q == list(ZERO) and qd == [0.0] * 5 and qdd == [0.0] * 5
    q, qd, _ = traj.sample(99.0)
    assert q[0] == 10.0 and qd == [0.0] * 5
    q, _, _ = traj.sample(1.0)
    assert q[0] == pytest.approx(5.0)


def test_interpolant_is_continuous_across_points():
    traj = JointTrajectory(ramp())
    left = traj._eval_segment(0, 1.0)
    right = traj._eval_segment(1, 0.0)
    assert left[0] == pytest.approx(right[0])
    assert left[1] == pytest.approx(right[1])   # C1 at an interior point


# ── Validation ────────────────────────────────────────────────────────


def test_good_trajectory_is_valid_and_summarised():
    r = validate_joint_trajectory(ramp(), limits(), start_joints=ZERO)
    assert r["valid"] is True and r["errors"] == []
    s = r["summary"]
    assert s["points"] == 3 and s["duration_s"] == 2.0
    assert s["interpolation"] == "cubic_estimated"
    assert s["samples"] > 200
    assert 0 < s["max_joint_speed_deg_s"] < 90
    assert r["start_state"]["joint_error_deg"] == 0.0
    assert r["execution_model"] is JOINT_EXECUTION_MODEL
    assert "not a collision-safety proof" in JOINT_EXECUTION_MODEL["collision_checking"]


def test_shape_errors_short_circuit():
    r = validate_joint_trajectory([p(0, ZERO)], limits(), start_joints=ZERO)
    assert codes(r) == ["too_few_points"]
    r = validate_joint_trajectory([p(0, ZERO), p(1, (0, 0, 0, 0))], limits(), start_joints=ZERO)
    assert codes(r) == ["joint_count"]
    r = validate_joint_trajectory([p(0, ZERO), p(1, (0, math.nan, 0, 0, 0))], limits(), start_joints=ZERO)
    assert codes(r) == ["non_finite"]


def test_derivatives_must_be_all_or_none():
    pts = [p(0, ZERO, ZERO), p(1, (1, 0, 0, 0, 0))]
    assert "inconsistent_derivatives" in codes(validate_joint_trajectory(pts, limits(), start_joints=ZERO))
    pts = [p(0, ZERO, None, ZERO), p(1, (1, 0, 0, 0, 0), None, ZERO)]
    assert "inconsistent_derivatives" in codes(validate_joint_trajectory(pts, limits(), start_joints=ZERO))


def test_timing_rules():
    late = [p(0.5, ZERO), p(1.5, (1, 0, 0, 0, 0))]
    assert codes(validate_joint_trajectory(late, limits(), start_joints=ZERO)) == ["time_not_from_zero"]
    back = [p(0, ZERO), p(1, (1, 0, 0, 0, 0)), p(1, (2, 0, 0, 0, 0))]
    assert codes(validate_joint_trajectory(back, limits(), start_joints=ZERO)) == ["time_not_increasing"]
    dense = [p(0, ZERO), p(0.005, ZERO), p(1, ZERO)]   # 5 ms < 10 ms period at 100 Hz
    assert codes(validate_joint_trajectory(dense, limits(), start_joints=ZERO)) == ["segment_too_short"]
    assert validate_joint_trajectory(dense, limits(servo_rate_hz=200), start_joints=ZERO)["valid"]
    long = [p(0, ZERO), p(601, ZERO)]
    assert codes(validate_joint_trajectory(long, limits(), start_joints=ZERO)) == ["duration_exceeded"]


def test_ends_must_be_at_rest_when_derivatives_are_given():
    moving_end = [p(0, ZERO, ZERO), p(1, (1, 0, 0, 0, 0), (1, 0, 0, 0, 0))]
    r = validate_joint_trajectory(moving_end, limits(), start_joints=ZERO)
    assert "not_at_rest" in codes(r)
    accel_start = [p(0, ZERO, ZERO, (5, 0, 0, 0, 0)), p(1, (1, 0, 0, 0, 0), ZERO, ZERO)]
    assert "not_at_rest" in codes(validate_joint_trajectory(accel_start, limits(), start_joints=ZERO))


def test_speed_is_checked_on_the_interpolant_not_the_chord():
    # Two points 1 s apart and 80 deg apart: the chord is 80 deg/s, but a
    # rest-to-rest cubic peaks at 1.5x the chord = 120 deg/s.
    pts = [p(0, ZERO), p(1, (80, 0, 0, 0, 0))]
    r = validate_joint_trajectory(pts, limits(max_joint_speed_deg_s=90, max_joint_acc_deg_s2=5000),
                                  start_joints=ZERO)
    assert codes(r) == ["joint_speed_too_high"]
    assert r["summary"]["max_joint_speed_deg_s"] == pytest.approx(120.0, rel=1e-3)


def test_acceleration_limit():
    pts = [p(0, ZERO), p(0.5, (20, 0, 0, 0, 0))]   # cubic rest-to-rest: 6 D/T^2 = 480 deg/s^2
    assert validate_joint_trajectory(pts, limits(max_joint_acc_deg_s2=500), start_joints=ZERO)["valid"]
    r = validate_joint_trajectory(pts, limits(max_joint_acc_deg_s2=400), start_joints=ZERO)
    assert codes(r) == ["joint_acc_too_high"]


def test_overshoot_between_points_is_caught():
    # J3's upper limit is 11 deg. Every point is within it, but leaving
    # 10 deg at +30 deg/s the cubic peaks at 12.8 deg before coming back.
    pts = [p(0, ZERO, ZERO), p(1, (0, 0, 10, 0, 0), (0, 0, 30, 0, 0)), p(2, ZERO, ZERO)]
    r = validate_joint_trajectory(pts, limits(max_joint_acc_deg_s2=1e6), start_joints=ZERO)
    assert "joint_out_of_range" in codes(r)
    assert "overshoots" in [e for e in r["errors"] if e["code"] == "joint_out_of_range"][0]["message"]


def test_point_out_of_range():
    pts = [p(0, ZERO), p(1, (0, 200, 0, 0, 0))]
    assert "joint_out_of_range" in codes(validate_joint_trajectory(pts, limits(), start_joints=ZERO))


def test_start_state_rules():
    pts = [p(0, (0.4, 0, 0, 0, 0)), p(1, (1, 0, 0, 0, 0))]
    assert validate_joint_trajectory(pts, limits(), start_joints=ZERO)["valid"]   # 0.5 default
    r = validate_joint_trajectory(pts, limits(), start_joints=ZERO, start_tolerance_deg=0.2)
    assert codes(r) == ["start_joint_mismatch"]
    assert codes(validate_joint_trajectory(pts, limits(), start_joints=None)) == ["start_state_unavailable"]
    assert validate_joint_trajectory(pts, limits(), start_joints=None, check_start=False)["valid"]


def test_error_list_is_capped():
    pts = [p(0, ZERO)] + [p(i * 0.01, (0, 500, 0, 0, 0)) for i in range(1, 300)]
    r = validate_joint_trajectory(pts, limits(max_joint_speed_deg_s=1e9, max_joint_acc_deg_s2=1e12),
                                  start_joints=ZERO)
    assert len(r["errors"]) == MAX_REPORTED_ERRORS + 1
    assert r["errors"][-1]["code"] == "too_many_errors"


# ── Lead-in and stop ─────────────────────────────────────────────────


def test_lead_in_respects_limits_and_ends_at_rest():
    q_from, q_to = ZERO, (0.4, 0, -0.3, 0, 0)
    seg = lead_in(q_from, q_to, max_speed_deg_s=90, max_acc_deg_s2=500)
    assert seg is not None and seg.kind == "quintic"
    assert seg.duration == pytest.approx(lead_in_duration(q_from, q_to, 90, 500))
    peak_v = peak_a = 0.0
    for _, q, qd, qdd in seg.segment_samples(1000):
        peak_v = max(peak_v, max(abs(v) for v in qd))
        peak_a = max(peak_a, max(abs(a) for a in qdd))
    assert peak_v <= 90 + 1e-6 and peak_a <= 500 * (1 + 1e-3)
    assert seg.sample(seg.duration)[0] == pytest.approx(list(q_to))
    assert lead_in(ZERO, ZERO, 90, 500) is None


def test_stop_deceleration():
    assert stop_deceleration(0.0, 0.0, 500) == float("inf")
    # Budget 400 over 40 deg/s -> 10 /s^2 (0.1 s to stop).
    assert stop_deceleration(40.0, 100.0, 500) == pytest.approx(10.0)
    # No budget left: floored so a stop takes at most 2 s.
    assert stop_deceleration(40.0, 500.0, 500) == pytest.approx(0.5)
