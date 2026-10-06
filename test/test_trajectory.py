"""Pure validation of coordinated rail + arm trajectories (no hardware)."""

from __future__ import annotations

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core.trajectory import (
    EXECUTION_MODEL, StartState, StartTolerance, TrajectoryLimits, Waypoint,
    validate_trajectory,
)

JOINT_LIMITS_5 = [(-360, 360), (-118, 120), (-225, 11), (-97, 180), (-360, 360)]


def limits(**overrides):
    kw = dict(joint_limits_deg=JOINT_LIMITS_5, max_joint_speed_deg_s=90.0)
    kw.update(overrides)
    return TrajectoryLimits(**kw)


def wp(t, rail, joints):
    return Waypoint(t=t, rail_mm=rail, joints_deg=tuple(joints))


ZERO = [0.0] * 5
START = StartState(joints_deg=ZERO, rail_mm=0.0)


def codes(report):
    return [e["code"] for e in report["errors"]]


def test_good_trajectory_is_valid_and_summarised():
    traj = [
        wp(0.0, 0.0, ZERO),
        wp(1.0, 20.0, [10, 0, 0, 0, 0]),
        wp(2.0, 20.0, [10, 5, 0, 0, 0]),   # rail parked, arm moves
    ]
    r = validate_trajectory(traj, limits(), START)
    assert r["valid"] is True and r["errors"] == []
    s = r["summary"]
    assert s["waypoints"] == 3 and s["duration_s"] == 2.0
    assert s["rail_travel_mm"] == 20.0
    assert s["max_rail_speed_mm_s"] == 20.0
    assert s["max_joint_speed_deg_s"] == 10.0
    assert [seg["rail_speed_mm_s"] for seg in s["segments"]] == [20.0, 0.0]
    assert r["start_state"]["joint_error_deg"] == 0.0
    assert r["start_state"]["rail_error_mm"] == 0.0
    assert r["execution_model"] is EXECUTION_MODEL


def test_collects_every_violation_not_just_the_first():
    traj = [
        wp(0.5, 800.0, [0, 0, 0, 0]),          # late start, rail OOR, joint count
        wp(0.5, -5.0, [0, 200, 0, 0, 0]),      # time not increasing (reported after shape pass)
    ]
    r = validate_trajectory(traj, limits(), START)
    assert r["valid"] is False
    # Shape errors short-circuit the rest: a 4-joint waypoint cannot be bound-checked.
    assert codes(r) == ["joint_count"]

    traj = [
        wp(0.5, 800.0, ZERO),
        wp(0.5, -5.0, [0, 200, 0, 0, 0]),
    ]
    r = validate_trajectory(traj, limits(), START)
    got = codes(r)
    for expected in ("time_not_from_zero", "time_not_increasing", "rail_out_of_range",
                     "joint_out_of_range", "start_rail_mismatch"):
        assert expected in got, (expected, got)


def test_too_few_waypoints():
    r = validate_trajectory([wp(0, 0, ZERO)], limits(), START)
    assert codes(r) == ["too_few_waypoints"]


def test_non_finite_rejected():
    r = validate_trajectory([wp(0, 0, ZERO), wp(1, math.nan, ZERO)], limits(), START)
    assert "non_finite" in codes(r)


def test_segment_shorter_than_rail_poll_rejected():
    r = validate_trajectory([wp(0, 0, ZERO), wp(0.05, 1, ZERO)], limits(), START)
    assert codes(r) == ["segment_too_short"]


def test_duration_limit():
    r = validate_trajectory([wp(0, 0, ZERO), wp(601, 0, ZERO)], limits(), START)
    assert codes(r) == ["duration_exceeded"]


def test_rail_speed_limits_both_ends():
    # 1000 mm/s cap; sub-1 mm/s is below the servo's commandable speed.
    too_fast = [wp(0, 0, ZERO), wp(0.5, 600, ZERO)]
    too_slow = [wp(0, 0, ZERO), wp(10, 5, ZERO)]
    assert codes(validate_trajectory(too_fast, limits(), START)) == ["rail_speed_too_high"]
    assert codes(validate_trajectory(too_slow, limits(), START)) == ["rail_speed_too_low"]
    # A rail that stays still is fine at any duration.
    still = [wp(0, 0, ZERO), wp(10, 0, [5, 0, 0, 0, 0])]
    assert validate_trajectory(still, limits(), START)["valid"]


def test_joint_speed_uses_safety_scaled_limit():
    traj = [wp(0, 0, ZERO), wp(1, 0, [100, 0, 0, 0, 0])]
    assert codes(validate_trajectory(traj, limits(max_joint_speed_deg_s=90), START)) == ["joint_speed_too_high"]
    assert validate_trajectory(traj, limits(max_joint_speed_deg_s=180), START)["valid"]


def test_rail_danger_zone_blocks():
    zones = [{"name": "sash", "start": 300, "end": 400, "block_movement": True}]
    traj = [wp(0, 0, ZERO), wp(20, 350, ZERO)]
    r = validate_trajectory(traj, limits(rail_danger_zones=zones), START)
    assert codes(r) == ["rail_danger_zone"]


def test_start_state_tolerances():
    traj = [wp(0, 10.0, [2.0, 0, 0, 0, 0]), wp(1, 10.0, ZERO)]
    r = validate_trajectory(traj, limits(), START)                     # 1 deg / 2 mm defaults
    assert sorted(codes(r)) == ["start_joint_mismatch", "start_rail_mismatch"]
    assert r["start_state"]["joint_error_deg"] == 2.0
    assert r["start_state"]["rail_error_mm"] == 10.0
    r = validate_trajectory(traj, limits(), START, StartTolerance(joint_deg=3, rail_mm=15))
    assert r["valid"]


def test_unreadable_start_state_is_a_refusal_not_a_pass():
    traj = [wp(0, 0, ZERO), wp(1, 0, ZERO)]
    r = validate_trajectory(traj, limits(), StartState(joints_deg=None, rail_mm=None))
    assert codes(r) == ["start_state_unavailable", "start_state_unavailable"]
    r = validate_trajectory(traj, limits(), StartState(joints_deg=[0, 0, 0], rail_mm=0))
    assert codes(r) == ["start_state_unavailable"]


def test_waypoint_from_mapping_coerces():
    w = Waypoint.from_mapping({"t": 1, "rail_mm": "2.5", "joints_deg": [1, 2, 3, 4, 5]})
    assert w == Waypoint(1.0, 2.5, (1.0, 2.0, 3.0, 4.0, 5.0))
