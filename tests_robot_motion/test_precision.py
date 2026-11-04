"""Offline tests for submillimetre Cartesian moves: the completion tolerance
that also decides whether a move is sent, commissioned precision profiles,
and the no-op report.

Replays request 2c809d77 (2026-11-04, UR5e): a 0.457 mm downward probe at
0.5 mm/s answered completed: true, moved: false under the default 0.5 mm
tolerance, and nothing was sent. The fake arm of test_motion starts at TCP
(300, 0, 300) mm with zero rotation; limits are fixture values.
"""

import math
from uuid import uuid4

import pytest
from pydantic import ValidationError

from robot_motion.drivers.ur_motion import MotionLimits, PrecisionProfile
from tests_robot_motion.test_motion import HEADERS, MOTION, Rig, connected, ik, settings

pytest.importorskip("scipy")

LINEAR = "/control/freehand/linear"
JOG = "/control/freehand/relative"
HOME = {"x": 300.0, "y": 0.0, "z": 300.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
PROBE = {
    "position_tolerance_mm": 0.05,
    "orientation_tolerance_deg": 0.05,
    "max_translation_mm": 2.0,
    "max_rotation_deg": 0.5,
    "max_linear_speed_mm_s": 2.0,
    "commissioning_note": "offline fixture",
}


def probe_rig(**profile):
    return Rig(settings({"precision_profiles": {"probe": {**PROBE, **profile}}}))


def limits(client):
    return client.get("/status").json()["details"]["control_session"]["limits"]


# ── the observed failure ─────────────────────────────────────────────
def test_the_reported_probe_is_an_explicit_no_op_under_the_default_tolerance():
    rig = Rig()
    with rig.client() as client:
        held = connected(client)
        probe = {**HOME, "z": 300 - 0.457, "speed": 0.5, "request_id": str(uuid4())}
        reply = client.post(LINEAR, json=probe, headers=held)
        assert reply.status_code == 200, reply.text
        body = reply.json()
        # The old contract is kept: completed, not moved.
        assert body["completed"] is True and body["moved"] is False
        assert body["dispatched"] is False
        assert body["no_op_reason"] == "within_completion_tolerance"
        assert body["requested_distance_mm"] == pytest.approx(0.457, abs=1e-6)
        assert body["displacement_mm"] == 0
        assert body["completion"] == {
            "precision": None, "position_tolerance_mm": 0.5, "orientation_tolerance_deg": 0.2,
        }
        assert "0.457 mm away" in body["message"] and "nothing was sent" in body["message"]
        assert rig.control.moves == []


def test_status_publishes_the_completion_tolerance_and_no_profiles_by_default():
    rig = Rig()
    with rig.client() as client:
        published = limits(client)
        assert published["completion"] == {
            "position_tolerance_mm": 0.5,
            "orientation_tolerance_deg": 0.2,
            "position_tolerance_deg": 0.05,
            "no_dispatch_within_mm": 0.5,
            "no_dispatch_within_deg": 0.05,
        }
        # Nothing commissioned: a client knows before it starts a probe sequence.
        assert published["precision_profiles"] == {}


def test_status_publishes_each_commissioned_profile():
    rig = probe_rig()
    with rig.client() as client:
        assert limits(client)["precision_profiles"] == {"probe": PROBE}


# ── refusals: precision unavailable or out of bounds ─────────────────
def test_a_precision_that_is_not_commissioned_is_refused_before_anything_is_sent():
    rig = Rig()
    with rig.client() as client:
        held = connected(client)
        reply = client.post(LINEAR, json={**HOME, "z": 299.75, "precision": "probe"}, headers=held)
        assert reply.status_code == 422
        detail = reply.json()["detail"]
        assert detail["error_code"] == "precision_unavailable"
        assert detail["available"] == [] and detail["default_position_tolerance_mm"] == 0.5
        assert "no precision profiles" in detail["reason"]
        assert rig.control.moves == [] and rig.control.queries == []


def test_an_unknown_profile_names_the_ones_that_exist():
    rig = probe_rig()
    with rig.client() as client:
        held = connected(client)
        detail = client.post(JOG, json={"dz": -0.25, "precision": "fine"}, headers=held).json()["detail"]
        assert detail["error_code"] == "precision_unavailable" and detail["available"] == ["probe"]
        assert rig.control.moves == []


@pytest.mark.parametrize(
    "move, exceeded",
    [
        ({"dz": -3.0}, "translation_mm"),
        ({"dz": -0.25, "speed": 5}, "speed_mm_s"),
    ],
)
def test_a_jog_outside_the_profile_bounds_is_refused(move, exceeded):
    rig = probe_rig()
    with rig.client() as client:
        held = connected(client)
        reply = client.post(JOG, json={**move, "precision": "probe"}, headers=held)
        assert reply.status_code == 422
        detail = reply.json()["detail"]
        assert detail["error_code"] == "precision_bounds_exceeded"
        assert list(detail["exceeded"]) == [exceeded]
        assert rig.control.moves == []


def test_a_turn_outside_the_profile_bounds_is_refused():
    rig = probe_rig()
    with rig.client() as client:
        held = connected(client)
        reply = client.post(LINEAR, json={**HOME, "z": 299.75, "yaw": 1.0, "precision": "probe"}, headers=held)
        detail = reply.json()["detail"]
        assert detail["error_code"] == "precision_bounds_exceeded" and "rotation_deg" in detail["exceeded"]
        assert rig.control.moves == []


def test_the_controller_safety_limits_still_refuse_a_precision_move():
    rig = probe_rig()
    rig.control.pose_ok = False
    with rig.client() as client:
        held = connected(client)
        refused = client.post(LINEAR, json={**HOME, "z": 299.75, "precision": "probe"}, headers=held)
        assert refused.status_code == 422
        assert refused.json()["detail"]["error_code"] == "outside_controller_limits"
        assert rig.control.moves == []


def test_the_workspace_box_still_refuses_a_precision_move():
    # The fixture box tops out at z 450 mm; start 0.1 mm below it, probe 0.25 mm up.
    rig = probe_rig()
    rig.world.q = ik([0.3, 0.0, 0.4499, 0.0, 0.0, 0.0])
    with rig.client() as client:
        held = connected(client)
        reply = client.post(JOG, json={"dz": 0.25, "precision": "probe"}, headers=held)
        assert reply.status_code == 422
        assert reply.json()["detail"]["error_code"] == "target_outside_workspace"
        assert rig.control.moves == []


# ── dispatch and the measured result ─────────────────────────────────
@pytest.mark.parametrize("probe_mm", [0.1, 0.25, 0.5])
def test_a_submillimetre_probe_is_sent_once_and_reports_its_displacement(probe_mm):
    rig = probe_rig()
    with rig.client() as client:
        held = connected(client)
        reply = client.post(
            LINEAR, json={**HOME, "z": 300 - probe_mm, "speed": 0.5, "precision": "probe"}, headers=held
        )
        assert reply.status_code == 200, reply.text
        body = reply.json()
        assert body["moved"] is True and body["dispatched"] is True and body["no_op_reason"] is None
        assert len(rig.control.moves) == 1
        kind, pose, speed, _, _ = rig.control.moves[0]
        assert kind == "moveL" and pose[2] == pytest.approx(0.3 - probe_mm / 1000)
        assert speed == pytest.approx(0.0005)
        assert body["start_tcp_mm_rpy_deg"][2] == pytest.approx(300)
        assert body["measured_tcp_mm_rpy_deg"][2] == pytest.approx(300 - probe_mm)
        assert body["displacement_xyz_mm"] == pytest.approx([0, 0, -probe_mm], abs=1e-4)
        assert body["displacement_mm"] == pytest.approx(probe_mm, abs=1e-4)
        assert body["requested_distance_mm"] == pytest.approx(probe_mm, abs=1e-4)
        assert body["target_error_mm"] <= 0.05
        assert body["completion"] == {
            "precision": "probe", "position_tolerance_mm": 0.05, "orientation_tolerance_deg": 0.05,
        }
        assert body["message"] == "Move complete"


def test_a_relative_probe_takes_the_profile_speed_cap_by_default():
    rig = probe_rig()
    with rig.client() as client:
        held = connected(client)
        body = client.post(JOG, json={"dz": -0.25, "precision": "probe"}, headers=held).json()
        assert body["moved"] is True and body["speed"] == 2.0
        assert rig.control.moves[0][2] == pytest.approx(0.002)


def test_within_the_profile_tolerance_is_still_an_explicit_no_op():
    rig = probe_rig()
    with rig.client() as client:
        held = connected(client)
        body = client.post(JOG, json={"dz": -0.03, "precision": "probe"}, headers=held).json()
        assert body["moved"] is False and body["no_op_reason"] == "within_completion_tolerance"
        assert body["completion"]["position_tolerance_mm"] == 0.05
        assert "0.05 mm completion tolerance" in body["message"]
        assert rig.control.moves == []


def test_a_repeated_request_id_is_never_sent_twice():
    # A client that timed out and resends the same id gets a refusal, not a second probe.
    rig = probe_rig()
    with rig.client() as client:
        held = connected(client)
        probe = {"dz": -0.25, "precision": "probe", "request_id": str(uuid4())}
        assert client.post(JOG, json=probe, headers=held).json()["moved"] is True
        again = client.post(JOG, json=probe, headers=held)
        assert again.status_code == 422 and again.json()["detail"]["error_code"] == "request_replayed"
        assert len(rig.control.moves) == 1


def test_an_arm_that_stops_short_of_the_profile_tolerance_is_not_counted_as_arrived():
    # The controller settles 0.1 mm short: within the default 0.5 mm, outside
    # the 0.05 mm profile. The move fails at its deadline and stops; it is
    # never reported as a completed probe.
    rig = probe_rig()
    original = rig.control.moveL

    def short(pose, speed, acceleration, asynchronous):
        accepted = original(pose, speed, acceleration, asynchronous)
        rig.world.command(ik([pose[0], pose[1], pose[2] + 0.0001, *pose[3:]]), speed / 0.3)
        return accepted

    rig.control.moveL = short
    with rig.client() as client:
        held = connected(client)
        failed = client.post(JOG, json={"dz": -0.25, "speed": 2, "precision": "probe"}, headers=held)
        assert failed.status_code == 500 and "deadline" in failed.json()["reason"]
        assert rig.control.stops and rig.control.stops[0][0] == "stopL"


# ── configuration ────────────────────────────────────────────────────
@pytest.mark.parametrize(
    "motion, profile",
    [
        ({}, {"position_tolerance_mm": 0.5}),  # not below the default
        ({"max_linear_speed_mm_s": 1}, {}),  # profile speed over the arm's linear cap
        ({"max_jog_mm": 1}, {}),  # profile translation over the jog cap
        ({}, {"position_tolerance_mm": 0.01}),  # below what a UR measures repeatably
        ({}, {"position_tolerance_mm": 0.3, "max_translation_mm": 0.2}),  # no move could ever be sent
        ({}, {"max_translation_mm": 6}),  # profiles are for probes, not moves
    ],
)
def test_precision_profiles_must_be_coherent(motion, profile):
    with pytest.raises(ValidationError):
        MotionLimits(**{**MOTION, **motion, "precision_profiles": {"probe": {**PROBE, **profile}}})


def test_a_profile_needs_its_commissioning_note():
    with pytest.raises(ValidationError):
        PrecisionProfile(**{k: v for k, v in PROBE.items() if k != "commissioning_note"})


def test_profile_names_are_plain_identifiers():
    with pytest.raises(ValidationError):
        MotionLimits(**{**MOTION, "precision_profiles": {"Probe Fine": PROBE}})
    assert math.isclose(MotionLimits(**{**MOTION, "precision_profiles": {"probe_fine": PROBE}})
                        .precision_profiles["probe_fine"].position_tolerance_mm, 0.05)


# ── reading an outcome back instead of resending ─────────────────────
def test_a_move_and_a_no_op_are_kept_under_their_request_id(tmp_path):
    config = settings({"precision_profiles": {"probe": PROBE}})
    audit = tmp_path / "audit.jsonl"
    config = config.model_copy(update={"control": config.control.model_copy(update={"audit_file": str(audit)})})
    rig = Rig(config)
    moved_id, no_op_id = str(uuid4()), str(uuid4())
    with rig.client() as client:
        held = connected(client)
        client.post(JOG, json={"dz": -0.25, "precision": "probe", "request_id": moved_id}, headers=held)
        client.post(LINEAR, json={**HOME, "z": 299.5 + 0.043, "request_id": no_op_id}, headers=held)
        kept = client.get(f"/control/diagnostics/{moved_id}", headers=held).json()["records"]
        assert [r["kind"] for r in kept] == ["arm.jog_linear", "arm.jog_linear_done"]
        assert kept[0]["status"] is None and kept[0]["precision"] == "probe"
        assert kept[1]["status"] == 200 and kept[1]["moved"] is True
        assert kept[1]["displacement_mm"] == pytest.approx(0.25, abs=1e-4)
        noop = client.get(f"/control/diagnostics/{no_op_id}", headers=held).json()["records"][-1]
        assert noop["moved"] is False and noop["no_op_reason"] == "within_completion_tolerance"
    # After a restart, from the audit file.
    with Rig(rig.config).client() as client:
        kept = client.get(f"/control/diagnostics/{moved_id}", headers=HEADERS).json()["records"]
        assert [r["kind"] for r in kept] == ["arm.jog_linear", "arm.jog_linear_done"]
        assert kept[1]["source"] == "audit_file" and kept[1]["moved"] is True
        assert kept[1]["completion"]["precision"] == "probe"


def test_a_failed_move_is_kept_under_its_request_id():
    rig = Rig()
    rig.world.stuck = True
    request_id = str(uuid4())
    with rig.client() as client:
        held = connected(client)
        failed = client.post(JOG, json={"dz": 2, "request_id": request_id}, headers=held)
        assert failed.status_code == 500
        kept = client.get(f"/control/diagnostics/{request_id}", headers=held).json()["records"]
        assert [r["kind"] for r in kept] == ["arm.jog_linear", "arm_failed"]
        assert kept[1]["latched"] is True and kept[1]["status"] == 500
