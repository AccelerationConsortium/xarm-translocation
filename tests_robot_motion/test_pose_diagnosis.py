"""Offline tests for the pose diagnosis (POST /control/diagnose/pose), the
classification of the controller's boolean safety queries (ur_checks.py) and
the request records (GET /control/diagnostics/{request_id}).

The fake controller of test_motion answers its safety queries about a target
as each test sets, and True about the arm's own current state (the probe)
unless probe_ok is False. Every diagnosis test checks that nothing moved,
stopped, reset, connected or entered teach mode.
"""

import json
import math
from uuid import uuid4

import pytest

from robot_motion.drivers.ur_checks import BOOLEAN_ONLY
from tests_robot_motion.test_motion import HEADERS, Rig, connected, settings, wait_for

pytest.importorskip("scipy")

TARGET = {"x": 310.0, "y": 0.0, "z": 300.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
CONTROLLER_CHECKS = ("ik_solution", "software_envelope_at_ik", "controller_joint_limits_at_ik", "controller_pose_limits")


def audited_rig(tmp_path):
    config = settings()
    audit = tmp_path / "audit.jsonl"
    config = config.model_copy(update={"control": config.control.model_copy(update={"audit_file": str(audit)})})
    return Rig(config), audit


def kinds(audit):
    return [json.loads(line)["kind"] for line in audit.read_text().splitlines()]


def diagnose(client, headers=HEADERS, **change):
    request_id = str(uuid4())
    response = client.post("/control/diagnose/pose", json={**TARGET, **change, "request_id": request_id}, headers=headers)
    assert response.status_code == 200, response.text
    body = response.json()
    assert body["request_id"] == request_id and body["moved"] is False
    return body


def untouched(rig, audit=None, connects=1):
    """The diagnosis sent no move, stop, reset, connect or teach command."""
    control = rig.control
    assert control.moves == [] and control.stops == []
    assert control.teach_calls == [] and control.zeroed == 0 and control.script_stops == 0
    if audit is not None:
        events = kinds(audit)
        assert events.count("connect") == connects
        assert not {"reset", "disconnect", "stop"} & set(events)
        assert not any(kind.startswith(("arm.", "gripper")) for kind in events)


def results(body):
    return {name: check["result"] for name, check in body["checks"].items()}


def test_a_reachable_pose_passes_every_check_with_its_evidence(tmp_path):
    rig, audit = audited_rig(tmp_path)
    rig.observer.read_versions = lambda: {"polyscope": "URSoftware 5.17.3.0 (fixture)"}
    with rig.client() as client:
        connected(client)
        body = diagnose(client)
        assert body["verdict"] == "passed" and body["decided_by"] == []
        assert set(results(body).values()) == {"passed"}
        # What was checked, in what units, against which state.
        assert body["target"]["requested"] == TARGET
        assert body["target"]["used_pose_m_rotvec"] == pytest.approx([0.31, 0, 0.3, 0, 0, 0])
        assert "rotation vector" in body["target"]["units"]["used"]
        assert body["state"]["joints_deg"] == [0.0] * 6
        assert body["state"]["tcp_mm_rpy_deg"] == pytest.approx([300, 0, 300, 0, 0, 0])
        assert body["state"]["translation_mm"] == pytest.approx(10.0)
        ik = body["checks"]["ik_solution"]
        assert len(ik["joints_deg"]) == 6 and ik["qnear_joints_deg"] == [0.0] * 6
        assert body["tcp_config"]["tcp_offset_mm_rpy_deg"] == pytest.approx([0, 0, 200, 0, 0, 0])
        versions = body["versions"]
        assert versions["controller"] == {"polyscope": "URSoftware 5.17.3.0 (fixture)"}
        assert versions["robot_model"] == "ur5e" and versions["service"]
        assert "ur_rtde" in versions  # None here: the SDK is not installed offline
        # Kept under its id, and readable without asking again.
        kept = client.get(f"/control/diagnostics/{body['request_id']}", headers=HEADERS).json()["records"]
        assert [record["kind"] for record in kept] == ["pose_diagnosis"]
        assert kept[0]["verdict"] == "passed"
        untouched(rig, audit)
    assert "pose_diagnosis" in kinds(audit)


def test_a_confirmed_safety_rejection_says_the_specific_limit_is_unknown(tmp_path):
    rig, audit = audited_rig(tmp_path)
    rig.control.pose_ok = False
    with rig.client() as client:
        connected(client)
        body = diagnose(client)
        untouched(rig, audit)
    assert body["verdict"] == "rejected" and body["decided_by"] == ["controller_pose_limits"]
    pose = body["checks"]["controller_pose_limits"]
    assert pose["result"] == "rejected" and pose["completed"] is True and pose["answer"] is False
    assert pose["probe"] == {"of": "current TCP pose", "completed": True, "answer": True, "error": None}
    assert pose["specific_limit"] == BOOLEAN_ONLY and pose["specific_limit"]["known"] is False
    assert results(body)["controller_joint_limits_at_ik"] == "passed"


def test_a_false_the_probe_cannot_confirm_is_indeterminate_not_a_limit(tmp_path):
    rig, audit = audited_rig(tmp_path)
    rig.control.pose_ok = False
    rig.control.probe_ok = False  # the arm's own pose answers false too
    with rig.client() as client:
        connected(client)
        body = diagnose(client)
        untouched(rig, audit)
    assert body["verdict"] == "indeterminate"
    pose = body["checks"]["controller_pose_limits"]
    assert pose["result"] == "indeterminate" and pose["probe"]["answer"] is False
    assert "cannot be told from a query that did not complete" in pose["reason"]


@pytest.mark.parametrize("break_it, reason", [
    (lambda control: control.raising.update(isPoseWithinSafetyLimits="RTDE command timed out"),
     "RTDE command timed out"),
    (lambda control: control.drop_link.add("isPoseWithinSafetyLimits"), "control_script_stopped after the query"),
])
def test_an_sdk_query_that_does_not_complete_is_query_failed(tmp_path, break_it, reason):
    rig, audit = audited_rig(tmp_path)
    break_it(rig.control)
    with rig.client() as client:
        connected(client)
        body = diagnose(client)
        untouched(rig, audit)
    assert body["verdict"] == "query_failed" and body["decided_by"] == ["controller_pose_limits"]
    pose = body["checks"]["controller_pose_limits"]
    assert pose["result"] == "query_failed" and pose["completed"] is False
    assert pose["reason"] == reason
    assert "probe" not in pose  # a failed query is not probed or called a refusal


def test_no_ik_solution_is_confirmed_and_skips_the_checks_that_need_one(tmp_path):
    rig, audit = audited_rig(tmp_path)
    rig.control.has_solution = False
    with rig.client() as client:
        connected(client)
        body = diagnose(client)
        untouched(rig, audit)
    assert body["verdict"] == "rejected" and body["decided_by"] == ["ik_solution"]
    checks = results(body)
    assert checks["ik_solution"] == "rejected"
    assert checks["software_envelope_at_ik"] == checks["controller_joint_limits_at_ik"] == "not_checked"
    assert checks["controller_pose_limits"] == "passed"  # still asked: it needs no IK solution
    assert rig.control.ik_calls == 0  # getInverseKinematics is never asked without a solution


def test_without_a_usable_connection_nothing_is_asked(tmp_path):
    rig, audit = audited_rig(tmp_path)
    with rig.client() as client:
        wait_for(lambda: client.get("/status").json()["details"].get("robotmode"), what="first observation")
        # No session: the diagnosis does not connect one.
        body = diagnose(client)
        assert body["verdict"] == "not_checked"
        assert body["checks"]["connection"]["reason"].startswith("no_control_session")
        assert all(results(body)[name] == "not_checked" for name in CONTROLLER_CHECKS)
        assert results(body)["software_workspace"] == "passed"  # local, always checked
        assert rig.receiver_calls == [] and body["state"] is None
        # A session whose control script stopped: no query is sent, because
        # ur_rtde would re-upload its script to answer one.
        connected(client)
        rig.control.program_running = False
        rig.control.queries.clear()
        body = diagnose(client)
        assert body["checks"]["connection"]["reason"].startswith("control_script_stopped")
        assert rig.control.queries == []
        untouched(rig, audit)


def test_no_queries_while_the_arm_is_commanded_or_in_manual_mode(tmp_path):
    rig, audit = audited_rig(tmp_path)
    with rig.client() as client:
        connected(client)
        control = client.app.state.control
        rig.control.queries.clear()
        with control.executor._lock:
            body = diagnose(client)
        assert body["checks"]["connection"]["reason"].startswith("motion_in_progress")
        control.executor._manual = True
        body = diagnose(client)
        assert body["checks"]["connection"]["reason"].startswith("manual_mode")
        control.executor._manual = False
        assert rig.control.queries == []
        untouched(rig, audit)


def test_a_target_outside_the_software_box_is_rejected_without_a_session(tmp_path):
    rig, audit = audited_rig(tmp_path)
    with rig.client() as client:
        wait_for(lambda: client.get("/status").json()["details"].get("robotmode"), what="first observation")
        body = diagnose(client, x=600.0)
        untouched(rig, audit, connects=0)
    assert body["verdict"] == "rejected" and body["decided_by"] == ["software_workspace"]
    box = body["checks"]["software_workspace"]
    assert box["axes"] == ["x"] and box["workspace_mm"]["x"] == [150, 450]


def test_diagnosis_needs_login_but_no_claim(tmp_path):
    rig, audit = audited_rig(tmp_path)
    with rig.client() as client:
        connected(client)  # someone else holds the claim
        assert client.post("/control/diagnose/pose", json=TARGET).status_code == 401
        assert diagnose(client, headers=HEADERS)["verdict"] == "passed"
        untouched(rig, audit)


# ── moves: a false from the controller is classified before it is reported ─
@pytest.mark.parametrize("arrange, status, error", [
    (lambda control: setattr(control, "pose_ok", False), 422, "outside_controller_limits"),
    (lambda control: (setattr(control, "pose_ok", False), setattr(control, "probe_ok", False)),
     502, "controller_check_indeterminate"),
    (lambda control: control.raising.update(isPoseWithinSafetyLimits="RTDE command timed out"),
     502, "planning_failed"),
    (lambda control: (setattr(control, "pose_ok", False), control.drop_link.add("isPoseWithinSafetyLimits")),
     502, "controller_query_failed"),
])
def test_a_refused_move_is_a_limit_only_when_the_controller_confirms_it(tmp_path, arrange, status, error):
    rig, audit = audited_rig(tmp_path)
    arrange(rig.control)
    request_id = str(uuid4())
    with rig.client() as client:
        held = connected(client)
        refused = client.post("/control/freehand/linear", json={**TARGET, "request_id": request_id}, headers=held)
        assert refused.status_code == status, refused.text
        detail = refused.json()["detail"]
        assert detail["error_code"] == error and detail["request_id"] == request_id
        if error != "planning_failed":  # an exception from the fast query stays planning_failed
            assert detail["controller_check"]["query"] == "isPoseWithinSafetyLimits"
        assert rig.control.moves == []
        # Read back by id; no need to resend the move to see why.
        kept = client.get(f"/control/diagnostics/{request_id}", headers=HEADERS).json()["records"]
        assert kept[-1]["kind"] == "arm_refused" and kept[-1]["error_code"] == error
        assert kept[-1]["status"] == status and kept[-1]["request"]["x"] == TARGET["x"]


def test_records_are_read_back_from_the_audit_file_after_a_restart(tmp_path):
    rig, audit = audited_rig(tmp_path)
    rig.control.pose_ok = False
    with rig.client() as client:
        held = connected(client)
        diagnosed = diagnose(client)
        move_id = str(uuid4())
        client.post("/control/freehand/linear", json={**TARGET, "request_id": move_id}, headers=held)
        bad_id = str(uuid4())
        client.post("/control/freehand/linear", json={"x": 1, "request_id": bad_id}, headers=held)
    restarted = Rig(rig.config)
    with restarted.client() as client:
        assert client.get(f"/control/diagnostics/{uuid4()}", headers=HEADERS).status_code == 404
        record = client.get(f"/control/diagnostics/{diagnosed['request_id']}", headers=HEADERS).json()["records"]
        assert record[0]["source"] == "audit_file" and record[0]["verdict"] == "rejected"
        assert record[0]["checks"]["controller_pose_limits"]["result"] == "rejected"
        refused = client.get(f"/control/diagnostics/{move_id}", headers=HEADERS).json()["records"]
        # The move as received, then its refusal.
        assert [r["kind"] for r in refused] == ["arm.move_linear", "arm_refused"]
        assert refused[1]["error_code"] == "outside_controller_limits"
        invalid = client.get(f"/control/diagnostics/{bad_id}", headers=HEADERS).json()["records"]
        assert invalid[0]["kind"] == "request_invalid" and "y" in invalid[0]["fields"]
        assert client.get(f"/control/diagnostics/{move_id}").status_code == 401


# ── trajectory paths and the controller version ──────────────────────
class _Path:
    """A straight joint path of one second (degrees), as path_errors samples it."""

    duration = 1.0

    def __init__(self, start, end):
        self.start, self.end = start, end

    def sample(self, t):
        return ([a + (b - a) * t for a, b in zip(self.start, self.end)],)


class _PathControl:
    def __init__(self, refuse_from_deg, probe_ok=True):
        self.refuse_from_deg, self.probe_ok = refuse_from_deg, probe_ok
        self.start = None

    def isConnected(self):
        return True

    def isProgramRunning(self):
        return True

    def isJointsWithinSafetyLimits(self, q):
        if self.start is None:
            self.start = list(q)
        if q == self.start:
            return self.probe_ok
        return math.degrees(q[0]) < self.refuse_from_deg

    def getForwardKinematics(self, q):
        return [0.3, 0.0, 0.3, 0.0, 0.0, 0.0]


@pytest.mark.parametrize("probe_ok, code", [(True, "outside_controller_limits"), (False, "controller_check_indeterminate")])
def test_a_trajectory_path_is_refused_as_a_limit_only_when_confirmed(probe_ok, code):
    from robot_motion.drivers.ur_motion import MotionLimits
    from robot_motion.trajectory.ur_backend import path_errors
    from tests_robot_motion.test_motion import MOTION

    errors = path_errors(_Path([0.0] * 6, [10.0, 0, 0, 0, 0, 0]), _PathControl(5.0, probe_ok),
                         MotionLimits(**MOTION), step_deg=2.0)
    assert [error["code"] for error in errors] == [code]
    assert errors[0]["controller_check"]["probe"]["of"] == "trajectory's first sample"


def test_the_controller_version_is_a_fixed_read_only_dashboard_query():
    import io

    from robot_motion.drivers.ur import URObserver

    sent = []

    class Conn:
        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

        def sendall(self, data):
            sent.append(data)

        def makefile(self, mode):
            return io.BytesIO(b"Connected: Universal Robots Dashboard Server\nURSoftware 5.17.3.0 (fixture)\n")

    observer = URObserver(settings(), connector=lambda address, timeout: Conn())
    assert observer.read_versions() == {"polyscope": "URSoftware 5.17.3.0 (fixture)"}
    assert sent == [b"PolyscopeVersion\n"]


def test_the_move_slot_is_held_while_the_diagnosis_queries_run(tmp_path):
    rig, audit = audited_rig(tmp_path)
    with rig.client() as client:
        connected(client)
        executor = client.app.state.control.executor
        busy = []
        rig.control.on_plan = lambda: busy.append(executor.busy)  # called inside the joint-limit query
        diagnose(client)
        assert busy == [True]  # no move could start, so no STOP waits behind a query
        assert executor.busy is False
        untouched(rig, audit)
