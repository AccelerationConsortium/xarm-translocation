"""Pose diagnosis: what the controller says about a target pose, without moving.

The same questions a Cartesian move's planner asks of the target (ur_motion.py)
are asked on the already open control session and classified by ur_checks:
the software workspace box, the controller's inverse kinematics near the
current joints, the commissioned envelope and the controller's joint limits
at that solution, and the controller's pose limits. Nothing here connects,
resets, moves the arm or touches the gripper; with no usable session every
controller check is reported not_checked. It checks the target only, not the
straight line to it, which a move checks as well.
"""

from __future__ import annotations

import math
import time

from . import ur_checks
from .lle_rtde import tcp_to_mm_deg, vector6
from .ur_checks import INDETERMINATE, NOT_CHECKED, PASSED, QUERY_FAILED, REJECTED
from .ur_motion import orientation_error_deg, rpy_to_rotvec

UNITS = {
    "requested": "position mm, orientation roll/pitch/yaw degrees (UR RPY: extrinsic x-y-z), base frame",
    "used": "position m, orientation rotation vector rad (what ur_rtde receives), base frame",
    "joints": "degrees",
}
CHECK_ORDER = (
    "connection", "software_workspace", "ik_solution", "software_envelope_at_ik",
    "controller_joint_limits_at_ik", "controller_pose_limits",
)


def _not_checked(reason):
    return {"result": NOT_CHECKED, "reason": reason}


def _verdict(checks):
    results = {name: check["result"] for name, check in checks.items()}
    rejected = [name for name in CHECK_ORDER if results.get(name) == REJECTED]
    if rejected:
        return REJECTED, rejected
    for outcome in (QUERY_FAILED, INDETERMINATE):
        failed = [name for name in CHECK_ORDER if results.get(name) == outcome]
        if failed:
            return outcome, failed
    if all(result == PASSED for result in results.values()):
        return PASSED, []
    return NOT_CHECKED, [name for name in CHECK_ORDER if results.get(name) == NOT_CHECKED]


VERDICT_REASONS = {
    PASSED: "Every check passed for the target pose (the straight line to it is checked by the move itself)",
    REJECTED: "Confirmed refused by {names}",
    QUERY_FAILED: "{names}: the controller query did not complete; no limit is claimed",
    INDETERMINATE: "{names}: the controller answered false without confirming it; no limit is claimed",
    NOT_CHECKED: "Not checked: {reason}",
}


def diagnose_pose(*, request, limits, control=None, latest=None, blocked=None, clock=time.monotonic):
    """{target, state, checks, verdict, verdict_reason, decided_by}.

    control: the session's serialized control interface, or None without a
    session. latest: the newest control feedback packet (or None). blocked:
    why controller queries must not run now (a move in flight, manual mode),
    or None.
    """
    requested = [request.x, request.y, request.z, request.roll, request.pitch, request.yaw]
    target = [request.x / 1000, request.y / 1000, request.z / 1000] + rpy_to_rotvec(requested[3:])
    report = {
        "target": {
            "requested": dict(zip(("x", "y", "z", "roll", "pitch", "yaw"), requested)),
            "used_pose_m_rotvec": [round(v, 9) for v in target],
            "units": UNITS,
        },
        "state": None,
        "checks": {},
    }
    checks = report["checks"]
    point = [v * 1000 for v in target[:3]]
    axes = limits.workspace.outside(point)
    workspace = {"x": list(limits.workspace.x_mm), "y": list(limits.workspace.y_mm), "z": list(limits.workspace.z_mm)}
    checks["software_workspace"] = (
        {"result": REJECTED, "axes": axes, "tcp_mm": [round(v, 3) for v in point], "workspace_mm": workspace,
         "reason": f"outside the commissioned workspace box on {''.join(axes)}"}
        if axes else {"result": PASSED, "workspace_mm": workspace}
    )

    problem = blocked
    sample = None
    if problem is None and control is None:
        problem = "no_control_session: Connect first (diagnosis never connects)"
    if problem is None:
        link = ur_checks.link_problem(control)
        if link is not None:
            problem = f"{link}: nothing was asked (ur_rtde would re-upload its script to answer)"
    if problem is None:
        raw = latest() if latest is not None else None
        try:
            sample = None if raw is None else {
                **raw,
                "joints_deg": vector6(raw["joints_deg"]),
                "tcp_pose": vector6(raw["tcp_pose"]),
            }
        except (KeyError, TypeError, ValueError):
            sample = None
        if sample is None:
            problem = "feedback_unavailable: no control feedback to compare against"
        else:
            age = clock() - sample["received_monotonic_s"]
            if not 0 <= age <= limits.feedback_max_age_s:
                problem = f"feedback_stale: newest control feedback is {age:.2f} s old"
            elif sample.get("robot_mode") != "RUNNING" or sample.get("safety_mode") != "NORMAL":
                # The probes rely on a controller that accepts the arm's own pose.
                problem = (
                    f"robot_not_ready: controller {sample.get('robot_mode')} / safety {sample.get('safety_mode')}"
                )
            report["state"] = {
                "joints_deg": [round(v, 4) for v in sample["joints_deg"]],
                "tcp_mm_rpy_deg": [round(v, 4) for v in tcp_to_mm_deg(sample["tcp_pose"])],
                "tcp_pose_m_rotvec": [round(v, 9) for v in sample["tcp_pose"]],
                "feedback_age_s": round(age, 4),
                "robot_mode": sample.get("robot_mode"),
                "safety_mode": sample.get("safety_mode"),
                "orientation_change_deg": round(orientation_error_deg(sample["tcp_pose"][3:], target[3:]), 4),
                "translation_mm": round(math.dist(sample["tcp_pose"][:3], target[:3]) * 1000, 3),
            }
    if problem is not None:
        checks["connection"] = _not_checked(problem)
        for name in CHECK_ORDER[2:]:
            checks[name] = _not_checked("connection not usable")
    else:
        checks["connection"] = {"result": PASSED}
        _controller_checks(checks, control, limits, target, sample)
    verdict, names = _verdict(checks)
    report["verdict"] = verdict
    report["decided_by"] = names
    report["verdict_reason"] = VERDICT_REASONS[verdict].format(
        names=", ".join(names), reason=checks["connection"].get("reason") or ", ".join(names)
    )
    report["checks"] = {name: checks[name] for name in CHECK_ORDER}
    return report


def _controller_checks(checks, control, limits, target, sample):
    q_now = [math.radians(v) for v in sample["joints_deg"]]
    p_now = list(sample["tcp_pose"])
    ik = ur_checks.check(
        control, "getInverseKinematicsHasSolution", [list(target), q_now], [p_now, q_now], "current TCP pose"
    )
    checks["ik_solution"] = ik
    if ik["result"] == PASSED:
        solved = ur_checks.ask(control, "getInverseKinematics", list(target), q_now)
        try:
            if not solved["completed"]:
                raise ValueError(solved["error"])
            q = vector6(solved["answer"])
        except (TypeError, ValueError) as exc:
            ik.update(result=QUERY_FAILED, reason=f"getInverseKinematics: {exc}")
        else:
            ik["joints_deg"] = [round(math.degrees(v), 4) for v in q]
            ik["qnear_joints_deg"] = [round(v, 4) for v in sample["joints_deg"]]
    if ik["result"] == PASSED:
        ik_deg = ik["joints_deg"]
        outside = [
            i + 1 for i, (value, lo, hi) in enumerate(zip(ik_deg, limits.joint_lower_deg, limits.joint_upper_deg))
            if not lo <= value <= hi
        ]
        envelope = {"lower_deg": list(limits.joint_lower_deg), "upper_deg": list(limits.joint_upper_deg)}
        checks["software_envelope_at_ik"] = (
            {"result": REJECTED, "joints": outside, "envelope": envelope,
             "reason": f"joints {outside} of the IK solution are outside the commissioned envelope"}
            if outside else {"result": PASSED, "envelope": envelope}
        )
        joints = ur_checks.check(
            control, "isJointsWithinSafetyLimits", [[math.radians(v) for v in ik_deg]], [q_now], "current joints"
        )
        joints["specific_limit"] = ur_checks.BOOLEAN_ONLY
        checks["controller_joint_limits_at_ik"] = joints
    else:
        reason = "no confirmed IK solution to check"
        checks["software_envelope_at_ik"] = _not_checked(reason)
        checks["controller_joint_limits_at_ik"] = _not_checked(reason)
    pose = ur_checks.check(control, "isPoseWithinSafetyLimits", [list(target)], [p_now], "current TCP pose")
    pose["specific_limit"] = ur_checks.BOOLEAN_ONLY
    checks["controller_pose_limits"] = pose
