"""Read-only SDK queries. No connection, motion, enable, or configuration writes."""
import inspect
from datetime import datetime, timezone
from importlib.metadata import version

from fastapi import HTTPException
from xarm.x3.code import APIState


UNITS = {"position": "mm", "angle": "degree", "pose_order": ["x", "y", "z", "roll", "pitch", "yaw"]}


def read_result(arm, method, *args, **kwargs):
    """Preserve SDK status; never expose invalid payloads as successful data."""
    fn = getattr(arm, method, None)
    if not callable(fn):
        return {"available": False, "code": None, "data": None, "reason": "sdk_method_unavailable"}
    try:
        code, data = fn(*args, **kwargs)
    except Exception as exc:
        return {"available": False, "code": None, "data": None,
                "reason": "sdk_exception", "message": str(exc)}
    names = [k for k, v in vars(APIState).items() if k.isupper() and v == code]
    return {"available": code == 0, "code": code, "data": data if code == 0 else None,
            "reason": None if code == 0 else (names[0] if names else "controller_query_failed")}


def reference_supported(arm):
    return (tuple(arm.version_number) >= (2, 7, 103)
            and "ref_angles" in inspect.signature(arm.get_inverse_kinematics).parameters)


def validate_joints(values, axis):
    if len(values) != axis:
        raise HTTPException(422, detail={"error": "joint_count_mismatch", "expected": axis})
    return list(values)


def metadata():
    return {"read_only": True, "sampled_at": datetime.now(timezone.utc).isoformat(), "units": UNITS}


def configuration(arm):
    return {**metadata(), "model": f"xArm{arm.axis}", "axis": arm.axis,
            "device_type": arm.device_type, "sdk_version": version("xarm-python-sdk"),
            "firmware": read_result(arm, "get_version"),
            "tcp_offset": list(arm.tcp_offset), "world_offset": list(arm.world_offset),
            "tcp_payload": {"mass_kg": arm.tcp_load[0], "center_of_gravity_mm": list(arm.tcp_load[1])},
            "offset_source": "sdk_controller_report_cache",
            "rotation_convention": {"axes": "roll=X, pitch=Y, yaw=Z",
                                    "composition": None, "status": "composition_not_verified"},
            "reference_angles_supported": reference_supported(arm),
            "dh_parameters": {**read_result(arm, "get_dh_params"),
                              "layout": "7 slots, 4 values per slot; raw controller order",
                              "parameter_convention": None,
                              "calibration_completeness": "not_verified"}}


def limits(arm):
    result = read_result(arm, "get_reduced_states", is_radian=False)
    reduced = {"query": result, "enabled": None, "joint_ranges_raw": None}
    if result["available"]:
        values = result["data"]
        reduced.update(enabled=bool(values[0]), tcp_boundary_mm=values[1],
                       max_tcp_speed_mm_s=values[2], max_joint_speed_deg_s=values[3])
        if len(values) >= 7:
            reduced.update(joint_ranges_raw=values[4], safety_boundary_enabled=bool(values[5]),
                           collision_rebound_enabled=bool(values[6]))
    return {**metadata(), "ordinary_joint_limits": {"available": False, "ranges": None,
            "reason": "No verified SDK getter for controller-effective ordinary joint bounds"},
            "reduced": reduced,
            "joint_range_mapping": "Raw SDK slot order; xArm5 J4/J5 mapping is not verified. Do not substitute for ordinary limits."}


def forward(arm, joints):
    return {**metadata(), "result": read_result(arm, "get_forward_kinematics", joints,
            input_is_radian=False, return_is_radian=False)}


def inverse(arm, pose, reference_angles, limited):
    if not reference_supported(arm):
        raise HTTPException(409, detail={"error": "reference_angles_unsupported",
            "required_firmware": "2.7.103", "required_sdk": "1.18.4",
            "message": "Reference angles will not be silently ignored."})
    if reference_angles is None:
        current = read_result(arm, "get_servo_angle", is_radian=False)
        if not current["available"]:
            return {**metadata(), "result": current, "reference_angles": None,
                    "joint_limit_check": None, "violating_joints": None}
        reference_angles = current["data"][:arm.axis]
    result = read_result(arm, "get_inverse_kinematics", pose, input_is_radian=False,
                         return_is_radian=False, limited=limited, ref_angles=reference_angles)
    check = None
    if result["available"]:
        result["data"] = result["data"][:arm.axis]
        check = read_result(arm, "is_joint_limit", result["data"], is_radian=False)
    return {**metadata(), "result": result, "reference_angles": reference_angles,
            "limited": limited, "joint_limit_check": check, "violating_joints": None,
            "diagnostic_scope": "SDK returns a limit boolean and status code, not per-joint reasons. This is not collision/path validation."}
