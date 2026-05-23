"""Build a STATUS_SPEC v1.0 ``EquipmentStatus`` envelope from controller state.

This module is the single, side-effect-free source of truth for the spec
``GET /status`` response. It MUST NOT call any controller method that
mutates hardware state (no ``connect``, no ``motion_enable``, no movement
queries that round-trip to the arm). All readings come from cached
attributes the controller updates as part of its normal operation.

See ``docs/STATUS_SPEC.md`` in ac-organic-lab for the contract.
"""

from __future__ import annotations

import socket
import time
from datetime import datetime, timezone
from typing import TYPE_CHECKING, Any

from .models import (
    PROTOCOL_VERSION,
    ComponentStatus,
    EquipmentStatus,
    ErrorInfo,
    MetricValue,
)

if TYPE_CHECKING:  # pragma: no cover - import-time only for type hints
    from .xarm_controller import XArmController


EQUIPMENT_ID = "xarm_translocation"
EQUIPMENT_NAME = "UFactory xArm5"
EQUIPMENT_KIND = "robot_arm"


# Process start time used for ``uptime_seconds``. Captured at import so each
# new uvicorn worker reports its own uptime.
_PROCESS_START_TIME = time.time()


def _component_state(controller: XArmController, key: str) -> str:
    """Read a ComponentState enum value defensively as a snake_case string."""
    state = controller.states.get(key)
    if state is None:
        return "unknown"
    value = getattr(state, "value", None)
    if value is None:
        return str(state)
    return str(value)


def _safe_hostname() -> str | None:
    try:
        return socket.gethostname()
    except OSError:
        return None


def _disconnected_envelope() -> EquipmentStatus:
    """Envelope returned when no controller object exists yet.

    The FastAPI process is up but no ``XArmController`` has been
    instantiated, so the only honest answer is ``requires_init`` with
    ``required_actions: ["connect"]`` (matching the SDK's pre-migration
    ``LegacyXArmAdapter`` mapping).
    """
    return EquipmentStatus(
        protocol_version=PROTOCOL_VERSION,
        equipment_id=EQUIPMENT_ID,
        equipment_name=EQUIPMENT_NAME,
        equipment_kind=EQUIPMENT_KIND,
        host=_safe_hostname(),
        equipment_status="requires_init",
        message="Controller not instantiated. POST /connect to initialize.",
        required_actions=["connect"],
        device_time=datetime.now(timezone.utc),
        uptime_seconds=time.time() - _PROCESS_START_TIME,
        components={
            "arm": ComponentStatus(connected=False, state="disabled"),
            "gripper": ComponentStatus(connected=False, state="disabled"),
            "track": ComponentStatus(connected=False, state="disabled"),
            "force_torque": ComponentStatus(connected=False, state="disabled"),
        },
    )


def build_status(controller: XArmController | None) -> EquipmentStatus:
    """Map controller state to an ``EquipmentStatus`` envelope.

    Side-effect-free: only cached attributes are read. The controller MUST
    NOT be re-initialized, polled, or otherwise mutated by this call.

    State precedence (top-most match wins):

    1. ``controller is None`` -> ``requires_init``.
    2. controller has an active error code or string -> ``error``.
    3. ``controller._motion_in_progress`` -> ``busy``.
    4. ``controller.alive`` and arm enabled -> ``ready``.
    5. otherwise -> ``degraded``.
    """
    if controller is None:
        return _disconnected_envelope()

    arm_state = _component_state(controller, "arm")
    gripper_state = _component_state(controller, "gripper")
    track_state = _component_state(controller, "track")
    force_torque_state = _component_state(controller, "force_torque")
    connection_state = _component_state(controller, "connection")
    arm_connected = connection_state == "enabled"

    last_error_code = getattr(controller, "last_error_code", 0) or 0
    last_error_text = getattr(controller, "last_error", None)
    if isinstance(last_error_text, int):
        last_error_text = None

    motion_in_progress = bool(getattr(controller, "_motion_in_progress", False))
    alive = bool(getattr(controller, "alive", False))

    # State derivation.
    last_error: ErrorInfo | None = None
    required_actions: list[str] = []

    if last_error_code != 0 or last_error_text:
        equipment_status = "error"
        message = (
            f"Controller reports error: {last_error_text}"
            if last_error_text
            else f"Controller reports error code {last_error_code}."
        )
        required_actions = ["clear_errors"]
        last_error = ErrorInfo(
            code=str(last_error_code) if last_error_code else None,
            message=str(last_error_text) if last_error_text else f"error_code={last_error_code}",
            severity="error",
            timestamp=datetime.now(timezone.utc),
        )
    elif connection_state != "enabled":
        equipment_status = "requires_init"
        message = "Controller not connected. POST /connect to initialize."
        required_actions = ["connect"]
    elif motion_in_progress:
        equipment_status = "busy"
        message = "Robot motion in progress."
    elif alive and arm_state == "enabled":
        equipment_status = "ready"
        message = "Idle"
    else:
        equipment_status = "degraded"
        message = "Controller connected but not fully alive."

    # Components.
    components: dict[str, ComponentStatus] = {
        "arm": ComponentStatus(connected=arm_connected, state=arm_state),
    }
    if controller.has_gripper():
        components["gripper"] = ComponentStatus(
            connected=arm_connected,
            state=gripper_state,
            message=str(getattr(controller, "gripper_type", None) or ""),
        )
    if controller.has_track():
        components["track"] = ComponentStatus(
            connected=arm_connected,
            state=track_state,
        )
    if controller.has_force_torque_sensor():
        components["force_torque"] = ComponentStatus(
            connected=arm_connected,
            state=force_torque_state,
            message=(
                "calibrated"
                if getattr(controller, "force_torque_calibrated", False)
                else "uncalibrated"
            ),
        )

    # Metrics (numeric values with units).
    metrics: dict[str, MetricValue] = {}
    if controller.has_track():
        track_pos = getattr(controller, "last_track_position", None)
        if isinstance(track_pos, (int, float)):
            metrics["track_position"] = MetricValue(value=float(track_pos), unit="mm")

    last_ft = getattr(controller, "last_force_torque", None)
    if (
        controller.has_force_torque_sensor()
        and isinstance(last_ft, (list, tuple))
        and len(last_ft) >= 3
    ):
        try:
            fx, fy, fz = (float(last_ft[0]), float(last_ft[1]), float(last_ft[2]))
            magnitude = (fx * fx + fy * fy + fz * fz) ** 0.5
            metrics["force_magnitude"] = MetricValue(value=magnitude, unit="N")
        except (TypeError, ValueError):
            pass

    tcp_speed = getattr(controller, "tcp_speed", None)
    if isinstance(tcp_speed, (int, float)):
        metrics["tcp_speed"] = MetricValue(value=float(tcp_speed), unit="mm/s")
    angle_speed = getattr(controller, "angle_speed", None)
    if isinstance(angle_speed, (int, float)):
        metrics["angle_speed"] = MetricValue(value=float(angle_speed), unit="deg/s")

    # Details (debug-only, free-form).
    details: dict[str, Any] = {
        "current_position": list(getattr(controller, "last_position", []) or []) or None,
        "current_joints": list(getattr(controller, "last_joints", []) or []) or None,
        "model_name": getattr(controller, "model_name", None),
        "num_joints": getattr(controller, "num_joints", None),
        "gripper_type": getattr(controller, "gripper_type", None),
    }
    # Carry connection details for the local web UI's panel. Not contracted.
    details["connection_details"] = _build_connection_details(controller)
    # Motion-graph state (Phase 1: introspection only; absent when no graph
    # is loaded so the field doesn't pollute /status for unmigrated configs).
    motion_graph_block = _build_motion_graph_details(controller)
    if motion_graph_block is not None:
        details["motion_graph"] = motion_graph_block

    return EquipmentStatus(
        protocol_version=PROTOCOL_VERSION,
        equipment_id=EQUIPMENT_ID,
        equipment_name=EQUIPMENT_NAME,
        equipment_kind=EQUIPMENT_KIND,
        host=_safe_hostname(),
        equipment_status=equipment_status,  # type: ignore[arg-type]
        message=message,
        required_actions=required_actions,
        device_time=datetime.now(timezone.utc),
        uptime_seconds=time.time() - _PROCESS_START_TIME,
        components=components,
        metrics=metrics,
        last_error=last_error,
        details=details,
    )


def _build_motion_graph_details(controller: XArmController) -> dict[str, Any] | None:
    """Read-only snapshot of the motion-graph layer for the dashboard.

    Returns None when no graph is loaded (legacy / unmigrated configs)
    so the field doesn't appear in /status.details at all. When loaded:

        current_node     graph node id matching the controller's 4-tuple,
                         or None when off-grid (post-STOP, after raw moves)
        reachable_nodes  outgoing target ids from current_node (empty list
                         when current_node is None)
        graph_mode       "off" | "advisory" | "strict"
        declared_payload operator-declared payload identity (always
                         "empty" in Phase 1; will be settable later)
    """
    graph = getattr(controller, "motion_graph", None)
    if graph is None:
        return None
    return {
        "current_node": controller.current_node,
        "reachable_nodes": controller.reachable_node_ids(),
        "graph_mode": getattr(controller, "graph_mode").value,
        "declared_payload": getattr(controller, "declared_payload", "empty"),
        "arm_pose_name": getattr(controller, "last_arm_pose_name", None),
        "rail_location_name": getattr(controller, "last_rail_location_name", None),
    }


def _build_connection_details(controller: XArmController) -> dict[str, Any] | None:
    """Best-effort connection panel data for the local web UI.

    Lives under ``details`` (not contracted by STATUS_SPEC) to keep the
    UI's "Host: ... Port: ..." widget working without breaking the spec
    envelope.
    """
    host = getattr(controller, "host", None)
    if host is None:
        return None
    cfg = getattr(controller, "xarm_config", {}) or {}
    return {
        "host": host,
        "port": cfg.get("port", 18333),
        "profile_name": getattr(controller, "profile_name", None) or "unknown",
        "gripper_type": getattr(controller, "gripper_type", None) or "N/A",
        "gripper_config": getattr(controller, "current_gripper_config", {}),
    }
