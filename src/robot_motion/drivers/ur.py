"""Read-only UR Dashboard transport; no RTDE control construction or commands.

CB3/e-Series queries mirror the lab's existing dashboard monitors.
A playing program does not prove physical motion. No loaded program paths,
credentials, or arbitrary-command endpoints are exposed.
"""

from __future__ import annotations

import socket
from contextlib import ExitStack
from datetime import datetime, timezone

from sdl_lab_contract import ComponentStatus, ErrorInfo

from ..config import MODELS, Settings

READ_COMMANDS = frozenset(
    {
        "robotmode",
        "safetystatus",
        "safetymode",
        "programState",
        # e-Series only (PolyScope 5): whether external control is allowed at
        # all, and MANUAL/AUTOMATIC. Read-only, like the rest.
        "is in remote control",
        "get operational mode",
    }
)


SAFEGUARD_STOPS = frozenset({"SAFEGUARD_STOP", "AUTOMATIC_MODE_SAFEGUARD_STOP"})


def _readline(stream):
    raw = stream.readline(4097)
    if not raw or len(raw) > 4096 or not raw.endswith(b"\n"):
        raise OSError("Incomplete or oversized UR Dashboard response")
    return raw.decode("utf-8", errors="strict").strip()


def _value(reply, prefix):
    if not reply.lower().startswith(prefix.lower() + ":"):
        raise OSError(f"Unrecognized UR {prefix} response")
    return reply.split(":", 1)[1].strip().upper()


class URObserver:
    def __init__(self, settings: Settings, *, connector=socket.create_connection):
        self.settings = settings
        self.connector = connector

    def read(self):
        with ExitStack() as stack:
            conn = stack.enter_context(
                self.connector(
                    (self.settings.robot_host, 29999), timeout=self.settings.timeout_s
                )
            )
            stream = stack.enter_context(conn.makefile("rb"))
            banner = _readline(stream)
            if "Universal Robots Dashboard Server" not in banner:
                raise OSError("Unexpected Dashboard Server identity")

            def query(command):
                if command not in READ_COMMANDS:
                    raise ValueError(
                        "Only fixed read-only Dashboard queries are permitted"
                    )
                conn.sendall((command + "\n").encode("ascii"))
                return _readline(stream)

            mode = _value(query("robotmode"), "robotmode")
            source = "safetystatus"
            safety_reply = query(source)
            if not safety_reply.lower().startswith("safetystatus:"):
                source = "safetymode"
                safety_reply = query(source)
            safety = _value(safety_reply, source)
            program = query("programState").split(" ", 1)[0].upper()
            if program not in {"PLAYING", "PAUSED", "STOPPED"}:
                raise OSError("Unrecognized UR program state")
            remote = operational = None
            if MODELS[self.settings.model]["generation"] == "e_series":
                remote = {"true": True, "false": False}.get(query("is in remote control").strip().lower())
                operational = query("get operational mode").strip().upper()
                if operational not in {"MANUAL", "AUTOMATIC", "NONE"}:
                    operational = None
        return interpret(mode, safety, program, source, remote_control=remote, operational_mode=operational)


def interpret(mode, safety, program, safety_source="safetystatus", *, remote_control=None, operational_mode=None):
    state, activity = "unknown", "unknown"
    message = "Unrecognized UR controller state"
    fault = None
    if safety in {"ROBOT_EMERGENCY_STOP", "SYSTEM_EMERGENCY_STOP"}:
        state, activity, message = "e_stop", "idle", f"Emergency stop: {safety}"
        fault = ErrorInfo(
            code="ur_estop",
            message=message,
            severity="critical",
            timestamp=datetime.now(timezone.utc),
        )
    elif safety in SAFEGUARD_STOPS:
        # A safeguard stop is the bench's safety-stop input doing its job, not
        # a fault: the arm is held until the input is released (and reset, if
        # the installation asks for it), so it needs init rather than an error.
        # STATUS_SPEC §2.3: requires_init ⇒ idle, even if a program was playing.
        state, activity, message = (
            "requires_init",
            "idle",
            f"Safeguard stop ({safety}): release the safety stop to resume",
        )
    elif safety not in {"NORMAL", "REDUCED"}:
        known_faults = {
            "PROTECTIVE_STOP",
            "RECOVERY",
            "VIOLATION",
            "FAULT",
            "SYSTEM_THREE_POSITION_ENABLING_STOP",
        }
        if safety in known_faults:
            state, message = "error", f"Safety stop: {safety}"
            activity = "running" if program == "PLAYING" else "idle"
            fault = ErrorInfo(
                code="ur_safety_stop",
                message=message,
                severity="error",
                timestamp=datetime.now(timezone.utc),
            )
        else:
            message = "Unknown safety state; readiness cannot be established"
    elif mode in {
        "POWER_OFF",
        "POWER_ON",
        "IDLE",
        "BOOTING",
        "CONFIRM_SAFETY",
        "BACKDRIVE",
    }:
        state, activity, message = "requires_init", "idle", f"Controller mode: {mode}"
    elif mode == "RUNNING":
        activity = "running" if program == "PLAYING" else "idle"
        if safety == "REDUCED" or program == "PAUSED":
            state, message = (
                "degraded",
                f"Controller safety {safety}; program {program}",
            )
        else:
            state = "busy" if program == "PLAYING" else "ready"
            message = f"Read-only observation: program {program.lower()}"
    return {
        "equipment_status": state,
        "activity": activity,
        "message": message,
        "last_error": fault,
        "components": {
            "controller": ComponentStatus(
                connected=mode not in {"NO_CONTROLLER", "DISCONNECTED"},
                state=mode.lower(),
            ),
            "safety": ComponentStatus(connected=True, state=safety.lower()),
            "program": ComponentStatus(connected=True, state=program.lower()),
        },
        "details": {
            "robotmode": mode,
            "safetystatus": safety,
            "program_state": program,
            "safety_source": safety_source,
            # None: not reported (CB3, or an unrecognized reply).
            "remote_control": remote_control,
            "operational_mode": operational_mode,
        },
    }


def _is_safeguard(details):
    return (details or {}).get("safetystatus") in SAFEGUARD_STOPS


def _reconnect_note(message, *, safeguard):
    text = message or ""
    if "Disconnect and connect" in text:
        return text
    if safeguard:
        return text + " Disconnect and connect once the safety stop is released."
    return text + " Disconnect and connect to resume."


# Trajectory session states that are moving the arm. A created or finished
# session is not work (STATUS_SPEC: activity is doing work, not "connected").
_TRAJECTORY_ACTIVE = frozenset({"starting", "running", "stopping"})


def work_in_flight(*, arm_busy=False, active_move=None, trajectory_state=None, gripper_busy=False):
    """True while this service is actually moving the arm or the fingers."""
    if arm_busy or active_move:
        return True
    if trajectory_state in _TRAJECTORY_ACTIVE:
        return True
    return bool(gripper_busy)


def _report_control_script(view, *, motion_active):
    """Dashboard programState does not see ur_rtde's external control script.

    The script is sent on the secondary interface (port 30002). Dashboard
    ``programState`` reports the loaded .urp program, which stays STOPPED
    while that script runs — measured on the UR5 CB3 (PolyScope 3.15.8) and
    the same split documented for e-Series. The acknowledged watchdog is the
    script answering, so /status reports program_state PLAYING. That is not
    work: activity stays idle until a move, trajectory, or gripper motion is
    in flight (ready ⇒ idle, busy ⇒ running). The cached observation is left
    alone: /connect still requires the loaded program to be STOPPED.
    """
    details = view["details"]
    corrected = interpret(
        details["robotmode"],
        details["safetystatus"],
        "PLAYING",
        details.get("safety_source") or "safetystatus",
        remote_control=details.get("remote_control"),
        operational_mode=details.get("operational_mode"),
    )
    extra = {
        key: value for key, value in details.items() if key not in corrected["details"]
    }
    components = dict(view["components"])
    components.update(corrected["components"])
    note = (
        "ur_rtde control script is running; Dashboard programState "
        "does not report an external script"
    )
    program = components["program"]
    components["program"] = program.model_copy(update={"message": note})
    # interpret(PLAYING) calls a running program "busy". The control script
    # is up whenever the session is; only in-flight motion is busy/running.
    status = corrected["equipment_status"]
    if motion_active:
        activity = "running"
        if status == "ready":
            status = "busy"
    else:
        activity = "idle"
        if status == "busy":
            status = "ready"
    return {
        **view,
        "equipment_status": status,
        "activity": activity,
        "message": corrected["message"],
        "last_error": corrected["last_error"],
        "components": components,
        "details": {
            **corrected["details"],
            **extra,
            "program_state_source": "control_session",
        },
    }


def present_status(
    observation,
    *,
    control_enabled,
    session_open,
    watchdog_ok,
    watchdog_error,
    motion_active=False,
):
    """/status view of a cached Dashboard observation. Does not mutate it.

    A requires_init state names the action that gets out of it (STATUS_SPEC
    §4.1). An open control session has to be disconnected before connect will
    run, including after a safeguard stop clears. A session whose watchdog is
    no longer acknowledged is not a healthy stopped program: motion has
    already dropped out of allowed_actions, and the envelope says why.
    e-stop and other safety faults are left as reported.
    """
    view = {
        "equipment_status": observation.get("equipment_status", "unknown"),
        "activity": observation.get("activity", "unknown"),
        "message": observation.get("message"),
        "last_error": observation.get("last_error"),
        "components": dict(observation.get("components") or {}),
        "details": dict(observation.get("details") or {}),
        "required_actions": [],
        "control_message": None,
    }
    details = view["details"]
    script_running = session_open and watchdog_ok
    dashboard_hides_script = (
        script_running
        and details.get("robotmode") == "RUNNING"
        and details.get("safetystatus") in {"NORMAL", "REDUCED"}
        and details.get("program_state") == "STOPPED"
    )
    if dashboard_hides_script:
        view = _report_control_script(view, motion_active=motion_active)

    if (
        session_open
        and not watchdog_ok
        and view["equipment_status"] not in {"e_stop", "error"}
    ):
        # ready/busy/degraded/unknown here means the Dashboard no longer shows
        # the stop, but this session's script is gone. requires_init, not a
        # quiet ready with the moves missing.
        if view["equipment_status"] != "requires_init":
            reason = (
                watchdog_error
                or "the controller stopped acknowledging the control script"
            )
            view["equipment_status"] = "requires_init"
            view["activity"] = "idle"
            view["message"] = (
                f"Control session is open but not usable ({reason}). "
                "Disconnect and connect to resume."
            )

    safeguard = _is_safeguard(view["details"])
    if view["equipment_status"] != "requires_init":
        actions = []
    elif session_open:
        actions = ["disconnect", "connect"]
    elif control_enabled:
        actions = ["connect"]
    elif safeguard:
        actions = ["release_safety_stop"]
    else:
        actions = []
    view["required_actions"] = actions
    if session_open and view["equipment_status"] == "requires_init":
        view["message"] = _reconnect_note(view["message"], safeguard=safeguard)
        view["control_message"] = view["message"]
    return view
