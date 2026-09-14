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

from ..config import Settings

READ_COMMANDS = frozenset({"robotmode", "safetystatus", "safetymode", "programState"})


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
        return interpret(mode, safety, program, source)


def interpret(mode, safety, program, safety_source="safetystatus"):
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
    elif safety not in {"NORMAL", "REDUCED"}:
        known_faults = {
            "PROTECTIVE_STOP",
            "SAFEGUARD_STOP",
            "RECOVERY",
            "VIOLATION",
            "FAULT",
            "AUTOMATIC_MODE_SAFEGUARD_STOP",
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
        },
    }
