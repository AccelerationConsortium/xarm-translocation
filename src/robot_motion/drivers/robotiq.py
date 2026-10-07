"""Robotiq adaptive gripper through the Robotiq URCap's socket server.

The URCap installed on the UR controller listens on TCP 63352 and speaks a
line protocol (the same one ur_rtde's doc example robotiq_gripper.py uses):
``GET VAR`` answers ``VAR <value>``; ``SET VAR value [VAR value ...]``
answers ``ack``. Registers used here:

- ACT activate request, GTO go-to request (0 stops a move), ATR auto-release
  (only ever written as 0, as part of a reset);
- STA status (0 reset, 1 activating, 3 active), OBJ object detection
  (0 moving, 1 stopped on contact while opening, 2 while closing, 3 at the
  requested position), FLT fault code, POS actual position, PRE echo of
  the requested position;
- POS/SPE/FOR requests: position 0 (open) .. 255 (closed), speed and force
  0..255.

Nothing here needs the arm's RTDE control script or the controller's program
slot. Status polling is read-only (GET). Commands are only issued by the
config-gated, claimed routes in gripper.py. Activation and moves physically
move the fingers.
"""

from __future__ import annotations

import logging
import socket
import threading
import time
from datetime import datetime, timezone
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

log = logging.getLogger(__name__)

GRIPPER_MODELS = {
    "robotiq_2f85": {"name": "Robotiq 2F-85", "stroke_mm": 85.0},
    "robotiq_2f140": {"name": "Robotiq 2F-140", "stroke_mm": 140.0},
    "robotiq_hande": {"name": "Robotiq Hand-E", "stroke_mm": 50.0},
}
GET_VARS = ("ACT", "GTO", "STA", "OBJ", "FLT", "POS", "PRE", "SPE", "FOR")
SET_VARS = frozenset({"ACT", "GTO", "POS", "SPE", "FOR", "ATR"})
OBJECT_STATES = {
    0: "moving",
    1: "contact_opening",
    2: "contact_closing",
    3: "at_requested_position",
}
GRIPPER_STATES = {0: "reset", 1: "activating", 2: "activating", 3: "active"}
# Robotiq gFLT codes (2F-85/140 and Hand-E manuals).
FAULTS = {
    0x05: "action delayed; activation must complete first",
    0x07: "activation bit must be set before this action",
    0x08: "maximum operating temperature exceeded; wait for cool-down",
    0x09: "no communication for at least 1 s",
    0x0A: "under minimum operating voltage",
    0x0B: "automatic release in progress",
    0x0C: "internal fault; contact Robotiq support",
    0x0D: "activation fault; check for interference",
    0x0E: "overcurrent triggered",
    0x0F: "automatic release completed",
}


class GripperSettings(BaseModel):
    """Local gripper config. Speeds and forces are percent of the model's range.

    max_speed_pct / max_force_pct are the commissioned caps: a request above
    them is refused, never clamped. open_raw / closed_raw map the position
    register to the finger opening in mm (linear estimate); leave the
    defaults until the stroke has been measured on the real fingers.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)
    model: Literal["robotiq_2f85", "robotiq_2f140", "robotiq_hande"]
    port: int = Field(default=63352, ge=1, le=65535)
    poll_interval_s: float = Field(default=1.0, ge=0.2, le=10, allow_inf_nan=False)
    timeout_s: float = Field(default=2.0, ge=0.2, le=5, allow_inf_nan=False)
    move_timeout_s: float = Field(default=6.0, ge=1, le=30, allow_inf_nan=False)
    activation_timeout_s: float = Field(default=15.0, ge=5, le=30, allow_inf_nan=False)
    default_speed_pct: float = Field(default=50, ge=1, le=100, allow_inf_nan=False)
    default_force_pct: float = Field(default=20, ge=0, le=100, allow_inf_nan=False)
    max_speed_pct: float = Field(default=100, ge=1, le=100, allow_inf_nan=False)
    max_force_pct: float = Field(default=50, ge=0, le=100, allow_inf_nan=False)
    open_raw: int = Field(default=0, ge=0, le=255)
    closed_raw: int = Field(default=255, ge=0, le=255)

    @model_validator(mode="after")
    def coherent(self):
        if self.closed_raw - self.open_raw < 50:
            raise ValueError("closed_raw must exceed open_raw by at least 50 counts")
        if self.default_speed_pct > self.max_speed_pct:
            raise ValueError("default_speed_pct exceeds max_speed_pct")
        if self.default_force_pct > self.max_force_pct:
            raise ValueError("default_force_pct exceeds max_force_pct")
        return self

    @property
    def name(self):
        return GRIPPER_MODELS[self.model]["name"]

    @property
    def stroke_mm(self):
        return GRIPPER_MODELS[self.model]["stroke_mm"]


class GripperRefused(Exception):
    """The gripper was not in a state to accept the command; nothing was sent."""


class GripperBusy(GripperRefused):
    """Another gripper command is still running; requests are never queued."""


class GripperFailed(Exception):
    """A command was sent and did not complete; a stop was attempted."""

    def __init__(self, message, *, stop_attempted, stop_error=None, sample=None):
        super().__init__(message)
        self.stop_attempted = stop_attempted
        self.stop_error = stop_error
        self.sample = sample


def fault_text(code):
    if code in (None, 0):
        return None
    return FAULTS.get(code, f"fault 0x{code:02X}")


def _parse_value(text):
    # Values are decimal; FLT is two digits and some URCap builds print hex.
    try:
        return int(text, 10)
    except ValueError:
        return int(text, 16)


class RobotiqClient:
    """One serialized socket to the URCap. Fails closed and reconnects lazily."""

    def __init__(self, host, port, timeout, *, connector=socket.create_connection):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._connector = connector
        self._sock = None
        self._buffer = b""
        self._lock = threading.Lock()

    def _ensure(self):
        if self._sock is None:
            self._sock = self._connector((self.host, self.port), timeout=self.timeout)
            self._buffer = b""
        return self._sock

    def _drop(self):
        sock, self._sock, self._buffer = self._sock, None, b""
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def _readline(self, sock, bare_ack=False):
        # GET replies end in a newline (measured on the UR5e's URCap). SET is
        # answered "ack", which ur_rtde's reference client compares without
        # one, so accept a bare "ack" and drop a newline that trails it later.
        while True:
            self._buffer = self._buffer.lstrip(b"\r\n")
            if b"\n" in self._buffer:
                line, self._buffer = self._buffer.split(b"\n", 1)
                return line.decode("ascii", errors="strict").strip()
            if bare_ack and self._buffer == b"ack":
                self._buffer = b""
                return "ack"
            chunk = sock.recv(256)
            if not chunk:
                raise OSError("Robotiq URCap closed the connection")
            self._buffer += chunk
            if len(self._buffer) > 1024:
                raise OSError("Oversized Robotiq URCap reply")

    def _exchange(self, line, bare_ack=False):
        with self._lock:
            try:
                sock = self._ensure()
                sock.sendall((line + "\n").encode("ascii"))
                return self._readline(sock, bare_ack)
            except (OSError, UnicodeDecodeError) as exc:
                self._drop()
                raise OSError(f"Robotiq URCap exchange failed: {exc}") from exc

    def get(self, var):
        if var not in GET_VARS:
            raise ValueError(f"Unsupported Robotiq register {var!r}")
        reply = self._exchange(f"GET {var}")
        parts = reply.split()
        if len(parts) != 2 or parts[0] != var:
            self._drop()
            raise OSError(f"Unexpected Robotiq reply to GET {var}: {reply!r}")
        try:
            value = _parse_value(parts[1])
        except ValueError as exc:
            self._drop()
            raise OSError(f"Unexpected Robotiq value for {var}: {reply!r}") from exc
        if not 0 <= value <= 255:
            raise OSError(f"Robotiq {var} out of range: {value}")
        return value

    def set(self, pairs):
        for var, value in pairs:
            if var not in SET_VARS:
                raise ValueError(f"Unsupported Robotiq request register {var!r}")
            if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 255:
                raise ValueError(f"Robotiq {var} must be an integer 0..255")
            if var == "ATR" and value != 0:
                raise ValueError("Auto-release is never requested by this service")
        reply = self._exchange(
            "SET " + " ".join(f"{var} {value}" for var, value in pairs), bare_ack=True
        )
        if reply != "ack":
            raise OSError(f"Robotiq SET not acknowledged: {reply!r}")

    def registers(self):
        return {var: self.get(var) for var in GET_VARS}

    def close(self):
        with self._lock:
            self._drop()


class RobotiqGripper:
    """Cached read-only status plus bounded, cancellable commands."""

    def __init__(self, settings: GripperSettings, host, *, client=None, clock=time.monotonic, sleep=time.sleep):
        self.settings = settings
        self.client = client or RobotiqClient(host, settings.port, settings.timeout_s)
        self._clock = clock
        self._sleep = sleep
        self._sample = None
        self._sampled_at = None
        self._sample_time = None
        self._error = None
        self._command = threading.Lock()
        self._cancel = threading.Event()
        self._busy = False
        self._stop_poll = threading.Event()
        self._thread = None

    # ── conversions ─────────────────────────────────────────────────
    def raw_to_mm(self, raw):
        s = self.settings
        span = s.closed_raw - s.open_raw
        fraction = (s.closed_raw - raw) / span
        return round(min(max(fraction, 0.0), 1.0) * s.stroke_mm, 1)

    def mm_to_raw(self, mm):
        s = self.settings
        span = s.closed_raw - s.open_raw
        return int(round(s.closed_raw - (mm / s.stroke_mm) * span))

    @staticmethod
    def pct_to_raw(pct):
        return int(round(pct / 100 * 255))

    # ── read-only status ────────────────────────────────────────────
    def poll_once(self):
        try:
            registers = self.client.registers()
        except Exception as exc:  # noqa: BLE001 - any failure means "unknown"
            message = str(exc)
            if message != self._error:
                log.warning("Robotiq status unavailable: %s", message)
            self._sample, self._error = None, message
            self._sampled_at = self._clock()
            return None
        if self._error is not None:
            log.info("Robotiq status restored")
        self._sample, self._error = registers, None
        self._sampled_at = self._clock()
        self._sample_time = datetime.now(timezone.utc)
        return registers

    def start(self):
        if self._thread is not None:
            return

        def run():
            while not self._stop_poll.is_set():
                self.poll_once()
                self._stop_poll.wait(self.settings.poll_interval_s)

        self._thread = threading.Thread(target=run, name="robotiq-status", daemon=True)
        self._thread.start()

    def close(self):
        self._stop_poll.set()
        if self._thread is not None:
            self._thread.join(timeout=self.settings.timeout_s * 10)
        self.client.close()

    def sample_age_s(self):
        return None if self._sampled_at is None else self._clock() - self._sampled_at

    def fresh(self):
        age = self.sample_age_s()
        limit = max(3 * self.settings.poll_interval_s, 3.0)
        return self._sample is not None and age is not None and age <= limit

    @property
    def busy(self):
        return self._busy

    def summary(self):
        """Cached state for /status; never does I/O."""
        r = self._sample if self.fresh() else None
        base = {
            "model": self.settings.model,
            "name": self.settings.name,
            "reachable": r is not None,
            "error": self._error,
            "sample_age_s": self.sample_age_s(),
            "sampled_at": self._sample_time.isoformat() if self._sample_time else None,
            "busy": self._busy,
        }
        if r is None:
            return {**base, "state": "unknown"}
        return {
            **base,
            "state": self.state_name(r),
            "activated": r["ACT"] == 1 and r["STA"] == 3,
            "status_code": r["STA"],
            "status": GRIPPER_STATES.get(r["STA"], "unknown"),
            "fault_code": r["FLT"],
            "fault": fault_text(r["FLT"]),
            "position_raw": r["POS"],
            "requested_raw": r["PRE"],
            "opening_mm": self.raw_to_mm(r["POS"]),
            "opening_mm_estimate": "linear from the position register (open_raw/closed_raw)",
            "object": OBJECT_STATES.get(r["OBJ"], "unknown") if r["GTO"] == 1 else None,
            "speed_raw": r["SPE"],
            "force_raw": r["FOR"],
        }

    @staticmethod
    def state_name(r):
        if r["FLT"] != 0:
            return "fault"
        if r["ACT"] == 1 and r["STA"] == 3:
            return "enabled"
        if r["ACT"] == 1:
            return "activating"
        return "disabled"

    # ── commands ────────────────────────────────────────────────────
    def request_stop(self):
        """Cancel an in-flight command (GTO 0). An idle gripper is left alone,
        so a stop never releases or re-grips a held part."""
        if not self._busy:
            return False
        self._cancel.set()
        try:
            self.client.set([("GTO", 0)])
        except Exception as exc:  # noqa: BLE001
            log.warning("Robotiq stop request failed: %s", exc)
        return True

    def _halt(self):
        try:
            self.client.set([("GTO", 0)])
            return None
        except Exception as exc:  # noqa: BLE001
            return str(exc)

    def _begin(self):
        if not self._command.acquire(blocking=False):
            raise GripperBusy("another gripper command is in progress")
        self._busy = True
        self._cancel.clear()

    def _end(self):
        self._busy = False
        self._command.release()

    def move(self, position_raw, speed_raw, force_raw):
        """Go to a position and wait until the fingers stop (target or contact)."""
        for value in (position_raw, speed_raw, force_raw):
            if not 0 <= value <= 255:
                raise ValueError("Robotiq requests are 0..255")
        self._begin()
        try:
            try:
                r = self.client.registers()
            except OSError as exc:
                raise GripperRefused(f"gripper status unavailable: {exc}") from exc
            if r["FLT"] != 0:
                raise GripperRefused(f"gripper fault: {fault_text(r['FLT'])}")
            if not (r["ACT"] == 1 and r["STA"] == 3):
                raise GripperRefused("gripper is not activated")
            started = self._clock()
            try:
                self.client.set(
                    [("POS", position_raw), ("SPE", speed_raw), ("FOR", force_raw), ("GTO", 1)]
                )
            except OSError as exc:
                # The request may or may not have reached the gripper.
                raise GripperFailed(
                    f"move request not confirmed: {exc}",
                    stop_attempted=True,
                    stop_error=self._halt(),
                ) from exc
            return self._wait_motion(position_raw, started)
        finally:
            self._end()

    def _wait_motion(self, target, started):
        deadline = started + self.settings.move_timeout_s
        acknowledged_at = None
        seen_moving = False
        while True:
            if self._cancel.is_set():
                raise GripperFailed("move cancelled by stop", stop_attempted=True, stop_error=self._halt())
            try:
                r = self.client.registers()
            except OSError as exc:
                raise GripperFailed(
                    f"lost gripper status during move: {exc}",
                    stop_attempted=True,
                    stop_error=self._halt(),
                ) from exc
            now = self._clock()
            if r["FLT"] != 0:
                raise GripperFailed(
                    f"gripper fault during move: {fault_text(r['FLT'])}",
                    stop_attempted=True,
                    stop_error=self._halt(),
                    sample=r,
                )
            if acknowledged_at is None and r["PRE"] == target:
                acknowledged_at = now
            if acknowledged_at is not None:
                if r["OBJ"] == 0:
                    seen_moving = True
                # OBJ can still show the previous move's result in the first
                # sample after the echo; accept it only once motion was seen,
                # the fingers are already at the request, or it has persisted.
                elif (
                    seen_moving
                    or abs(r["POS"] - target) <= 5
                    or now - acknowledged_at >= 0.3
                ):
                    self._sample, self._sampled_at = r, now
                    self._sample_time = datetime.now(timezone.utc)
                    return {
                        "position_raw": r["POS"],
                        "requested_raw": target,
                        "opening_mm": self.raw_to_mm(r["POS"]),
                        "object": OBJECT_STATES.get(r["OBJ"], "unknown"),
                        "object_detected": r["OBJ"] in (1, 2),
                        "elapsed_s": round(now - started, 3),
                    }
            if now >= deadline:
                raise GripperFailed(
                    f"move did not finish within {self.settings.move_timeout_s} s",
                    stop_attempted=True,
                    stop_error=self._halt(),
                    sample=r,
                )
            self._sleep(0.02)

    def activate(self):
        """Activate (or re-activate after a fault). The fingers sweep through
        their stroke to calibrate. No-op when already active and fault-free."""
        self._begin()
        try:
            try:
                r = self.client.registers()
            except OSError as exc:
                raise GripperRefused(f"gripper status unavailable: {exc}") from exc
            if r["ACT"] == 1 and r["STA"] == 3 and r["FLT"] == 0:
                return {"already_active": True, "position_raw": r["POS"]}
            started = self._clock()
            deadline = started + self.settings.activation_timeout_s
            try:
                self.client.set([("ACT", 0), ("ATR", 0)])
                while True:
                    r = self.client.registers()
                    if r["ACT"] == 0 and r["STA"] == 0:
                        break
                    if self._clock() >= deadline:
                        raise GripperFailed("gripper did not reset", stop_attempted=False, sample=r)
                    self._sleep(0.05)
                self.client.set([("ACT", 1)])
                # Activation runs to completion on the gripper once requested;
                # GTO 0 does not interrupt it, so a stop is not offered here.
                while True:
                    r = self.client.registers()
                    if r["FLT"] not in (0, 0x05, 0x07) or (r["ACT"] == 1 and r["STA"] == 3):
                        break
                    if self._clock() >= deadline:
                        raise GripperFailed(
                            f"activation did not complete within {self.settings.activation_timeout_s} s",
                            stop_attempted=False,
                            sample=r,
                        )
                    self._sleep(0.05)
            except OSError as exc:
                raise GripperFailed(f"lost gripper during activation: {exc}", stop_attempted=False) from exc
            if r["FLT"] != 0 or r["STA"] != 3:
                raise GripperFailed(
                    f"activation ended with {fault_text(r['FLT']) or 'status ' + str(r['STA'])}",
                    stop_attempted=False,
                    sample=r,
                )
            self._sample, self._sampled_at = r, self._clock()
            self._sample_time = datetime.now(timezone.utc)
            return {"already_active": False, "position_raw": r["POS"], "elapsed_s": round(self._clock() - started, 3)}
        finally:
            self._end()
