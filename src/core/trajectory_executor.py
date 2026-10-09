"""Sessions and the fixed-rate executor for joint trajectories.

A session collects a joint trajectory (one or more chunks, ending with a
``final`` chunk), validates it against the measured start, and executes
it on a dedicated thread at a fixed command rate through a backend:

- ``servoj``           xArm mode 1, ``set_servo_angle_j``: the executor
                       shapes all motion; the controller follows the newest
                       target at full speed.
- ``online_planning``  xArm mode 6, ``set_servo_angle(wait=False)``: the
                       controller replans to each new target with the
                       speed and acceleration we send.

Version 1 executes only a complete trajectory: start is refused until the
final chunk is in, and chunks cannot be appended while running. The
contract (sequence numbers, idempotent uploads) already covers appending,
so adding it later changes no field. See
``src/docs/SERVOJ_TRAJECTORY_PLAN.md``.

Safety model, in short:

- STOP, the sash watchdog and disconnect call :meth:`notify_hard_stop`,
  which bumps a stop *generation*. A run captures the generation before its
  gates and checks it before entering servo mode, before every send, and
  before and after every command of its own that could re-enable the arm
  (``set_state(0)``). A stop is never cleared, so one that lands during
  start-up, prepare, settle or the mode restore is still honoured.
  Nothing is sent after it.
- Cancel, claim loss, graph mode back to STRICT and timing faults slow the
  trajectory clock to zero along the planned path within the acceleration
  limit (a constrained stop), then restore mode 0.
- A non-zero SDK code, a controller error or warning, a state other than
  moving/ready (paused, stopping, stopped), or the arm leaving the
  expected mode is a hard failure: no further sends, emergency stop.
- Every sample is re-checked against the joint limits, a speed bound and an
  acceleration bound before it is sent, as defence in depth.
- The arm is re-measured after entering servo mode; the lead-in is planned
  from that measurement, and the run fails before moving if the arm is no
  longer within the start tolerance.
"""

from __future__ import annotations

import array
import gc
import hashlib
import json
import logging
import math
import os
import socket
import threading
import time
import uuid
from dataclasses import dataclass, field, fields
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Optional, Sequence

try:
    from .joint_trajectory import (
        DEFAULT_MAX_JOINT_ACC_DEG_S2, JointLimits, JointPoint, JointTrajectory, lead_in,
        stop_deceleration, validate_joint_trajectory,
    )
except ImportError:  # pragma: no cover - flat ``core.*`` import path
    from core.joint_trajectory import (
        DEFAULT_MAX_JOINT_ACC_DEG_S2, JointLimits, JointPoint, JointTrajectory, lead_in,
        stop_deceleration, validate_joint_trajectory,
    )

logger = logging.getLogger(__name__)

CREATED, STARTING, RUNNING, STOPPING = "created", "starting", "running", "stopping"
COMPLETED, CANCELLED, STOPPED, FAILED, EXPIRED = (
    "completed", "cancelled", "stopped", "failed", "expired",
)
TERMINAL = frozenset({COMPLETED, CANCELLED, STOPPED, FAILED, EXPIRED})
ACTIVE = frozenset({STARTING, RUNNING, STOPPING})
# Reported arm states that are fine while streaming: 1 moving, 2 ready.
# 3 is paused, 4 and 5 stopping/stopped.
HEALTHY_STATES = (1, 2)
HARD_STOP_REASONS = ("stop", "disconnect")
# The lead-in closes a small gap to the first point. Sizing it at half the
# limits keeps it gentle, and leaves acceleration budget for a constrained
# stop during it (a lead-in sized to the full limit left none, so a stop
# fell back to the slowest ramp; seen on the arm 2026-10-09).
LEAD_IN_FRACTION = 0.5

BACKENDS = ("servoj", "online_planning")
SETTINGS_PATH = os.path.join("src", "settings", "trajectory.yaml")


def _utc(ts: Optional[float]) -> Optional[str]:
    if ts is None:
        return None
    return datetime.fromtimestamp(ts, tz=timezone.utc).isoformat()


class TrajectoryError(Exception):
    """A refusal the API maps to an HTTP status with a structured body."""

    def __init__(self, status_code: int, error: str, message: str, **extra: Any):
        super().__init__(message)
        self.status_code = status_code
        self.detail = {"error": error, "message": message, **extra}


class BackendError(Exception):
    """The backend could not enter or leave its mode, or a command failed."""

    def __init__(self, message: str, code: Optional[int] = None):
        super().__init__(message)
        self.code = code


class StopRequested(Exception):
    """A STOP arrived while the backend was entering its mode."""


# ── Settings ─────────────────────────────────────────────────────────

# (min, max) for each numeric setting; booleans and strings are checked apart.
_BOUNDS = {
    "servo_rate_hz": (50.0, 250.0),
    "max_servo_rate_hz": (50.0, 250.0),
    "online_rate_hz": (5.0, 100.0),
    "online_lookahead_periods": (0.0, 10.0),
    "max_joint_acc_deg_s2": (1.0, 5000.0),
    "start_tolerance_deg": (0.01, 5.0),
    "command_timeout_s": (0.01, 1.0),
    "max_lateness_periods": (1.0, 50.0),
    "settle_tolerance_deg": (0.01, 5.0),
    "settle_timeout_s": (0.1, 30.0),
    "unstarted_ttl_s": (1.0, 3600.0),
    "max_stop_s": (0.1, 10.0),
    "max_points": (2, 200000),
    "max_points_per_chunk": (1, 200000),
    "max_duration_s": (0.1, 3600.0),
    "watch_interval_s": (0.01, 1.0),
}


@dataclass
class TrajectorySettings:
    """``src/settings/trajectory.yaml``; every key optional. Disabled by default."""

    enabled: bool = False
    backend: str = "servoj"
    servo_rate_hz: float = 100.0
    # Stage 0a (2026-10-09): 200 Hz showed no late tick with the service's
    # background polling suspended. Above that is unmeasured.
    max_servo_rate_hz: float = 200.0
    online_rate_hz: float = 50.0
    online_lookahead_periods: float = 2.0
    max_joint_acc_deg_s2: float = DEFAULT_MAX_JOINT_ACC_DEG_S2
    start_tolerance_deg: float = 0.5
    command_timeout_s: float = 0.1
    max_lateness_periods: float = 5.0
    settle_tolerance_deg: float = 0.2
    settle_timeout_s: float = 3.0
    unstarted_ttl_s: float = 300.0
    max_stop_s: float = 2.0
    max_points: int = 60000
    max_points_per_chunk: int = 5000
    max_duration_s: float = 600.0
    watch_interval_s: float = 0.1
    log_dir: str = os.path.join("logs", "trajectory")
    realtime_report: bool = True

    @classmethod
    def from_mapping(cls, data: Optional[Dict[str, Any]]) -> "TrajectorySettings":
        data = dict(data or {})
        known = {f.name for f in fields(cls)}
        unknown = sorted(set(data) - known)
        if unknown:
            raise ValueError(f"Unknown trajectory settings: {', '.join(unknown)}")
        settings = cls(**data)
        settings.check()
        return settings

    @classmethod
    def load(cls, path: str = SETTINGS_PATH) -> "TrajectorySettings":
        try:
            import yaml
            with open(path, encoding="utf-8") as handle:
                return cls.from_mapping(yaml.safe_load(handle) or {})
        except FileNotFoundError:
            return cls()

    def check(self) -> None:
        for name in ("enabled", "realtime_report"):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f"{name} must be true or false, got {getattr(self, name)!r}")
        if self.backend not in BACKENDS:
            raise ValueError(f"backend must be one of {BACKENDS}, got {self.backend!r}")
        if not isinstance(self.log_dir, str) or not self.log_dir:
            raise ValueError("log_dir must be a non-empty path")
        for name, (lo, hi) in _BOUNDS.items():
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{name} must be a number, got {value!r}")
            if not lo <= value <= hi:
                raise ValueError(f"{name} must be within {lo}-{hi}, got {value!r}")
        for name in ("max_points", "max_points_per_chunk"):
            if not isinstance(getattr(self, name), int):
                raise ValueError(f"{name} must be a whole number")
        if self.servo_rate_hz > self.max_servo_rate_hz:
            raise ValueError("servo_rate_hz must not exceed max_servo_rate_hz")
        if self.max_points_per_chunk > self.max_points:
            raise ValueError("max_points_per_chunk must not exceed max_points")

    def rate_range(self) -> tuple:
        if self.backend == "servoj":
            return (50.0, self.max_servo_rate_hz)
        return (5.0, 100.0)

    def default_rate(self) -> float:
        return self.servo_rate_hz if self.backend == "servoj" else self.online_rate_hz


# ── Backends ─────────────────────────────────────────────────────────


class XArmBackend:
    """Shared plumbing for the two xArm streaming modes.

    Health is read from the SDK's cached report values, never from a
    command, so checking it costs nothing on the command channel."""

    mode = 0

    def __init__(self, arm, command_timeout_s: float = 0.1):
        self.arm = arm
        self.command_timeout_s = command_timeout_s
        self._saved_timeout = None
        self.mode_changed = False

    def _check(self, code, what):
        if code not in (0, None):
            raise BackendError(f"{what} returned code {code}", code)

    def prepare(self, abort: Callable[[], bool] = lambda: False, wait_s: float = 1.5,
                sleep=time.sleep, clock=time.monotonic) -> None:
        """Enter the streaming mode. Refuses an arm that is not idle and
        healthy, and checks ``abort`` (a STOP) before the one command that
        could re-enable a stopped arm, ``set_state(0)``."""
        arm = self.arm
        if getattr(arm, "error_code", 0) or getattr(arm, "warn_code", 0):
            raise BackendError(f"arm has error {arm.error_code} / warning {arm.warn_code}; clear it first")
        if arm.state != 2:
            raise BackendError(f"arm is not idle (state {arm.state}); it must be ready and still")
        if abort():
            raise StopRequested()
        # set_timeout(0) changes nothing and returns the current value.
        self._saved_timeout = arm.set_timeout(0)
        arm.set_timeout(self.command_timeout_s)
        # From here on a failure must put the arm back in mode 0.
        self.mode_changed = True
        self._check(arm.set_mode(self.mode), f"set_mode({self.mode})")
        if abort():
            raise StopRequested()
        self._check(arm.set_state(0), "set_state(0)")
        deadline = clock() + wait_s
        while clock() < deadline:
            if abort():
                raise StopRequested()
            if arm.mode == self.mode and arm.state in HEALTHY_STATES:
                return
            sleep(0.02)
        raise BackendError(f"arm did not report mode {self.mode} within {wait_s} s "
                           f"(mode {arm.mode}, state {arm.state})")

    def finish(self) -> None:
        self._check(self.arm.set_mode(0), "set_mode(0)")
        self._check(self.arm.set_state(0), "set_state(0)")

    def restore(self) -> None:
        if self._saved_timeout is not None:
            try:
                self.arm.set_timeout(self._saved_timeout)
            except Exception as exc:  # noqa: BLE001 - restore must not raise
                logger.warning(f"[trajectory] could not restore the SDK timeout: {exc}")

    def emergency_stop(self) -> None:
        try:
            self.arm.emergency_stop()
        except Exception as exc:  # noqa: BLE001 - stop path must not raise
            logger.error(f"[trajectory] emergency_stop raised: {exc}")

    def read_joints(self, n: int) -> Optional[List[float]]:
        code, angles = self.arm.get_servo_angle(is_radian=False)
        if code != 0 or angles is None:
            return None
        values = [float(a) for a in list(angles)[:n]]
        return values if all(math.isfinite(v) for v in values) else None

    def health(self) -> Dict[str, Any]:
        arm = self.arm
        return {
            "connected": bool(getattr(arm, "connected", False)),
            "state": getattr(arm, "state", None),
            "mode": getattr(arm, "mode", None),
            "error_code": getattr(arm, "error_code", 0),
            "warn_code": getattr(arm, "warn_code", 0),
        }


class XArmServoJBackend(XArmBackend):
    """Mode 1: the controller moves at full speed to each newest target."""

    name = "servoj"
    mode = 1

    def send(self, q: Sequence[float], speed_deg_s: float, acc_deg_s2: float) -> int:
        return self.arm.set_servo_angle_j(angles=list(q), is_radian=False)


class XArmOnlinePlanningBackend(XArmBackend):
    """Mode 6: each new target interrupts the current move and is replanned
    by the controller with the speed and acceleration sent with it."""

    name = "online_planning"
    mode = 6

    def send(self, q: Sequence[float], speed_deg_s: float, acc_deg_s2: float) -> int:
        return self.arm.set_servo_angle(angle=list(q), speed=speed_deg_s, mvacc=acc_deg_s2,
                                        wait=False, is_radian=False)


def make_backend(name: str, arm, command_timeout_s: float):
    if name == "servoj":
        return XArmServoJBackend(arm, command_timeout_s)
    if name == "online_planning":
        return XArmOnlinePlanningBackend(arm, command_timeout_s)
    raise ValueError(f"unknown backend {name!r}")


class RealtimeJointMonitor:
    """Reads the controller's 100 Hz real-time report (port 30003).

    Read-only: nothing is ever sent on the socket. Frames start with a
    4-byte big-endian length; byte 4 holds state (low nibble) and mode
    (high nibble); bytes 7..34 are seven little-endian float32 joint angles
    in radians (as parsed by the SDK). Frames with non-finite or absurd
    angles are dropped. On this cell frames arrive in bursts about every
    50 ms, so a sample's receive time can lag its measurement by that much.
    """

    def __init__(self, host: str, num_joints: int, port: int = 30003, keep: int = 200000):
        self.host, self.port, self.num_joints = host, port, num_joints
        self._keep = keep
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self.latest: Optional[tuple] = None   # (perf_counter, joints_deg, state, mode)
        self.samples: List[tuple] = []
        self.dropped = 0
        self.error: Optional[str] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="xarm-trajectory-monitor", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        import struct
        try:
            with socket.create_connection((self.host, self.port), timeout=2.0) as sock:
                sock.settimeout(0.2)
                buffer = b""
                while not self._stop.is_set():
                    try:
                        chunk = sock.recv(65536)
                    except socket.timeout:
                        continue
                    if not chunk:
                        self.error = "report stream closed"
                        return
                    buffer += chunk
                    while len(buffer) >= 4:
                        size = int.from_bytes(buffer[:4], "big")
                        if size < 35 or size > 65536:
                            self.error = f"unexpected report frame size {size}"
                            return
                        if len(buffer) < size:
                            break
                        frame, buffer = buffer[:size], buffer[size:]
                        state, mode = frame[4] & 0x0F, frame[4] >> 4
                        radians = struct.unpack("<7f", frame[7:35])
                        joints = [math.degrees(r) for r in radians[: self.num_joints]]
                        if not all(math.isfinite(v) and abs(v) < 1000.0 for v in joints):
                            self.dropped += 1
                            continue
                        sample = (time.perf_counter(), joints, state, mode)
                        with self._lock:
                            self.latest = sample
                            if len(self.samples) < self._keep:
                                self.samples.append(sample)
        except OSError as exc:
            self.error = str(exc)


# ── Sessions ─────────────────────────────────────────────────────────


def chunk_digest(points: List[Dict[str, Any]], final: bool) -> str:
    canonical = json.dumps({"points": points, "final": bool(final)}, sort_keys=True,
                           separators=(",", ":"), allow_nan=True)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


@dataclass
class TrajectorySession:
    id: str
    owner_key: Optional[str]   # the claim's session_id that created it
    owner: Optional[str]
    backend: str
    rate_hz: float
    num_joints: int
    start_tolerance_deg: float
    created_at: float
    created_mono: float
    state: str = CREATED
    reason: Optional[str] = None
    chunk_digests: Dict[int, str] = field(default_factory=dict)
    raw_points: List[Dict[str, Any]] = field(default_factory=list)
    final_received: bool = False
    report: Optional[Dict[str, Any]] = None
    started_at: Optional[float] = None       # wall time of tau = 0 (after the lead-in)
    began_at: Optional[float] = None         # wall time streaming began (lead-in start)
    ended_at: Optional[float] = None
    t_exec: float = 0.0
    duration_s: Optional[float] = None
    lead_in_s: float = 0.0
    cancel_requested: bool = False
    events: List[Dict[str, Any]] = field(default_factory=list)
    stats: Dict[str, Any] = field(default_factory=dict)
    final: Optional[Dict[str, Any]] = None
    measured: Optional[Dict[str, Any]] = None
    log_path: Optional[str] = None
    record: Optional[Dict[str, Any]] = None

    def event(self, kind: str, **data: Any) -> None:
        self.events.append({"at": _utc(time.time()), "event": kind, **data})

    @property
    def next_seq(self) -> int:
        return len(self.chunk_digests)

    def status(self) -> Dict[str, Any]:
        fraction = None
        if self.duration_s:
            fraction = round(min(1.0, self.t_exec / self.duration_s), 4)
        return {
            "session_id": self.id,
            "state": self.state,
            "reason": self.reason,
            "backend": self.backend,
            "rate_hz": self.rate_hz,
            "owner": self.owner,
            "created_at_utc": _utc(self.created_at),
            "received": {
                "last_seq": self.next_seq - 1 if self.chunk_digests else None,
                "points": len(self.raw_points),
                "final_received": self.final_received,
                "t_end_received": self.raw_points[-1]["t"] if self.raw_points else None,
            },
            "executed": {
                "t_exec": round(self.t_exec, 4),
                "duration_s": self.duration_s,
                "fraction": fraction,
                "lead_in_s": round(self.lead_in_s, 4),
            },
            "began_at_utc": _utc(self.began_at),
            "started_at_utc": _utc(self.started_at),
            "ended_at_utc": _utc(self.ended_at),
            "measured": self.measured,
            "servo": self.stats.get("servo"),
            "tracking": self.stats.get("tracking"),
            "final": self.final,
            "events": self.events[-20:],
        }


class _Ticks:
    """Per-tick record in compact arrays (150k ticks fit in about 10 MB)."""

    def __init__(self, num_joints: int):
        self.scheduled = array.array("d")
        self.sent = array.array("d")
        self.round_trip = array.array("d")
        self.tau = array.array("d")
        self.code = array.array("i")
        self.q = [array.array("d") for _ in range(num_joints)]

    def add(self, scheduled, sent, round_trip, tau, code, q):
        self.scheduled.append(scheduled)
        self.sent.append(sent)
        self.round_trip.append(round_trip)
        self.tau.append(tau)
        try:
            self.code.append(int(code if code is not None else 0))
        except (TypeError, ValueError, OverflowError):
            self.code.append(-1)
        for j, value in enumerate(q):
            self.q[j].append(value)

    def __len__(self):
        return len(self.sent)


def _stats_ms(values: Sequence[float]) -> Optional[Dict[str, float]]:
    if not values:
        return None
    ms = sorted(v * 1000.0 for v in values)

    def pct(p):
        return round(ms[min(len(ms) - 1, int(p / 100.0 * len(ms)))], 3)

    return {"n": len(ms), "mean": round(sum(ms) / len(ms), 3), "p50": pct(50),
            "p99": pct(99), "max": round(ms[-1], 3)}


# ── Manager and executor ─────────────────────────────────────────────


class TrajectoryManager:
    """One per controller. Holds at most one open session and runs it.

    Sessions are bound to an *owner key* (the API passes the claim's
    session_id), so a client that renews its claim token keeps its session.
    ``watch()`` returns a soft-stop reason (e.g. ``"claim_lost"``) or None;
    it is called every ``watch_interval_s`` while running.
    ``on_finish(session)`` runs exactly once when a started session ends,
    after the mode has been restored and before the terminal state is
    reported; the API uses it to release the motion slot.
    """

    def __init__(
        self,
        settings: TrajectorySettings,
        *,
        clock: Callable[[], float] = time.perf_counter,
        sleep: Callable[[float], None] = time.sleep,
        wall: Callable[[], float] = time.time,
        keep_sessions: int = 5,
    ):
        self.settings = settings
        self._clock, self._sleep, self._wall = clock, sleep, wall
        self._lock = threading.Lock()
        self._sessions: Dict[str, TrajectorySession] = {}
        self._order: List[str] = []
        self._keep = keep_sessions
        self._current: Optional[str] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_lock = threading.Lock()
        self._stop_gen = 0
        self._stop_reason: Optional[str] = None

    # ── Stops ────────────────────────────────────────────────────────

    def stop_generation(self) -> int:
        """Capture before a start's gates; pass to :meth:`start`."""
        with self._stop_lock:
            return self._stop_gen

    def notify_hard_stop(self, reason: str) -> None:
        """STOP, the sash watchdog or disconnect: send nothing more, now.

        Unconditional and never cleared: a start that captured an older
        generation will refuse or stop. Safe from any thread."""
        with self._stop_lock:
            self._stop_gen += 1
            self._stop_reason = reason

    def _stopped_since(self, generation: int) -> Optional[str]:
        with self._stop_lock:
            return (self._stop_reason or "stop") if self._stop_gen != generation else None

    # ── Queries ──────────────────────────────────────────────────────

    @property
    def running(self) -> bool:
        """True from start until the run has fully ended (mode restored and
        motion slot released). Background SDK traffic pauses while True."""
        session = self._current_session()
        return session is not None and session.state in ACTIVE

    def _current_session(self) -> Optional[TrajectorySession]:
        with self._lock:
            return self._sessions.get(self._current) if self._current else None

    def get(self, session_id: str) -> TrajectorySession:
        with self._lock:
            self._expire_unstarted()
            session = self._sessions.get(session_id)
        if session is None:
            raise TrajectoryError(404, "session_not_found", f"No trajectory session {session_id!r}")
        return session

    def summary(self) -> Optional[Dict[str, Any]]:
        """Compact view for ``details.trajectory`` in /status."""
        session = self._current_session()
        base = {"enabled": self.settings.enabled, "backend": self.settings.backend}
        if session is None:
            return {**base, "session": None}
        return {
            **base,
            "session": {
                "session_id": session.id,
                "state": session.state,
                "reason": session.reason,
                "t_exec": round(session.t_exec, 3),
                "duration_s": session.duration_s,
                "started_at_utc": _utc(session.started_at),
            },
        }

    # ── Session lifecycle ────────────────────────────────────────────

    def _expire_unstarted(self) -> None:
        """Caller holds the lock."""
        now = self._clock()
        for session in self._sessions.values():
            if session.state == CREATED and now - session.created_mono > self.settings.unstarted_ttl_s:
                session.state, session.reason = EXPIRED, "not started in time"
                session.event("expired")

    @staticmethod
    def _check_owner(session: TrajectorySession, key: Optional[str]) -> None:
        if session.owner_key != key:
            raise TrajectoryError(
                423, "session_owner_mismatch",
                "This session belongs to a different claim; only its creator may upload to, "
                "start or cancel it",
            )

    def create(self, *, key: Optional[str], owner: Optional[str], num_joints: int,
               rate_hz: Optional[float] = None,
               start_tolerance_deg: Optional[float] = None) -> TrajectorySession:
        settings = self.settings
        rate = float(rate_hz) if rate_hz is not None else settings.default_rate()
        lo, hi = settings.rate_range()
        if not (math.isfinite(rate) and lo <= rate <= hi):
            raise TrajectoryError(
                422, "rate_out_of_range",
                f"rate_hz {rate} is outside {lo:g}-{hi:g} Hz for the {settings.backend} backend",
            )
        tolerance = settings.start_tolerance_deg if start_tolerance_deg is None else float(start_tolerance_deg)
        if not (math.isfinite(tolerance) and 0 < tolerance <= settings.start_tolerance_deg):
            raise TrajectoryError(
                422, "start_tolerance_out_of_range",
                f"start_tolerance_deg must be within (0, {settings.start_tolerance_deg}]",
            )
        with self._lock:
            self._expire_unstarted()
            current = self._sessions.get(self._current) if self._current else None
            if current is not None and current.state not in TERMINAL:
                raise TrajectoryError(
                    409, "session_open",
                    f"Session {current.id} is {current.state}; cancel it or let it finish first",
                    session_id=current.id,
                )
            session = TrajectorySession(
                id=uuid.uuid4().hex[:16], owner_key=key, owner=owner, backend=settings.backend,
                rate_hz=rate, num_joints=num_joints, start_tolerance_deg=tolerance,
                created_at=self._wall(), created_mono=self._clock(),
            )
            session.event("created", rate_hz=rate, backend=settings.backend)
            self._sessions[session.id] = session
            self._order.append(session.id)
            self._current = session.id
            while len(self._order) > self._keep:
                old = self._order.pop(0)
                if old != self._current:
                    self._sessions.pop(old, None)
            return session

    def validation_limits(self, session: TrajectorySession, joint_limits, max_joint_speed) -> JointLimits:
        # Mode 6 commands at a low rate but samples a dense path; validate the
        # path at no less than 100 Hz either way.
        rate = session.rate_hz if session.backend == "servoj" else max(session.rate_hz, 100.0)
        return JointLimits(
            joint_limits_deg=list(joint_limits),
            max_joint_speed_deg_s=float(max_joint_speed),
            max_joint_acc_deg_s2=self.settings.max_joint_acc_deg_s2,
            servo_rate_hz=rate,
            max_duration_s=self.settings.max_duration_s,
            max_points=self.settings.max_points,
        )

    def add_chunk(self, session_id: str, seq: int, key: Optional[str], points: List[Dict[str, Any]],
                  final: bool, limits: JointLimits) -> Dict[str, Any]:
        session = self.get(session_id)
        self._check_owner(session, key)
        digest = chunk_digest(points, final)
        with self._lock:
            known = session.chunk_digests.get(seq)
            if known is not None:
                if known == digest:
                    return {"accepted": True, "duplicate": True, "seq": seq, "session": session.status()}
                raise TrajectoryError(409, "chunk_conflict",
                                      f"Chunk {seq} was already received with different content", seq=seq)
            if session.state in ACTIVE:
                raise TrajectoryError(409, "session_running",
                                      "Appending chunks while a trajectory runs is not supported in "
                                      "this version; upload the whole trajectory before start")
            if session.state in TERMINAL or session.final_received:
                raise TrajectoryError(409, "session_closed",
                                      f"Session is {session.state}"
                                      + ("; the final chunk is already in" if session.final_received else ""))
            if seq != session.next_seq:
                raise TrajectoryError(409, "chunk_out_of_order",
                                      f"Expected chunk {session.next_seq}, got {seq}",
                                      expected_seq=session.next_seq)
            if not points:
                raise TrajectoryError(422, "trajectory_invalid", "A chunk needs at least one point")
            if len(points) > self.settings.max_points_per_chunk:
                raise TrajectoryError(422, "trajectory_invalid",
                                      f"{len(points)} points in one chunk; limit is "
                                      f"{self.settings.max_points_per_chunk}")
            if len(session.raw_points) + len(points) > self.settings.max_points:
                raise TrajectoryError(422, "trajectory_invalid",
                                      f"The trajectory would exceed {self.settings.max_points} points")
            combined = session.raw_points + list(points)
        # Validate everything received so far, without the start-state check
        # (that happens at start, because the arm may move in between).
        parsed = [JointPoint.from_mapping(p) for p in combined]
        report = validate_joint_trajectory(parsed, limits, check_start=False, partial=not final)
        if not report["valid"]:
            raise TrajectoryError(422, "trajectory_invalid",
                                  f"{len(report['errors'])} violation(s); see report.errors", report=report)
        with self._lock:
            if session.state != CREATED or session.chunk_digests.get(seq) is not None \
                    or seq != session.next_seq:
                raise TrajectoryError(409, "chunk_out_of_order",
                                      f"Expected chunk {session.next_seq}, got {seq}",
                                      expected_seq=session.next_seq)
            session.chunk_digests[seq] = digest
            session.raw_points = combined
            session.final_received = bool(final)
            session.report = report
            session.event("chunk", seq=seq, points=len(points), final=bool(final))
        return {"accepted": True, "duplicate": False, "seq": seq, "report": report,
                "session": session.status()}

    # ── Start, cancel ────────────────────────────────────────────────

    def start(self, session_id: str, key: Optional[str], *, limits: JointLimits,
              start_joints: Optional[Sequence[float]], backend, watch: Callable[[], Optional[str]],
              on_finish: Callable[[TrajectorySession], None],
              monitor: Optional[RealtimeJointMonitor] = None,
              on_measured: Optional[Callable[[List[float]], None]] = None,
              stop_generation: Optional[int] = None) -> Dict[str, Any]:
        """Validate against the measured start and launch the executor.

        ``stop_generation`` is what the caller captured *before* its gates;
        a STOP since then refuses the start. The caller has reserved the
        motion slot; on any refusal here it must release the slot itself.
        Once this returns, ``on_finish`` owns the release."""
        generation = self.stop_generation() if stop_generation is None else stop_generation
        session = self.get(session_id)
        self._check_owner(session, key)
        with self._lock:
            if self._stopped_since(generation):
                raise TrajectoryError(409, "stopped_during_start",
                                      "A STOP arrived while the start was being prepared; nothing moved")
            if session.state != CREATED:
                raise TrajectoryError(409, "session_not_startable", f"Session is {session.state}")
            if not session.final_received:
                raise TrajectoryError(409, "trajectory_incomplete",
                                      "Upload the final chunk before starting; this version executes "
                                      "only complete trajectories")
            if self._thread is not None and self._thread.is_alive():
                raise TrajectoryError(409, "motion_in_progress", "A trajectory is still finishing")
            session.state = STARTING
            session.cancel_requested = False

        def back_to_created():
            with self._lock:
                if session.state == STARTING:
                    session.state = CREATED

        try:
            parsed = [JointPoint.from_mapping(p) for p in session.raw_points]
            report = validate_joint_trajectory(parsed, limits, start_joints=start_joints,
                                               start_tolerance_deg=session.start_tolerance_deg)
            session.report = report
            if not report["valid"]:
                raise TrajectoryError(422, "trajectory_invalid",
                                      f"{len(report['errors'])} violation(s); see report.errors",
                                      report=report)
            trajectory = JointTrajectory(parsed)
        except Exception:
            back_to_created()
            raise
        with self._lock:
            if self._stopped_since(generation):
                session.state = CREATED
                raise TrajectoryError(409, "stopped_during_start",
                                      "A STOP arrived while the start was being prepared; nothing moved")
            if session.cancel_requested or session.state != STARTING:
                session.state, session.reason = CANCELLED, "cancelled before start"
                session.event("cancel")
                raise TrajectoryError(409, "session_cancelled", "The session was cancelled during start")
            session.state = RUNNING
            session.duration_s = round(trajectory.duration, 6)
            session.event("start")
            self._thread = threading.Thread(
                target=self._run, name="xarm-trajectory",
                args=(session, trajectory, limits, backend, watch, on_finish, monitor, on_measured,
                      generation),
                daemon=True,
            )
            self._thread.start()
        return session.status()

    def cancel(self, session_id: str, key: Optional[str]) -> Dict[str, Any]:
        session = self.get(session_id)
        self._check_owner(session, key)
        with self._lock:
            if session.state == CREATED:
                session.state, session.reason = CANCELLED, "cancelled before start"
                session.event("cancel")
            elif session.state in ACTIVE:
                session.cancel_requested = True
                session.event("cancel")
        return session.status()

    def wait_idle(self, timeout_s: float = 5.0) -> bool:
        thread = self._thread
        if thread is None:
            return True
        thread.join(timeout=timeout_s)
        return not thread.is_alive()

    # ── The executor ─────────────────────────────────────────────────

    def _wait_until(self, deadline: float, generation: int) -> None:
        """Sleep until ``deadline`` in short slices so a STOP is seen within
        5 ms even at a 5 Hz command rate."""
        while True:
            remaining = deadline - self._clock()
            if remaining <= 0 or self._stopped_since(generation):
                return
            self._sleep(min(remaining, 0.005))

    def _run(self, session, trajectory, limits, backend, watch, on_finish, monitor, on_measured,
             generation):
        settings = self.settings
        clock, wall = self._clock, self._wall
        n = session.num_joints
        rate = session.rate_hz
        dt = 1.0 / rate
        vmax, amax = limits.max_joint_speed_deg_s, limits.max_joint_acc_deg_s2
        expected_mode = backend.mode
        online = session.backend == "online_planning"
        lookahead = settings.online_lookahead_periods * dt if online else 0.0
        ticks = _Ticks(n)
        ctx = {"hard": None, "soft": None, "prepared": False, "late_total": 0.0, "late_ticks": 0,
               "q_last": None, "lead": None}

        def stopped():
            return self._stopped_since(generation)

        def hard_stop(reason, estop=True):
            ctx["hard"] = reason
            if estop:
                backend.emergency_stop()

        if monitor is not None:
            try:
                monitor.start()
            except Exception as exc:  # noqa: BLE001 - records only
                logger.warning(f"[trajectory] real-time monitor did not start: {exc}")
                monitor = None
        try:
            if stopped():
                hard_stop(stopped(), estop=False)
                return
            try:
                backend.prepare(abort=lambda: bool(stopped()))
                ctx["prepared"] = True
            except StopRequested:
                reason = stopped() or "stop"
                # STOP issues its own emergency stop; a disconnect does not.
                hard_stop(reason, estop=reason != "stop")
                return
            except Exception as exc:  # noqa: BLE001
                ctx["hard"] = f"prepare_failed: {exc}"
                return
            if stopped():
                # The STOP raced our set_state(0): stop the arm again.
                hard_stop(stopped())
                return
            # Re-measure now that the arm is in its streaming mode, and plan
            # the lead-in from this, not from the reading taken at start.
            measured = backend.read_joints(n)
            if measured is None:
                ctx["soft"] = "start_unreadable"
                return
            gap = max(abs(a - b) for a, b in zip(measured, trajectory.start_q))
            if gap > session.start_tolerance_deg:
                ctx["soft"] = f"start_moved: {gap:.3f} deg from the first point"
                return
            lead = lead_in(measured, trajectory.start_q, LEAD_IN_FRACTION * vmax, LEAD_IN_FRACTION * amax)
            ctx["lead"] = lead
            lead_s = lead.duration if lead else 0.0
            session.lead_in_s = lead_s
            total = lead_s + trajectory.duration
            summary = session.report.get("summary", {}) if session.report else {}
            # The stop rate depends on the peaks of the part being stopped in:
            # the main trajectory, or (for a stop during the lead-in, which
            # may carry on into the trajectory) the stricter of the two.
            alpha_main = stop_deceleration(summary.get("max_joint_speed_deg_s") or 0.0,
                                           summary.get("max_joint_acc_deg_s2") or 0.0,
                                           amax, settings.max_stop_s)
            alpha_lead = alpha_main
            if lead:
                distance = max(abs(a - b) for a, b in zip(lead.start_q, lead.final_q))
                lead_v = 1.875 * distance / lead_s
                lead_a = (10.0 / math.sqrt(3.0)) * distance / (lead_s * lead_s)
                alpha_lead = min(alpha_main, stop_deceleration(lead_v, lead_a, amax, settings.max_stop_s))
            alpha = None
            acc_bound = 1.5 * amax + vmax / settings.max_stop_s + 1e-6

            def sample(t):
                if t < lead_s:
                    return lead.sample(t)
                return trajectory.sample(t - lead_s)

            q_prev = list(measured)
            q_prev2 = list(measured)
            ctx["q_last"] = list(measured)
            # A cyclic garbage collection can pause every thread for tens of
            # ms (two 29-40 ms command stalls on the arm, 2026-10-09). The
            # executor allocates no cycles, so pause the collector while
            # streaming and restore it in _finalise.
            ctx["gc_was_enabled"] = gc.isenabled()
            gc.disable()
            monitor_armed = False
            rate_scale = 1.0
            tau = 0.0
            last_watch = -1e9
            last_push = -1e9
            session.began_at = wall()
            session.event("streaming", mode=expected_mode, lead_in_s=round(lead_s, 4))
            if lead_s <= 0:
                session.started_at = session.began_at
            next_deadline = clock()
            k = 0
            while True:
                reason = stopped()
                if reason:
                    hard_stop(reason, estop=reason not in ("stop",))
                    break
                now = clock()
                if ctx["soft"] is None:
                    if session.cancel_requested:
                        ctx["soft"] = "cancelled"
                    elif now - last_watch >= settings.watch_interval_s:
                        last_watch = now
                        try:
                            watched = watch()
                        except Exception as exc:  # noqa: BLE001
                            watched = f"watch_error: {exc}"
                        if watched:
                            ctx["soft"] = watched
                    if ctx["soft"]:
                        session.event("constrained_stop", reason=ctx["soft"],
                                      t_exec=round(max(0.0, tau - lead_s), 4))
                        session.state = STOPPING
                fault, monitor_armed = _health_fault(backend.health(), expected_mode, monitor, monitor_armed)
                if fault:
                    hard_stop(f"controller_fault: {fault}")
                    break
                self._wait_until(next_deadline, generation)
                reason = stopped()
                if reason:
                    hard_stop(reason, estop=reason not in ("stop",))
                    break
                woke = clock()
                lateness = woke - next_deadline
                if lateness > settings.max_lateness_periods * dt and ctx["soft"] is None:
                    ctx["soft"] = "timing"
                    session.state = STOPPING
                    session.event("constrained_stop", reason="timing",
                                  lateness_ms=round(lateness * 1000, 3))
                if lateness > 0.5 * dt:
                    # Delay rather than skip: re-anchor so the next samples are
                    # not sent in a burst that would make the arm catch up.
                    ctx["late_total"] += lateness
                    ctx["late_ticks"] += 1
                    next_deadline = woke
                if ctx["soft"] is not None:
                    if alpha is None:
                        alpha = alpha_lead if tau < lead_s else alpha_main
                        ctx["stop_alpha"] = alpha
                    rate_scale = max(0.0, rate_scale - alpha * dt)
                tau = min(total, tau + rate_scale * dt)
                if session.started_at is None and tau >= lead_s:
                    session.started_at = wall()
                q, _, _ = sample(min(total, tau + lookahead))
                # Defence in depth: never send a sample outside the joint
                # limits, or one that implies a speed or acceleration the
                # validated trajectory cannot. In mode 6 the first target is
                # a lookahead ahead of the measured pose by design.
                step_dt = dt + (lookahead if (online and k == 0) else 0.0)
                bad = _sample_fault(q, q_prev, q_prev2 if not (online and k < 2) else None,
                                    step_dt, dt, limits, vmax, acc_bound)
                if bad:
                    hard_stop(f"internal_check: {bad}")
                    break
                speed = max(abs(a - b) for a, b in zip(q, q_prev)) / step_dt
                send_speed = min(vmax, max(1.0, speed))
                sent = clock()
                code = backend.send(q, send_speed, amax)
                round_trip = clock() - sent
                ticks.add(next_deadline, sent, round_trip, tau, code, q)
                if code not in (0, None):
                    hard_stop(f"sdk_error: code {code}")
                    break
                q_prev2, q_prev = q_prev, q
                ctx["q_last"] = q
                session.t_exec = max(0.0, tau - lead_s)
                if monitor is not None and monitor.latest is not None:
                    t_m, joints_m, _, _ = monitor.latest
                    session.measured = {"joints_deg": [round(v, 4) for v in joints_m],
                                        "age_ms": round((clock() - t_m) * 1000, 1)}
                    if on_measured is not None and woke - last_push >= 0.1:
                        last_push = woke
                        try:
                            on_measured(joints_m)
                        except Exception:  # noqa: BLE001 - display only
                            pass
                if tau >= total:
                    break
                if ctx["soft"] is not None and rate_scale <= 0.0:
                    break
                next_deadline += dt
                k += 1
        except Exception as exc:  # noqa: BLE001 - the run must always finalise
            if ctx["hard"] is None:
                logger.error(f"[trajectory] executor error: {exc}", exc_info=True)
                hard_stop(f"executor_error: {exc}")
        finally:
            self._finalise(session, backend, trajectory, ticks, ctx, monitor, on_finish, generation)

    def _finalise(self, session, backend, trajectory, ticks, ctx, monitor, on_finish, generation):
        """Settle, restore mode 0, release, report. Always reaches on_finish
        and a terminal state, whatever raises on the way."""
        settings = self.settings
        n = session.num_joints
        hard, soft = ctx["hard"], ctx["soft"]
        mode_restored = False
        settle_s = None
        measured_final = None
        target = list(ctx["q_last"]) if ctx["q_last"] is not None else list(trajectory.final_q)
        try:
            prepared = ctx["prepared"]
            stop_reason = self._stopped_since(generation)
            if (hard is not None and not prepared and hard.startswith("prepare_failed")
                    and not stop_reason and getattr(backend, "mode_changed", False)):
                # Nothing was streamed, but the mode was changed: put it back.
                # (A prepare refused before set_mode leaves the arm alone, so
                # a stopped arm is never re-enabled here.)
                try:
                    backend.finish()
                    mode_restored = True
                except Exception as exc:  # noqa: BLE001
                    logger.error(f"[trajectory] mode restore after a failed prepare: {exc}")
            elif hard is None and prepared:
                if not stop_reason:
                    begin = self._clock()
                    while self._clock() - begin < settings.settle_timeout_s:
                        stop_reason = self._stopped_since(generation)
                        if stop_reason:
                            break
                        measured_final = backend.read_joints(n)
                        if measured_final is not None and max(
                                abs(a - b) for a, b in zip(measured_final, target)) <= settings.settle_tolerance_deg:
                            settle_s = round(self._clock() - begin, 3)
                            break
                        self._sleep(0.05)
                # Re-check right before the restore: its set_state(0) would
                # re-enable an arm that a STOP has just halted.
                stop_reason = stop_reason or self._stopped_since(generation)
                if stop_reason:
                    # STOP during the run's tail: leave the stopped state alone.
                    hard = stop_reason
                else:
                    try:
                        backend.finish()
                        mode_restored = True
                    except Exception as exc:  # noqa: BLE001
                        hard = f"mode_restore_failed: {exc}"
                        backend.emergency_stop()
                    late_stop = self._stopped_since(generation)
                    if late_stop and mode_restored:
                        # The STOP landed during our set_state(0); honour it.
                        backend.emergency_stop()
                        hard, mode_restored = late_stop, False
        except Exception as exc:  # noqa: BLE001
            logger.error(f"[trajectory] finalise error: {exc}", exc_info=True)
            if hard is None:
                hard = f"finalise_error: {exc}"
                backend.emergency_stop()
        finally:
            if ctx.get("gc_was_enabled"):
                gc.enable()
            for step in (backend.restore, monitor.stop if monitor is not None else None):
                if step is None:
                    continue
                try:
                    step()
                except Exception as exc:  # noqa: BLE001
                    logger.warning(f"[trajectory] cleanup step failed: {exc}")
            if hard is not None:
                state = STOPPED if hard in HARD_STOP_REASONS else FAILED
            elif soft == "cancelled":
                state = CANCELLED
            elif soft is not None:
                state = FAILED
            else:
                state = COMPLETED
            try:
                final_error = None
                if measured_final is not None:
                    final_error = round(max(abs(a - b) for a, b in zip(measured_final, target)), 4)
                session.final = {
                    "measured_joints_deg": [round(v, 4) for v in measured_final] if measured_final else None,
                    "error_to_last_target_deg": final_error,
                    "settle_s": settle_s,
                    "mode_restored": mode_restored,
                    "note": None if hard is None else "Clear errors restores mode 0 and state 0",
                }
                session.stats = self._stats(session, ticks, ctx, monitor)
            except Exception as exc:  # noqa: BLE001
                logger.warning(f"[trajectory] could not summarise the run: {exc}")
            session.reason = hard or (None if soft == "cancelled" else soft)
            session.ended_at = self._wall()
            # Release the motion slot before reporting a terminal state, so a
            # client that sees "completed" can move the arm straight away.
            try:
                on_finish(session)
            except Exception as exc:  # noqa: BLE001
                logger.error(f"[trajectory] on_finish failed: {exc}", exc_info=True)
            session.state = state
            session.event("end", state=state, reason=session.reason)
            try:
                session.record = self._record(session, ticks, monitor)
                self._write_log(session)
            except Exception as exc:  # noqa: BLE001
                logger.warning(f"[trajectory] could not build the session record: {exc}")

    def _stats(self, session, ticks, ctx, monitor):
        sent = list(ticks.sent)
        intervals = [b - a for a, b in zip(sent, sent[1:])]
        lateness = [max(0.0, s - d) for s, d in zip(sent, ticks.scheduled)]
        servo = {
            "rate_hz_configured": session.rate_hz,
            "ticks": len(ticks),
            "period_ms": _stats_ms(intervals),
            "lateness_ms": _stats_ms(lateness),
            "round_trip_ms": _stats_ms(list(ticks.round_trip)),
            "late_ticks": ctx["late_ticks"],
            "cumulative_delay_ms": round(ctx["late_total"] * 1000, 3),
            "sdk_errors": sum(1 for c in ticks.code if c != 0),
            "gc_paused_while_streaming": "gc_was_enabled" in ctx,
            "stop_rate_per_s2": (None if ctx.get("stop_alpha") is None or math.isinf(ctx["stop_alpha"])
                                 else round(ctx["stop_alpha"], 3)),
        }
        tracking = None
        if monitor is not None and monitor.samples and len(ticks):
            errors = []
            sent_arr = ticks.sent
            idx = 0
            for t_m, joints, _, _ in monitor.samples:
                if t_m < sent_arr[0]:
                    continue
                while idx + 1 < len(sent_arr) and sent_arr[idx + 1] <= t_m:
                    idx += 1
                commanded = [ticks.q[j][idx] for j in range(session.num_joints)]
                errors.append(max(abs(a - b) for a, b in zip(commanded, joints)))
            if errors:
                tracking = {
                    "samples": len(errors),
                    "max_error_deg": round(max(errors), 4),
                    "last_error_deg": round(errors[-1], 4),
                    "note": "commanded minus reported; reports arrive in ~50 ms bursts, so this "
                            "includes report latency",
                    "monitor_error": monitor.error,
                    "monitor_dropped_frames": monitor.dropped,
                }
        return {"servo": servo, "tracking": tracking}

    def _record(self, session, ticks, monitor):
        record = {
            "session": session.status(),
            "events": session.events,
            "ticks": {
                "scheduled_s": list(ticks.scheduled),
                "sent_s": list(ticks.sent),
                "round_trip_s": list(ticks.round_trip),
                "tau_s": list(ticks.tau),
                "code": list(ticks.code),
                "commanded_deg": [list(col) for col in ticks.q],
            },
            "reported": None,
            "clock": "perf_counter seconds; sent_s and reported.received_s share it",
        }
        if monitor is not None:
            record["reported"] = {
                "received_s": [s[0] for s in monitor.samples],
                "joints_deg": [s[1] for s in monitor.samples],
                "state": [s[2] for s in monitor.samples],
                "mode": [s[3] for s in monitor.samples],
                "dropped_frames": monitor.dropped,
                "error": monitor.error,
            }
        return record

    def _write_log(self, session):
        try:
            os.makedirs(self.settings.log_dir, exist_ok=True)
            path = os.path.join(self.settings.log_dir, f"{session.id}.json")
            with open(path, "w", encoding="utf-8") as handle:
                json.dump(session.record, handle)
            session.log_path = path
        except (OSError, TypeError, ValueError) as exc:
            logger.warning(f"[trajectory] could not write the session log: {exc}")


def _health_fault(health: Dict[str, Any], expected_mode: int, monitor, armed: bool):
    """(fault or None, monitor armed). The real-time monitor is only trusted
    once one of its frames shows the expected mode in a healthy state;
    frames from before the mode change must not count as a fault."""
    if not health.get("connected", False):
        return "disconnected", armed
    if health.get("error_code"):
        return f"error code {health['error_code']}", armed
    if health.get("warn_code"):
        return f"warning code {health['warn_code']}", armed
    state = health.get("state")
    if state not in HEALTHY_STATES:
        return f"arm state {state}", armed
    mode = health.get("mode")
    if mode != expected_mode:
        return f"arm left mode {expected_mode} (now {mode})", armed
    if monitor is not None and monitor.latest is not None:
        _, _, m_state, m_mode = monitor.latest
        if not armed:
            if m_mode == expected_mode and m_state in HEALTHY_STATES:
                armed = True
        elif m_state not in HEALTHY_STATES:
            return f"arm state {m_state} (real-time report)", armed
        elif m_mode != expected_mode:
            return f"arm left mode {expected_mode} (real-time report: {m_mode})", armed
    return None, armed


def _sample_fault(q, q_prev, q_prev2, step_dt, dt, limits: JointLimits, vmax: float,
                  acc_bound: float) -> Optional[str]:
    for j, (angle, (lo, hi)) in enumerate(zip(q, limits.joint_limits_deg)):
        if not math.isfinite(angle) or angle < lo - 1e-6 or angle > hi + 1e-6:
            return f"J{j + 1} sample {angle} outside [{lo}, {hi}]"
    step = max(abs(a - b) for a, b in zip(q, q_prev))
    if step > 1.2 * vmax * step_dt + 1e-6:
        return f"step of {step:.4f} deg in one tick exceeds the speed bound"
    if q_prev2 is not None:
        acc = max(abs(a - 2 * b + c) for a, b, c in zip(q, q_prev, q_prev2)) / (dt * dt)
        if acc > acc_bound:
            return f"implied acceleration {acc:.0f} deg/s^2 exceeds the bound {acc_bound:.0f}"
    return None
