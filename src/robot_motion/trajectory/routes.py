"""Joint-trajectory routes for the UR service (ur_rtde servoJ).

The xArm's HTTP contract (src/docs/TRAJECTORY_API.md) on a UR: upload a
time-stamped joint trajectory under /control/freehand/trajectory, then start
it; the service streams it with servoJ on its own clock. Installed by
URControl only with a motion block and a control.trajectory block, and
refused (412 trajectory_disabled) unless that block says enabled.

Limits come from the commissioned motion block: the joint envelope, the
joint speed cap and the joint acceleration. Before a start the planned path
is also checked against the workspace box with the controller's forward
kinematics, and while it runs every feedback packet is held to the same
envelope, box, overspeed and force guard as a moveJ (ur_backend.py).

A run holds the motion executor's move slot, so every other move, manual
mode, Clear errors and Disconnect are refused (409) until it ends; STOP
(/control/stop) stays open and halts it. See src/docs/UR5E_TRAJECTORY_PLAN.md.
"""

from __future__ import annotations

import asyncio
import json
import logging
import threading
from typing import Annotated, List, Optional

from fastapi import HTTPException
from fastapi.responses import Response
from pydantic import BaseModel, ConfigDict, Field

from .executor import TERMINAL, TrajectoryError, TrajectoryManager, TrajectorySettings
from .joint_trajectory import JointLimits, JointPoint, JointTrajectory, validate_joint_trajectory
from .ur_backend import FeedbackMonitor, URServoJBackend, path_errors

log = logging.getLogger(__name__)
PREFIX = "/control/freehand/trajectory"
FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]


class JointTrajectoryPoint(BaseModel):
    """One point of a joint trajectory (UR: J1..J6, base to wrist)."""

    model_config = ConfigDict(extra="forbid")
    t: FiniteFloat = Field(ge=0, description="Seconds from the trajectory start; strictly increasing.")
    joints_deg: List[FiniteFloat] = Field(min_length=1, max_length=7)
    velocities_deg_s: Optional[List[FiniteFloat]] = Field(default=None, min_length=1, max_length=7)
    accelerations_deg_s2: Optional[List[FiniteFloat]] = Field(default=None, min_length=1, max_length=7)


class ValidateRequest(BaseModel):
    points: List[JointTrajectoryPoint] = Field(min_length=2, max_length=200000)
    rate_hz: Optional[FiniteFloat] = None
    start_tolerance_deg: Optional[FiniteFloat] = None


class SessionRequest(BaseModel):
    rate_hz: Optional[FiniteFloat] = None
    start_tolerance_deg: Optional[FiniteFloat] = None


class ChunkRequest(BaseModel):
    points: List[JointTrajectoryPoint] = Field(min_length=1, max_length=200000)
    final: bool = False


def executor_settings(config, motion) -> TrajectorySettings:
    """The shared executor's settings from the UR config blocks."""
    settings = TrajectorySettings(
        enabled=config.enabled,
        backend="servoj",
        servo_rate_hz=float(config.rate_hz),
        max_servo_rate_hz=float(config.max_rate_hz),
        max_joint_acc_deg_s2=float(motion.joint_accel_deg_s2),
        start_tolerance_deg=float(config.start_tolerance_deg),
        max_points=int(config.max_points),
        max_points_per_chunk=int(config.max_points_per_chunk),
        max_duration_s=float(config.max_duration_s),
        log_dir=config.log_dir,
        realtime_report=False,  # xArm-only port 30003; UR feedback is the RTDE receive stream
    )
    settings.check()
    return settings


def _http(exc: TrajectoryError) -> HTTPException:
    return HTTPException(status_code=exc.status_code, detail=exc.detail)


class URTrajectory:
    num_joints = 6

    def __init__(self, ur):
        self.ur = ur  # URControl
        self.config = ur.config.trajectory
        self.motion = ur.arm_limits
        self.settings = executor_settings(self.config, self.motion)
        self.manager = TrajectoryManager(self.settings)

    # ── helpers ─────────────────────────────────────────────────────
    @property
    def joint_limits(self):
        return list(zip(self.motion.joint_lower_deg, self.motion.joint_upper_deg))

    def summary(self):
        return self.manager.summary()

    def running(self):
        return self.manager.running

    def advertised_actions(self, move_block):
        """The lab-skills catalog names for /status allowed_actions.

        STATUS_SPEC 6.2: listed means a call is not state-refused right now,
        so clients that gate on the list (the dashboard's skill catalog,
        agents) see the same gates the routes apply. move_block is
        arm_block("arm.move_joints"), the check that start shares with a
        moveJ. Per-request refusals (an unknown session id, a chunk out of
        order, a path outside the box) are not predicted here.
        """
        if not self.settings.enabled:
            return []
        actions = ["trajectory.limits", "trajectory.status"]
        summary = self.manager.summary() or {}
        session = summary.get("session")
        if session is not None and session.get("state") not in TERMINAL:
            # A session is open (created, uploading or streaming): only its
            # cancel is offered; create answers 409 session_open meanwhile.
            actions.append("trajectory.cancel")
            if session.get("state") == "created":
                actions.append("trajectory.upload_chunk")
                if move_block is None:
                    actions += ["trajectory.validate", "trajectory.start"]
            return actions
        actions += ["trajectory.create", "trajectory.upload_chunk"]
        if move_block is None:
            actions += ["trajectory.validate", "trajectory.start"]
        return actions

    def hard_stop(self, reason):
        self.manager.notify_hard_stop(reason)

    def _enabled(self):
        if not self.settings.enabled:
            raise HTTPException(status_code=412, detail={
                "error": "trajectory_disabled",
                "message": "Joint-trajectory execution is disabled on this robot "
                           "(control.trajectory.enabled is false in the local config).",
            })

    def _holder(self):
        if self.ur.claims.enforced is not True:
            raise HTTPException(status_code=412, detail={
                "error": "claim_enforcement_off",
                "message": "Joint trajectories need hard claim enforcement",
            })
        holder = self.ur.claims.claimed_by()
        if not isinstance(holder, dict) or not holder.get("session_id"):
            raise HTTPException(status_code=423, detail={"error": "claim_required", "claimed_by": None})
        return holder

    def _session(self, session_id):
        try:
            return self.manager.get(session_id)
        except TrajectoryError as exc:
            raise _http(exc)

    def _arm(self):
        """(session, executor, latest feedback sample), or 409 without a session."""
        session, executor = self.ur.session, self.ur.executor
        if session is None or not session.is_open or executor is None:
            raise HTTPException(status_code=409, detail={
                "error": "no_control_session",
                "message": "No arm session; Connect first",
                "hint": "POST /connect first",
            })
        return session, executor, session.feedback.latest()

    def _rate(self, rate_hz):
        rate = float(rate_hz) if rate_hz is not None else self.settings.default_rate()
        lo, hi = self.settings.rate_range()
        if not lo <= rate <= hi:
            raise HTTPException(status_code=422, detail={
                "error": "rate_out_of_range", "message": f"rate_hz {rate:g} is outside {lo:g}-{hi:g} Hz"})
        return rate

    def _limits(self, rate):
        return JointLimits(
            joint_limits_deg=self.joint_limits,
            max_joint_speed_deg_s=float(self.motion.max_joint_speed_deg_s),
            max_joint_acc_deg_s2=self.settings.max_joint_acc_deg_s2,
            servo_rate_hz=rate,
            max_duration_s=self.settings.max_duration_s,
            max_points=self.settings.max_points,
        )

    def _path_errors(self, control, points, rate):
        unverified = getattr(control, "kinematics_error", None)
        if unverified:
            raise HTTPException(status_code=412, detail={
                "error": "kinematics_unverified",
                "message": f"The workspace check needs forward kinematics, which is unavailable: {unverified}",
                "hint": "Disconnect and Connect again; check the TCP offset on the pendant"})
        trajectory = JointTrajectory(points)
        return path_errors(trajectory, control, self.motion, step_deg=self.config.path_check_step_deg,
                           max_checks=self.config.max_fk_checks, rate_hz=rate)

    def limits_body(self):
        lo, hi = self.settings.rate_range()
        motion = self.motion
        return {
            "enabled": self.settings.enabled,
            "backend": "ur_servoj",
            "units": {"t": "s", "joints": "deg", "velocities": "deg/s", "accelerations": "deg/s^2"},
            "num_joints": self.num_joints,
            "joint_limits_deg": [list(r) for r in self.joint_limits],
            "max_joint_speed_deg_s": float(motion.max_joint_speed_deg_s),
            "max_joint_acc_deg_s2": self.settings.max_joint_acc_deg_s2,
            "workspace_mm": {"x": list(motion.workspace.x_mm), "y": list(motion.workspace.y_mm),
                             "z": list(motion.workspace.z_mm)},
            "force_guard_n": motion.force_guard_n,
            "commissioning_id": motion.commissioning_id,
            "max_duration_s": self.settings.max_duration_s,
            "max_points": self.settings.max_points,
            "max_points_per_chunk": self.settings.max_points_per_chunk,
            "rate_hz": {"default": self.settings.default_rate(), "min": lo, "max": hi},
            "min_point_spacing_s": round(1.0 / self.settings.default_rate(), 6),
            "start_tolerance_deg": self.settings.start_tolerance_deg,
            "rules": [
                "t starts at 0 and strictly increases; points at least one command period apart",
                "first and last points at rest (zero velocity and acceleration)",
                "first point within start_tolerance_deg of the arm's measured pose",
                "velocities and accelerations optional, on every point or none (both give a quintic)",
                "the TCP stays inside workspace_mm along the whole path (checked with the "
                "controller's forward kinematics before start, and on every packet while running)",
                "the pendant speed slider must be at 100 %",
                "no collision checking: the planner owns it",
            ],
        }

    # ── start ───────────────────────────────────────────────────────
    def _start(self, session_id, identity):
        mgr = self.manager
        # Captured before anything else: a STOP from here on cancels the start.
        generation = mgr.stop_generation()
        self._enabled()
        holder = self._holder()
        key = holder["session_id"]
        traj = self._session(session_id)
        if traj.owner_key != key:
            raise HTTPException(status_code=423, detail={
                "error": "session_owner_mismatch",
                "message": "Only the claim that created this session may start it"})
        blocked = self.ur.arm_block("arm.move_joints")
        if blocked is not None:
            self.ur._refuse(identity, "trajectory.start", blocked)
        session, executor, sample = self._arm()
        if not traj.final_received:
            raise HTTPException(status_code=409, detail={
                "error": "trajectory_incomplete", "message": "Upload the final chunk before starting"})
        points = [JointPoint.from_mapping(p) for p in traj.raw_points]
        try:
            errors = self._path_errors(session.control, points, traj.rate_hz)
        except HTTPException:
            raise
        except Exception as exc:  # a kinematics query failed; nothing was sent
            raise HTTPException(status_code=502, detail={
                "error": "planning_failed", "message": f"Controller kinematics query failed: {exc}"})
        if errors:
            raise HTTPException(status_code=422, detail={
                "error": "trajectory_invalid", "message": errors[0]["message"],
                "report": {"valid": False, "errors": errors}})
        if not executor.reserve({"action": "trajectory", "kind": "trajectory", "session_id": traj.id}):
            raise HTTPException(status_code=409, detail={
                "error": "motion_busy", "reason": "Another arm move is running; commands are not queued"})
        released = threading.Event()

        def release(_session=None):
            if not released.is_set():
                released.set()
                executor.release()
                self.ur.event("trajectory_end", identity, session_id=traj.id, state=traj.state,
                              reason=traj.reason)

        def watch():
            current = self.ur.claims.claimed_by()
            if not isinstance(current, dict) or current.get("session_id") != key:
                return "claim_lost"
            if current.get("owner") not in self.ur.config.authorized_operators:
                return "not_authorized"
            if self.ur.session is not session or not session.is_open or not session.watchdog_ok:
                return "control_session_lost"
            if self.ur.executor is not executor or executor.latched is not None:
                return "latched"
            return None

        try:
            backend = URServoJBackend(
                session.control, session.feedback, period_s=1.0 / traj.rate_hz,
                num_joints=self.num_joints, lookahead_s=self.config.lookahead_s, gain=self.config.gain,
                stop_decel_deg_s2=self.motion.stop_joint_decel_deg_s2,
                emergency_decel_deg_s2=self.motion.stop_joint_decel_deg_s2,
                max_feedback_age_s=self.motion.feedback_max_age_s, guard=self.motion,
            )
            start_joints = list(sample["joints_deg"]) if sample else None
            status = mgr.start(
                session_id, key, limits=self._limits(traj.rate_hz), start_joints=start_joints,
                backend=backend, watch=watch, on_finish=release, stop_generation=generation,
                monitor=FeedbackMonitor(session.feedback, self.num_joints),
            )
        except TrajectoryError as exc:
            release()
            raise _http(exc)
        except Exception as exc:
            release()
            log.exception("UR trajectory start failed")
            raise HTTPException(status_code=500, detail=f"Trajectory start failed: {exc}")
        self.ur.event("trajectory_start", identity, session_id=traj.id, rate_hz=traj.rate_hz,
                      duration_s=status["executed"]["duration_s"])
        return status

    # ── routes ──────────────────────────────────────────────────────
    def install(self, app, login, claim):
        tags = ["trajectory"]

        @app.get(f"{PREFIX}/limits", tags=tags)
        def trajectory_limits(identity: dict = login):
            """The limits a joint trajectory must meet on this arm. Read first.
            Moves nothing and makes no controller calls."""
            return self.limits_body()

        @app.post(f"{PREFIX}/validate", tags=tags)
        async def validate(body: ValidateRequest, identity: dict = login):
            """Check a whole trajectory against the limits, the measured start
            and (forward kinematics) the workspace box. Moves nothing; needs an
            open arm session but no claim."""
            if self.running():
                raise HTTPException(status_code=409, detail={
                    "error": "trajectory_running", "message": "A joint trajectory is streaming"})
            session, _executor, sample = self._arm()
            rate = self._rate(body.rate_hz)
            tolerance = (float(body.start_tolerance_deg) if body.start_tolerance_deg is not None
                         else self.settings.start_tolerance_deg)
            if not 0 < tolerance <= self.settings.start_tolerance_deg:
                raise HTTPException(status_code=422, detail={
                    "error": "start_tolerance_out_of_range",
                    "message": f"start_tolerance_deg must be within (0, {self.settings.start_tolerance_deg}]"})
            points = [JointPoint.from_mapping(p.model_dump(exclude_none=True)) for p in body.points]
            start = list(sample["joints_deg"]) if sample else None
            report = await asyncio.to_thread(
                validate_joint_trajectory, points, self._limits(rate), start, tolerance)
            if report.get("valid"):
                try:
                    errors = await asyncio.to_thread(self._path_errors, session.control, points, rate)
                except HTTPException:
                    raise
                except Exception as exc:
                    raise HTTPException(status_code=502, detail={
                        "error": "planning_failed", "message": f"Controller kinematics query failed: {exc}"})
                if errors:
                    report = {**report, "valid": False, "errors": errors}
            if not report.get("valid"):
                raise HTTPException(status_code=422, detail={
                    "error": "trajectory_invalid",
                    "message": f"{len(report.get('errors', []))} violation(s); see report.errors",
                    "report": report})
            return report

        @app.post(PREFIX, status_code=201, tags=tags, dependencies=[claim])
        def create(body: Optional[SessionRequest] = None, identity: dict = login):
            """Open a trajectory session bound to the caller's claim (one per arm)."""
            self._enabled()
            holder = self._holder()
            body = body or SessionRequest()
            try:
                traj = self.manager.create(
                    key=holder["session_id"], owner=holder.get("owner"), num_joints=self.num_joints,
                    rate_hz=body.rate_hz, start_tolerance_deg=body.start_tolerance_deg)
            except TrajectoryError as exc:
                raise _http(exc)
            self.ur.event("trajectory_create", identity, session_id=traj.id, rate_hz=traj.rate_hz)
            return traj.status()

        async def chunk(session_id: str, seq: int, body: ChunkRequest, identity: dict = login):
            """Upload chunk seq (0, 1, 2...); the last carries final: true.
            Identical re-sends are harmless duplicates. PUT and POST are the
            same handler (the dashboard proxy forwards GET/POST/DELETE only)."""
            self._enabled()
            if seq < 0:
                raise HTTPException(status_code=422, detail={"error": "chunk_out_of_order", "message": "seq starts at 0"})
            holder = self._holder()
            traj = self._session(session_id)
            limits = self.manager.validation_limits(traj, self.joint_limits, self.motion.max_joint_speed_deg_s)
            points = [p.model_dump(exclude_none=True) for p in body.points]
            try:
                return await asyncio.to_thread(
                    self.manager.add_chunk, session_id, seq, holder["session_id"], points, body.final, limits)
            except TrajectoryError as exc:
                raise _http(exc)

        app.put(f"{PREFIX}/{{session_id}}/chunks/{{seq}}", tags=tags, dependencies=[claim])(chunk)
        app.post(f"{PREFIX}/{{session_id}}/chunks/{{seq}}", tags=tags, dependencies=[claim])(chunk)

        @app.post(f"{PREFIX}/{{session_id}}/start", tags=tags, dependencies=[claim])
        async def start(session_id: str, identity: dict = login):
            """Start the complete trajectory. Refused unless the arm takes moves
            right now (the same checks as a moveJ, 409/412), the path stays in
            the workspace box (422) and the arm is at the first point (422)."""
            return await asyncio.to_thread(self._start, session_id, identity)

        @app.post(f"{PREFIX}/{{session_id}}/cancel", tags=tags, dependencies=[claim])
        def cancel(session_id: str, identity: dict = login):
            """Constrained stop along the path. For an immediate halt use
            POST /control/stop, which needs no claim."""
            holder = self._holder()
            try:
                status = self.manager.cancel(session_id, holder["session_id"])
            except TrajectoryError as exc:
                raise _http(exc)
            self.ur.event("trajectory_cancel", identity, session_id=session_id)
            return status

        @app.get(f"{PREFIX}/{{session_id}}", tags=tags)
        def status(session_id: str, identity: dict = login):
            """State, progress (executed.t_exec) and the latest measured joints,
            for one-way sync. Poll at 10-20 Hz at most."""
            traj = self._session(session_id)
            body = traj.status()
            body["log_available"] = traj.record is not None
            return body

        @app.get(f"{PREFIX}/{{session_id}}/log", tags=tags)
        async def session_log(session_id: str, identity: dict = login):
            """The finished session's record. Refused while a trajectory runs."""
            if self.running():
                raise HTTPException(status_code=409, detail={
                    "error": "trajectory_running", "message": "Fetch logs after the run"})
            traj = self._session(session_id)
            if traj.record is None:
                raise HTTPException(status_code=409, detail={
                    "error": "log_not_ready", "message": f"Session is {traj.state}; the log is written when it ends"})
            body = await asyncio.to_thread(json.dumps, traj.record)
            return Response(content=body, media_type="application/json")
