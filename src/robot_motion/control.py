"""Config-gated claim surface and the arm and gripper control routes.

Installed only when Settings.control_enabled is true. Identity is the
dashboard edge's (X-Auth-User plus X-Edge-Auth matching
ROBOT_MOTION_EDGE_SHARED_SECRET); without a configured secret every control
route refuses, so a directly reachable service never trusts client headers.
Hard claims (core.claims) gate every arm move and the gripper commands
(gripper.py). /connect and /disconnect follow the xArm order (Connect, then
Take Control): any listed operator may call them while nobody holds control,
but never against someone else's claim. /control/stop needs identity only.

The arm runs in one of two modes, chosen by the control block: joint_step
(single tiny steps, the first commissioning primitive) or motion (joint and
Cartesian moves and jogs inside commissioned limits, plus manual (teach) mode
for guiding the arm by hand, drivers/ur_motion.py).
The gripper routes exist only with a gripper block. arm_block() is the one
precondition helper behind both the arm routes and allowed_actions
(STATUS_SPEC section 6.2). Nothing here is a safety-rated stop.
"""

from __future__ import annotations

import asyncio
import json
import logging
import threading
import time
from datetime import datetime, timezone

from fastapi import Depends, Header, HTTPException, Request, Response
from fastapi.responses import JSONResponse
from sdl_lab_contract import ClaimRejection, ClaimRequest, ClaimResponse
from core.claims import ClaimConflict, ClaimManager, InvalidClaimToken

from .drivers.joint_step import (
    JointStep,
    JointStepExecutor,
    JointStepFailed,
    JointStepRefused,
)
from .drivers.lle_rtde import tcp_to_mm_deg
from .drivers.ur_control import ControlSession
from .drivers.ur_motion import (
    JOINT_ACTIONS,
    MOVE_ACTIONS,
    JointJog,
    JointMove,
    LinearJog,
    LinearMove,
    ManualMode,
    MotionExecutor,
    MotionFailed,
    MotionRefused,
)
from .edge import SECRET_ENV, configured_secret, edge_identity

log = logging.getLogger(__name__)
STOP_NOTICE = (
    "Software stop request only; not a safety-rated stop and not confirmed by "
    "measurement. Use the teach pendant or hardware stop for the real thing."
)


IDLE_STATE = {"robotmode": "RUNNING", "safetystatus": "NORMAL", "program_state": "STOPPED"}


class URControl:
    def __init__(self, settings, *, session_factory=None, edge_secret=None, observe=None, gripper=None):
        self.settings = settings
        self.config = settings.control
        # joint_step or motion limits, whichever the config names (never both).
        self.arm_limits = self.config.arm
        self.arm_mode = (
            "joint_step" if self.config.joint_step is not None
            else "motion" if self.config.motion is not None
            else None
        )
        self.gripper_control = None
        if gripper is not None:
            from .gripper import GripperControl

            self.gripper_control = GripperControl(self, gripper)
        # Latest cached Dashboard observation: (observation, stale). Used to
        # refuse /connect while anything else runs or owns the robot.
        self._observe = observe or (lambda: (None, True))
        self.claims = ClaimManager(enforce=True)
        self.secret = configured_secret(edge_secret)
        self._session_factory = session_factory or (lambda: ControlSession(settings))
        self._lock = threading.Lock()
        self.session = None
        self.executor = None
        self.tcp_offset = None
        self.last_event = None

    # ── identity and gates ──────────────────────────────────────────
    def identity(self, request):
        return edge_identity(request, self.secret)

    def require_login(self, request: Request):
        if not self.secret:
            raise HTTPException(
                status_code=503,
                detail={
                    "error": "edge_identity_not_configured",
                    "hint": f"set {SECRET_ENV} for the service; control refuses without it",
                },
            )
        identity = self.identity(request)
        if identity is None:
            raise HTTPException(
                status_code=401,
                detail={
                    "error": "login_required",
                    "hint": "control requests must arrive through the authenticated dashboard edge",
                },
            )
        if identity["email"] not in self.config.authorized_operators:
            raise HTTPException(
                status_code=403,
                detail={
                    "error": "operator_not_authorized",
                    "hint": "only operators listed in the local control config may command this robot",
                },
            )
        request.state.identity = identity
        return identity

    def require_claim(self, x_claim_token: str | None = Header(default=None)):
        try:
            self.claims.verify_token(x_claim_token)
        except InvalidClaimToken:
            holder = self.claims.claimed_by()
            raise HTTPException(
                status_code=423,
                detail={
                    "error": "claim_required",
                    "claimed_by": (
                        {"session_id": holder["session_id"], "owner": holder["owner"]}
                        if holder
                        else None
                    ),
                    "hint": "POST /control/claim first and present its X-Claim-Token",
                },
            )

    def require_claim_if_held(self, x_claim_token: str | None = Header(default=None)):
        # Session routes: open to any listed operator while control is free,
        # the holder's token only once someone holds it. Opening a session
        # moves nothing; every move still needs the claim.
        if self.claims.claimed_by() is not None:
            self.require_claim(x_claim_token)

    def authorize(self, request, target, commissioning_id):
        # Rechecked by the executors before dispatch, at the exact target, and
        # on every feedback sample: the holder must still be a listed operator
        # and the session the move started under must still be the open one.
        holder = self.claims.claimed_by()
        return (
            self.arm_limits is not None
            and commissioning_id == self.arm_limits.commissioning_id
            and holder is not None
            and holder["owner"] in self.config.authorized_operators
            and self.session is not None
            and self.session.is_open
        )

    def connect_preconditions(self):
        """Why /connect must refuse right now, or None.

        Uploading the control script takes the controller's program slot, so
        the robot must be observed idle first: controller RUNNING, safety
        NORMAL and no program PLAYING or PAUSED (an LLE demo, a pendant
        program or another ur_rtde client would show as PLAYING). An e-Series
        must also be in Remote Control, or the controller refuses the script.
        A stale or failed observation refuses too; "unknown" is not "idle".
        """
        current, stale = self._observe()
        details = (current or {}).get("details") or {}
        if stale or not details.get("robotmode"):
            return {
                "error": "observation_unavailable",
                "reason": "No fresh Dashboard observation; cannot verify the robot is idle",
                "hint": "Wait for the next observation or check the controller connection",
            }
        observed = {key: details.get(key) for key in IDLE_STATE}
        if observed != IDLE_STATE:
            return {
                "error": "robot_not_idle",
                "reason": "The robot is not idle (RUNNING, safety NORMAL, no program)",
                "observed": observed,
                "required": IDLE_STATE,
                "hint": "Stop the running program / other client on the pendant before connecting",
            }
        if details.get("remote_control") is False:
            return {
                "error": "remote_control_off",
                "reason": "The controller is in Local Control; external control is refused",
                "operational_mode": details.get("operational_mode"),
                "hint": "Switch the teach pendant to Remote Control (top-right icon) before connecting",
            }
        return None

    @property
    def session_open(self):
        return self.session is not None and self.session.is_open

    def observed_robot(self):
        """(observed idle-state dict, problem body or None) from the cached
        Dashboard observation. Our own open control session counts as idle:
        its script is the program the controller reports."""
        current, stale = self._observe()
        details = (current or {}).get("details") or {}
        if stale or not details.get("robotmode"):
            return None, {
                "detail": "No fresh robot observation; cannot verify the robot is idle",
                "error": "observation_unavailable",
            }
        observed = {key: details.get(key) for key in IDLE_STATE}
        required = dict(IDLE_STATE)
        if self.session_open:
            observed.pop("program_state")
            required.pop("program_state")
        if observed != required:
            return observed, {
                "detail": "Robot is not idle under this service",
                "error": "robot_not_idle",
                "observed": observed,
                "required": required,
            }
        return observed, None

    def arm_actions(self):
        """Arm actions in allowed_actions order for the configured mode."""
        if self.arm_mode == "joint_step":
            return ("connect", "disconnect", "control.joint_step")
        if self.arm_mode == "motion":
            return (
                "connect", "disconnect", *MOVE_ACTIONS, "arm.manual_mode", "control.reset", "arm.zero_force_sensor"
            )
        return ()

    def arm_block(self, action):
        """(HTTP status, body) for why `action` cannot run now, or None.

        The single precondition helper behind the arm routes and
        allowed_actions. 409 is a state conflict (no session, a move running),
        412 a precondition (STATUS_SPEC section 6.1). Request-specific limits
        (a target outside the envelope, a speed over the cap) are 422s decided
        per request and are not part of this.
        """
        if action == "connect":
            if self.session_open:
                return 409, {
                    "error": "control_session_open",
                    "reason": "An arm session is already open",
                    "hint": "POST /disconnect first",
                }
            problem = self.connect_preconditions()
            return (412, problem) if problem is not None else None
        # Read the session's parts once: a concurrent disconnect clears them.
        session, executor = self.session, self.executor
        control = getattr(session, "control", None)
        feedback = getattr(session, "feedback", None)
        if executor is None or control is None or feedback is None:
            return 409, {
                "error": "no_control_session",
                "reason": "No arm session; Connect first",
                "hint": "POST /connect first",
            }
        busy = getattr(executor, "busy", False)
        if action == "disconnect":
            if busy:
                return 409, {
                    "error": "motion_in_progress",
                    "reason": "An arm move is running; STOP it first",
                }
            return None
        if action == "control.joint_step":
            if executor.latched is not None or not session.watchdog_ok:
                return 412, {
                    "error": "joint_step_refused",
                    "reason": executor.latched or f"Control link fault: {session.watchdog_error}",
                }
            return None
        if busy:
            return 409, {
                "error": "motion_busy",
                "reason": "Another arm move is running; commands are not queued",
            }
        manual = getattr(executor, "manual", False)
        if action == "arm.manual_mode":
            if manual:
                return None  # turning it off is always offered
            refusal = executor.manual_block(feedback.latest())
            return (refusal.status, refusal.body()) if refusal is not None else None
        if manual and action in ("control.reset", "arm.zero_force_sensor"):
            return 412, {
                "error": "manual_mode",
                "reason": "Manual mode is on; turn it off first",
            }
        if action == "control.reset":
            return self._reset_block(session, control, feedback)
        if action == "arm.zero_force_sensor":
            if not session.watchdog_ok:
                return 412, {"error": "control_link_down", "reason": f"Control link fault: {session.watchdog_error}"}
            sample = feedback.latest()
            if sample is None or not self._still(sample):
                return 412, {"error": "robot_moving", "reason": "Zero the sensor only with the arm still"}
            return None
        refusal = executor.state_block(feedback.latest())
        return (refusal.status, refusal.body()) if refusal is not None else None

    def _still(self, sample):
        limit = self.arm_limits.stationary_speed_deg_s
        return all(abs(v) <= limit for v in sample["velocities_deg_s"])

    def _reset_block(self, session, control, feedback):
        if not session.watchdog_ok:
            return 412, {
                "error": "control_link_down",
                "reason": f"Control link fault: {session.watchdog_error}",
                "hint": "Disconnect and Connect to start a new session",
            }
        if control.isProgramRunning() is not True:
            return 412, {
                "error": "control_script_stopped",
                "reason": "The control script is not running (for example after a protective stop)",
                "hint": "Clear the stop on the pendant, then Disconnect and Connect",
            }
        sample = feedback.latest()
        if (
            sample is None
            or time.monotonic() - sample["received_monotonic_s"] > 1.0
            or sample["robot_mode"] != "RUNNING"
            or sample["safety_mode"] != "NORMAL"
        ):
            return 412, {
                "error": "robot_not_ready",
                "reason": "Controller must be RUNNING with safety NORMAL and fresh feedback",
            }
        if not self._still(sample):
            return 412, {"error": "robot_moving", "reason": "The arm is moving; wait until it is still"}
        return None

    def _refuse(self, identity, action, blocked):
        status, body = blocked
        kind = "connect_refused" if action == "connect" else "arm_refused"
        self.event(kind, identity, action=action, reason=body.get("error"))
        raise HTTPException(status_code=status, detail=body)

    def event(self, kind, identity, **extra):
        self.last_event = {
            "kind": kind,
            "operator": identity["email"] if identity else None,
            "at": datetime.now(timezone.utc).isoformat(),
            **extra,
        }
        log.info("UR control %s by %s %s", kind, self.last_event["operator"], extra)
        if self.config.audit_file:
            # Durable trail; a logging failure must never block or hide a stop.
            try:
                with open(self.config.audit_file, "a", encoding="utf-8") as handle:
                    handle.write(json.dumps(self.last_event, default=str) + "\n")
            except OSError:
                log.exception("UR control audit append failed")

    # ── state for /status ───────────────────────────────────────────
    def state(self):
        session = self.session
        session_open = session is not None and session.is_open
        executor = self.executor
        latched = executor.latched if executor is not None else None
        limits = self.arm_limits
        allowed = ["control.stop"]
        move_block = None
        if self.arm_mode == "motion":
            # The four moves share one state check; ask once.
            move_block = self.arm_block(MOVE_ACTIONS[0])
        for action in self.arm_actions():
            if action in MOVE_ACTIONS:
                blocked = move_block
            elif action == "control.reset" and latched is None:
                continue  # nothing to clear
            else:
                blocked = self.arm_block(action)
            if blocked is None:
                allowed.append(action)
        if self.gripper_control is not None:
            allowed += self.gripper_control.allowed_actions()
        return {
            "claimed_by": self.claims.claimed_by(),
            "control_session": {
                "arm_control": limits is not None,
                "mode": self.arm_mode,
                "open": session_open,
                "busy": bool(getattr(executor, "busy", False)),
                "active_move": getattr(executor, "active", None),
                "manual_mode": bool(getattr(executor, "manual", False)),
                "latched": latched,
                "watchdog_ok": session.watchdog_ok if session_open else None,
                "watchdog_error": session.watchdog_error if session_open else None,
                "commissioning_id": limits.commissioning_id if limits else None,
                "max_step_deg": self.config.joint_step.max_step_deg if self.config.joint_step else None,
                "limits": self.config.motion.summary() if self.config.motion else None,
                "tcp_offset_mm_rpy_deg": self.tcp_offset if session_open else None,
                "last_event": self.last_event,
            },
            "allowed_actions": allowed,
        }

    def shutdown(self):
        with self._lock:
            session, executor = self.session, self.executor
            self.session = self.executor = None
        if executor is not None:
            executor.request_stop()
        if session is not None:
            # Service stopping: halt any move before the script goes, rather
            # than leaving it to the controller watchdog.
            try:
                session.stop(self._stop_joint_decel())
            except Exception:
                log.exception("Stop at shutdown failed")
            session.close()
            log.info("Arm session stopped and closed at shutdown")

    def _new_executor(self, session, seen=None):
        common = dict(
            control=session.control,
            read_feedback=session.feedback.read,
            claims=self.claims,
            authorize=self.authorize,
            limits=self.arm_limits,
        )
        if self.arm_mode == "motion":
            return MotionExecutor(**common, seen=seen, latest_feedback=session.feedback.latest)
        return JointStepExecutor(**common)

    # ── routes ──────────────────────────────────────────────────────
    def install(self, app):
        login = Depends(self.require_login)
        claim = Depends(self.require_claim)
        claim_if_held = Depends(self.require_claim_if_held)

        @app.post("/control/claim", responses={409: {"model": ClaimRejection}}, tags=["control"])
        def acquire_claim(request: ClaimRequest, identity: dict = login):
            # The verified identity is the owner; the client's owner is ignored.
            try:
                record = self.claims.acquire(
                    owner=identity["email"], session_id=request.session_id, ttl_s=request.ttl_s
                )
            except ClaimConflict as exc:
                rejection = ClaimRejection(
                    detail=str(exc),
                    claimed_by=exc.holder.to_claimed_by_dict(),
                    retry_after_s=exc.retry_after_s,
                )
                return JSONResponse(
                    status_code=409,
                    content=rejection.model_dump(mode="json"),
                    headers={"Retry-After": str(int(exc.retry_after_s) + 1)},
                )
            self.event("claim", identity, session_id=request.session_id)
            return ClaimResponse(
                claim_token=record.token,
                heartbeat_interval_s=self.claims.heartbeat_interval_s,
                expires_at=datetime.fromtimestamp(record.expires_at, tz=timezone.utc),
            )

        @app.post("/control/heartbeat", status_code=204, tags=["control"])
        def heartbeat(x_claim_token: str = Header(...)):
            try:
                self.claims.heartbeat(token=x_claim_token)
            except InvalidClaimToken:
                raise HTTPException(status_code=401, detail="invalid or expired claim token")
            return Response(status_code=204)

        @app.post("/control/release", status_code=204, tags=["control"])
        def release(x_claim_token: str = Header(...)):
            # Token-only route: the audit operator is the holder being released.
            holder = self.claims.claimed_by()
            if self.claims.release(token=x_claim_token) and holder is not None:
                self.event("release", {"email": holder["owner"]}, session_id=holder["session_id"])
            return Response(status_code=204)

        if self.arm_limits is not None:
            self._install_session(app, login, claim_if_held)
        if self.arm_mode == "joint_step":
            self._install_joint_step(app, login, claim, self.arm_limits)
        if self.arm_mode == "motion":
            self._install_motion(app, login, claim)
        if self.gripper_control is not None:
            self.gripper_control.install(app, login, claim)

        @app.post("/control/stop", tags=["control"])
        @app.post("/move/stop", tags=["control"])
        def stop(identity: dict = login):
            """Identity-gated safety floor: no claim needed to ask for a stop."""
            executor, session = self.executor, self.session
            requested, error = False, None
            if executor is not None:
                # Returns only after any dispatch in progress, so the stop
                # below can never land before the move it should stop.
                executor.request_stop()
                requested = True
            linear = getattr(executor, "active_kind", None) == "linear"
            if session is not None and session.is_open:
                try:
                    if linear:
                        stopped = session.stop_linear(self.arm_limits.stop_linear_decel_mm_s2)
                    else:
                        stopped = session.stop(self._stop_joint_decel())
                    requested = stopped or requested
                except Exception as exc:
                    error = str(exc)
            gripper_stopped = False
            if self.gripper_control is not None:
                gripper_stopped = self.gripper_control.gripper.request_stop()
                requested = requested or gripper_stopped
            self.event("stop", identity, requested=requested, gripper=gripper_stopped, error=error)
            return {
                "stop_requested": requested,
                "stop_confirmed": False,
                "stop_error": error,
                "gripper_stop_requested": gripper_stopped,
                "control_session": session is not None and session.is_open,
                "message": "Stop requested" if requested else "Nothing to stop",
                "notice": STOP_NOTICE,
            }

        return self

    def _stop_joint_decel(self):
        if self.arm_mode == "motion":
            return self.arm_limits.stop_joint_decel_deg_s2
        return self.arm_limits.stop_deceleration_deg_s2

    def _install_session(self, app, login, claim_if_held):
        @app.post("/connect", tags=["control"], dependencies=[claim_if_held])
        def connect(identity: dict = login):
            """Open the owned RTDE control session (uploads the control script)."""
            with self._lock:
                blocked = self.arm_block("connect")
                if blocked is not None:
                    self._refuse(identity, "connect", blocked)
                session = self._session_factory()
                try:
                    session.open()
                except Exception as exc:
                    log.exception("UR control session failed to open")
                    raise HTTPException(
                        status_code=502,
                        detail={"error": "control_connect_failed", "message": str(exc)},
                    )
                self.session = session
                # A fresh executor per session: latches clear only by this
                # explicit, identified reconnect (or, for motion, the claimed
                # /control/reset).
                self.executor = self._new_executor(session)
                self.tcp_offset = None
                if self.arm_mode == "motion":
                    try:
                        self.tcp_offset = [round(v, 3) for v in tcp_to_mm_deg(session.control.getTCPOffset())]
                    except Exception:
                        log.exception("Could not read the active TCP offset")
            self.event("connect", identity, tcp_offset_mm_rpy_deg=self.tcp_offset)
            return {
                "connected": True,
                "message": "Arm session open",
                "commissioning_id": self.arm_limits.commissioning_id,
                "tcp_offset_mm_rpy_deg": self.tcp_offset,
                "notice": "RTDE control script uploaded; the controller program slot is owned by this service",
            }

        @app.post("/disconnect", tags=["control"], dependencies=[claim_if_held])
        def disconnect(identity: dict = login):
            with self._lock:
                # Closing mid-move would end the control script under the
                # arm; STOP first. Without a session this is a harmless no-op.
                executor = self.executor
                retire = getattr(executor, "retire", None)
                if retire is not None and not retire():  # holds the move lock if free
                    self._refuse(identity, "disconnect", (409, {
                        "error": "motion_in_progress",
                        "reason": "An arm move is running; STOP it first",
                    }))
                session, self.session, self.executor = self.session, None, None
                self.tcp_offset = None
            # Teach mode off before the script goes (closing ends it too).
            end_manual = getattr(executor, "end_manual", None)
            if end_manual is not None:
                end_manual("Disconnected")
            error = None
            if session is not None:
                try:
                    session.close()
                except Exception as exc:
                    error = str(exc)
            self.event("disconnect", identity, error=error)
            return {"connected": False, "close_error": error, "message": "Arm session closed"}

    def _install_joint_step(self, app, login, claim, limits):
        @app.post("/control/joint_step", tags=["control"], dependencies=[claim])
        async def joint_step(
            request: JointStep,
            identity: dict = login,
            x_claim_token: str | None = Header(default=None),
        ):
            executor = self.executor
            if executor is None:
                raise HTTPException(
                    status_code=409,
                    detail={"error": "no_control_session", "hint": "POST /connect first"},
                )
            self.event("joint_step", identity, request_id=str(request.request_id), joint=request.joint, delta_deg=request.delta_deg)
            try:
                result = await asyncio.to_thread(executor.execute, request, claim_token=x_claim_token)
            except JointStepRefused as exc:
                self.event("joint_step_refused", identity, request_id=str(request.request_id), reason=str(exc))
                raise HTTPException(
                    status_code=412, detail={"error": "joint_step_refused", "reason": str(exc)}
                )
            except JointStepFailed as exc:
                self.event("joint_step_failed", identity, request_id=str(request.request_id), reason=str(exc))
                return JSONResponse(
                    status_code=500,
                    content={
                        "error": "joint_step_failed",
                        "reason": str(exc),
                        "stop_attempted": exc.stop_attempted,
                        "stop_confirmed": exc.stop_confirmed,
                        "stop_error": exc.stop_error,
                        "latched": True,
                        "hint": "Verify the robot is stationary and safe on the pendant; "
                        "POST /disconnect then /connect to reconcile before any further step",
                    },
                )
            return {
                **result,
                "target_deg": list(result["target_deg"]),
                "measured_deg": list(result["measured_deg"]),
                "commissioning_id": limits.commissioning_id,
            }

    # ── arm motion (ur_motion.py) ───────────────────────────────────
    async def _run_motion(self, identity, action, body, token):
        blocked = await asyncio.to_thread(self.arm_block, action)
        if blocked is not None:
            self._refuse(identity, action, blocked)
        limits = self.arm_limits
        joint = action in JOINT_ACTIONS
        unit = "deg/s" if joint else "mm/s"
        default = limits.joint_speed_default if joint else limits.linear_speed_default
        cap = limits.max_joint_speed_deg_s if joint else limits.max_linear_speed_mm_s
        speed = body.speed if body.speed is not None else default
        if speed > cap:
            self.event("arm_refused", identity, action=action, reason="above_commissioned_limit")
            raise HTTPException(
                status_code=422,
                detail={
                    "error": "above_commissioned_limit",
                    "reason": f"Speed {speed:g} {unit} exceeds the commissioned {cap:g} {unit}",
                    "requested": speed,
                    "limit": cap,
                    "units": unit,
                },
            )
        executor = self.executor
        self.event(action, identity, **{**body.model_dump(mode="json", exclude_none=True), "speed": speed})
        try:
            result = await asyncio.to_thread(executor.execute, action, body, claim_token=token, speed=speed)
        except MotionRefused as exc:
            self.event("arm_refused", identity, action=action, reason=exc.error)
            raise HTTPException(status_code=exc.status, detail=exc.body())
        except MotionFailed as exc:
            self.event("arm_failed", identity, action=action, reason=str(exc), stop_error=exc.stop_error)
            return JSONResponse(
                status_code=500,
                content={
                    "error": "arm_move_failed",
                    "reason": str(exc),
                    "stop_attempted": exc.stop_attempted,
                    "stop_confirmed": exc.stop_confirmed,
                    "stop_error": exc.stop_error,
                    "latched": True,
                    "hint": "Check the arm on the pendant, then Clear errors (POST /control/reset) "
                    "before the next move",
                },
            )
        self.event(
            f"{action}_done",
            identity,
            moved=result["moved"],
            measured_joints_deg=[round(q, 3) for q in result["measured_joints_deg"]],
            measured_tcp_mm_rpy_deg=[round(v, 2) for v in result["measured_tcp_mm_rpy_deg"]],
            peak_force_change_n=result["peak_force_change_n"],
            elapsed_s=result["elapsed_s"],
        )
        message = "Move complete" if result["moved"] else "Already at the target"
        return {"ok": True, "action": action, "message": message, **result}

    def _install_motion(self, app, login, claim):
        tags = ["arm"]

        @app.post("/control/freehand/joints", tags=tags, dependencies=[claim])
        async def move_joints(
            body: JointMove, identity: dict = login, x_claim_token: str | None = Header(default=None)
        ):
            """moveJ to absolute angles for all six joints (degrees; speed deg/s)."""
            return await self._run_motion(identity, "arm.move_joints", body, x_claim_token)

        @app.post("/control/freehand/joint_jog", tags=tags, dependencies=[claim])
        async def jog_joint(
            body: JointJog, identity: dict = login, x_claim_token: str | None = Header(default=None)
        ):
            """moveJ one joint by a signed delta (degrees; speed deg/s)."""
            return await self._run_motion(identity, "arm.jog_joint", body, x_claim_token)

        @app.post("/control/freehand/relative", tags=tags, dependencies=[claim])
        async def jog_linear(
            body: LinearJog, identity: dict = login, x_claim_token: str | None = Header(default=None)
        ):
            """moveL by dx/dy/dz in the base frame, orientation kept (mm; speed mm/s)."""
            return await self._run_motion(identity, "arm.jog_linear", body, x_claim_token)

        @app.post("/control/freehand/linear", tags=tags, dependencies=[claim])
        async def move_linear(
            body: LinearMove, identity: dict = login, x_claim_token: str | None = Header(default=None)
        ):
            """moveL to an absolute TCP pose (mm and roll/pitch/yaw degrees; speed mm/s)."""
            return await self._run_motion(identity, "arm.move_linear", body, x_claim_token)

        @app.post("/control/reset", tags=tags, dependencies=[claim])
        @app.post("/clear/errors", tags=tags, dependencies=[claim])
        def reset(identity: dict = login):
            """Clear a stop/fault latch once the arm is still and the control
            script is running. The panel's Clear errors button calls this."""
            with self._lock:
                if self.executor is not None and self.executor.latched is None:
                    return {"ok": True, "reset": False, "message": "Nothing to clear"}
                blocked = self.arm_block("control.reset")
                if blocked is not None:
                    self._refuse(identity, "control.reset", blocked)
                cleared = self.executor.latched
                self.executor = self._new_executor(self.session, seen=self.executor.seen)
            self.event("reset", identity, cleared=cleared)
            return {"ok": True, "reset": True, "cleared": cleared, "message": "Arm moves re-enabled"}

        @app.post("/control/manual", tags=tags, dependencies=[claim])
        @app.post("/robot/manual", tags=tags, dependencies=[claim])
        async def manual_mode(
            body: ManualMode, identity: dict = login, x_claim_token: str | None = Header(default=None)
        ):
            """Manual (teach) mode. On: the arm can be guided by hand and every
            move is refused. Off: position control again. STOP and Disconnect
            also turn it off. The panel's Manual switch calls this."""
            action = "arm.manual_mode"
            executor = self.executor
            if not body.enable and executor is not None and not executor.manual:
                return {"ok": True, "manual_mode": False, "changed": False, "message": "Manual mode already off"}
            blocked = await asyncio.to_thread(self.arm_block, action)
            if blocked is not None:
                self._refuse(identity, action, blocked)
            executor = self.executor
            if executor is None:  # disconnected meanwhile
                self._refuse(identity, action, (409, {"error": "no_control_session", "hint": "POST /connect first"}))
            self.event(action, identity, enable=body.enable)
            try:
                changed = await asyncio.to_thread(executor.set_manual, body.enable, claim_token=x_claim_token)
            except MotionRefused as exc:
                self.event("arm_refused", identity, action=action, reason=exc.error)
                raise HTTPException(status_code=exc.status, detail=exc.body())
            except MotionFailed as exc:
                self.event("arm_failed", identity, action=action, reason=str(exc), stop_error=exc.stop_error)
                return JSONResponse(
                    status_code=500,
                    content={
                        "error": "manual_mode_failed",
                        "reason": str(exc),
                        "stop_error": exc.stop_error,
                        "manual_mode": executor.manual,
                        "latched": True,
                        "hint": "Check the arm. If it can still be pushed by hand, press STOP or "
                        "Disconnect (or use the pendant), then Clear errors",
                    },
                )
            on = executor.manual
            self.event(f"{action}_done", identity, manual_mode=on, changed=changed)
            if not changed:
                message = f"Manual mode already {'on' if on else 'off'}"
            else:
                message = "Manual mode on: guide the arm by hand" if on else "Manual mode off"
            return {"ok": True, "manual_mode": on, "changed": changed, "message": message}

        @app.post("/control/force/zero", tags=tags, dependencies=[claim])
        async def zero_force_sensor(identity: dict = login):
            """Zero the flange force/torque sensor (the arm must be still and
            hold nothing it should weigh). Moves nothing."""
            blocked = await asyncio.to_thread(self.arm_block, "arm.zero_force_sensor")
            if blocked is not None:
                self._refuse(identity, "arm.zero_force_sensor", blocked)
            try:
                await asyncio.to_thread(self.session.control.zeroFtSensor)
            except Exception as exc:
                raise HTTPException(status_code=502, detail={"error": "zero_failed", "reason": str(exc)})
            self.event("force_zero", identity)
            return {"ok": True, "message": "Force/torque sensor zeroed"}
