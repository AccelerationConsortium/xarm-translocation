"""Config-gated claim surface and bounded joint-step control routes.

Installed only when Settings.control_enabled is true. Identity is the
dashboard edge's (X-Auth-User plus X-Edge-Auth matching
ROBOT_MOTION_EDGE_SHARED_SECRET); without a configured secret every control
route refuses, so a directly reachable service never trusts client headers.
Hard claims (core.claims) gate /connect, /disconnect and /control/joint_step;
/control/stop needs identity only. Nothing here is a safety-rated stop.
"""

from __future__ import annotations

import asyncio
import json
import logging
import threading
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
from .drivers.ur_control import ControlSession
from .edge import SECRET_ENV, configured_secret, edge_identity

log = logging.getLogger(__name__)
STOP_NOTICE = (
    "Software stop request only; not a safety-rated stop and not confirmed by "
    "measurement. Use the teach pendant or hardware stop for the real thing."
)


IDLE_STATE = {"robotmode": "RUNNING", "safetystatus": "NORMAL", "program_state": "STOPPED"}


class URControl:
    def __init__(self, settings, *, session_factory=None, edge_secret=None, observe=None):
        self.settings = settings
        self.config = settings.control
        # Latest cached Dashboard observation: (observation, stale). Used to
        # refuse /connect while anything else runs or owns the robot.
        self._observe = observe or (lambda: (None, True))
        self.claims = ClaimManager(enforce=True)
        self.secret = configured_secret(edge_secret)
        self._session_factory = session_factory or (lambda: ControlSession(settings))
        self._lock = threading.Lock()
        self.session = None
        self.executor = None
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

    def authorize(self, request, target, commissioning_id):
        # Rechecked by the executor before dispatch, at the exact target, and
        # on every feedback sample: the holder must still be a listed operator
        # and the session the step started under must still be the open one.
        holder = self.claims.claimed_by()
        return (
            commissioning_id == self.config.joint_step.commissioning_id
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
        program or another ur_rtde client would show as PLAYING). A stale or
        failed observation refuses too; "unknown" is not "idle".
        """
        current, stale = self._observe()
        details = (current or {}).get("details") or {}
        if stale or not details.get("robotmode"):
            return {
                "error": "observation_unavailable",
                "hint": "No fresh Dashboard observation; cannot verify the robot is idle",
            }
        observed = {key: details.get(key) for key in IDLE_STATE}
        if observed != IDLE_STATE:
            return {
                "error": "robot_not_idle",
                "observed": observed,
                "required": IDLE_STATE,
                "hint": "Stop the running program / other client on the pendant before connecting",
            }
        return None

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
        session_open = self.session is not None and self.session.is_open
        latched = self.executor.latched if self.executor is not None else None
        allowed = ["control.stop"]
        if session_open and latched is None and self.session.watchdog_ok:
            allowed.append("control.joint_step")
        if not session_open:
            allowed.append("connect")
        return {
            "claimed_by": self.claims.claimed_by(),
            "control_session": {
                "open": session_open,
                "latched": latched,
                "watchdog_ok": self.session.watchdog_ok if session_open else None,
                "watchdog_error": self.session.watchdog_error if session_open else None,
                "commissioning_id": self.config.joint_step.commissioning_id,
                "max_step_deg": self.config.joint_step.max_step_deg,
                "last_event": self.last_event,
            },
            "allowed_actions": allowed,
        }

    def shutdown(self):
        with self._lock:
            session, self.session, self.executor = self.session, None, None
        if session is not None:
            session.close()

    # ── routes ──────────────────────────────────────────────────────
    def install(self, app):
        login = Depends(self.require_login)
        claim = Depends(self.require_claim)
        limits = self.config.joint_step

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
            self.claims.release(token=x_claim_token)
            return Response(status_code=204)

        @app.post("/connect", tags=["control"], dependencies=[claim])
        def connect(identity: dict = login):
            """Open the owned RTDE control session (uploads the control script)."""
            with self._lock:
                if self.session is not None and self.session.is_open:
                    raise HTTPException(
                        status_code=409,
                        detail={"error": "control_session_open", "hint": "POST /disconnect first"},
                    )
                problem = self.connect_preconditions()
                if problem is not None:
                    self.event("connect_refused", identity, **problem)
                    raise HTTPException(status_code=409, detail=problem)
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
                # explicit, claimed and identified reconnect.
                self.executor = JointStepExecutor(
                    control=session.control,
                    read_feedback=session.feedback.read,
                    claims=self.claims,
                    authorize=self.authorize,
                    limits=limits,
                )
            self.event("connect", identity)
            return {
                "connected": True,
                "commissioning_id": limits.commissioning_id,
                "notice": "RTDE control script uploaded; the controller program slot is owned by this service",
            }

        @app.post("/disconnect", tags=["control"], dependencies=[claim])
        def disconnect(identity: dict = login):
            with self._lock:
                session, self.session, self.executor = self.session, None, None
            error = None
            if session is not None:
                try:
                    session.close()
                except Exception as exc:
                    error = str(exc)
            self.event("disconnect", identity, error=error)
            return {"connected": False, "close_error": error}

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
                    detail={"error": "no_control_session", "hint": "POST /connect under your claim first"},
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

        @app.post("/control/stop", tags=["control"])
        @app.post("/move/stop", tags=["control"])
        def stop(identity: dict = login):
            """Identity-gated safety floor: no claim needed to ask for a stop."""
            executor, session = self.executor, self.session
            requested, error = False, None
            if executor is not None:
                executor.request_stop()
                requested = True
            if session is not None and session.is_open:
                try:
                    requested = session.stop(limits.stop_deceleration_deg_s2) or requested
                except Exception as exc:
                    error = str(exc)
            self.event("stop", identity, requested=requested, error=error)
            return {
                "stop_requested": requested,
                "stop_confirmed": False,
                "stop_error": error,
                "control_session": session is not None and session.is_open,
                "notice": STOP_NOTICE,
            }

        return self
