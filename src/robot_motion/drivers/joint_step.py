"""Bounded joint-step execution primitive; NOT wired into the live service.

No SDK imports, connections, script uploads, or HTTP routes. A future reviewed
SDK integration must supply an exclusively owned RTDE control interface, fresh
feedback, hard claims, and an authorization check bound to a commissioned plan.
Tests use fakes only. See docs/JOINT_STEP_CONTROL.md before integrating.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Annotated
from uuid import UUID

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    StrictBool,
    StrictInt,
    model_validator,
)

Number = Annotated[float, Field(strict=True, allow_inf_nan=False)]
Six = tuple[Number, Number, Number, Number, Number, Number]


class JointStep(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    request_id: UUID
    joint: Annotated[StrictInt, Field(ge=1, le=6)]
    delta_deg: Number = 0.1

    @model_validator(mode="after")
    def bounded(self):
        if not 0.02 <= abs(self.delta_deg) <= 0.5:
            raise ValueError("Joint step magnitude must be 0.02..0.5 degrees")
        return self


class JointStepLimits(BaseModel):
    """Explicit commissioning envelope. Software caps are not safety ratings."""

    model_config = ConfigDict(extra="forbid", frozen=True)
    commissioning_id: str = Field(min_length=1, max_length=120)
    lower_deg: Six
    upper_deg: Six
    stop_deceleration_deg_s2: Number = Field(gt=0, le=180)
    max_step_deg: Number = Field(default=0.1, ge=0.02, le=0.5)
    speed_deg_s: Number = Field(default=0.5, gt=0, le=1)
    acceleration_deg_s2: Number = Field(default=1.0, gt=0, le=2)
    session_travel_deg: Number = Field(default=0.5, ge=0.02, le=1)
    position_tolerance_deg: Number = Field(default=0.005, gt=0, le=0.005)
    stationary_speed_deg_s: Number = Field(default=0.02, gt=0, le=0.05)
    feedback_max_age_s: Number = Field(default=0.2, gt=0, le=0.2)
    timeout_s: Number = Field(default=5.0, gt=0, le=10)

    @model_validator(mode="after")
    def envelope(self):
        if any(lo >= hi for lo, hi in zip(self.lower_deg, self.upper_deg)):
            raise ValueError("Every lower joint limit must be below its upper limit")
        if self.session_travel_deg < self.max_step_deg:
            raise ValueError("Session budget must allow at least one maximum step")
        return self


class JointFeedback(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    joints_deg: Six
    velocities_deg_s: Six
    controller_timestamp_s: Number = Field(gt=0)
    received_monotonic_s: Number = Field(ge=0)
    controller_connected: StrictBool
    robot_mode: str
    safety_mode: str


class JointStepRefused(Exception):
    """Pre-dispatch refusal: no command was submitted."""


class JointStepFailed(Exception):
    """Dispatch was attempted; movement/stop may be ambiguous. Never retry."""

    def __init__(self, reason, stop_error=None):
        super().__init__(reason)
        self.stop_error = stop_error
        self.stop_attempted = True
        # stopJ returning is NOT independent evidence that the robot stopped.
        self.stop_confirmed = False


class JointStepExecutor:
    """One bounded moveJ at a time, with no queue, retries, or automatic reset.

    control must be an already authorized, exclusively owned RTDE interface.
    read_feedback must be fresh, bounded and in the same monotonic clock domain.
    authorize(request, target, commissioning_id) must recheck the authenticated
    SDK session/plan; returning anything except True refuses or aborts the move.
    This callback boundary is not itself an authentication implementation.
    """

    def __init__(
        self,
        *,
        control,
        read_feedback,
        claims,
        authorize,
        limits: JointStepLimits,
        clock=time.monotonic,
        sleep=time.sleep,
    ):
        self.control = control
        self.read_feedback = read_feedback
        self.claims = claims
        self.authorize = authorize
        self.limits = JointStepLimits.model_validate(limits)
        self.clock = clock
        self.sleep = sleep
        self._lock = threading.Lock()
        self._cancelled = threading.Event()
        self._fault = None
        self._spent = 0.0
        self._seen = set()

    def request_stop(self):
        # RTDEControlInterface is not thread-safe: wake the owning move loop,
        # never issue a concurrent SDK call. This is NOT a safety-rated stop.
        self._cancelled.set()

    def _gate(self, token, request, target):
        if self._cancelled.is_set() or self._fault is not None:
            raise JointStepRefused(
                "Stop/fault is latched; operator reconciliation required"
            )
        if self.claims.enforced is not True:
            raise JointStepRefused("Hard claim enforcement is required")
        try:
            self.claims.verify_token(token)
        except Exception as exc:
            raise JointStepRefused("Control claim missing or expired") from exc
        if self.authorize(request, target, self.limits.commissioning_id) is not True:
            raise JointStepRefused(
                "Authorized SDK session and commissioned plan required"
            )
        if not self.control.isConnected():
            raise JointStepRefused("Owned control interface disconnected")

    def _sample(self, previous_timestamp=None):
        feedback = JointFeedback.model_validate(self.read_feedback())
        age = self.clock() - feedback.received_monotonic_s
        if not 0 <= age <= self.limits.feedback_max_age_s:
            raise JointStepRefused("Joint feedback is stale or from a different clock")
        if (
            previous_timestamp is not None
            and feedback.controller_timestamp_s <= previous_timestamp
        ):
            raise JointStepRefused("Controller feedback stopped advancing or restarted")
        if (
            not feedback.controller_connected
            or feedback.robot_mode != "RUNNING"
            or feedback.safety_mode != "NORMAL"
        ):
            raise JointStepRefused(
                "Controller or safety state does not permit a joint step"
            )
        if any(
            not lo <= q <= hi
            for q, lo, hi in zip(
                feedback.joints_deg,
                self.limits.lower_deg,
                self.limits.upper_deg,
            )
        ):
            raise JointStepRefused(
                "Measured joints are outside the commissioned envelope"
            )
        return feedback

    def _stationary(self, sample):
        return all(
            abs(v) <= self.limits.stationary_speed_deg_s
            for v in sample.velocities_deg_s
        )

    def execute(self, request: JointStep, *, claim_token):
        request = JointStep.model_validate(request)
        if not self._lock.acquire(blocking=False):
            raise JointStepRefused(
                "A joint step is already active; commands are not queued"
            )
        dispatched = False
        try:
            if request.request_id in self._seen:
                raise JointStepRefused(
                    "Duplicate request; joint steps are never replayed"
                )
            if len(self._seen) >= 100:
                raise JointStepRefused("Session request budget exhausted")
            if abs(request.delta_deg) > self.limits.max_step_deg:
                raise JointStepRefused(
                    "Requested step exceeds the commissioned step limit"
                )
            if (
                self._spent + abs(request.delta_deg)
                > self.limits.session_travel_deg + 1e-12
            ):
                raise JointStepRefused("Session travel budget exhausted")
            # No control commands during preflight, including a failed check.
            # Authorization is bound again to the exact calculated target below.
            self._gate(claim_token, request, None)
            initial = self._sample()
            if not self._stationary(initial):
                raise JointStepRefused("Robot is already moving")
            self.sleep(0.02)
            start = self._sample(initial.controller_timestamp_s)
            tolerance = self.limits.position_tolerance_deg
            if not self._stationary(start) or any(
                abs(a - b) > tolerance
                for a, b in zip(initial.joints_deg, start.joints_deg)
            ):
                raise JointStepRefused("Robot changed position during preflight")
            target = list(start.joints_deg)
            target[request.joint - 1] += request.delta_deg
            target = tuple(target)
            if any(
                not lo <= q <= hi
                for q, lo, hi in zip(
                    target, self.limits.lower_deg, self.limits.upper_deg
                )
            ):
                raise JointStepRefused("Target exceeds the commissioned joint envelope")
            self._gate(claim_token, request, target)
            # The authorization check may itself take time; recheck sample age.
            if (
                self.clock() - start.received_monotonic_s
                > self.limits.feedback_max_age_s
            ):
                raise JointStepRefused("Feedback became stale before dispatch")
            self._seen.add(request.request_id)
            self._spent += abs(request.delta_deg)
            dispatched = True  # even a thrown SDK exception can be ambiguous
            deadline = self.clock() + self.limits.timeout_s
            accepted = self.control.moveJ(
                [math.radians(q) for q in target],
                math.radians(self.limits.speed_deg_s),
                math.radians(self.limits.acceleration_deg_s2),
                True,
            )
            if accepted is not True:
                raise RuntimeError("RTDE moveJ did not acknowledge the step")
            previous = start.controller_timestamp_s
            settled_since = None
            settled_samples = 0
            while self.clock() < deadline:
                self._gate(claim_token, request, target)
                current = self._sample(previous)
                if self.clock() >= deadline:
                    raise TimeoutError(
                        "Joint step feedback/authorization exceeded the deadline"
                    )
                previous = current.controller_timestamp_s
                if any(
                    not min(a, b) - tolerance <= q <= max(a, b) + tolerance
                    for q, a, b in zip(current.joints_deg, start.joints_deg, target)
                ):
                    raise RuntimeError(
                        "Joint motion left the requested single-axis segment"
                    )
                at_target = all(
                    abs(q - t) <= tolerance for q, t in zip(current.joints_deg, target)
                )
                if at_target and self._stationary(current):
                    if settled_since is None:
                        settled_since = self.clock()
                    settled_samples += 1
                    if settled_samples >= 3 and self.clock() - settled_since >= 0.1:
                        self._gate(claim_token, request, target)
                        if (
                            self.clock() >= deadline
                            or self.clock() - current.received_monotonic_s
                            > self.limits.feedback_max_age_s
                        ):
                            raise TimeoutError(
                                "Completion feedback became stale before verification"
                            )
                        return {
                            "request_id": str(request.request_id),
                            "completed": True,
                            "target_deg": target,
                            "measured_deg": current.joints_deg,
                        }
                else:
                    settled_since, settled_samples = None, 0
                self.sleep(0.02)
            raise TimeoutError("Joint step did not reach and settle at the target")
        except Exception as exc:
            if not dispatched:
                raise
            self._fault = str(exc)
            self._cancelled.set()
            stop_error = None
            try:
                self.control.stopJ(math.radians(self.limits.stop_deceleration_deg_s2))
            except Exception as stop_exc:
                stop_error = str(stop_exc)
            raise JointStepFailed(str(exc), stop_error) from exc
        finally:
            self._lock.release()
