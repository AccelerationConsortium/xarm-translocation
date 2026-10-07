"""Bounded arm motion for the UR panel: joint moves and jogs (moveJ) and
Cartesian moves and jogs (moveL) inside a commissioned joint envelope and a
TCP workspace box.

Nothing here imports an SDK or opens a connection. The executor drives an
already open, exclusively owned RTDE control interface (ur_control.py) and
reads its dedicated feedback stream. Every limit comes from the local config;
the ceilings on the fields only stop a typo from becoming a fast arm. None of
this is a safety-rated function: the controller's safety configuration, the
pendant and the e-stop remain the safety system.

Planning uses the controller's own kinematics (forward kinematics along a
joint move, inverse kinematics along a straight line), so the active TCP set
on the pendant must be the real tool before any of these checks mean much.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Annotated
from uuid import UUID, uuid4

from pydantic import BaseModel, ConfigDict, Field, StrictInt, model_validator

from .lle_rtde import tcp_to_mm_deg, vector6

Number = Annotated[float, Field(strict=True, allow_inf_nan=False)]
Six = tuple[Number, Number, Number, Number, Number, Number]
Span = tuple[Number, Number]

JOINT_ACTIONS = ("arm.move_joints", "arm.jog_joint")
LINEAR_ACTIONS = ("arm.move_linear", "arm.jog_linear")
MOVE_ACTIONS = JOINT_ACTIONS + LINEAR_ACTIONS
MAX_PATH_SAMPLES = 200


class Workspace(BaseModel):
    """Axis-aligned box for the TCP point, base frame, millimetres.

    It bounds one point only: the tool body, the fingers and the arm links
    can still reach outside it. It is not collision checking.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)
    x_mm: Span
    y_mm: Span
    z_mm: Span

    @model_validator(mode="after")
    def ordered(self):
        for name in ("x_mm", "y_mm", "z_mm"):
            lo, hi = getattr(self, name)
            if not -2000 <= lo < hi <= 2000:
                raise ValueError(f"workspace {name} must be [min, max], min < max, within +/-2000 mm")
        return self

    def outside(self, point_mm, margin_mm=0.0):
        """Axes on which point_mm lies outside the box widened by margin_mm."""
        return [
            axis
            for axis, value, (lo, hi) in zip("xyz", point_mm, (self.x_mm, self.y_mm, self.z_mm))
            if not lo - margin_mm <= value <= hi + margin_mm
        ]


class MotionLimits(BaseModel):
    """Commissioning envelope for arm motion. Software limits, not safety ratings."""

    model_config = ConfigDict(extra="forbid", frozen=True)
    commissioning_id: str = Field(min_length=1, max_length=120)
    joint_lower_deg: Six
    joint_upper_deg: Six
    workspace: Workspace
    # A request above a cap is refused (422), never clamped. Every move uses
    # the configured acceleration.
    max_joint_speed_deg_s: Number = Field(gt=0, le=60)
    joint_accel_deg_s2: Number = Field(gt=0, le=180)
    max_linear_speed_mm_s: Number = Field(gt=0, le=250)
    linear_accel_mm_s2: Number = Field(gt=0, le=1500)
    stop_joint_decel_deg_s2: Number = Field(gt=0, le=720)
    stop_linear_decel_mm_s2: Number = Field(gt=0, le=10000)
    # Used when a request names no speed; None means a quarter of the cap.
    default_joint_speed_deg_s: Number | None = Field(default=None, gt=0)
    default_linear_speed_mm_s: Number | None = Field(default=None, gt=0)
    max_jog_joint_deg: Number = Field(default=10, gt=0, le=45)
    max_jog_mm: Number = Field(default=50, gt=0, le=200)
    # Abort a move when the measured TCP force changes by more than this from
    # its start. None: no software force guard (the controller's own force
    # limit still applies). A change, so a sensor offset does not trip it.
    force_guard_n: Number | None = Field(default=None, ge=5, le=150)
    position_tolerance_deg: Number = Field(default=0.05, ge=0.01, le=0.5)
    # How far a measured joint may stray from its start-to-target segment (and
    # past the envelope) while moving: tracking error, not completion.
    path_tolerance_deg: Number = Field(default=0.5, ge=0.05, le=5)
    position_tolerance_mm: Number = Field(default=0.5, ge=0.05, le=5)
    orientation_tolerance_deg: Number = Field(default=0.2, ge=0.01, le=2)
    # How far the measured TCP may stray from the planned straight line, and
    # outside the box while moving.
    path_tolerance_mm: Number = Field(default=2.0, ge=0.5, le=10)
    stationary_speed_deg_s: Number = Field(default=0.05, gt=0, le=0.5)
    feedback_max_age_s: Number = Field(default=0.2, gt=0, le=0.2)
    # Planning resolution: forward kinematics every step along a joint move,
    # inverse kinematics every step along a straight line.
    path_check_step_deg: Number = Field(default=2.0, ge=0.5, le=10)
    path_check_step_mm: Number = Field(default=10.0, ge=1, le=50)
    # A larger joint change between neighbouring line samples means a wrist
    # flip or a near-singular stretch: refused rather than swept.
    max_ik_jump_deg: Number = Field(default=15.0, ge=1, le=90)
    # The pendant speed slider scales every move. Below this fraction moves
    # are refused (they would crawl past any sensible deadline); above it the
    # deadline stretches by the slider.
    min_speed_fraction: Number = Field(default=0.1, gt=0, le=1)

    @model_validator(mode="after")
    def coherent(self):
        for lo, hi in zip(self.joint_lower_deg, self.joint_upper_deg):
            if not -363 <= lo < hi <= 363:
                raise ValueError("Every joint needs lower < upper within +/-363 degrees")
        if self.stop_joint_decel_deg_s2 < self.joint_accel_deg_s2:
            raise ValueError("stop_joint_decel_deg_s2 must be at least joint_accel_deg_s2")
        if self.stop_linear_decel_mm_s2 < self.linear_accel_mm_s2:
            raise ValueError("stop_linear_decel_mm_s2 must be at least linear_accel_mm_s2")
        if (self.default_joint_speed_deg_s or 0) > self.max_joint_speed_deg_s:
            raise ValueError("default_joint_speed_deg_s exceeds max_joint_speed_deg_s")
        if (self.default_linear_speed_mm_s or 0) > self.max_linear_speed_mm_s:
            raise ValueError("default_linear_speed_mm_s exceeds max_linear_speed_mm_s")
        return self

    @property
    def joint_speed_default(self):
        return self.default_joint_speed_deg_s or self.max_joint_speed_deg_s / 4

    @property
    def linear_speed_default(self):
        return self.default_linear_speed_mm_s or self.max_linear_speed_mm_s / 4

    def summary(self):
        """The limits an operator or agent needs to plan within (for /status)."""
        return {
            "joint_lower_deg": list(self.joint_lower_deg),
            "joint_upper_deg": list(self.joint_upper_deg),
            "workspace_mm": {
                "x": list(self.workspace.x_mm),
                "y": list(self.workspace.y_mm),
                "z": list(self.workspace.z_mm),
            },
            "max_joint_speed_deg_s": self.max_joint_speed_deg_s,
            "max_linear_speed_mm_s": self.max_linear_speed_mm_s,
            "default_joint_speed_deg_s": self.joint_speed_default,
            "default_linear_speed_mm_s": self.linear_speed_default,
            "max_jog_joint_deg": self.max_jog_joint_deg,
            "max_jog_mm": self.max_jog_mm,
            "force_guard_n": self.force_guard_n,
        }


# ── requests ──────────────────────────────────────────────────────────
class _Motion(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    # Optional: a request id is never run twice. The panel sends none.
    request_id: UUID | None = None
    # deg/s for joint moves, mm/s for linear moves; None: the default.
    speed: Number | None = Field(default=None, gt=0)


class JointMove(_Motion):
    """Absolute target for all six joints, degrees."""

    angles: Six


class JointJog(_Motion):
    """Relative change of one joint, degrees."""

    joint: Annotated[StrictInt, Field(ge=1, le=6)]
    delta: Number

    @model_validator(mode="after")
    def nonzero(self):
        if abs(self.delta) < 0.01:
            raise ValueError("delta must be at least 0.01 degrees")
        return self


class LinearJog(_Motion):
    """Relative TCP translation in the base frame, mm; orientation is kept."""

    dx: Number = 0.0
    dy: Number = 0.0
    dz: Number = 0.0

    @model_validator(mode="after")
    def nonzero(self):
        if math.sqrt(self.dx**2 + self.dy**2 + self.dz**2) < 0.01:
            raise ValueError("a jog needs at least 0.01 mm of translation")
        return self


class LinearMove(_Motion):
    """Absolute TCP pose: base-frame mm and roll/pitch/yaw degrees, the same
    convention as details.current_position."""

    x: Number
    y: Number
    z: Number
    roll: Number
    pitch: Number
    yaw: Number


# ── outcomes ──────────────────────────────────────────────────────────
class MotionRefused(Exception):
    """Nothing was sent to the robot.

    status 412: a state precondition (mirrored in allowed_actions); 422: this
    request cannot run under the commissioned limits; 409: a state conflict.
    """

    def __init__(self, error, reason, *, status=412, **extra):
        super().__init__(reason)
        self.error = error
        self.status = status
        self.extra = extra

    def body(self):
        return {"error": self.error, "reason": str(self), **self.extra}


class MotionFailed(Exception):
    """A move was sent and the arm may have moved. A stop was attempted and
    further moves are latched until an explicit reset."""

    def __init__(self, reason, *, stop_error=None, measured=None):
        super().__init__(reason)
        self.stop_attempted = True
        self.stop_confirmed = False  # a returned stop call is not measurement
        self.stop_error = stop_error
        self.measured = measured


# ── geometry ──────────────────────────────────────────────────────────
def _rotation():
    from scipy.spatial.transform import Rotation

    return Rotation


def rpy_to_rotvec(rpy_deg):
    return _rotation().from_euler("xyz", list(rpy_deg), degrees=True).as_rotvec().tolist()


def orientation_error_deg(rotvec_a, rotvec_b):
    rotation = _rotation()
    return math.degrees((rotation.from_rotvec(rotvec_a).inv() * rotation.from_rotvec(rotvec_b)).magnitude())


def interpolate_poses(start, end, count):
    """count + 1 poses from start to end: linear position, slerp orientation."""
    from scipy.spatial.transform import Slerp

    rotation = _rotation()
    slerp = Slerp([0.0, 1.0], rotation.from_rotvec([start[3:], end[3:]]))
    poses = []
    for i in range(count + 1):
        f = i / count
        position = [a + (b - a) * f for a, b in zip(start[:3], end[:3])]
        poses.append(position + slerp([f]).as_rotvec()[0].tolist())
    return poses


def distance_to_segment(point, a, b):
    ab = [y - x for x, y in zip(a, b)]
    ap = [y - x for x, y in zip(a, point)]
    length2 = sum(v * v for v in ab)
    f = 0.0 if length2 == 0 else max(0.0, min(1.0, sum(x * y for x, y in zip(ap, ab)) / length2))
    closest = [x + v * f for x, v in zip(a, ab)]
    return math.dist(point, closest)


def profile_time(distance, speed, accel):
    """Duration of a trapezoidal (or triangular) move from rest to rest."""
    if distance <= 0:
        return 0.0
    if distance >= speed * speed / accel:
        return distance / speed + speed / accel
    return 2 * math.sqrt(distance / accel)


@dataclass
class Plan:
    action: str
    kind: str  # "joint" | "linear"
    start_q_deg: tuple
    target_q_deg: tuple
    start_pose: list  # metres + rotation vector
    target_pose: list | None
    speed: float  # deg/s or mm/s
    timeout_s: float
    samples: int


# ── executor ──────────────────────────────────────────────────────────
class MotionExecutor:
    """One bounded move at a time; no queue, retries or automatic reset.

    control must be an exclusively owned RTDE control interface (serialized
    by ur_control); read_feedback returns a new controller packet per call
    with joints, velocities, modes, TCP pose and TCP force. authorize(request,
    target, commissioning_id) must recheck that the claim holder and session
    are still authorized; anything but True refuses or aborts.
    """

    def __init__(
        self,
        *,
        control,
        read_feedback,
        claims,
        authorize,
        limits: MotionLimits,
        clock=time.monotonic,
        sleep=time.sleep,
        seen=None,
    ):
        self.control = control
        self.read_feedback = read_feedback
        self.claims = claims
        self.authorize = authorize
        self.limits = MotionLimits.model_validate(limits)
        self.clock = clock
        self.sleep = sleep
        self._lock = threading.Lock()
        # Held from the last cancel check through the moveJ/moveL call, so a
        # STOP either prevents the dispatch or lands after it (and stops it).
        self._dispatch_lock = threading.Lock()
        self._cancelled = threading.Event()
        self._fault = None
        # Request ids already run; handed on to the executor a reset creates.
        self._seen = seen if seen is not None else set()
        self.active = None

    # ── state ───────────────────────────────────────────────────────
    def request_stop(self):
        # The SDK is not thread-safe: wake the move loop, which issues the
        # stop itself. The STOP route also sends one through the SDK lock,
        # after this returns, so it can never precede a dispatch.
        with self._dispatch_lock:
            self._cancelled.set()

    def retire(self):
        """Take the move lock for good (disconnect): False if a move runs."""
        return self._lock.acquire(blocking=False)

    @property
    def seen(self):
        return self._seen

    @property
    def latched(self):
        if self._fault is not None:
            return self._fault
        return "Stop requested" if self._cancelled.is_set() else None

    @property
    def busy(self):
        return self._lock.locked()

    @property
    def active_kind(self):
        active = self.active
        return active["kind"] if active else None

    def state_block(self, sample, now=None):
        """MotionRefused for a state that refuses every move, or None.

        Request-independent, so /status allowed_actions calls it with the
        latest feedback packet and stays in step with the routes (STATUS_SPEC
        section 6.2). Request-specific limits are checked by the planner.
        """
        if self.latched is not None:
            return MotionRefused(
                "motion_latched",
                f"Moves are latched ({self.latched}); check the arm, then Clear errors",
                hint="POST /control/reset (the panel's Clear errors) after checking the arm",
            )
        if not self.control.isConnected():
            return MotionRefused("control_link_down", "Owned control interface is not connected")
        if self.control.isProgramRunning() is not True:
            return MotionRefused(
                "control_script_stopped",
                "The control script is not running (for example after a protective stop)",
                hint="Clear any stop on the pendant, then Disconnect and Connect",
            )
        if sample is None:
            return MotionRefused("feedback_unavailable", "No control feedback yet")
        now = self.clock() if now is None else now
        if not 0 <= now - sample["received_monotonic_s"] <= self.limits.feedback_max_age_s:
            return MotionRefused("feedback_stale", "Control feedback is stale")
        if not (
            sample["controller_connected"]
            and sample["robot_mode"] == "RUNNING"
            and sample["safety_mode"] == "NORMAL"
        ):
            return MotionRefused(
                "robot_not_ready",
                "Controller must be RUNNING with safety NORMAL",
                observed={"robot_mode": sample["robot_mode"], "safety_mode": sample["safety_mode"]},
            )
        fraction = sample.get("speed_fraction")
        if fraction is not None and fraction < self.limits.min_speed_fraction:
            return MotionRefused(
                "speed_slider_low",
                f"The pendant speed slider is at {fraction:.0%}; raise it to at least "
                f"{self.limits.min_speed_fraction:.0%}",
                speed_fraction=fraction,
            )
        if not self._stationary(sample):
            return MotionRefused("robot_moving", "The arm is moving; wait until it is still")
        joints = self._outside_envelope(sample["joints_deg"])
        if joints:
            return MotionRefused(
                "outside_envelope",
                f"Joints {joints} are outside the commissioned envelope; move the arm in with the pendant",
                joints=joints,
            )
        point = [v * 1000 for v in sample["tcp_pose"][:3]]
        axes = self.limits.workspace.outside(point)
        if axes:
            return MotionRefused(
                "outside_workspace",
                f"TCP is outside the workspace box on {''.join(axes)}; move it in with the pendant",
                tcp_mm=point,
                axes=axes,
            )
        return None

    # ── checks ──────────────────────────────────────────────────────
    def _outside_envelope(self, joints_deg, margin=0.0):
        return [
            i + 1
            for i, (q, lo, hi) in enumerate(zip(joints_deg, self.limits.joint_lower_deg, self.limits.joint_upper_deg))
            if not lo - margin <= q <= hi + margin
        ]

    def _require_claim_and_unlatched(self, token):
        if self._cancelled.is_set() or self._fault is not None:
            raise MotionRefused("motion_latched", f"Moves are latched ({self.latched})")
        if self.claims.enforced is not True:
            raise MotionRefused("claims_not_enforced", "Hard claim enforcement is required")
        try:
            self.claims.verify_token(token)
        except Exception as exc:
            raise MotionRefused("claim_lost", "Control claim missing or expired") from exc

    def _gate(self, token, target=None):
        self._require_claim_and_unlatched(token)
        if self.authorize(None, target, self.limits.commissioning_id) is not True:
            raise MotionRefused("not_authorized", "Authorized session and operator required")
        if not self.control.isConnected():
            raise MotionRefused("control_link_down", "Owned control interface is not connected")
        self._require_claim_and_unlatched(token)

    def _sample(self, previous_timestamp=None):
        raw = self.read_feedback()
        try:
            sample = {
                **raw,
                "joints_deg": tuple(vector6(raw["joints_deg"])),
                "velocities_deg_s": tuple(vector6(raw["velocities_deg_s"])),
                "tcp_pose": vector6(raw["tcp_pose"]),
                "tcp_force": vector6(raw["tcp_force"]),
            }
        except (KeyError, TypeError, ValueError) as exc:
            raise MotionRefused("feedback_invalid", f"Control feedback is incomplete: {exc}") from exc
        age = self.clock() - sample["received_monotonic_s"]
        if not 0 <= age <= self.limits.feedback_max_age_s:
            raise MotionRefused("feedback_stale", "Control feedback is stale or from a different clock")
        if previous_timestamp is not None and sample["controller_timestamp_s"] <= previous_timestamp:
            raise MotionRefused("feedback_stalled", "Controller feedback stopped advancing or restarted")
        if not (
            sample["controller_connected"]
            and sample["robot_mode"] == "RUNNING"
            and sample["safety_mode"] == "NORMAL"
        ):
            raise MotionRefused(
                "robot_not_ready",
                f"Controller is {sample['robot_mode']} / safety {sample['safety_mode']}",
            )
        return sample

    def _stationary(self, sample):
        return all(abs(v) <= self.limits.stationary_speed_deg_s for v in sample["velocities_deg_s"])

    def _settled_between(self, a, b):
        tolerance = self.limits.position_tolerance_deg
        if not (self._stationary(a) and self._stationary(b)) or any(
            abs(x - y) > tolerance for x, y in zip(a["joints_deg"], b["joints_deg"])
        ):
            raise MotionRefused("robot_moving", "The arm is moving; wait until it is still")

    # ── planning (controller kinematics, nothing moves) ─────────────
    def _plan(self, action, request, start, speed):
        lim = self.limits
        q0 = start["joints_deg"]
        p0 = start["tcp_pose"]
        if action in JOINT_ACTIONS:
            if action == "arm.jog_joint":
                if abs(request.delta) > lim.max_jog_joint_deg:
                    raise MotionRefused(
                        "jog_too_large", f"Joint jog is limited to {lim.max_jog_joint_deg} degrees",
                        status=422, max_jog_joint_deg=lim.max_jog_joint_deg,
                    )
                target = list(q0)
                target[request.joint - 1] += request.delta
            else:
                target = list(request.angles)
            return self._plan_joint(action, q0, tuple(target), p0, speed)
        if action == "arm.jog_linear":
            delta = [request.dx, request.dy, request.dz]
            if math.sqrt(sum(d * d for d in delta)) > lim.max_jog_mm:
                raise MotionRefused(
                    "jog_too_large", f"Cartesian jog is limited to {lim.max_jog_mm} mm",
                    status=422, max_jog_mm=lim.max_jog_mm,
                )
            target = [p + d / 1000 for p, d in zip(p0[:3], delta)] + list(p0[3:])
        else:
            target = [request.x / 1000, request.y / 1000, request.z / 1000] + rpy_to_rotvec(
                (request.roll, request.pitch, request.yaw)
            )
        return self._plan_linear(action, q0, p0, target, speed)

    def _plan_joint(self, action, q0, target, p0, speed):
        lim = self.limits
        outside = self._outside_envelope(target)
        if outside:
            raise MotionRefused(
                "target_outside_envelope", f"Target joints {outside} are outside the commissioned envelope",
                status=422, joints=outside,
            )
        radians = [math.radians(q) for q in target]
        if self.control.isJointsWithinSafetyLimits(radians) is not True:
            raise MotionRefused(
                "outside_controller_limits", "The controller's safety limits reject this joint target", status=422
            )
        travel = max(abs(b - a) for a, b in zip(q0, target))
        count = min(MAX_PATH_SAMPLES, max(1, math.ceil(travel / lim.path_check_step_deg)))
        for i in range(count + 1):
            q = [a + (b - a) * i / count for a, b in zip(q0, target)]
            pose = vector6(self.control.getForwardKinematics([math.radians(v) for v in q]))
            point = [v * 1000 for v in pose[:3]]
            axes = lim.workspace.outside(point)
            if axes:
                raise MotionRefused(
                    "path_leaves_workspace",
                    f"The TCP would leave the workspace box on {''.join(axes)} during this joint move",
                    status=422, fraction=round(i / count, 3), tcp_mm=[round(v, 1) for v in point],
                )
        duration = profile_time(travel, speed, lim.joint_accel_deg_s2)
        return Plan(action, "joint", q0, target, list(p0), None, speed, 1.5 * duration + 2.0, count + 1)

    def _plan_linear(self, action, q0, p0, target, speed):
        lim = self.limits
        point = [v * 1000 for v in target[:3]]
        axes = lim.workspace.outside(point)
        if axes:
            raise MotionRefused(
                "target_outside_workspace", f"Target is outside the workspace box on {''.join(axes)}",
                status=422, tcp_mm=[round(v, 1) for v in point], axes=axes,
            )
        if self.control.isPoseWithinSafetyLimits(list(target)) is not True:
            raise MotionRefused(
                "outside_controller_limits", "The controller's safety limits reject this pose", status=422
            )
        q_start = [math.radians(q) for q in q0]
        if self.control.getInverseKinematicsHasSolution(list(target), q_start) is not True:
            raise MotionRefused("unreachable", "No inverse kinematics solution for this pose", status=422)
        length = math.dist(p0[:3], target[:3]) * 1000
        turn = orientation_error_deg(p0[3:], target[3:])
        count = min(
            MAX_PATH_SAMPLES,
            max(1, math.ceil(length / lim.path_check_step_mm), math.ceil(turn / lim.path_check_step_deg)),
        )
        previous = q_start
        # Cruise time per sample step: the slower of translation (speed in
        # mm/s) and rotation (the controller reads the speed as rad/s then).
        step_time = max(length / count / speed, turn / count / math.degrees(speed / 1000))
        peak_rate = 0.0
        for i, pose in enumerate(interpolate_poses(p0, target, count)[1:], start=1):
            # URScript's inverse kinematics raises (ending the control script)
            # when there is no solution, so ask first.
            if self.control.getInverseKinematicsHasSolution(pose, previous) is not True:
                raise MotionRefused(
                    "unreachable", "No inverse kinematics solution along this line",
                    status=422, fraction=round(i / count, 3),
                )
            q = vector6(self.control.getInverseKinematics(pose, previous))
            degrees = [math.degrees(v) for v in q]
            outside = self._outside_envelope(degrees)
            if outside:
                raise MotionRefused(
                    "path_outside_envelope",
                    f"Joints {outside} would leave the commissioned envelope along this line",
                    status=422, fraction=round(i / count, 3), joints=outside,
                )
            jump = max(abs(math.degrees(a - b)) for a, b in zip(q, previous))
            if jump > lim.max_ik_jump_deg:
                raise MotionRefused(
                    "path_configuration_change",
                    "The straight line passes a wrist flip or near-singular stretch; use a joint move",
                    status=422, fraction=round(i / count, 3), jump_deg=round(jump, 1),
                )
            if step_time > 0:
                peak_rate = max(peak_rate, jump / step_time)
            previous = q
        # A straight line at a modest TCP speed can still spin a joint fast
        # (J1 far from the base, a wrist near a singularity): bound it by the
        # joint speed cap too.
        if peak_rate > lim.max_joint_speed_deg_s:
            raise MotionRefused(
                "joint_speed_exceeded",
                f"At {speed:g} mm/s a joint would turn about {peak_rate:.0f} deg/s along this line "
                f"(cap {lim.max_joint_speed_deg_s:g}); go slower or use a joint move",
                status=422,
                peak_joint_speed_deg_s=round(peak_rate, 1),
                max_speed_mm_s=round(speed * lim.max_joint_speed_deg_s / peak_rate, 1),
            )
        # Translation and rotation each bound the duration; for a pure turn the
        # controller reads the speed as rad/s, taken here in degrees as well.
        duration = max(
            profile_time(length, speed, lim.linear_accel_mm_s2),
            profile_time(turn, math.degrees(speed / 1000), math.degrees(lim.linear_accel_mm_s2 / 1000)),
        )
        target_q = tuple(math.degrees(v) for v in previous)
        return Plan(action, "linear", q0, target_q, list(p0), list(target), speed, 1.5 * duration + 3.0, count + 1)

    # ── execution ───────────────────────────────────────────────────
    def _stop(self, kind):
        try:
            if kind == "linear":
                self.control.stopL(self.limits.stop_linear_decel_mm_s2 / 1000)
            else:
                self.control.stopJ(math.radians(self.limits.stop_joint_decel_deg_s2))
            return None
        except Exception as exc:  # noqa: BLE001 - reported, never hidden
            return str(exc)

    def _dispatch(self, plan):
        lim = self.limits
        if plan.kind == "joint":
            return self.control.moveJ(
                [math.radians(q) for q in plan.target_q_deg],
                math.radians(plan.speed),
                math.radians(lim.joint_accel_deg_s2),
                True,
            )
        return self.control.moveL(plan.target_pose, plan.speed / 1000, lim.linear_accel_mm_s2 / 1000, True)

    def _monitor(self, plan, start, deadline, token):
        lim = self.limits
        tolerance = lim.position_tolerance_deg
        slack = lim.path_tolerance_deg
        overspeed = 1.25 * lim.max_joint_speed_deg_s + 1.0
        baseline = start["tcp_force"][:3]
        peak_force = 0.0
        previous = start["controller_timestamp_s"]
        settled_since, settled = None, 0
        start_mm = [v * 1000 for v in plan.start_pose[:3]]
        target_mm = [v * 1000 for v in (plan.target_pose or [])[:3]]
        while True:
            target = plan.target_q_deg if plan.kind == "joint" else plan.target_pose
            self._gate(token, target)
            sample = self._sample(previous)
            now = self.clock()
            if now >= deadline:
                raise TimeoutError("The move did not finish within its deadline")
            previous = sample["controller_timestamp_s"]
            joints = sample["joints_deg"]
            outside = self._outside_envelope(joints, margin=slack)
            if outside:
                raise RuntimeError(f"Joints {outside} left the commissioned envelope")
            fastest = max(abs(v) for v in sample["velocities_deg_s"])
            if fastest > overspeed:
                raise RuntimeError(
                    f"A joint turned at {fastest:.0f} deg/s, over the {lim.max_joint_speed_deg_s:g} deg/s cap"
                )
            point = [v * 1000 for v in sample["tcp_pose"][:3]]
            axes = lim.workspace.outside(point, margin_mm=lim.path_tolerance_mm)
            if axes:
                raise RuntimeError(f"TCP left the workspace box on {''.join(axes)}")
            if plan.kind == "joint":
                if any(
                    not min(a, b) - slack <= q <= max(a, b) + slack
                    for q, a, b in zip(joints, plan.start_q_deg, plan.target_q_deg)
                ):
                    raise RuntimeError("Joint motion left the commanded segment")
                at_target = all(abs(q - t) <= tolerance for q, t in zip(joints, plan.target_q_deg))
            else:
                if distance_to_segment(point, start_mm, target_mm) > lim.path_tolerance_mm:
                    raise RuntimeError("TCP left the planned straight line")
                at_target = (
                    math.dist(point, target_mm) <= lim.position_tolerance_mm
                    and orientation_error_deg(sample["tcp_pose"][3:], plan.target_pose[3:])
                    <= lim.orientation_tolerance_deg
                )
            force = math.dist(sample["tcp_force"][:3], baseline)
            peak_force = max(peak_force, force)
            if lim.force_guard_n is not None and force > lim.force_guard_n:
                raise RuntimeError(
                    f"Force guard: TCP force changed by {force:.1f} N (limit {lim.force_guard_n} N)"
                )
            if at_target and self._stationary(sample):
                if settled_since is None:
                    settled_since = now
                settled += 1
                if settled >= 3 and now - settled_since >= 0.1:
                    return sample, peak_force
            else:
                settled_since, settled = None, 0
            self.sleep(0.02)

    def execute(self, action, request, *, claim_token, speed):
        if action not in MOVE_ACTIONS:
            raise ValueError(f"Unknown arm action {action!r}")
        if not self._lock.acquire(blocking=False):
            raise MotionRefused(
                "motion_busy", "Another arm move is running; commands are not queued", status=409
            )
        kind = "joint" if action in JOINT_ACTIONS else "linear"
        dispatched = False
        started = self.clock()
        try:
            cap = self.limits.max_joint_speed_deg_s if kind == "joint" else self.limits.max_linear_speed_mm_s
            if not 0 < speed <= cap:
                raise MotionRefused(
                    "above_commissioned_limit", f"Speed {speed:g} is outside (0, {cap:g}]",
                    status=422, requested=speed, limit=cap,
                )
            if request.request_id is not None and request.request_id in self._seen:
                raise MotionRefused("request_replayed", "This request id already ran", status=422)
            if len(self._seen) >= 10000:
                raise MotionRefused("session_request_budget", "Reconnect to start a new session", status=409)
            self._gate(claim_token)
            first = self._sample()
            block = self.state_block(first)
            if block is not None:
                raise block
            self.sleep(0.02)
            start = self._sample(first["controller_timestamp_s"])
            self._settled_between(first, start)
            try:
                plan = self._plan(action, request, start, speed)
            except MotionRefused:
                raise
            except Exception as exc:  # a kinematics query failed; nothing was sent
                raise MotionRefused(
                    "planning_failed", f"Controller kinematics query failed: {exc}", status=502
                ) from exc
            # Within the completion tolerance already: report it rather than
            # send a move whose "arrival" could be read before it starts.
            lim = self.limits
            if (
                plan.kind == "joint"
                and max(abs(a - b) for a, b in zip(plan.start_q_deg, plan.target_q_deg))
                <= lim.position_tolerance_deg
            ) or (
                plan.kind == "linear"
                and math.dist(plan.start_pose[:3], plan.target_pose[:3]) * 1000 <= lim.position_tolerance_mm
                and orientation_error_deg(plan.start_pose[3:], plan.target_pose[3:])
                <= lim.orientation_tolerance_deg
            ):
                return self._result(plan, request, start, started, moved=False, peak_force=0.0)
            self._gate(claim_token, plan.target_q_deg)
            # Planning took controller round trips: re-measure, and only move
            # from the pose the plan was made for.
            fresh = self._sample()
            block = self.state_block(fresh)
            if block is not None:
                raise block
            self._settled_between(start, fresh)
            if request.request_id is not None:
                self._seen.add(request.request_id)
            self.active = {"action": action, "kind": kind, "target_joints_deg": list(plan.target_q_deg)}
            # The pendant slider slows the move; stretch the deadline with it.
            fraction = fresh.get("speed_fraction")
            scale = max(1.0 if fraction is None else fraction, lim.min_speed_fraction)
            deadline = self.clock() + plan.timeout_s / scale
            with self._dispatch_lock:
                # A STOP that got here first: nothing is sent.
                self._require_claim_and_unlatched(claim_token)
                dispatched = True  # even a thrown SDK error may have reached the arm
                accepted = self._dispatch(plan)
            if accepted is not True:
                raise RuntimeError("The controller did not accept the move")
            final, peak_force = self._monitor(plan, fresh, deadline, claim_token)
            return self._result(plan, request, final, started, moved=True, peak_force=peak_force)
        except Exception as exc:
            if not dispatched:
                raise
            self._fault = str(exc)
            self._cancelled.set()
            stop_error = self._stop(kind)
            raise MotionFailed(str(exc), stop_error=stop_error) from exc
        finally:
            self.active = None
            self._lock.release()

    def _result(self, plan, request, sample, started, *, moved, peak_force):
        return {
            "request_id": str(request.request_id or uuid4()),
            "completed": True,
            "moved": moved,
            "kind": plan.kind,
            "speed": plan.speed,
            "target_joints_deg": list(plan.target_q_deg),
            "target_tcp_mm_rpy_deg": tcp_to_mm_deg(plan.target_pose) if plan.target_pose else None,
            "measured_joints_deg": list(sample["joints_deg"]),
            "measured_tcp_mm_rpy_deg": tcp_to_mm_deg(sample["tcp_pose"]),
            "peak_force_change_n": round(peak_force, 2),
            "path_samples_checked": plan.samples,
            "elapsed_s": round(self.clock() - started, 3),
        }
