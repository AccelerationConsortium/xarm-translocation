"""Machine-local configuration and explicit model capabilities."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from .drivers.joint_step import JointStepLimits
from .drivers.robotiq import GripperSettings
from .drivers.ur_motion import MotionLimits

MODELS = {
    "xarm5": {
        "driver": "xarm",
        "joints": 5,
        "generation": "xarm",
        "implementation": "legacy",
    },
    "ur3e": {
        "driver": "ur",
        "joints": 6,
        "generation": "e_series",
        "implementation": "observe",
    },
    "ur5e": {
        "driver": "ur",
        "joints": 6,
        "generation": "e_series",
        "implementation": "observe",
    },
    "ur5_cb3": {
        "driver": "ur",
        "joints": 6,
        "generation": "cb3",
        "implementation": "observe",
    },
    "mg400": {
        "driver": "mg400",
        "joints": 4,
        "generation": "mg400",
        "implementation": "planned",
    },
}


class TrajectoryConfig(BaseModel):
    """Streamed joint trajectories (ur_rtde servoJ), the xArm's HTTP contract
    (src/docs/UR5E_TRAJECTORY_PLAN.md). Off unless enabled, and only with a
    motion block: joint bounds, speed and acceleration come from there. Rates
    and servoJ defaults are the stage 0b measurements on pc-05.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)
    enabled: bool = False
    # Stage 0b (2026-10-11): no late tick at 125 or 250 Hz; 250 Hz gains only
    # ~6 ms of lag, so 125 is the default.
    rate_hz: float = Field(default=125, ge=50, le=250, allow_inf_nan=False)
    max_rate_hz: float = Field(default=250, ge=50, le=250, allow_inf_nan=False)
    # Controller-side servoj smoothing (ur_rtde ranges). The arm lags by
    # about the lookahead plus 10 ms; tracking was the same at every pair
    # tried, so the shortest lookahead wins (0.03/1000: 36-42 ms lag).
    lookahead_s: float = Field(default=0.03, ge=0.03, le=0.2, allow_inf_nan=False)
    gain: float = Field(default=1000, ge=100, le=2000, allow_inf_nan=False)
    start_tolerance_deg: float = Field(default=0.5, gt=0, le=5, allow_inf_nan=False)
    max_duration_s: float = Field(default=600, gt=0, le=3600, allow_inf_nan=False)
    max_points: int = Field(default=60000, ge=2, le=200000)
    max_points_per_chunk: int = Field(default=5000, ge=1, le=200000)
    # Planning resolution of the workspace check: controller forward
    # kinematics wherever any joint has moved this far, at most max_fk_checks.
    path_check_step_deg: float = Field(default=2.0, ge=0.5, le=10, allow_inf_nan=False)
    max_fk_checks: int = Field(default=3000, ge=10, le=20000)
    # Session records, relative to the config file.
    log_dir: str = Field(default="logs/trajectory", min_length=1)

    @model_validator(mode="after")
    def coherent(self):
        if self.rate_hz > self.max_rate_hz:
            raise ValueError("rate_hz must not exceed max_rate_hz")
        if self.max_points_per_chunk > self.max_points:
            raise ValueError("max_points_per_chunk must not exceed max_points")
        return self


class ControlSettings(BaseModel):
    """Explicit commissioning inputs for the control routes.

    Nothing here is guessed: every joint bound, the stop deceleration and the
    commissioning reference come from the local config. The dashboard edge's
    shared secret comes from the environment. Who may command the robot is not
    configured here: the central edge decides (Caddy forward_auth with
    ``/auth/verify?equipment=<key>``, and the dashboard's per-equipment check),
    and the device trusts the identity it vouches for.
    The arm is driven by at most one of joint_step (single tiny steps, the
    first commissioning primitive) or motion (joint and Cartesian moves and
    jogs, ur_motion.py). Both are optional so the gripper can be commissioned
    on its own; without either there is no arm session (/connect) or motion.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)
    joint_step: JointStepLimits | None = None
    motion: MotionLimits | None = None
    # Dedicated receive stream for control feedback; the UI poll is too old.
    feedback_frequency_hz: float = Field(default=125, ge=50, le=500, allow_inf_nan=False)
    # ur_rtde communication watchdog: the controller stops the control script
    # when kicks stop arriving at this rate (e.g. a frozen or dead service).
    watchdog_hz: float = Field(default=10, ge=1, le=50, allow_inf_nan=False)
    # JSON-lines audit of every control event; relative to the config file.
    audit_file: str | None = None
    trajectory: TrajectoryConfig | None = None

    @model_validator(mode="before")
    @classmethod
    def no_local_operator_list(cls, data):
        # Named, not left to extra="forbid": a config that still carries the
        # old list must say why it no longer loads.
        if isinstance(data, dict) and "authorized_operators" in data:
            raise ValueError(
                "authorized_operators was removed: the central edge authorizes "
                "operators (grant a role on this equipment in the auth roster) "
                "- delete the key from the local config"
            )
        return data

    @model_validator(mode="after")
    def one_arm_mode(self):
        if self.joint_step is not None and self.motion is not None:
            raise ValueError("Configure joint_step or motion for the arm, not both")
        if self.trajectory is not None and self.trajectory.enabled:
            if self.motion is None:
                raise ValueError("trajectory needs the motion block (its joint bounds, speed and acceleration)")
            # A STOP during a trajectory is a servoStop, which holds the SDK
            # lock while the arm brakes; the watchdog kicker (every half
            # period) must still get in before the controller's deadline.
            braking_s = self.motion.max_joint_speed_deg_s / self.motion.stop_joint_decel_deg_s2
            if braking_s >= 1 / (2 * self.watchdog_hz):
                raise ValueError(
                    f"with trajectories, watchdog_hz must be below {1 / (2 * braking_s):.2f}: a STOP "
                    f"brakes for up to {braking_s:.2f} s while the watchdog cannot be kicked"
                )
        return self

    @property
    def arm(self):
        """The arm limits in force (joint_step or motion), or None."""
        return self.joint_step or self.motion


class LabCameraSettings(BaseModel):
    """The bench's network PTZ camera, as registered on the lab dashboard.

    Read through the dashboard's open /api/equipment snapshot; PTZ and preset
    recalls go through its audited control passthrough carrying the
    operator's own credential, so no camera credential lives here.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)
    # Dashboard (aggregator) origin, not the camera gateway, which binds to
    # loopback on the dashboard host.
    dashboard_base_url: str = Field(pattern=r"^https?://[^\s/?#]+$")
    camera_id: str = Field(pattern=r"^[a-z][a-z0-9_]{0,63}$")
    # Lens id from the camera's details.lenses (e.g. "wide" / "tele"); the
    # panel starts on it and offers a switch. None: the camera's first lens.
    lens: str | None = Field(default=None, pattern=r"^[A-Za-z0-9_-]{1,32}$")
    request_timeout_s: float = Field(default=5, ge=0.5, le=15, allow_inf_nan=False)


class StereoCameraSettings(BaseModel):
    """Panel metadata for one camera-service RealSense alias."""

    model_config = ConfigDict(extra="forbid", frozen=True)
    id: str = Field(pattern=r"^[a-z][a-z0-9_]{0,31}$")
    label: str = Field(min_length=1, max_length=100)
    short_label: str | None = Field(default=None, min_length=1, max_length=24)
    mount: dict[str, str] = Field(default_factory=dict, max_length=8)


class CameraServiceSettings(BaseModel):
    """RealSense cameras owned by the standalone SDL camera service.

    service_file names the private {"url", "token", "cameras"} JSON the xArm
    reads from XARM_CAMERA_SERVICE_CONFIG, relative to this config; the token
    never sits in this file. cameras must list exactly the ids it names.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)
    service_file: str = Field(min_length=1)
    cameras: tuple[StereoCameraSettings, ...] = Field(min_length=1, max_length=8)

    @field_validator("cameras")
    @classmethod
    def unique_ids(cls, cameras):
        if len({camera.id for camera in cameras}) != len(cameras):
            raise ValueError("Camera ids must be unique")
        return cameras


class Settings(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    equipment_id: str = Field(
        default="robot_motion_prototype", pattern=r"^[a-z][a-z0-9_]{0,63}$"
    )
    equipment_name: str = Field(default="Robot Motion", min_length=1, max_length=100)
    driver: Literal["none", "ur", "mg400"] = "none"
    model: Literal["ur3e", "ur5e", "ur5_cb3", "mg400"] | None = None
    robot_host: str | None = Field(default=None, min_length=1, max_length=253)
    observe: bool = False
    # Opt in explicitly; existing deployments keep their Dashboard-only reads.
    ur_transport: Literal["dashboard", "rtde"] = "dashboard"
    poll_interval_s: float = Field(default=10, ge=5, le=300, allow_inf_nan=False)
    # RTDE transport only: joints, TCP and TCP force are sampled this often for
    # /status (the Dashboard poll above stays slow).
    telemetry_interval_s: float = Field(default=0.5, ge=0.1, le=10, allow_inf_nan=False)
    timeout_s: float = Field(default=2, ge=0.1, le=5, allow_inf_nan=False)
    graph_file: str | None = None
    # Physical control cannot be switched on by a UI toggle or a lone flag:
    # it needs this flag AND a complete control block AND RTDE observation of
    # the same robot. Even then only the moves its limits allow exist.
    control_enabled: bool = False
    control: ControlSettings | None = None
    # Optional cameras for the shared panel's Tapo and Stereo tiles. Looking
    # is not actuation: neither block touches the robot or its control gates.
    lab_camera: LabCameraSettings | None = None
    camera_service: CameraServiceSettings | None = None
    # Tool-flange gripper. Status is read (GET only) whenever the robot is
    # observed; commands exist only under control_enabled.
    gripper: GripperSettings | None = None

    @model_validator(mode="after")
    def coherent(self):
        if self.control_enabled:
            if self.control is None:
                raise ValueError("control_enabled requires an explicit control block")
            if not (self.driver == "ur" and self.observe and self.ur_transport == "rtde"):
                raise ValueError("Control requires RTDE observation of the same UR robot")
            if self.control.arm is None and self.gripper is None:
                raise ValueError("control_enabled needs joint_step or motion limits, or a gripper, to control")
        if self.gripper is not None and not (self.driver == "ur" and self.observe):
            raise ValueError("A gripper needs an observed UR robot (it is reached through its controller)")
        if self.ur_transport == "rtde" and self.driver != "ur":
            raise ValueError("RTDE transport requires a UR driver")
        if self.driver == "none":
            if self.model is not None or self.observe or self.robot_host is not None:
                raise ValueError("Unconfigured mode cannot select or observe a robot")
        elif self.model is None or MODELS[self.model]["driver"] != self.driver:
            raise ValueError("Driver and robot model do not match")
        if self.observe and (self.driver != "ur" or not self.robot_host):
            raise ValueError("Observation requires an explicit UR model and robot_host")
        return self


def load_settings(path: Path | None) -> Settings:
    if path is None:
        return Settings()
    settings = Settings.model_validate(json.loads(path.read_text(encoding="utf-8-sig")))

    def beside_config(value):
        candidate = Path(value)
        if not candidate.is_absolute():
            candidate = path.parent / candidate
        return str(candidate.resolve())

    if settings.graph_file:
        settings = settings.model_copy(update={"graph_file": beside_config(settings.graph_file)})
    if settings.control and settings.control.audit_file:
        control = settings.control.model_copy(
            update={"audit_file": beside_config(settings.control.audit_file)}
        )
        settings = settings.model_copy(update={"control": control})
    if settings.control and settings.control.trajectory:
        trajectory = settings.control.trajectory
        trajectory = trajectory.model_copy(update={"log_dir": beside_config(trajectory.log_dir)})
        control = settings.control.model_copy(update={"trajectory": trajectory})
        settings = settings.model_copy(update={"control": control})
    if settings.camera_service:
        service = settings.camera_service.model_copy(
            update={"service_file": beside_config(settings.camera_service.service_file)}
        )
        settings = settings.model_copy(update={"camera_service": service})
    return settings
