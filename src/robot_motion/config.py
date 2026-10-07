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


class ControlSettings(BaseModel):
    """Explicit commissioning inputs for the control routes.

    Nothing here is guessed: the operator allowlist, every joint bound, the
    stop deceleration and the commissioning reference come from the local
    config. The dashboard edge's shared secret comes from the environment.
    The arm is driven by at most one of joint_step (single tiny steps, the
    first commissioning primitive) or motion (joint and Cartesian moves and
    jogs, ur_motion.py). Both are optional so the gripper can be commissioned
    on its own; without either there is no arm session (/connect) or motion.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)
    authorized_operators: tuple[str, ...] = Field(min_length=1, max_length=50)
    joint_step: JointStepLimits | None = None
    motion: MotionLimits | None = None
    # Dedicated receive stream for control feedback; the UI poll is too old.
    feedback_frequency_hz: float = Field(default=125, ge=50, le=500, allow_inf_nan=False)
    # ur_rtde communication watchdog: the controller stops the control script
    # when kicks stop arriving at this rate (e.g. a frozen or dead service).
    watchdog_hz: float = Field(default=10, ge=1, le=50, allow_inf_nan=False)
    # JSON-lines audit of every control event; relative to the config file.
    audit_file: str | None = None

    @field_validator("authorized_operators")
    @classmethod
    def email_identities(cls, operators):
        normalized = tuple(sorted({o.strip().lower() for o in operators}))
        if any(not o or "@" not in o or " " in o for o in normalized):
            raise ValueError("Authorized operators are e-mail identities")
        return normalized

    @model_validator(mode="after")
    def one_arm_mode(self):
        if self.joint_step is not None and self.motion is not None:
            raise ValueError("Configure joint_step or motion for the arm, not both")
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
    if settings.camera_service:
        service = settings.camera_service.model_copy(
            update={"service_file": beside_config(settings.camera_service.service_file)}
        )
        settings = settings.model_copy(update={"camera_service": service})
    return settings
