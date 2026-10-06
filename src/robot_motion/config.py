"""Machine-local configuration and explicit model capabilities."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from .drivers.joint_step import JointStepLimits

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
    """Explicit commissioning inputs for the joint-step control routes.

    Nothing here is guessed: the operator allowlist, every joint bound, the
    stop deceleration and the commissioning reference come from the local
    config. The dashboard edge's shared secret comes from the environment.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)
    authorized_operators: tuple[str, ...] = Field(min_length=1, max_length=50)
    joint_step: JointStepLimits
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
    timeout_s: float = Field(default=2, ge=0.1, le=5, allow_inf_nan=False)
    graph_file: str | None = None
    # Physical control cannot be switched on by a UI toggle or a lone flag:
    # it needs this flag AND a complete control block AND RTDE observation of
    # the same robot. Even then only bounded single-joint steps exist.
    control_enabled: bool = False
    control: ControlSettings | None = None

    @model_validator(mode="after")
    def coherent(self):
        if self.control_enabled:
            if self.control is None:
                raise ValueError("control_enabled requires an explicit control block")
            if not (self.driver == "ur" and self.observe and self.ur_transport == "rtde"):
                raise ValueError("Control requires RTDE observation of the same UR robot")
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
    return settings
