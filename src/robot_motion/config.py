"""Machine-local configuration and explicit model capabilities."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

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
    # This first release cannot be switched into physical control by a UI
    # toggle or config typo. A reviewed controller integration is still needed.
    control_enabled: Literal[False] = False

    @model_validator(mode="after")
    def coherent(self):
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
    if settings.graph_file:
        graph_path = Path(settings.graph_file)
        if not graph_path.is_absolute():
            graph_path = path.parent / graph_path
        settings = settings.model_copy(update={"graph_file": str(graph_path.resolve())})
    return settings
