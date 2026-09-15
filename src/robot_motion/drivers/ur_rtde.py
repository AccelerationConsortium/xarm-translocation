"""LLE URArm telemetry plus the existing fixed Dashboard status queries."""

import logging

from sdl_lab_contract import ComponentStatus

from .lle_rtde import URArm
from .ur import URObserver

log = logging.getLogger(__name__)


class URRTDEObserver:
    def __init__(self, settings, *, arm=None, dashboard=None):
        self.dashboard = dashboard if dashboard is not None else URObserver(settings)
        self.arm = (
            arm
            if arm is not None
            else URArm(
                settings.robot_host,
                timeout=settings.timeout_s,
            )
        )

    def read(self):
        # Retain established safety and program-state interpretation. A missing
        # telemetry stream is not a robot fault or evidence that motion is safe.
        result = self.dashboard.read()
        try:
            telemetry = self.arm.read()
            result["components"]["telemetry"] = ComponentStatus(
                connected=True, state="receiving"
            )
        except Exception:
            log.exception("RTDE receive-only telemetry unavailable")
            telemetry = {"valid": False, "source": "rtde_receive"}
            result["components"]["telemetry"] = ComponentStatus(
                connected=False,
                state="unknown",
                message="RTDE telemetry unavailable; see service log",
            )
        result["details"]["telemetry"] = telemetry
        return result

    def close(self):
        self.arm.disconnect()
