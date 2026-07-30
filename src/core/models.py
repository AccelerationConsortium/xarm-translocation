"""STATUS_SPEC v1.2 Pydantic types for xarm-translocation.

Wire-contract types are imported from the shared ``sdl-lab-contract`` package
and re-exported. No device-specific detail models are needed beyond the
contract types.
"""

from __future__ import annotations

from datetime import datetime

from pydantic import BaseModel, Field

from sdl_lab_contract import (
    ClaimedBy,
    ClaimRejection,
    ClaimRequest,
    ClaimResponse,
    ComponentStatus,
    EquipmentKind,
    EquipmentState,
    EquipmentStatus,
    ErrorInfo,
    ErrorSeverity,
    HealthResponse,
    MetricValue,
    ProbeResponse,
)

PROTOCOL_VERSION = "1.2"
