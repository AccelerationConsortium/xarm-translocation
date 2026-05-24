"""Lab equipment status spec v1.1.

This module is a vendored copy of the unified status contract defined in the
ac-organic-lab monorepo (``docs/STATUS_SPEC.md``). It MUST stay in sync with
that document until a shared ``lab-status-contract`` Python package is
published; once it is, replace this file with::

    from lab_status_contract import (
        EquipmentStatus, ProbeResponse, HealthResponse, ...
    )

Conformance: xarm-translocation REST API conforms to lab status spec v1.1
(read-only baseline + claim protocol). ``allowed_actions`` is populated
when the motion-graph layer is in STRICT mode (derived from the current
node's outgoing edges). Claim/heartbeat/release endpoints are exposed
under ``/control/*`` in advisory mode — existing ``/move/*``, ``/gripper/*``
etc. do not yet require ``X-Claim-Token`` (a future hardening step can
flip on enforcement once workflow callers have adopted the SDK's
``ClaimManager``).
"""

from __future__ import annotations

from datetime import datetime
from typing import Any, Literal

from pydantic import BaseModel, Field

PROTOCOL_VERSION = "1.1"


EquipmentKind = Literal[
    "solid_doser",
    "liquid_handler",
    "press",
    "fume_hood",
    "robot_arm",
    "environmental_sensor",
    "hplc",
    "plate_reader",
    "plate_sealer",
    "plate_stacker",
    "other",
]

EquipmentState = Literal[
    "ready",          # initialized, idle, can accept commands
    "busy",           # performing an operation
    "requires_init",  # service up but hardware not initialized
    "degraded",       # running but a sub-component is unhealthy
    "dry_run",        # simulation mode, no hardware connected
    "error",          # hardware reported an error
    "e_stop",         # emergency stopped
    "unknown",        # state cannot be determined
]

ErrorSeverity = Literal["info", "warning", "error", "critical"]


class ComponentStatus(BaseModel):
    connected: bool
    state: str
    message: str | None = None
    last_event_at: datetime | None = None


class MetricValue(BaseModel):
    value: float | int | str | bool
    unit: str | None = None
    timestamp: datetime | None = None


class ErrorInfo(BaseModel):
    code: str | None = None
    message: str
    severity: ErrorSeverity
    timestamp: datetime


class EquipmentStatus(BaseModel):
    """Unified equipment status envelope (spec v1.0).

    The :attr:`allowed_actions` field is an optional v1.1 forward-compat
    hook: a v1.0 device may leave it empty (the SDK then falls back to
    catalog-declared ``requires_states``); a future v1.1 migration will
    populate it with skill names the device will currently honor on
    ``/control/*``.
    """

    protocol_version: str = PROTOCOL_VERSION

    # Identity
    equipment_id: str
    equipment_name: str
    equipment_kind: EquipmentKind
    equipment_version: str | None = None
    host: str | None = None  # local hostname only

    # Operational state
    equipment_status: EquipmentState
    message: str | None = None
    required_actions: list[str] = Field(default_factory=list)
    allowed_actions: list[str] = Field(default_factory=list)

    # Timing
    device_time: datetime
    uptime_seconds: float | None = None

    # Sub-equipment / measurements
    components: dict[str, ComponentStatus] = Field(default_factory=dict)
    metrics: dict[str, MetricValue] = Field(default_factory=dict)
    last_error: ErrorInfo | None = None

    # Free-form per-equipment data; safe to display in a debug/details panel.
    details: dict[str, Any] = Field(default_factory=dict)


class ProbeResponse(BaseModel):
    """Body of ``GET /`` -- the cheapest possible identity probe."""

    equipment_id: str
    equipment_name: str
    protocol_version: str = PROTOCOL_VERSION


class HealthResponse(BaseModel):
    """Body of ``GET /health`` -- service liveness."""

    status: Literal["healthy"] = "healthy"


# ── v1.1: claim protocol ─────────────────────────────────────────────


class ClaimedBy(BaseModel):
    """Identity of the current claim holder, surfaced on /status.details
    so every reader sees who controls the device without a side-trip."""

    session_id: str
    owner: str
    expires_at: datetime


class ClaimRequest(BaseModel):
    """Body of ``POST /control/claim``."""

    owner: str = Field(description="Human or agent identifier; surfaced in details.claimed_by")
    session_id: str = Field(description="Opaque per-session id; UUID recommended")
    ttl_s: float = Field(default=30.0, ge=1.0, le=300.0, description="Claim lifetime in seconds; device may clamp")


class ClaimResponse(BaseModel):
    """Success body of ``POST /control/claim``."""

    claim_token: str
    heartbeat_interval_s: float = Field(description="Caller MUST heartbeat more often than this")
    expires_at: datetime = Field(description="Absolute UTC; claim dies at this time without a heartbeat")


class ClaimRejection(BaseModel):
    """Body of HTTP 409 / 423 when /control/claim is refused because
    another session already holds the claim."""

    detail: str
    claimed_by: ClaimedBy | None = None
    retry_after_s: float | None = None
