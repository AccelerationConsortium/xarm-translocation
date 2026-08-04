#!/usr/bin/env python3
"""
FastAPI Server for xArm Translocation Control

This module provides a REST API wrapper around the XArmController class
to enable web-based control and monitoring of xArm robots.
"""

# TODO: planning to implement DI/DO for safety light and additional e-stop

import asyncio
import hmac
import json
import logging
import os
from collections import deque
from contextlib import asynccontextmanager
from typing import Dict, List, Optional, Any
from datetime import datetime, timezone
from fastapi import Depends, FastAPI, HTTPException, BackgroundTasks, Header, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
import uvicorn

try:
    from .xarm_controller import XArmController, SafetyLevel, ComponentState
    from .xarm_utils import load_config
    from .models import (
        ClaimRequest,
        ClaimResponse,
        ClaimRejection,
        EquipmentStatus,
        HealthResponse,
        ProbeResponse,
        PROTOCOL_VERSION,
    )
    from .status_builder import (
        EQUIPMENT_ID,
        EQUIPMENT_NAME,
        build_status,
        build_telemetry,
    )
    from .motion_graph import (
        EdgeNotAllowedError, GraphError, GraphMode, GripperTransitionError,
        NoPathError, RecoveryMismatch, UnknownNodeError,
    )
    from .claims import ClaimConflict, InvalidClaimToken
    from .camera_tracker import CameraTracker
    from . import assistant_actions
    from . import assistant_llm
except ImportError:
    from core.xarm_controller import XArmController, SafetyLevel, ComponentState
    from core.xarm_utils import load_config
    from core.models import (
        ClaimRequest,
        ClaimResponse,
        ClaimRejection,
        EquipmentStatus,
        HealthResponse,
        ProbeResponse,
        PROTOCOL_VERSION,
    )
    from core.status_builder import (
        EQUIPMENT_ID,
        EQUIPMENT_NAME,
        build_status,
        build_telemetry,
    )
    from core.motion_graph import (
        EdgeNotAllowedError, GraphError, GraphMode, GripperTransitionError,
        NoPathError, RecoveryMismatch, UnknownNodeError,
    )
    from core.claims import ClaimConflict, InvalidClaimToken
    from core.camera_tracker import CameraTracker
    from core import assistant_actions
    from core import assistant_llm

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Failures that happen *inside* the WebSocket broadcast path are logged here,
# NOT through `logger`. `logger` has WebSocketLogHandler attached, so logging a
# broadcast failure through it would re-queue the message for broadcast, which
# re-fails, which logs again -- a self-amplifying loop that spams every client
# and starves the event loop. This logger has no such handler; it still reaches
# the root stderr handler, so the failure is recorded in the service log.
_ws_internal_logger = logging.getLogger("xarm.ws_internal")

# Add WebSocket log handler (will be set up after ConnectionManager is ready)
ws_handler = None

# Global controller instance
controller: Optional[XArmController] = None

# WebSocket connections for real-time updates
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def send_personal_message(self, message: str, websocket: WebSocket):
        await websocket.send_text(message)

    async def broadcast(self, message: str):
        # Iterate over a snapshot: send_text awaits, so other coroutines
        # (connect/disconnect) may mutate active_connections mid-loop.
        dead: List[WebSocket] = []
        for connection in list(self.active_connections):
            try:
                await connection.send_text(message)
            except Exception as exc:
                # Use the broadcast-internal logger, never `logger`, to avoid
                # the feedback loop documented on _ws_internal_logger.
                _ws_internal_logger.warning(
                    f"Dropping unreachable WebSocket client: {exc!r}"
                )
                dead.append(connection)
        # Prune dead connections so we stop retrying (and re-failing) them every
        # broadcast. Without this, one closed browser tab generates an endless
        # stream of "Error broadcasting message" errors.
        for connection in dead:
            self.disconnect(connection)

manager = ConnectionManager()

# Custom logging handler to broadcast logs to WebSocket clients
class WebSocketLogHandler(logging.Handler):
    # Bound the queue so that if broadcasting falls behind (or every client is
    # momentarily unreachable) it can never grow without limit. Oldest entries
    # are dropped first -- log streaming to a browser is best-effort.
    MAX_QUEUE = 200

    def __init__(self):
        super().__init__()
        self.setLevel(logging.INFO)
        formatter = logging.Formatter('%(levelname)s: %(message)s')
        self.setFormatter(formatter)
        self.log_queue: deque = deque(maxlen=self.MAX_QUEUE)

    def emit(self, record):
        try:
            msg = self.format(record)
            log_type = 'error' if record.levelno >= logging.ERROR else 'warning' if record.levelno >= logging.WARNING else 'info'
            self.log_queue.append({
                'type': 'log',
                'log_message': msg,
                'log_type': log_type,
                'timestamp': record.created,
            })
        except Exception:
            # Never let a logging failure propagate into application code.
            pass

# Pydantic models for request/response
class ConnectionRequest(BaseModel):
    """Request model for establishing a connection to the controller."""
    profile_name: Optional[str] = Field(default=None, description="Name of the connection profile to use.")
    host: Optional[str] = Field(default=None, description="IP address of the robot. Overrides profile.")
    model: Optional[int] = Field(default=None, description="Robot model: 5, 6, 7. Overrides profile.")
    gripper_type: Optional[str] = Field(default=None, description="Installed gripper type, e.g. bio_gen2.")
    safety_level: str = Field(default="MEDIUM", description="Set the safety validation level: LOW, MEDIUM, HIGH.")

    def get_safety_level_enum(self) -> SafetyLevel:
        """Convert string safety level to enum"""
        level_map = {
            "LOW": SafetyLevel.LOW,
            "MEDIUM": SafetyLevel.MEDIUM, 
            "HIGH": SafetyLevel.HIGH
        }
        return level_map.get(self.safety_level.upper(), SafetyLevel.MEDIUM)

class PositionRequest(BaseModel):
    """Request model for Cartesian position movement."""
    x: float = Field(description="X coordinate in mm")
    y: float = Field(description="Y coordinate in mm")
    z: float = Field(description="Z coordinate in mm")
    roll: Optional[float] = Field(default=None, description="Roll angle in degrees")
    pitch: Optional[float] = Field(default=None, description="Pitch angle in degrees")
    yaw: Optional[float] = Field(default=None, description="Yaw angle in degrees")
    speed: Optional[float] = Field(default=None, description="Movement speed (validated by safety level)")
    check_collision: bool = Field(default=True, description="Perform collision checking before movement.")
    wait: bool = Field(default=True, description="Wait for movement to complete.")

class JointRequest(BaseModel):
    """Request model for joint angle movement."""
    angles: List[float] = Field(description="List of joint angles in degrees")
    speed: Optional[float] = Field(default=None, description="Movement speed (validated by safety level)")
    acceleration: Optional[float] = Field(default=None, description="Movement acceleration (validated by safety level)")
    check_collision: bool = Field(default=True, description="Perform collision checking before movement.")
    wait: bool = Field(default=True, description="Wait for movement to complete.")

class RelativeRequest(BaseModel):
    """Request model for relative Cartesian movement."""
    dx: float = Field(default=0, description="Delta X in mm")
    dy: float = Field(default=0, description="Delta Y in mm")
    dz: float = Field(default=0, description="Delta Z in mm")
    droll: float = Field(default=0, description="Delta roll in degrees")
    dpitch: float = Field(default=0, description="Delta pitch in degrees")
    dyaw: float = Field(default=0, description="Delta yaw in degrees")
    speed: Optional[float] = Field(default=None, description="Movement speed (validated by safety level)")

class LocationRequest(BaseModel):
    """Request model for moving to a named location."""
    location_name: str = Field(description="Name of the location defined in joint_config.yaml")
    speed: Optional[float] = Field(default=None, description="Movement speed (validated by safety level)")

class TrackRequest(BaseModel):
    """Request model for linear track movement."""
    position: float = Field(description="Target position for the linear track in mm")
    speed: Optional[float] = Field(default=None, description="Movement speed for the track (validated by safety level)")
    wait: bool = Field(default=True, description="Wait for movement to complete.")

class TrackLocationRequest(BaseModel):
    """Request model for moving linear track to a named location."""
    location_name: str = Field(description="Name of the location from linear_track_config.yaml")
    speed: Optional[float] = Field(default=None, description="Movement speed for the track (validated by safety level)")
    wait: bool = Field(default=True, description="Wait for movement to complete.")

class GripperRequest(BaseModel):
    """Request model for gripper operations."""
    speed: Optional[float] = Field(default=None, description="Gripper speed (1-5000)")
    force: Optional[float] = Field(default=None, description="Gripper force, when supported")
    wait: bool = Field(default=True, description="Wait for operation to complete.")

class GripperStrokeRequest(BaseModel):
    """Request model for gripper stroke/position control."""
    stroke: float = Field(description="Target gripper stroke/position")
    speed: Optional[float] = Field(default=None, description="Gripper movement speed")
    force: Optional[float] = Field(default=None, description="Gripper force, when supported")
    wait: bool = Field(default=True, description="Wait for operation to complete.")

class GripperForceRequest(BaseModel):
    """Request model for setting gripper force."""
    force: float = Field(description="Target gripper force")

class VelocityRequest(BaseModel):
    """Request model for Cartesian velocity control."""
    vx: float = Field(default=0, description="Velocity in X direction (mm/s)")
    vy: float = Field(default=0, description="Velocity in Y direction (mm/s)")
    vz: float = Field(default=0, description="Velocity in Z direction (mm/s)")
    vroll: float = Field(default=0, description="Angular velocity around X axis (deg/s)")
    vpitch: float = Field(default=0, description="Angular velocity around Y axis (deg/s)")
    vyaw: float = Field(default=0, description="Angular velocity around Z axis (deg/s)")

class ComponentRequest(BaseModel):
    """Request model for enabling/disabling a component."""
    component: str = Field(description="Component to manage ('gripper', 'track', or 'force_torque')")

class ForceTorqueCalibrationRequest(BaseModel):
    """Request model for force torque sensor calibration."""
    samples: Optional[int] = Field(default=None, description="Number of calibration samples")
    delay: Optional[float] = Field(default=None, description="Delay between samples in seconds")

class ForceTorqueMovementRequest(BaseModel):
    """Request model for force-controlled movement."""
    direction: List[float] = Field(description="Direction vector [x, y, z] (normalized)")
    force_threshold: Optional[float] = Field(default=None, description="Force threshold in Newtons")
    speed: Optional[float] = Field(default=None, description="Movement speed in mm/s")
    timeout: float = Field(default=30.0, description="Maximum time to wait in seconds")

class JointTorqueMovementRequest(BaseModel):
    """Request model for torque-controlled joint movement."""
    joint_id: int = Field(description="Joint number (1-7)")
    target_angle: float = Field(description="Target angle in degrees")
    torque_threshold: Optional[float] = Field(default=None, description="Torque threshold in Nm")
    speed: Optional[float] = Field(default=None, description="Movement speed in deg/s")
    timeout: float = Field(default=30.0, description="Maximum time to wait in seconds")

class PlateLinearRequest(BaseModel):
    """Request model for linear movement from current position to target."""
    target_location: str = Field(description="Name of the target location from joint_config.yaml")
    speed: Optional[float] = Field(default=None, description="Movement speed (validated by safety level)")


class GraphModeRequest(BaseModel):
    """Request model for switching motion-graph enforcement mode."""
    mode: str = Field(description="One of: 'off', 'advisory', 'strict'")


class GraphRecordRequest(BaseModel):
    """Optional overrides for the edge being recorded from last_transition."""
    mode: Optional[str] = Field(
        default=None,
        description="'linear' or 'joint'; defaults to the mode the last move used",
    )
    speed: Optional[float] = Field(
        default=None, description="Override edge speed; defaults to the speed used"
    )
    comment: Optional[str] = Field(default=None, description="Free-text comment")
    preconditions: Optional[List[str]] = Field(
        default=None, description="List of named preconditions"
    )


class GraphRecoverRequest(BaseModel):
    """Body of POST /control/graph/recover_to (Phase 4)."""
    node_id: str = Field(description="Graph node id the operator declares as current")
    force: bool = Field(
        default=False,
        description=(
            "Skip the nearest-node sanity check. Use when the operator has "
            "verified position by other means (e.g., cartesian-dict presets "
            "that the joint-distance algo can't score)."
        ),
    )
    gripper_state: Optional[str] = Field(
        default=None,
        description=(
            "Optionally declare the gripper leaf (catalog state name, e.g. "
            "'empty' or 'grip_120'); must be allowed at the node. When "
            "omitted, the current commanded stroke is kept."
        ),
    )


class GraphGripperRequest(BaseModel):
    """Body of POST /control/graph/gripper — change the gripper state
    while parked at the current node (the only graph-sanctioned way to
    grip/release/narrow; the stroke never changes during arm motion)."""
    state: str = Field(description="Target catalog gripper state, e.g. 'grip_120'")


class GraphMoveToRequest(BaseModel):
    """Body of POST /control/graph/move_to.

    Wraps /move/location for graph-aware callers: takes a node id and
    looks up the underlying arm-pose preset name. Convenient for the
    web UI's reachable-node buttons since node ids and preset names
    can differ (e.g. node 'uplc_draw_approach' has arm 'uplc_draw_home').
    """
    node_id: str = Field(description="Graph node id to move to")
    speed: Optional[float] = Field(default=None, description="Movement speed (may be capped by edge.speed in STRICT)")


class GraphTravelToRequest(BaseModel):
    """Body of POST /control/graph/travel_to.

    Multi-hop counterpart of GraphMoveToRequest: the target may be any
    node reachable through the graph with the current gripper state,
    not just an adjacent one. The device plans a shortest hop path and
    executes it hop-by-hop.
    """
    node_id: str = Field(description="Graph node id to travel to (any reachable node)")
    speed: Optional[float] = Field(default=None, description="Per-hop movement speed (each hop may be capped by its edge.speed)")


class AssistantMessage(BaseModel):
    """One prior conversation turn passed back for light multi-turn context."""
    role: str = Field(description="'user' or 'assistant'")
    content: str = Field(description="The turn's plain text")


class AssistantPlanRequest(BaseModel):
    """Body of POST /assistant/plan.

    ``message`` is the operator's free-text request. ``history`` is an
    optional list of prior turns (user/assistant text) for context. The
    endpoint is read-only: it interprets the request via OpenRouter
    and computes a motion-graph step list WITHOUT moving the robot.
    """
    message: str = Field(description="Natural-language robot-motion request")
    history: Optional[List[AssistantMessage]] = Field(
        default=None, description="Prior conversation turns for context"
    )


class AssistantStep(BaseModel):
    """One concrete step from a plan, replayed to /assistant/execute.

    ``kind`` is 'move' (drive to graph node ``to``) or 'gripper' (change
    gripper to catalog ``state`` while parked). Extra fields the planner
    attaches (mode/speed) are advisory only — the controller re-derives
    and re-validates everything under STRICT enforcement at execution.
    """
    kind: str = Field(description="'move' or 'gripper'")
    to: Optional[str] = Field(default=None, description="Target node id (move steps)")
    state: Optional[str] = Field(default=None, description="Target gripper state (gripper steps)")
    mode: Optional[str] = Field(default=None, description="Advisory move mode")
    speed: Optional[float] = Field(default=None, description="Advisory move speed")


class AssistantExecuteRequest(BaseModel):
    """Body of POST /assistant/execute.

    Carries the exact step list the operator confirmed from a prior
    /assistant/plan preview. Claim-gated; each step is executed through
    the same STRICT-enforced controller machinery as the manual graph
    endpoints, so a tampered step list can never bypass the interlocks.
    """
    steps: List[AssistantStep] = Field(description="Ordered steps to execute")
    interpretation: Optional[str] = Field(
        default=None, description="Human-readable label echoed into progress logs"
    )
    speed: Optional[float] = Field(
        default=None, description="Optional per-move speed override (capped by edge.speed)"
    )


class GraphEdgeUpdateRequest(BaseModel):
    """Body of POST /control/graph/edge — in-place edit of an existing edge.

    Only mode and speed are editable from the viewer; node poses, grip/
    release actions, and topology stay with the control panel / record flow.
    A None field is left unchanged.
    """
    from_node: str = Field(description="Edge source node id")
    to_node: str = Field(description="Edge target node id")
    mode: Optional[str] = Field(
        default=None, description="'joint' or 'linear'; None leaves it unchanged"
    )
    speed: Optional[float] = Field(
        default=None, gt=0, description="Edge speed (>0); None leaves it unchanged"
    )


class GraphEdgeDeleteRequest(BaseModel):
    """Body of POST /control/graph/edge/delete — remove an existing edge.

    Identified by its ordered (from, to) pair, which is unique. Only the edge
    (a motion) is removed; both endpoint nodes stay. The candidate is validated
    and the file's comments preserved (ruamel round-trip) before the write.
    """
    from_node: str = Field(description="Edge source node id")
    to_node: str = Field(description="Edge target node id")


class GraphLayoutModel(BaseModel):
    """Body of POST /graph/layout — the viewer's saved geometry.

    Pure presentation data (node positions, which stations are expanded, the
    pan/zoom), stored device-side in src/settings/motion_graph_layout.json so every PC
    that connects sees the same arrangement. Deliberately separate from
    motion_graph.yaml (the validated interlock topology) and NOT claim-gated —
    rearranging the map moves no hardware. Unknown node ids are harmless (the
    viewer only applies positions for nodes that exist).
    """
    positions: Dict[str, Dict[str, float]] = Field(
        default_factory=dict, description="node id -> {x, y}"
    )
    expanded: Dict[str, bool] = Field(
        default_factory=dict, description="station -> true when its nodes are shown"
    )
    pan: Optional[Dict[str, float]] = Field(default=None, description="{x, y} viewport pan")
    zoom: Optional[float] = Field(default=None, description="viewport zoom factor")


class GraphNodeCreateRequest(BaseModel):
    """Body of POST /control/graph/node — add a new node to the graph.

    A node is an ARM POSITION: a named arm pose (from joint_config.yaml)
    at a rail location. The gripper is modelled as leaves on the node:
    ``gripper_states`` you may occupy there and ``gripper_transitions``
    allowed while parked there. Edges are added separately (record /
    edge editor). The loader validates the result.
    """
    id: str = Field(description="Unique node id (convention: the arm pose name)")
    arm: str = Field(description="Arm pose name from joint_config.yaml")
    rail: str = Field(description="Rail location name from linear_track_config.yaml")
    tags: Optional[List[str]] = Field(default=None, description="Optional tags; first tag groups/colours the node")
    gripper_states: Optional[List[str]] = Field(
        default=None,
        description="Catalog states occupiable at this node (default: ['empty'])",
    )
    gripper_transitions: Optional[List[List[str]]] = Field(
        default=None,
        description="[from, to] state changes allowed while parked here",
    )


class GraphEdgeCreateRequest(BaseModel):
    """Body of POST /control/graph/edge/create — add a new edge (motion)
    between two existing nodes.
    """
    from_node: str = Field(description="Source node id (must exist)")
    to_node: str = Field(description="Target node id (must exist)")
    mode: str = Field(description="'joint' or 'linear'")
    speed: Optional[float] = Field(default=None, gt=0, description="Edge speed (>0)")
    preconditions: Optional[List[str]] = Field(default=None, description="Named preconditions")
    comment: Optional[str] = Field(default=None, description="Free-text comment")


class EnforcementRequest(BaseModel):
    """Body of POST /control/claim/enforce (Phase 5 toggle)."""
    enabled: bool = Field(description="true to require X-Claim-Token on mutating endpoints")


class ManualModeRequest(BaseModel):
    """Body of POST /robot/manual."""
    enable: bool = Field(
        description=(
            "true to release the joint brakes for hand-guiding (drag/teach), "
            "false to return to position control"
        )
    )


# Phase 5: FastAPI dependency that enforces X-Claim-Token on mutating
# endpoints when claim enforcement is enabled. No-op otherwise (and
# no-op when no claim is held — the cooperative interpretation, see
# ClaimManager.verify_token docstring). Defined ABOVE the endpoints
# so the @app.post(..., dependencies=[Depends(require_claim)]) refs
# resolve at module import.
async def require_claim(x_claim_token: Optional[str] = Header(default=None)):
    c = get_controller()
    try:
        c.claim_manager.verify_token(x_claim_token)
    except InvalidClaimToken:
        holder = c.claim_manager.claimed_by()
        raise HTTPException(
            status_code=423,
            detail={
                "error": "claim_required",
                "claimed_by": (
                    {
                        "session_id": holder["session_id"],
                        "owner": holder["owner"],
                    } if holder else None
                ),
                "hint": (
                    "this endpoint is gated by an active claim; "
                    "POST /control/claim to acquire it, or include the "
                    "holder's X-Claim-Token header"
                ),
            },
        )


# Login gate for the mutating endpoints that are NOT claim-gated: /connect,
# /disconnect, and the safety floor (/move/stop, /clear/errors). Requires a
# verified identity (cookie -> /auth/me, or X-Api-Key -> /auth/verify), NOT
# the claim token — so any signed-in operator (or a keyed workflow) can stop
# the arm or connect/disconnect without first taking the claim. No-op unless
# XARM_REQUIRE_LOGIN is on and the sidecar is configured. Fails closed
# (sidecar unreachable -> 503). The verified email is stashed on
# request.state for audit logging. Motion endpoints deliberately use
# require_claim instead (the token already proves a logged-in holder).
async def require_login(request: Request):
    if not (REQUIRE_LOGIN and AUTH_SIDECAR_URL):
        return
    try:
        email = await _resolve_identity(request)
    except Exception:
        raise HTTPException(
            status_code=503,
            detail="Auth service unreachable; cannot verify identity.",
        )
    if not email:
        raise HTTPException(
            status_code=401,
            detail={"error": "login_required", "hint": _LOGIN_HINT},
        )
    request.state.identity_email = email


# Application lifespan management
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    logger.info("Starting xArm API Server")
    
    # Start background tasks
    log_task = asyncio.create_task(broadcast_logs())
    telemetry_task = asyncio.create_task(telemetry_loop())

    yield

    # Shutdown
    log_task.cancel()
    telemetry_task.cancel()
    global controller
    if controller:
        logger.info("Disconnecting from robot...")
        controller.disconnect()
    logger.info("xArm API Server shutdown complete")

# Create FastAPI app.
#
# NOTE on serving under the single Caddy edge (docs/SINGLE_EDGE_SSO_PLAN.md):
# we deliberately do NOT set a FastAPI `root_path`. The edge strips the public
# /xarm5 prefix before proxying, so this app always sees its bare paths
# (/status, /web/, /ws, /control/*) exactly as on direct :8000 access — and
# under Starlette 1.x an app-level root_path relocates the StaticFiles mount to
# /xarm5/web/, which would 404 once the edge has stripped the prefix. Base-path
# awareness lives entirely on the client instead: the panel's assets are
# relative and its JS derives the /xarm5 prefix from window.location (see
# main.js / graph.js / auth.js). Keep the edge strip-only (no /web rewrite).
app = FastAPI(
    title="xArm Translocation API",
    description="REST API for controlling xArm robots with gripper and linear track support",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify actual origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Set up WebSocket logging
ws_handler = WebSocketLogHandler()
logger.addHandler(ws_handler)

# Mount static files. The browser UI lives at /web/ -- the bare HTML page is
# no longer served from /, since GET / is the STATUS_SPEC v1.0 probe endpoint.
WEB_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "web")
try:
    if os.path.exists(WEB_DIR):
        app.mount(
            "/web",
            StaticFiles(directory=WEB_DIR, html=True),
            name="web",
        )
        # Keep /static/* for legacy bookmarks; harmless duplicate mount.
        app.mount("/static", StaticFiles(directory=WEB_DIR), name="static")
except Exception as e:
    logger.warning(f"Could not mount static files: {e}")


# ---------------------------------------------------------------------------
# Auth banner proxy (SDL2 Auth sidecar, ac_auth)
#
# The /web/ UI shows a sign-in banner backed by the lab's central auth
# sidecar (email one-time-code -> opaque session cookie). The browser can't
# talk to the sidecar directly: it serves no CORS headers and its session
# cookie is SameSite=Lax, so cross-origin fetches from this UI would carry
# neither. These endpoints proxy the four calls the banner needs on the
# SAME origin as the UI, holding the session token in a cookie scoped to
# this device's origin. The sidecar remains the single source of truth —
# tokens are validated server-side there on every /auth/me.
#
# Auth here is ADVISORY (a banner + claim-owner identity), not an access
# gate: the claim protocol stays the single gate on motion. Signing in
# lets the UI stamp `owner: <email>` into /control/claim, so the
# dashboard's claimed_by and the audit trail show a real person instead
# of the anonymous fallback.
#
# Disabled unless XARM_AUTH_URL is set (e.g. http://100.64.254.6:8009).
# ---------------------------------------------------------------------------

AUTH_SIDECAR_URL = os.environ.get("XARM_AUTH_URL", "").strip().rstrip("/")
AUTH_COOKIE_NAME = os.environ.get("XARM_AUTH_COOKIE_NAME", "ac_auth_session")
_AUTH_TIMEOUT_S = 5.0
# The device UI is plain http over the Tailnet (same posture as the
# dashboard), so the cookie can't be Secure. HttpOnly + SameSite=Lax.
_AUTH_COOKIE_MAX_AGE_S = 12 * 3600


def _env_truthy(val: Optional[str]) -> bool:
    return str(val or "").strip().lower() in {"1", "true", "yes", "on"}


# When set, the session cookie is scoped to this parent domain so a single
# sign-in covers every *.<domain> lab UI (deploy with tail6a1dd7.ts.net).
# logout must delete with the SAME Domain or the browser won't clear it.
# None -> host-only cookie (current single-UI behaviour).
AUTH_COOKIE_DOMAIN = os.environ.get("XARM_AUTH_COOKIE_DOMAIN", "").strip() or None

# Opt-in server-side login gate: when truthy AND the auth sidecar is
# configured, every mutating endpoint requires a verified identity (session
# cookie -> the sidecar's /auth/me, or X-Api-Key -> /auth/verify). It gates
# POST /control/claim (and stamps the verified email as the claim owner) and,
# via the require_login dependency, the endpoints that aren't claim-gated:
# /connect, /disconnect, and the safety floor /move/stop + /clear/errors.
# Motion endpoints stay claim-token-gated — the token already proves the
# holder logged in when they claimed, so they need no extra sidecar hop.
# Off by default so unconfigured/dev deployments are unchanged. The older
# XARM_REQUIRE_LOGIN_FOR_CLAIM name is still honored as an alias.
REQUIRE_LOGIN = _env_truthy(
    os.environ.get("XARM_REQUIRE_LOGIN")
    or os.environ.get("XARM_REQUIRE_LOGIN_FOR_CLAIM")
)


# ---------------------------------------------------------------------------
# Single-edge SSO — trust the identity injected by the Caddy edge.
#
# When the panel is reached through the lab's single Caddy edge
# (docs/SINGLE_EDGE_SSO_PLAN.md), the edge has already authenticated the human
# (forward_auth -> ac_auth /auth/verify) and injects the verified identity as
# `X-Auth-User` / `X-Auth-Role`. The device then trusts that identity directly,
# so the user is NOT prompted for a second login inside the panel (that is the
# whole point of "one login for every lab UI").
#
# Trust boundary: the device is still directly reachable on the Tailnet, where
# any caller could forge `X-Auth-User`. So we trust the edge headers ONLY when
# they arrive with a matching shared secret in `X-Edge-Auth` (the edge sets
# `header_up X-Edge-Auth <secret>`). A direct caller hitting :8000 carries no
# valid secret, so its identity headers are ignored and it falls back to the
# cookie / X-Api-Key path. Unset (XARM_EDGE_SHARED_SECRET absent) => edge trust
# is fully disabled and the headers are ignored, i.e. today's behaviour.
#
# This is the interim (shared-secret) trust from the SSO plan. When Phase 2
# lands (loopback-bind or Tailscale-ACL :8000 so ONLY the edge can reach it),
# the secret becomes belt-and-suspenders rather than the sole guard.
EDGE_USER_HEADER = "X-Auth-User"
EDGE_ROLE_HEADER = "X-Auth-Role"
EDGE_TRUST_HEADER = "X-Edge-Auth"
EDGE_SHARED_SECRET = os.environ.get("XARM_EDGE_SHARED_SECRET", "").strip() or None

# Shared 401 hint so the claim gate and the require_login dependency speak
# with one voice.
_LOGIN_HINT = (
    "sign in via /web/ (email one-time code) or present an X-Api-Key header "
    "to use the controls on this device"
)


def _auth_sidecar_call(method: str, path: str, body: Optional[dict] = None,
                       cookie_token: Optional[str] = None, *,
                       api_key: Optional[str] = None):
    """Blocking sidecar round-trip (run via asyncio.to_thread).

    Returns (status_code, parsed_json | None, session_token | None) where
    session_token is extracted from the sidecar's Set-Cookie, if any.
    ``cookie_token`` forwards the human session cookie; ``api_key`` forwards
    a machine principal's key as ``X-Api-Key``. Raises OSError/URLError on
    transport failure.
    """
    import urllib.request
    from http.cookies import SimpleCookie

    request = urllib.request.Request(
        f"{AUTH_SIDECAR_URL}{path}",
        data=json.dumps(body).encode("utf-8") if body is not None else None,
        headers={
            "Content-Type": "application/json",
            **({"Cookie": f"{AUTH_COOKIE_NAME}={cookie_token}"} if cookie_token else {}),
            **({"X-Api-Key": api_key} if api_key else {}),
        },
        method=method,
    )
    try:
        resp = urllib.request.urlopen(request, timeout=_AUTH_TIMEOUT_S)
    except urllib.error.HTTPError as exc:
        resp = exc  # non-2xx still carries a JSON body (FastAPI detail)
    with resp:
        status = resp.getcode() if hasattr(resp, "getcode") else resp.code
        try:
            payload = json.loads(resp.read().decode("utf-8"))
        except Exception:
            payload = None
        token = None
        for header in resp.headers.get_all("Set-Cookie") or []:
            jar = SimpleCookie()
            jar.load(header)
            if AUTH_COOKIE_NAME in jar:
                token = jar[AUTH_COOKIE_NAME].value
        return status, payload, token


def _auth_passthrough(status: int, payload):
    """Mirror the sidecar's response (status + body) to the browser."""
    return JSONResponse(payload if payload is not None else {}, status_code=status)


def _identity_email(payload) -> Optional[str]:
    """Pull the verified email out of a sidecar identity payload.

    Tolerates both the /auth/me shape ({identity: {email, role}}) and a
    flatter {email: ...} shape a machine-principal /auth/verify may use.
    Returns None when no email is present (e.g. anonymous /auth/me).
    """
    if not isinstance(payload, dict):
        return None
    ident = payload.get("identity")
    if isinstance(ident, dict) and ident.get("email"):
        return ident["email"]
    if payload.get("email"):
        return payload["email"]
    return None


def _edge_identity(request: Request) -> Optional[dict]:
    """Identity injected by the trusted Caddy edge, or None.

    Returns ``{"email": ..., "role": ...}`` when the request carries an
    ``X-Auth-User`` accompanied by an ``X-Edge-Auth`` that matches the
    configured shared secret; otherwise None. When no secret is configured
    (``XARM_EDGE_SHARED_SECRET`` unset) edge trust is disabled and this always
    returns None, so a directly-reachable device never trusts client-supplied
    identity headers. Constant-time secret compare to avoid a timing oracle.
    """
    if not EDGE_SHARED_SECRET:
        return None
    presented = request.headers.get(EDGE_TRUST_HEADER)
    if not presented or not hmac.compare_digest(presented, EDGE_SHARED_SECRET):
        return None
    email = (request.headers.get(EDGE_USER_HEADER) or "").strip()
    if not email:
        return None
    role = (request.headers.get(EDGE_ROLE_HEADER) or "").strip() or None
    return {"email": email, "role": role}


async def _resolve_identity(request: Request) -> Optional[str]:
    """Resolve a verified principal email for a control request.

    Checks, in order: the trusted edge identity (``X-Auth-User`` +
    ``X-Edge-Auth``, see ``_edge_identity``), then an ``X-Api-Key`` header
    (machine principals / future SDK workflows -> the sidecar's
    GET /auth/verify), then the ``ac_auth_session`` cookie (humans ->
    GET /auth/me). Returns the verified email, or None when no credential was
    presented / it didn't validate.

    Fails closed: the sidecar round-trip runs off the event loop and any
    transport exception is propagated to the caller (which maps it to 503),
    never silently allowed. (The edge path is header-only — no sidecar hop —
    since the edge already verified the session.)
    """
    edge = _edge_identity(request)
    if edge:
        return edge["email"]
    api_key = request.headers.get("X-Api-Key")
    if api_key:
        status, payload, _ = await asyncio.to_thread(
            _auth_sidecar_call, "GET", "/auth/verify", None, None, api_key=api_key,
        )
        return _identity_email(payload) if status == 200 else None
    token = request.cookies.get(AUTH_COOKIE_NAME)
    if token:
        status, payload, _ = await asyncio.to_thread(
            _auth_sidecar_call, "GET", "/auth/me", None, token,
        )
        return _identity_email(payload) if status == 200 else None
    return None


class AuthEmailIn(BaseModel):
    email: str = Field(min_length=3, max_length=254)


class AuthVerifyIn(BaseModel):
    email: str = Field(min_length=3, max_length=254)
    code: str = Field(min_length=1, max_length=16)


@app.get("/auth/config", tags=["auth"])
async def auth_config() -> dict:
    """Banner bootstrap: is the auth integration configured on this device?

    Enabled when either the auth sidecar is configured (direct on-host login)
    or edge trust is configured (login happens once at the edge). Behind the
    edge, /auth/me resolves the injected identity so the banner shows the
    signed-in user with no second login form.
    """
    return {"enabled": bool(AUTH_SIDECAR_URL) or bool(EDGE_SHARED_SECRET)}


@app.get("/auth/me", tags=["auth"])
async def auth_me(request: Request):
    # Reached through the trusted edge: the human already signed in there.
    # Report that identity so the banner shows them signed-in and never renders
    # its own login form (the "single source of login" outcome). `via: edge`
    # lets the banner hide its sign-out button — you sign out at the edge.
    edge = _edge_identity(request)
    if edge:
        return {
            "authenticated": True,
            "identity": {
                "email": edge["email"],
                "role": edge["role"] or "user",
                "via": "edge",
            },
        }
    if not AUTH_SIDECAR_URL:
        return {"authenticated": False, "identity": None}
    token = request.cookies.get(AUTH_COOKIE_NAME)
    if not token:
        return {"authenticated": False, "identity": None}
    try:
        status, payload, _ = await asyncio.to_thread(
            _auth_sidecar_call, "GET", "/auth/me", None, token
        )
    except Exception:
        raise HTTPException(status_code=502, detail="Auth service unreachable.")
    return _auth_passthrough(status, payload)


@app.get("/auth/users", tags=["auth"])
async def auth_users():
    """Active human accounts, for the banner's email picker."""
    if not AUTH_SIDECAR_URL:
        raise HTTPException(status_code=501, detail="Auth not configured on this device.")
    try:
        status, payload, _ = await asyncio.to_thread(
            _auth_sidecar_call, "GET", "/auth/users", None, None
        )
    except Exception:
        raise HTTPException(status_code=502, detail="Auth service unreachable.")
    return _auth_passthrough(status, payload)


@app.post("/auth/request-code", tags=["auth"])
async def auth_request_code(body: AuthEmailIn):
    if not AUTH_SIDECAR_URL:
        raise HTTPException(status_code=501, detail="Auth not configured on this device.")
    try:
        status, payload, _ = await asyncio.to_thread(
            _auth_sidecar_call, "POST", "/auth/request-code", {"email": body.email}, None
        )
    except Exception:
        raise HTTPException(status_code=502, detail="Auth service unreachable.")
    return _auth_passthrough(status, payload)


@app.post("/auth/verify-code", tags=["auth"])
async def auth_verify_code(body: AuthVerifyIn, response: Response):
    if not AUTH_SIDECAR_URL:
        raise HTTPException(status_code=501, detail="Auth not configured on this device.")
    try:
        status, payload, token = await asyncio.to_thread(
            _auth_sidecar_call, "POST", "/auth/verify-code",
            {"email": body.email, "code": body.code}, None,
        )
    except Exception:
        raise HTTPException(status_code=502, detail="Auth service unreachable.")
    if status == 200 and token:
        # Re-issue the sidecar's session token as a cookie on OUR origin;
        # the sidecar's own Set-Cookie would be scoped to :8009 and Secure.
        response.set_cookie(
            AUTH_COOKIE_NAME, token, max_age=_AUTH_COOKIE_MAX_AGE_S,
            httponly=True, samesite="lax", path="/",
            domain=AUTH_COOKIE_DOMAIN,
        )
        return payload if payload is not None else {"ok": True}
    return _auth_passthrough(status, payload)


@app.post("/auth/logout", tags=["auth"])
async def auth_logout(request: Request, response: Response):
    token = request.cookies.get(AUTH_COOKIE_NAME)
    if AUTH_SIDECAR_URL and token:
        try:
            await asyncio.to_thread(
                _auth_sidecar_call, "POST", "/auth/logout", None, token
            )
        except Exception:
            pass  # revoking best-effort; the local cookie still dies
    # Must match the Domain used on set_cookie or the browser keeps it.
    response.delete_cookie(AUTH_COOKIE_NAME, path="/", domain=AUTH_COOKIE_DOMAIN)
    return {"ok": True}


# ---------------------------------------------------------------------------
# STATUS_SPEC v1.0 endpoints (GET /, GET /health, GET /status)
# ---------------------------------------------------------------------------


@app.get("/", response_model=ProbeResponse, tags=["spec"])
async def probe() -> ProbeResponse:
    """Cheapest possible identity probe. Always 200 unless the process is broken."""
    return ProbeResponse(
        equipment_id=EQUIPMENT_ID,
        equipment_name=EQUIPMENT_NAME,
        protocol_version=PROTOCOL_VERSION,
    )


@app.get("/health", response_model=HealthResponse, tags=["spec"])
async def health() -> HealthResponse:
    """Service liveness from the dashboard's perspective."""
    return HealthResponse()

# Periodic task to broadcast queued logs
async def broadcast_logs():
    """Periodically broadcast queued logs to WebSocket clients"""
    while True:
        try:
            queue = getattr(ws_handler, 'log_queue', None)
            if queue:
                # Drain via popleft so an append racing in from a logging
                # thread isn't silently dropped by a copy()+clear() window.
                # Snapshot the count first so a flood can't keep us here
                # forever (and starve the rest of the event loop).
                for _ in range(len(queue)):
                    try:
                        log_data = queue.popleft()
                    except IndexError:
                        break
                    await manager.broadcast(json.dumps(log_data))
        except Exception as e:
            print(f"Error broadcasting logs: {e}")

        await asyncio.sleep(0.5)  # Check every 500ms

# Start log broadcasting task
async def start_background_tasks():
    """Start background tasks for the application"""
    asyncio.create_task(broadcast_logs())

# Helper functions
def get_controller() -> XArmController:
    """Get the global controller instance"""
    global controller
    if not controller:
        raise HTTPException(status_code=400, detail="Robot not connected. Please connect first.")
    return controller

def reserve_motion() -> XArmController:
    """Claim the single motion slot, or refuse the request with HTTP 409.

    Only one motion may be in flight at a time: two overlapping commands
    to the same arm are a collision risk, and STATUS_SPEC v1.2 §2.3
    requires a device to refuse a second concurrent run.

    The slot is *reserved* here rather than inferred from the controller,
    because a motion endpoint accepts and returns before the arm has
    necessarily started moving — several schedule the SDK call as a
    background task. A caller firing two moves back to back would find the
    controller still idle on the second one. Reserving synchronously in the
    request handler closes that window: handlers and background tasks all
    run on the single event loop thread, so the check-and-set below is
    atomic (there is no await between the two).

    Every caller MUST release with ``exit_motion()`` in a ``finally`` —
    ``/status`` reports ``activity: running`` and withholds move actions
    for as long as the slot is held.

    Raises 409 rather than 412: this is a device-state conflict, not an
    unmet precondition the caller could have satisfied beforehand (spec
    §6.1 reserves 412 for the latter).
    """
    c = get_controller()
    if c._motion_in_progress:
        raise HTTPException(
            status_code=409,
            detail={
                "error": "motion_in_progress",
                "message": (
                    "A motion is already in flight. Wait for it to finish, "
                    "or POST /move/stop to abort it."
                ),
            },
        )
    c.enter_motion()
    return c


def create_error_response(message: str, status_code: int = 500) -> JSONResponse:
    """Create standardized error response"""
    return JSONResponse(
        status_code=status_code,
        content={"error": message, "timestamp": datetime.now().isoformat()}
    )

async def _safe_disconnect(ctrl: Optional["XArmController"]) -> None:
    """Disconnect a controller's SDK session, swallowing any error.

    Used on the /connect failure paths so a controller that failed to
    initialize still closes its sockets to the control box instead of
    leaking them (which would saturate the xArm's single control session).
    Runs the blocking SDK disconnect in a worker thread so it never stalls
    the event loop.
    """
    if ctrl is None:
        return
    try:
        await asyncio.to_thread(ctrl.disconnect)
    except Exception as exc:  # best-effort cleanup; never mask the original error
        logger.warning(f"Cleanup disconnect after failed connect raised: {exc}")

async def broadcast_status_update():
    """Broadcast a STATUS_SPEC v1.0 envelope to all connected WebSocket clients.

    The browser UI in ``src/web/main.js`` consumes the same shape as the
    HTTP ``GET /status`` response so push and poll are interchangeable.
    """
    try:
        envelope = build_status(controller)
        message = {
            "type": "status_update",
            "data": envelope.model_dump(mode="json"),
        }
        await manager.broadcast(json.dumps(message))
    except Exception as e:
        logger.error(f"Error broadcasting status: {e}")


async def broadcast_telemetry():
    """Broadcast a compact live-telemetry message to WebSocket clients.

    Smaller than the full ``status_update`` envelope and applied by the browser
    with a cheap field-diff, so the high-frequency push (telemetry_loop) stays
    light. Action-driven updates still use ``broadcast_status_update``.
    """
    try:
        message = {"type": "telemetry", "data": build_telemetry(controller)}
        await manager.broadcast(json.dumps(message))
    except Exception as e:
        logger.error(f"Error broadcasting telemetry: {e}")


# Live-telemetry push. Refreshes the cached joint/pose readings from the arm
# and pushes the status envelope to connected WebSocket clients at a fixed
# rate, so the UI shows genuinely live motion (including hand-guided manual
# mode) without the browser polling /status. The loop idles cheaply when the
# arm is down or no client is attached. Rate is configurable; 0 disables it.
TELEMETRY_HZ = float(os.environ.get("XARM_TELEMETRY_HZ", "10"))


async def telemetry_loop():
    interval = 1.0 / TELEMETRY_HZ if TELEMETRY_HZ > 0 else None
    if interval is None:
        logger.info("Telemetry loop disabled (XARM_TELEMETRY_HZ=0)")
        return
    logger.info(f"Telemetry loop running at {TELEMETRY_HZ} Hz")

    def _refresh(c):
        # Side-effecting *reads* only: pull live joint/pose (and track) into the
        # controller's cache that build_status() then reads. Runs in a worker
        # thread so the serial round-trips don't block the event loop.
        c._update_positions()
        if getattr(c, "enable_track", False):
            c._update_track_position()

    while True:
        try:
            await asyncio.sleep(interval)
            c = controller
            if c is None or not getattr(c, "is_alive", False):
                continue
            if not manager.active_connections:
                continue
            await asyncio.to_thread(_refresh, c)
            await broadcast_telemetry()
        except asyncio.CancelledError:
            break
        except Exception as e:
            logger.error(f"Telemetry loop error: {e}")
            # Back off so a persistent fault doesn't spin the loop hot.
            await asyncio.sleep(0.5)

# API Routes

@app.get("/api")
async def root():
    """Root endpoint with API information"""
    return {
        "message": "xArm Translocation API",
        "version": "1.0.0",
        "status": "running",
        "connected": controller is not None and controller.is_alive
    }

@app.get("/api/configurations")
async def get_configurations():
    """Scan and return available connection profiles from the main config file."""
    # Try multiple possible paths for main config
    possible_paths = [
        os.path.join('src', 'settings', 'xarm_config.yaml'),
        os.path.join('settings', 'xarm_config.yaml'),
        os.path.join(os.path.dirname(__file__), '..', 'settings', 'xarm_config.yaml')
    ]
    
    for config_path in possible_paths:
        resolved = os.path.abspath(config_path)
        if not os.path.exists(resolved):
            continue
        try:
            full_config = load_config(config_path)
            profiles = full_config.get('profiles', {})
            return sorted(list(profiles.keys()))
        except Exception as e:
            logger.error(f"Failed to read profiles from {config_path}: {e}")
            continue
    
    raise HTTPException(status_code=404, detail="Main xarm_config.yaml not found in any expected location.")


@app.post("/connect", dependencies=[Depends(require_login)])
async def connect_robot(request: ConnectionRequest, background_tasks: BackgroundTasks):
    """Connect to the robot controller.

    Initializes the ``XArmController`` against the configured profile and
    sets the initial safety level.
    """
    global controller
    
    if controller and controller.is_alive:
        raise HTTPException(status_code=400, detail="A robot is already connected. Please disconnect first.")
    
    try:
        # Create and initialize the controller instance
        controller = XArmController(
            profile_name=request.profile_name,
            host=request.host,
            model=request.model,
            gripper_type=request.gripper_type,
            safety_level=request.get_safety_level_enum()
        )

        # initialize() opens the SDK sockets and runs the connect/enable
        # handshake (retries + sleeps -- up to ~12 s). Run it in a worker
        # thread so /health, /status and STOP stay responsive while a slow
        # or failing connect is in progress.
        if await asyncio.to_thread(controller.initialize):
            background_tasks.add_task(broadcast_status_update)
            return {
                "message": "Successfully connected.",
                "connection_details": {
                    "host": controller.host,
                    "port": controller.xarm_config.get('port', 18333),
                    "profile_name": request.profile_name or 'custom',
                },
                "model": controller.model_name,
                "num_joints": controller.num_joints,
                "gripper_type": controller.gripper_type if hasattr(controller, 'gripper_type') else 'N/A',
                "gripper_config": getattr(controller, 'current_gripper_config', {}),
                "has_track": controller.has_track(),
                "component_states": controller.get_component_states(),
                "safety_level": controller.safety_level.name
            }
        else:
            # initialize() failed: tear down the half-open SDK connection so its
            # sockets to the control box (port 502 control + 30002 report) are
            # closed. The xArm grants a single control session; leaking a socket
            # on every failed connect saturates the box, after which every later
            # attempt fails the version handshake ("failed to check version,
            # close"). Just dropping the reference is not enough -- the SDK's
            # background threads keep the socket ESTABLISHED until disconnect().
            await _safe_disconnect(controller)
            controller = None
            raise HTTPException(status_code=500, detail="Failed to initialize robot connection. Check logs for details.")

    except HTTPException:
        # Already a structured error from the branch above; don't re-wrap it
        # (re-wrapping buried the real 500 detail inside a generic message).
        raise
    except Exception as e:
        await _safe_disconnect(controller)
        controller = None
        logger.error(f"Connection failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"An unexpected error occurred during connection: {e}")

@app.post("/disconnect", dependencies=[Depends(require_login)])
async def disconnect_robot():
    """Disconnect from the robot and ensure the state is cleaned up."""
    global controller
    
    connection_info = None
    if controller:
        # Capture connection info before disconnecting
        connection_info = {
            "host": controller.host,
            "port": controller.xarm_config.get('port', 18333),
            "profile_name": getattr(controller, 'profile_name', 'unknown')
        }
    
    message = "Robot was not connected."
    if controller:
        try:
            await asyncio.to_thread(controller.disconnect)
            message = f"Successfully disconnected from {connection_info['host']}:{connection_info['port']}"
        except Exception as e:
            logger.error(f"Disconnect failed: {e}", exc_info=True)
            # Still proceed to set controller to None
            message = f"Disconnected from {connection_info['host']}:{connection_info['port']} (with errors)"
        finally:
            controller = None
    
    # Broadcast the post-disconnect spec envelope so the UI re-syncs.
    await broadcast_status_update()
    
    return {
        "message": message,
        "connection_details": connection_info
    }

@app.get("/status", response_model=EquipmentStatus, tags=["spec"])
async def get_status() -> EquipmentStatus:
    """Return the spec v1.0 ``EquipmentStatus`` envelope.

    Side-effect-free: only cached controller state is read (see
    ``status_builder.build_status``). Always HTTP 200 unless the process
    itself is broken; ``requires_init``, ``error``, ``busy``, etc. are all
    *states*, not failures.
    """
    return build_status(controller)

@app.get("/positions")
async def get_all_positions():
    """Read-only snapshot of every position sensor: joints, Cartesian pose,
    linear track, and gripper — no movement is performed."""
    c = get_controller()
    has_track = c.has_track()

    # All four reads hit the SDK (get_servo_angle / get_position /
    # get_linear_track_pos / gripper query). Gather them in a single worker
    # thread so the serial round-trips don't block the event loop.
    def _read_all():
        return (
            c.get_current_joints(),
            c.get_current_position(),
            c.get_track_position() if has_track else None,
            c.get_gripper_position(),
        )

    joints_raw, cart_raw, track_pos, gripper_pos = await asyncio.to_thread(_read_all)

    # Joints
    joints = joints_raw[:c.num_joints] if joints_raw else None

    # Cartesian pose
    if cart_raw and len(cart_raw) >= 6:
        cartesian = {"x": cart_raw[0], "y": cart_raw[1], "z": cart_raw[2],
                     "roll": cart_raw[3], "pitch": cart_raw[4], "yaw": cart_raw[5]}
    else:
        cartesian = None

    # Linear track
    if has_track:
        track = {"available": True, "position": track_pos}
    else:
        track = {"available": False, "position": None}

    # Gripper
    gripper = {
        "available": gripper_pos is not None,
        "position": gripper_pos,
    }

    return {
        "joints": joints,
        "cartesian": cartesian,
        "track": track,
        "gripper": gripper,
    }

@app.get("/locations")
async def get_locations():
    """Get all named arm positions from the position config file."""
    try:
        # Try multiple possible paths for position config
        possible_paths = [
            os.path.join('src', 'settings', 'joint_config.yaml'),
            os.path.join('settings', 'joint_config.yaml'),
            os.path.join(os.path.dirname(__file__), '..', 'settings', 'joint_config.yaml')
        ]
        
        position_config = None
        for path in possible_paths:
            try:
                position_config = load_config(path)
                break
            except FileNotFoundError:
                continue
        
        if position_config:
            locations = list(position_config.get('positions', {}).keys())
            positions = position_config.get('positions', {})
        else:
            logger.warning("joint_config.yaml not found in any expected location, returning empty list.")
            locations = []
            positions = {}
        
        return {"locations": locations, "positions": positions}
    except Exception as e:
        logger.error(f"Get arm positions failed: {e}")
        raise HTTPException(status_code=500, detail=f"Get arm positions failed: {str(e)}")

# Movement endpoints
@app.post("/move/position", dependencies=[Depends(require_claim)])
async def move_to_position(request: PositionRequest, background_tasks: BackgroundTasks):
    """Move the robot to a specific Cartesian position.

    Returns 409 (motion_in_progress) when a motion is already in flight.
    """
    c = reserve_motion()

    async def move_task():
        # Run the blocking SDK call in a worker thread so the event loop
        # stays free to handle STOP and status polls while the arm moves.
        try:
            success = await asyncio.to_thread(
                c.move_to_position,
                x=request.x, y=request.y, z=request.z,
                roll=request.roll, pitch=request.pitch, yaw=request.yaw,
                speed=request.speed,
                check_collision=request.check_collision,
                wait=request.wait,
            )
            if not success:
                logger.error("Failed to move to position.")
        finally:
            c.exit_motion()
        await broadcast_status_update()

    background_tasks.add_task(move_task)
    return {"message": "Move to position command accepted."}

@app.post("/move/joints", dependencies=[Depends(require_claim)])
async def move_joints(request: JointRequest, background_tasks: BackgroundTasks):
    """Move the robot to a specific joint configuration.

    Returns 409 (motion_in_progress) when a motion is already in flight.
    """
    c = reserve_motion()

    async def move_task():
        try:
            success = await asyncio.to_thread(
                c.move_joints,
                angles=request.angles,
                speed=request.speed,
                acceleration=request.acceleration,
                check_collision=request.check_collision,
                wait=request.wait,
            )
            if not success:
                logger.error("Failed to move joints.")
        finally:
            c.exit_motion()
        await broadcast_status_update()

    background_tasks.add_task(move_task)
    return {"message": "Move joints command accepted."}

@app.post("/move/relative", dependencies=[Depends(require_claim)])
async def move_relative(request: RelativeRequest, background_tasks: BackgroundTasks):
    """Move the robot relative to its current position.

    Returns 409 (motion_in_progress) when a motion is already in flight.
    """
    c = reserve_motion()

    async def move_task():
        try:
            success = await asyncio.to_thread(
                c.move_relative,
                dx=request.dx, dy=request.dy, dz=request.dz,
                droll=request.droll, dpitch=request.dpitch, dyaw=request.dyaw,
                speed=request.speed,
            )
            if not success:
                logger.error("Failed to move relative.")
        finally:
            c.exit_motion()
        await broadcast_status_update()

    background_tasks.add_task(move_task)
    return {"message": "Move relative command accepted."}

@app.post("/move/location", dependencies=[Depends(require_claim)])
async def move_to_location(request: LocationRequest, background_tasks: BackgroundTasks):
    """Move the robot to a pre-defined named location.

    Runs synchronously via to_thread so a STRICT-mode rejection surfaces
    as HTTP 409 (rather than a silent background failure) and the caller
    learns the actual outcome. STOP remains responsive because the move
    runs in a worker thread, not on the event loop.

    Also returns 409 (motion_in_progress) when a motion is already in
    flight.
    """
    c = reserve_motion()

    try:
        success = await asyncio.to_thread(
            c.move_to_named_location,
            location_name=request.location_name,
            speed=request.speed,
        )
    except EdgeNotAllowedError as exc:
        raise HTTPException(
            status_code=409,
            detail={
                "error": "edge_not_allowed",
                "current_node": exc.current,
                "target": exc.target,
                "reason": exc.reason,
            },
        )
    finally:
        c.exit_motion()

    background_tasks.add_task(broadcast_status_update)
    if not success:
        logger.error(f"Failed to move to named location: {request.location_name}")
        raise HTTPException(
            status_code=500,
            detail=f"Move to '{request.location_name}' failed",
        )
    return {"message": f"Moved to '{request.location_name}'."}

@app.post("/move/home", dependencies=[Depends(require_claim)])
async def move_home(background_tasks: BackgroundTasks):
    """Move robot to home position.

    Returns 409 (motion_in_progress) when a motion is already in flight.
    """
    ctrl = reserve_motion()

    try:
        # go_home() blocks until the move completes; offload to a worker
        # thread so STOP requests can interrupt the event loop.
        result = await asyncio.to_thread(ctrl.go_home)

        if result:
            background_tasks.add_task(broadcast_status_update)
            return {
                "message": "Successfully moved to home position",
                "timestamp": datetime.now().isoformat()
            }
        else:
            raise HTTPException(status_code=500, detail="Home movement failed")

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Home movement failed: {e}")
        raise HTTPException(status_code=500, detail=f"Home movement failed: {str(e)}")
    finally:
        ctrl.exit_motion()

@app.post("/move/stop", dependencies=[Depends(require_login)])
async def stop_movement(request: Request, background_tasks: BackgroundTasks):
    """Stop all robot motion immediately.

    Login-gated (require_login) but NOT claim-gated: any signed-in operator
    can halt the arm without holding the claim. The hardware e-stop remains
    the credential-free backstop.
    """
    c = get_controller()

    try:
        # Execute stop immediately (not in background) for fastest response.
        # Run via to_thread so the SDK call doesn't block the event loop
        # if the underlying TCP/USB layer stalls.
        stopped = await asyncio.to_thread(c.stop_motion)
        if not stopped:
            raise HTTPException(status_code=500, detail="Stop command failed.")
        # Audit who halted the arm (identity stashed by require_login).
        actor = getattr(request.state, "identity_email", None) or "unauthenticated"
        logger.info(f"Stop command issued immediately by {actor}.")
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Stop command failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Stop command failed: {str(e)}")
    
    # Only use background task for status update
    async def status_update_task():
        await broadcast_status_update()
    
    background_tasks.add_task(status_update_task)
    return {"message": "Stop command executed immediately."}

@app.post("/clear/errors", dependencies=[Depends(require_login)])
async def clear_errors(background_tasks: BackgroundTasks):
    """Clear all robot errors and warnings"""
    ctrl = get_controller()
    
    try:
        result = await asyncio.to_thread(ctrl.clear_errors)
        
        if result:
            background_tasks.add_task(broadcast_status_update)
            return {
                "message": "All errors and warnings cleared successfully",
                "timestamp": datetime.now().isoformat()
            }
        else:
            raise HTTPException(status_code=500, detail="Failed to clear all errors")
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Clear errors failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Clear errors failed: {str(e)}")

@app.post("/robot/enable", dependencies=[Depends(require_claim)])
async def enable_robot():
    """Re-enable robot motion after emergency stop."""
    c = get_controller()

    def _reenable() -> int | None:
        if hasattr(c.arm, 'clean_error'):
            c.arm.clean_error()
        if hasattr(c.arm, 'clean_warn'):
            c.arm.clean_warn()
        code: int | None = None
        if hasattr(c.arm, 'motion_enable'):
            code = c.arm.motion_enable(enable=True)
        # The xArm SDK puts the arm in state 4 (stopped) after an
        # emergency_stop. Mode/state must be re-asserted before motion
        # commands will be honored again.
        if hasattr(c.arm, 'set_mode'):
            c.arm.set_mode(0)
        if hasattr(c.arm, 'set_state'):
            c.arm.set_state(0)
        return code

    result = await asyncio.to_thread(_reenable)
    if result not in (None, 0):
        logger.warning(f"Motion enable returned code: {result}")

    # Reset alive state
    c.alive = True
    c.states['arm'] = ComponentState.ENABLED
    logger.info("Robot motion re-enabled after emergency stop")

    await broadcast_status_update()
    return {"message": "Robot motion enabled successfully."}

@app.post("/robot/manual", dependencies=[Depends(require_claim)])
async def set_manual_mode(request: ManualModeRequest):
    """Toggle manual (drag/teach) mode -- mirrors the factory UI's Manual button.

    Manual mode (xArm SDK mode 2) releases the joint brakes so the arm can be
    moved by hand. Disabling returns to position control (mode 0). Run via
    to_thread so the SDK round-trip doesn't block the event loop.
    """
    c = get_controller()
    ok = await asyncio.to_thread(c.set_manual_mode, request.enable)
    if not ok:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to {'enable' if request.enable else 'disable'} manual mode.",
        )

    logger.info(f"Manual mode {'enabled' if request.enable else 'disabled'}")
    await broadcast_status_update()
    return {
        "message": f"Manual mode {'enabled' if request.enable else 'disabled'}.",
        "manual_mode": request.enable,
    }

@app.post("/component/enable", dependencies=[Depends(require_claim)])
async def enable_component(request: ComponentRequest):
    """Enable a specific component (gripper, track, or force_torque)."""
    c = get_controller()
    component = request.component.lower()
    success = False
    if component == 'gripper':
        success = await asyncio.to_thread(c.enable_gripper_component)
    elif component == 'track':
        success = await asyncio.to_thread(c.enable_track_component)
    elif component == 'force_torque':
        success = await asyncio.to_thread(c.enable_force_torque_sensor)
    else:
        raise HTTPException(status_code=400, detail="Invalid component specified. Use 'gripper', 'track', or 'force_torque'.")
    
    await broadcast_status_update()
    if success:
        return {"message": f"Component '{component}' enabled successfully."}
    else:
        raise HTTPException(status_code=500, detail=f"Failed to enable component '{component}'.")

@app.post("/component/disable", dependencies=[Depends(require_claim)])
async def disable_component(request: ComponentRequest):
    """Disable a specific component (gripper, track, or force_torque)."""
    c = get_controller()
    component = request.component.lower()
    success = False
    if component == 'gripper':
        success = await asyncio.to_thread(c.disable_gripper_component)
    elif component == 'track':
        success = await asyncio.to_thread(c.disable_track_component)
    elif component == 'force_torque':
        success = await asyncio.to_thread(c.disable_force_torque_sensor)
    else:
        raise HTTPException(status_code=400, detail="Invalid component specified. Use 'gripper', 'track', or 'force_torque'.")

    await broadcast_status_update()
    if success:
        return {"message": f"Component '{component}' disabled successfully."}
    else:
        raise HTTPException(status_code=500, detail=f"Failed to disable component '{component}'.")

@app.post("/velocity/cartesian", dependencies=[Depends(require_claim)])
async def set_cartesian_velocity(request: VelocityRequest):
    """Set the Cartesian velocity of the robot arm."""
    c = get_controller()
    velocities = [request.vx, request.vy, request.vz, request.vroll, request.vpitch, request.vyaw]

    if not await asyncio.to_thread(c.set_cartesian_velocity, velocities):
        raise HTTPException(status_code=500, detail="Failed to set Cartesian velocity.")
    
    return {"message": "Cartesian velocity set successfully."}

# Gripper endpoints
@app.post("/gripper/open", dependencies=[Depends(require_claim)])
async def open_gripper(request: Optional[GripperRequest] = None):
    """Open the attached gripper."""
    c = get_controller()
    request = request or GripperRequest()

    try:
        success = await asyncio.to_thread(
            c.open_gripper, speed=request.speed, force=request.force, wait=request.wait
        )
        await broadcast_status_update()
        if not success:
            raise HTTPException(status_code=500, detail="Failed to open gripper.")
        return {"message": "Open gripper command completed."}
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Open gripper failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Open gripper failed: {str(e)}")

@app.post("/gripper/close", dependencies=[Depends(require_claim)])
async def close_gripper(request: Optional[GripperRequest] = None):
    """Close the attached gripper."""
    c = get_controller()
    request = request or GripperRequest()

    try:
        success = await asyncio.to_thread(
            c.close_gripper, speed=request.speed, force=request.force, wait=request.wait
        )
        await broadcast_status_update()
        if not success:
            raise HTTPException(status_code=500, detail="Failed to close gripper.")
        return {"message": "Close gripper command completed."}
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Close gripper failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Close gripper failed: {str(e)}")

@app.post("/gripper/move/stroke", dependencies=[Depends(require_claim)])
async def move_gripper_stroke(request: GripperStrokeRequest):
    """Move gripper to a specific stroke position."""
    c = get_controller()

    try:
        success = await asyncio.to_thread(
            c.move_gripper_to_stroke,
            stroke=request.stroke,
            speed=request.speed,
            force=request.force,
            wait=request.wait,
        )
        await broadcast_status_update()
        if not success:
            raise HTTPException(status_code=500, detail=f"Failed to move gripper to stroke {request.stroke}.")
        return {"message": f"Move gripper to stroke {request.stroke} command completed."}
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Move gripper to stroke failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Move gripper to stroke failed: {str(e)}")

@app.post("/gripper/force", dependencies=[Depends(require_claim)])
async def set_gripper_force(request: GripperForceRequest):
    """Set gripping force for grippers that support force control."""
    c = get_controller()
    if not await asyncio.to_thread(c.set_gripper_force, request.force):
        raise HTTPException(status_code=500, detail="Failed to set gripper force.")
    await broadcast_status_update()
    return {"message": f"Gripper force set to {request.force}."}

@app.get("/gripper/position")
async def get_gripper_position():
    """Get gripper stroke/position when supported by the installed gripper."""
    c = get_controller()
    position = await asyncio.to_thread(c.get_gripper_position)
    if position is None:
        raise HTTPException(status_code=404, detail="Gripper position is not available.")
    return {"position": position}

# Linear track endpoints
@app.post("/track/move", dependencies=[Depends(require_claim)])
async def move_track(request: TrackRequest, background_tasks: BackgroundTasks):
    """Move the linear track to a specific position.

    Returns 409 (motion_in_progress) when a motion is already in flight.
    """
    c = reserve_motion()

    async def track_task():
        try:
            success = await asyncio.to_thread(
                c.move_track_to_position,
                position=request.position, speed=request.speed, wait=request.wait,
            )
            if not success:
                logger.error("Failed to move linear track.")
        finally:
            c.exit_motion()
        await broadcast_status_update()

    background_tasks.add_task(track_task)
    return {"message": "Move track command accepted."}

@app.post("/track/move/location", dependencies=[Depends(require_claim)])
async def move_track_to_location(request: TrackLocationRequest, background_tasks: BackgroundTasks):
    """Move the linear track to a pre-configured named location.

    Synchronous via to_thread (same rationale as /move/location). Returns
    409 (motion_in_progress) when a motion is already in flight.
    """
    c = reserve_motion()

    try:
        success = await asyncio.to_thread(
            c.move_track_to_named_location,
            location_name=request.location_name,
            speed=request.speed,
            wait=request.wait,
        )
    except EdgeNotAllowedError as exc:
        raise HTTPException(
            status_code=409,
            detail={
                "error": "edge_not_allowed",
                "current_node": exc.current,
                "target": exc.target,
                "reason": exc.reason,
            },
        )
    finally:
        c.exit_motion()

    background_tasks.add_task(broadcast_status_update)
    if not success:
        logger.error(f"Failed to move track to named location: {request.location_name}")
        raise HTTPException(
            status_code=500,
            detail=f"Track move to '{request.location_name}' failed",
        )
    return {"message": f"Moved track to '{request.location_name}'."}

@app.get("/track/position")
async def get_track_position():
    """Get current linear track position"""
    c = get_controller()
    
    if not c.has_track():
        raise HTTPException(status_code=400, detail="Linear track is not enabled.")
    return {"position": await asyncio.to_thread(c.get_track_position)}

@app.get("/track/locations")
async def get_track_locations():
    """Get a list of all available named locations for the linear track from its config file."""
    try:
        # Try multiple possible paths for track config
        possible_paths = [
            os.path.join('src', 'settings', 'linear_track_config.yaml'),
            os.path.join('settings', 'linear_track_config.yaml'),
            os.path.join(os.path.dirname(__file__), '..', 'settings', 'linear_track_config.yaml')
        ]
        
        track_config = None
        for path in possible_paths:
            try:
                track_config = load_config(path)
                break
            except FileNotFoundError:
                continue
        
        if track_config:
            locations = list(track_config.get('locations', {}).keys())
            positions = track_config.get('locations', {})
            return {"locations": locations, "positions": positions}
        else:
            logger.warning("linear_track_config.yaml not found in any expected location, returning empty list.")
            return {"locations": [], "positions": {}}
    except Exception as e:
        logger.error(f"Get track locations failed: {e}")
        raise HTTPException(status_code=500, detail=f"Get track locations failed: {str(e)}")

# Force Torque Sensor endpoints
@app.post("/force-torque/enable", dependencies=[Depends(require_claim)])
async def enable_force_torque_sensor():
    """Enable the 6-axis force torque sensor."""
    c = get_controller()
    
    if not c.has_force_torque_sensor():
        raise HTTPException(status_code=400, detail="Force torque sensor is not available or disabled in configuration.")

    success = await asyncio.to_thread(c.enable_force_torque_sensor)
    await broadcast_status_update()
    
    if success:
        return {"message": "Force torque sensor enabled successfully."}
    else:
        raise HTTPException(status_code=500, detail="Failed to enable force torque sensor.")

@app.post("/force-torque/disable", dependencies=[Depends(require_claim)])
async def disable_force_torque_sensor():
    """Disable the 6-axis force torque sensor."""
    c = get_controller()

    success = await asyncio.to_thread(c.disable_force_torque_sensor)
    await broadcast_status_update()

    if success:
        return {"message": "Force torque sensor disabled successfully."}
    else:
        raise HTTPException(status_code=500, detail="Failed to disable force torque sensor.")

@app.post("/force-torque/calibrate", dependencies=[Depends(require_claim)])
async def calibrate_force_torque_sensor(request: ForceTorqueCalibrationRequest, background_tasks: BackgroundTasks):
    """Calibrate the force torque sensor to zero."""
    c = get_controller()

    async def calibration_task():
        success = await asyncio.to_thread(
            c.calibrate_force_torque_sensor,
            samples=request.samples,
            delay=request.delay,
        )
        if not success:
            logger.error("Failed to calibrate force torque sensor.")
        await broadcast_status_update()

    background_tasks.add_task(calibration_task)
    return {"message": "Force torque sensor calibration started."}

@app.get("/force-torque/data")
async def get_force_torque_data():
    """Get current force torque sensor data."""
    c = get_controller()
    
    if not c.is_component_enabled('force_torque'):
        raise HTTPException(status_code=400, detail="Force torque sensor is not enabled.")

    data = await asyncio.to_thread(c.get_force_torque_data)
    if data is None:
        raise HTTPException(status_code=500, detail="Failed to get force torque data.")

    # magnitude/direction each re-read the sensor (get_ft_sensor_data); fetch
    # them in one worker hop so neither blocks the event loop.
    magnitude, direction = await asyncio.to_thread(
        lambda: (c.get_force_torque_magnitude(), c.get_force_torque_direction())
    )
    return {
        "data": data,
        "magnitude": magnitude,
        "direction": direction,
        "calibrated": c.force_torque_calibrated
    }

@app.get("/force-torque/status")
async def get_force_torque_status():
    """Get comprehensive force torque sensor status."""
    c = get_controller()
    
    return c.get_force_torque_status()

@app.post("/force-torque/check-safety", dependencies=[Depends(require_claim)])
async def check_force_torque_safety():
    """Check if force/torque exceeds safety thresholds and trigger alerts."""
    c = get_controller()
    
    if not c.is_component_enabled('force_torque'):
        raise HTTPException(status_code=400, detail="Force torque sensor is not enabled.")
    
    violation_detected = await asyncio.to_thread(c.check_force_torque_safety)

    return {
        "violation_detected": violation_detected,
        "message": "Safety check completed."
    }

@app.post("/force-torque/move-until-force", dependencies=[Depends(require_claim)])
async def move_until_force(request: ForceTorqueMovementRequest, background_tasks: BackgroundTasks):
    """Move in a linear direction until a force threshold is reached.

    Returns 409 (motion_in_progress) when a motion is already in flight.
    """
    c = reserve_motion()

    async def force_movement_task():
        try:
            success = await asyncio.to_thread(
                c.move_until_force,
                direction=request.direction,
                force_threshold=request.force_threshold,
                speed=request.speed,
                timeout=request.timeout,
            )
            if not success:
                logger.error("Force-controlled movement failed or timed out.")
        finally:
            c.exit_motion()
        await broadcast_status_update()

    background_tasks.add_task(force_movement_task)
    return {"message": "Force-controlled movement started."}

@app.post("/force-torque/move-joint-until-torque", dependencies=[Depends(require_claim)])
async def move_joint_until_torque(request: JointTorqueMovementRequest, background_tasks: BackgroundTasks):
    """Move a specific joint until a torque threshold is reached.

    Returns 409 (motion_in_progress) when a motion is already in flight.
    """
    c = reserve_motion()

    async def torque_movement_task():
        try:
            success = await asyncio.to_thread(
                c.move_joint_until_torque,
                joint_id=request.joint_id,
                target_angle=request.target_angle,
                torque_threshold=request.torque_threshold,
                speed=request.speed,
                timeout=request.timeout,
            )
            if not success:
                logger.error("Torque-controlled joint movement failed or timed out.")
        finally:
            c.exit_motion()
        await broadcast_status_update()

    background_tasks.add_task(torque_movement_task)
    return {"message": "Torque-controlled joint movement started."}

@app.post("/move/plate_linear", dependencies=[Depends(require_claim)])
async def move_plate_linear(request: PlateLinearRequest, background_tasks: BackgroundTasks):
    """Move linearly from current position to target with constant tool orientation.

    Returns 409 (motion_in_progress) when a motion is already in flight.
    """
    c = reserve_motion()

    async def plate_linear_task():
        try:
            success = await asyncio.to_thread(
                c.move_plate_linear,
                target_location=request.target_location,
                speed=request.speed,
            )
            if not success:
                logger.error(f"Failed to move linearly to {request.target_location}")
        finally:
            c.exit_motion()
        await broadcast_status_update()

    background_tasks.add_task(plate_linear_task)
    return {"message": f"Linear movement to '{request.target_location}' command accepted."}

# =============================================================================
# CLAIM PROTOCOL (STATUS_SPEC v1.1, Phase 3 — advisory)
# =============================================================================

@app.post("/control/claim", responses={409: {"model": ClaimRejection}})
async def acquire_claim(request: ClaimRequest, http_request: Request):
    """Take the cooperative claim on this device.

    Returns 200 + ClaimResponse on success. Returns 409 + ClaimRejection
    when another active session holds the claim. Idempotent for the same
    session_id (token is rotated, TTL refreshed).

    When XARM_REQUIRE_LOGIN_FOR_CLAIM is on (and the auth sidecar is
    configured), the caller must present a verified identity — a signed-in
    session cookie or an X-Api-Key — or the request is refused with 401.
    On success the verified email OVERRIDES the client-supplied owner, so
    details.claimed_by and the audit trail always name the real principal.
    heartbeat/release stay token-only (the token already proves the holder).
    """
    # Identity is hardware-independent, so the login gate runs BEFORE the
    # connection check: an unauthenticated caller gets a clean 401 and never
    # learns whether the arm is connected. Authenticated-but-disconnected
    # still falls through to get_controller()'s 400 ("connect first").
    owner = request.owner
    if REQUIRE_LOGIN and AUTH_SIDECAR_URL:
        try:
            email = await _resolve_identity(http_request)
        except Exception:
            # Fail closed: sidecar unreachable -> refuse, never silently allow.
            raise HTTPException(
                status_code=503,
                detail="Auth service unreachable; cannot verify identity for claim.",
            )
        if not email:
            raise HTTPException(
                status_code=401,
                detail={"error": "login_required", "hint": _LOGIN_HINT},
            )
        owner = email  # ignore client-supplied owner; keep client's session_id
    c = get_controller()
    try:
        record = c.claim_manager.acquire(
            owner=owner,
            session_id=request.session_id,
            ttl_s=request.ttl_s,
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
    return ClaimResponse(
        claim_token=record.token,
        heartbeat_interval_s=c.claim_manager.heartbeat_interval_s,
        expires_at=datetime.fromtimestamp(record.expires_at, tz=timezone.utc),
    )


@app.post("/control/heartbeat")
async def heartbeat_claim(x_claim_token: str = Header(...)):
    """Extend the holder's TTL. Returns 204 on success, 401 when the
    token is unknown / expired / belongs to a different session (per
    spec: client MUST treat the claim as lost)."""
    c = get_controller()
    try:
        c.claim_manager.heartbeat(token=x_claim_token)
    except InvalidClaimToken:
        raise HTTPException(status_code=401, detail="invalid or expired claim token")
    return Response(status_code=204)


@app.post("/control/release")
async def release_claim(x_claim_token: str = Header(...)):
    """Release the claim. Idempotent — returns 204 whether or not the
    token matched, so callers can always cleanly retire a session."""
    c = get_controller()
    c.claim_manager.release(token=x_claim_token)
    return Response(status_code=204)


@app.post("/control/claim/enforce", dependencies=[Depends(require_claim)])
async def set_claim_enforcement(request: EnforcementRequest):
    """Enable or disable claim enforcement at runtime.

    Gated by require_claim itself: disabling enforcement while it's on
    requires holding the current claim, so a random client can't quietly
    drop the lock. Enabling from off-state is open (no claim to enforce
    against yet).
    """
    c = get_controller()
    if request.enabled:
        c.claim_manager.enable_enforcement()
    else:
        c.claim_manager.disable_enforcement()
    return {"enforced": c.claim_manager.enforced}


# =============================================================================
# MOTION GRAPH (Phase 2)
# =============================================================================

def _load_standalone_graph():  # -> Optional[MotionGraph]
    """Load motion_graph.yaml directly from disk, no controller required.

    The motion graph is pure data; the graph viewer must render with no
    hardware connected (the controller — and thus c.motion_graph — only
    exists after /connect). When disconnected we read the file here so
    GET /graph still returns the full topology. Returns None if the file
    is missing or fails validation (same disabled-graph semantics the
    controller uses at init)."""
    try:
        from .motion_graph import MotionGraph, DEFAULT_PRECONDITIONS, GraphError
    except ImportError:
        from core.motion_graph import MotionGraph, DEFAULT_PRECONDITIONS, GraphError
    path = os.path.join("src", "settings", "motion_graph.yaml")
    try:
        return MotionGraph.from_yaml(path, preconditions=DEFAULT_PRECONDITIONS)
    except (FileNotFoundError, GraphError) as exc:
        logger.warning(f"Standalone motion_graph load failed: {exc}")
        return None


@app.get("/graph")
async def get_graph_state():
    """Snapshot of the motion-graph layer's current state.

    Useful for the web UI, debugging, and tests. The full node/edge
    topology is always returned (loaded standalone from YAML when no
    controller is connected, so the viewer works with no hardware); the
    live fields (current_node, reachable_nodes, payload, mode) are only
    meaningful once connected and are nulled/defaulted otherwise. Returns
    404 only when no graph is loadable at all. The same data also rides
    in /status.details.motion_graph.
    """
    c = controller  # module global; None until /connect
    graph = c.motion_graph if c is not None else _load_standalone_graph()
    if graph is None:
        raise HTTPException(status_code=404, detail="motion_graph.yaml not loaded")
    return {
        # Live state — populated from the controller when connected,
        # neutral defaults when the viewer is open against a cold service.
        "graph_mode": c.graph_mode.value if c is not None else "off",
        "current_node": c.current_node if c is not None else None,
        "reachable_nodes": c.reachable_node_ids() if c is not None else [],
        "travel_targets": (
            graph.reachable_set(c.current_node, c.current_gripper_state)
            if c is not None else []
        ),
        "gripper_stroke": c.last_gripper_position if c is not None else None,
        "gripper_state": c.current_gripper_state if c is not None else None,
        "allowed_gripper_targets": c.allowed_gripper_targets() if c is not None else [],
        "arm_pose_name": c.last_arm_pose_name if c is not None else None,
        "rail_location_name": c.last_rail_location_name if c is not None else None,
        "last_transition": c.last_transition if c is not None else None,
        "adjacency": graph.adjacency_summary(),
        # The global gripper-state catalog (name -> stroke + intent).
        "gripper_state_catalog": {
            gs.name: {"stroke": gs.stroke, "intent": gs.intent.value}
            for gs in graph.gripper_states
        },
        # Full node/edge detail so the graph viewer can render the whole
        # topology (and Phase B can edit it) without a second call.
        "nodes": [
            {
                "id": n.id, "arm": n.arm, "rail": n.rail,
                "gripper_states": list(n.gripper_states),
                "gripper_transitions": [list(t) for t in n.gripper_transitions],
                "tags": list(n.tags),
            }
            for n in graph.nodes
        ],
        "edges": [
            {
                "from": e.from_node, "to": e.to_node, "mode": e.mode.value,
                "speed": e.speed,
                "preconditions": list(e.preconditions), "comment": e.comment,
            }
            for e in graph.edges
        ],
    }


def _graph_layout_path() -> str:
    return os.path.join("src", "settings", "motion_graph_layout.json")


@app.get("/graph/layout")
async def get_graph_layout():
    """The viewer's saved geometry (positions / expanded / pan / zoom).

    Read-only, NOT claim-gated — it's presentation data shared by every PC
    that connects, stored device-side so the layout no longer lives in one
    browser's localStorage. Returns empty defaults when nothing's been saved
    yet (so a fresh device just lays itself out from scratch).
    """
    path = _graph_layout_path()
    if not os.path.exists(path):
        return {"positions": {}, "expanded": {}, "pan": None, "zoom": None}
    try:
        with open(path) as fh:
            data = json.load(fh)
    except (json.JSONDecodeError, OSError):
        # Corrupt/unreadable layout must never break the viewer — fall back to
        # empty and let the next save overwrite it.
        return {"positions": {}, "expanded": {}, "pan": None, "zoom": None}
    return data


@app.post("/graph/layout")
async def save_graph_layout(layout: GraphLayoutModel):
    """Persist the viewer's geometry to src/settings/motion_graph_layout.json.

    NOT claim-gated: rearranging the map moves no hardware, and a read-only
    viewer (no claim) must still be able to save layout. Written atomically
    (temp file + replace) so a concurrent reader never sees a half-written
    file. Last write wins — layout is not safety-critical.
    """
    path = _graph_layout_path()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(layout.model_dump(), fh, indent=2)
    os.replace(tmp, path)
    return {"saved": True}


@app.post("/control/graph/move_to", dependencies=[Depends(require_claim)])
async def graph_move_to(request: GraphMoveToRequest, background_tasks: BackgroundTasks):
    """Move to a graph node by id.

    Pure arm motion — the gripper stroke is invariant along edges and is
    never touched here. Grip/release/narrow happens separately via
    POST /control/graph/gripper while parked at a node.

    Returns 409 (edge_not_allowed) when STRICT mode refuses the transition
    (including edges the current gripper state may not ride), 409
    (motion_in_progress) when a motion is already in flight, or 500 when
    the move fails.
    """
    c = get_controller()
    if c.motion_graph is None:
        raise HTTPException(status_code=404, detail="motion_graph.yaml not loaded")
    try:
        c.motion_graph.node(request.node_id)  # validate early
    except UnknownNodeError:
        raise HTTPException(status_code=409, detail=f"unknown node: {request.node_id!r}")

    # Reserved after the cheap validation above so a bad node id does not
    # take (and have to release) the motion slot. A cross-rail edge is two
    # sub-moves; the reservation nests over both, so the slot is not freed
    # between them.
    reserve_motion()
    try:
        success = await asyncio.to_thread(
            c.move_to_node,
            node_id=request.node_id,
            speed=request.speed,
        )
    except UnknownNodeError:
        raise HTTPException(status_code=409, detail=f"unknown node: {request.node_id!r}")
    except EdgeNotAllowedError as exc:
        raise HTTPException(
            status_code=409,
            detail={
                "error": "edge_not_allowed",
                "current_node": exc.current,
                "target": exc.target,
                "reason": exc.reason,
            },
        )
    finally:
        c.exit_motion()

    background_tasks.add_task(broadcast_status_update)
    if not success:
        raise HTTPException(
            status_code=500,
            detail=f"Move to node '{request.node_id}' failed",
        )
    return {"message": f"Moved to node '{request.node_id}'", "current_node": c.current_node}


@app.post("/control/graph/travel_to", dependencies=[Depends(require_claim)])
async def graph_travel_to(request: GraphTravelToRequest, background_tasks: BackgroundTasks):
    """Multi-hop travel to any reachable graph node.

    Plans a shortest hop path (current gripper state held throughout)
    and executes it hop-by-hop through the same machinery as
    /control/graph/move_to. Blocks until the journey completes,
    matching the repo's blocking-move convention; live progress is
    visible on the /ws status stream.

    Returns 409 for an unknown node, no path (no_path), a STRICT
    refusal (edge_not_allowed — e.g. off-grid), or a motion already in
    flight (motion_in_progress); 500 when a hop fails mid-journey (the arm
    is parked at the last completed node).
    """
    c = get_controller()
    if c.motion_graph is None:
        raise HTTPException(status_code=404, detail="motion_graph.yaml not loaded")
    try:
        c.motion_graph.node(request.node_id)  # validate early
    except UnknownNodeError:
        raise HTTPException(status_code=409, detail=f"unknown node: {request.node_id!r}")

    # One reservation for the whole journey: the per-hop primitives nest
    # inside it, so the slot is held from the first hop to the last rather
    # than being released at every intermediate node.
    reserve_motion()
    try:
        result = await asyncio.to_thread(
            c.travel_to_node,
            node_id=request.node_id,
            speed=request.speed,
        )
    except UnknownNodeError:
        raise HTTPException(status_code=409, detail=f"unknown node: {request.node_id!r}")
    except NoPathError as exc:
        raise HTTPException(
            status_code=409,
            detail={
                "error": "no_path",
                "from": exc.from_node,
                "to": exc.to_node,
                "gripper_state": exc.gripper_state,
                "reason": exc.reason,
            },
        )
    except EdgeNotAllowedError as exc:
        raise HTTPException(
            status_code=409,
            detail={
                "error": "edge_not_allowed",
                "current_node": exc.current,
                "target": exc.target,
                "reason": exc.reason,
            },
        )
    finally:
        c.exit_motion()

    background_tasks.add_task(broadcast_status_update)

    # Surface edge speed-cap clamps: the requested max is only a ceiling of
    # the caller's own, and STRICT clamps it per hop to edge.speed. Logging
    # through `logger` puts it on the panel's log stream (bare prints in the
    # controller never reach the browser).
    for clamp in result.get("speed_clamps") or []:
        logger.warning(
            "speed clamped %s->%s: %s -> %s deg/s (graph edge limit)",
            clamp["from"], clamp["to"], clamp["requested"], clamp["applied"],
        )

    if not result["success"]:
        raise HTTPException(
            status_code=500,
            detail={
                "error": "travel_failed",
                "failed_hop": result["failed_hop"],
                "completed": result["completed"],
                "current_node": c.current_node,
            },
        )
    return {
        "message": f"Traveled to node '{request.node_id}' "
                   f"({len(result['path'])} hop(s))",
        "path": result["path"],
        "current_node": c.current_node,
    }


# =============================================================================
# LAB ASSISTANT (natural-language motion control via OpenRouter)
#
# The assistant is a thin convenience over the motion-graph endpoints: the
# LLM only maps text -> a structured intent (assistant_llm), the intent is
# turned into a concrete step list deterministically (assistant_actions), and
# execution replays those steps through the SAME STRICT-enforced controller
# machinery as the manual graph endpoints. The model never touches hardware
# and never sees the claim token.
# =============================================================================


def _assistant_graph_and_state():
    """Resolve the graph plus live (node, gripper_state) for planning.

    Uses the connected controller's live state when available; otherwise
    falls back to the standalone graph (so the catalog/enabled-state can
    be shown before /connect, though every action will read as off-grid).
    Returns ``(graph, current_node, current_gripper_state)`` or
    ``(None, None, None)`` when no graph is loadable.
    """
    c = controller  # module global; None until /connect
    if c is not None and c.motion_graph is not None:
        return c.motion_graph, c.current_node, c.current_gripper_state
    return _load_standalone_graph(), None, None


@app.get("/assistant/status")
async def assistant_status():
    """Whether the assistant can run, plus the place catalog for the UI.

    Read-only and un-gated: the corner widget calls this on load to
    decide whether to enable itself and to show the operator which
    places it understands. ``enabled`` is False (with ``reason``) when
    the openai lib/OpenRouter key is missing or the feature is toggled off.
    """
    graph, _, _ = _assistant_graph_and_state()
    places = []
    if graph is not None:
        catalog = assistant_actions.build_place_catalog(graph)
        places = [
            {"key": p.key, "label": p.label}
            for p in sorted(catalog.values(), key=lambda x: x.key)
        ]
    return {
        "enabled": assistant_llm.is_enabled(),
        "reason": assistant_llm.disabled_reason(),
        "model": assistant_llm.model_name(),
        "graph_loaded": graph is not None,
        "places": places,
    }


# =============================================================================
# CAMERA TRACKING (pan the lab camera to follow the arm on the motion graph)
#
# The arm panel calls GET /camera/config on load (and polls it) to decide
# whether to show a camera card + live stream + "Follow arm" toggle. Actuation
# lives entirely in core/camera_tracker.py; these endpoints are a thin view +
# runtime toggle over the controller's CameraTracker. Config is readable before
# /connect (via a standalone tracker) so the card + video can appear while
# disconnected; the follow *toggle* needs a connected controller, since
# following only acts on arm motion.
# =============================================================================


class CameraFollowRequest(BaseModel):
    """Body of POST /camera/follow."""
    enabled: bool = Field(description="true to make the camera follow the arm, false to pause")


# A read-only CameraTracker built straight from the YAML, used to answer
# /camera/config before the controller exists (pre-/connect). Cached so we
# don't re-read the file on every poll. NOT used for actuation.
_standalone_camera_tracker: Optional[CameraTracker] = None


def _camera_tracker_for_read() -> CameraTracker:
    """The controller's tracker when connected, else a standalone read view."""
    global _standalone_camera_tracker
    if controller is not None and getattr(controller, "camera_tracker", None) is not None:
        return controller.camera_tracker
    if _standalone_camera_tracker is None:
        cfg_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), "settings", "camera_tracking.yaml"
        )
        _standalone_camera_tracker = CameraTracker.from_config_file(cfg_path)
    return _standalone_camera_tracker


@app.get("/camera/config")
async def camera_config():
    """Whether the arm panel should show the camera card + follow toggle.

    Read-only and un-gated (like /assistant/status). ``configured`` decides
    whether the card appears at all; ``available`` (a short-cached live probe
    of the camera's /status through the dashboard) decides whether the stream
    + toggle are usable right now; ``stream_url`` is the absolute go2rtc
    ws(s):// URL the panel's player connects to. The probe never raises.
    """
    tracker = _camera_tracker_for_read()
    info = await asyncio.to_thread(tracker.availability)
    info["connected"] = controller is not None
    return info


@app.post("/camera/follow", dependencies=[Depends(require_login)])
async def camera_follow(request: CameraFollowRequest):
    """Turn "follow the arm" on/off. Requires a connected controller.

    Login-gated (no-op unless XARM_REQUIRE_LOGIN) but NOT claim-gated:
    panning a read-only camera is not hardware actuation, so it does not
    contend for the arm claim. When following is switched on we re-aim the
    camera at the arm's current node immediately, rather than waiting for
    the next move.
    """
    c = get_controller()
    tracker = getattr(c, "camera_tracker", None)
    if tracker is None or not tracker.configured:
        raise HTTPException(
            status_code=409,
            detail={"error": "camera_not_configured",
                    "hint": "set enabled + dashboard_base_url + camera_id in "
                            "src/settings/camera_tracking.yaml"},
        )
    following = tracker.set_following(request.enabled)
    # Best-effort immediate re-aim to the current node when turning on.
    if following and c.current_node:
        try:
            tracker.notify_node(c.motion_graph.node(c.current_node))
        except Exception as exc:  # noqa: BLE001 - re-aim is best-effort
            logger.info(f"camera re-aim skipped: {exc}")
    return {"following": following, "configured": tracker.configured}


@app.post("/camera/ptz", dependencies=[Depends(require_login)])
async def camera_ptz(body: dict):
    """Operator PTZ for the lab camera, forwarded to the dashboard's
    audited control passthrough (the camera gateway is loopback-bound on
    the dashboard host, so this is the only route in).

    Body is passed through verbatim: either a continuous move
    ``{direction, speed, duration_ms}`` or a stop ``{pan, tilt, zoom}``.
    """
    tracker = _camera_tracker_for_read()
    if tracker is None or not tracker.configured:
        raise HTTPException(status_code=404, detail="camera tracking not configured")
    try:
        await asyncio.to_thread(tracker.send_control, "ptz", dict(body or {}))
    except Exception as exc:  # noqa: BLE001 - surface the passthrough's failure
        raise HTTPException(status_code=502, detail=f"camera ptz failed: {exc}")
    return {"ok": True}


@app.post("/assistant/plan")
async def assistant_plan(request: AssistantPlanRequest):
    """Interpret a natural-language request into a previewable step list.

    Read-only with respect to hardware: no motion happens here. Returns
    the model's interpretation plus the concrete motion-graph steps the
    operator can confirm and replay to /assistant/execute. When the
    model needs clarification or the request is out of scope, ``action``
    is null and ``reply`` carries a plain-text message.

    503 when the assistant is unavailable (no key / lib / disabled),
    502 on an LLM runtime error, 404 when no motion graph is loaded.
    """
    graph, current_node, current_gripper_state = _assistant_graph_and_state()
    if graph is None:
        raise HTTPException(status_code=404, detail="motion_graph.yaml not loaded")

    catalog = assistant_actions.build_place_catalog(graph)
    gripper_states = [gs.name for gs in graph.gripper_states]
    history = (
        [{"role": m.role, "content": m.content} for m in request.history]
        if request.history else None
    )

    try:
        interp = await asyncio.to_thread(
            assistant_llm.interpret,
            request.message,
            catalog=catalog,
            gripper_states=gripper_states,
            current_node=current_node,
            current_gripper_state=current_gripper_state,
            history=history,
        )
    except assistant_llm.AssistantDisabled as exc:
        raise HTTPException(
            status_code=503,
            detail={"error": "assistant_disabled", "reason": exc.reason},
        )
    except assistant_llm.AssistantError as exc:
        raise HTTPException(
            status_code=502,
            detail={"error": "assistant_error", "reason": str(exc)},
        )

    # The model chose to reply in text (clarification / refusal) instead
    # of committing to an action.
    if interp.action is None:
        return {
            "feasible": False,
            "action": None,
            "reply": interp.reply,
            "interpretation": None,
            "steps": [],
            "reason": None,
            "connected": controller is not None,
        }

    result = assistant_actions.resolve_action(
        catalog, graph, current_node, current_gripper_state,
        action=interp.action,
        place=interp.args.get("place"),
        gripper_state=interp.args.get("gripper_state"),
    )
    return {
        "feasible": result.feasible,
        "action": interp.action,
        "place": result.place,
        "interpretation": result.interpretation,
        "reason": result.reason,
        "reply": interp.reply,
        "steps": result.steps,
        "connected": controller is not None,
    }


@app.post("/assistant/execute", dependencies=[Depends(require_claim)])
async def assistant_execute(request: AssistantExecuteRequest, background_tasks: BackgroundTasks):
    """Execute a confirmed assistant step list, hop by hop.

    Claim-gated. Each 'move' step drives to a graph node via
    ``move_to_node`` and each 'gripper' step changes the gripper via
    ``set_gripper_state`` — both under STRICT enforcement, so an invalid
    or tampered step is refused (409) exactly as a manual graph call
    would be. Fail-fast: the first failed step stops the run, leaving the
    arm parked at the last completed node. Progress streams over /ws as
    log lines.

    Returns 409 for a graph refusal (edge_not_allowed / no_path /
    gripper_transition_not_allowed), 422 for a malformed step, 500 for a
    mid-run actuation/verification failure.
    """
    c = get_controller()
    if c.motion_graph is None:
        raise HTTPException(status_code=404, detail="motion_graph.yaml not loaded")
    if not request.steps:
        raise HTTPException(status_code=422, detail="no steps to execute")

    label = request.interpretation or "assistant command"
    total = len(request.steps)
    completed: List[Dict[str, Any]] = []
    logger.info(f"[assistant] executing: {label} ({total} step(s))")

    def _run_step(step: AssistantStep) -> bool:
        if step.kind == "move":
            return c.move_to_node(node_id=step.to, speed=request.speed)
        if step.kind == "gripper":
            return c.set_gripper_state(step.state)
        raise ValueError(f"unknown step kind {step.kind!r}")

    # One reservation for the whole step list: a plate transfer is a
    # sequence, and freeing the slot between steps would let another
    # caller interleave a move into the middle of it.
    reserve_motion()
    try:
        for idx, step in enumerate(request.steps, start=1):
            # Validate step shape up front so a bad payload is a clean 422.
            if step.kind == "move" and not step.to:
                raise HTTPException(status_code=422, detail=f"step {idx}: move step missing 'to'")
            if step.kind == "gripper" and not step.state:
                raise HTTPException(status_code=422, detail=f"step {idx}: gripper step missing 'state'")
            if step.kind not in ("move", "gripper"):
                raise HTTPException(status_code=422, detail=f"step {idx}: unknown kind {step.kind!r}")

            descriptor = (
                f"move to {step.to}" if step.kind == "move" else f"set gripper {step.state}"
            )
            logger.info(f"[assistant] step {idx}/{total}: {descriptor}")

            try:
                success = await asyncio.to_thread(_run_step, step)
            except EdgeNotAllowedError as exc:
                logger.error(f"[assistant] step {idx}/{total} refused: {exc.reason}")
                raise HTTPException(
                    status_code=409,
                    detail={
                        "error": "edge_not_allowed",
                        "current_node": exc.current,
                        "target": exc.target,
                        "reason": exc.reason,
                        "completed": completed,
                        "failed_step": idx,
                    },
                )
            except GripperTransitionError as exc:
                logger.error(f"[assistant] step {idx}/{total} refused: {exc.reason}")
                raise HTTPException(
                    status_code=409,
                    detail={
                        "error": "gripper_transition_not_allowed",
                        "node": exc.node,
                        "from_state": exc.from_state,
                        "to_state": exc.to_state,
                        "reason": exc.reason,
                        "completed": completed,
                        "failed_step": idx,
                    },
                )
            except (UnknownNodeError, NoPathError, GraphError, ValueError) as exc:
                logger.error(f"[assistant] step {idx}/{total} failed: {exc}")
                raise HTTPException(
                    status_code=409,
                    detail={"error": "step_invalid", "reason": str(exc),
                            "completed": completed, "failed_step": idx},
                )

            if not success:
                logger.error(f"[assistant] step {idx}/{total} failed at {c.current_node!r}")
                background_tasks.add_task(broadcast_status_update)
                raise HTTPException(
                    status_code=500,
                    detail={
                        "error": "step_failed",
                        "failed_step": idx,
                        "descriptor": descriptor,
                        "completed": completed,
                        "current_node": c.current_node,
                    },
                )
            completed.append({"step": idx, "kind": step.kind,
                              "descriptor": descriptor, "current_node": c.current_node})

        logger.info(f"[assistant] done: {label} — parked at {c.current_node!r}")
        background_tasks.add_task(broadcast_status_update)
        return {
            "message": f"Executed {total} step(s): {label}",
            "completed": completed,
            "current_node": c.current_node,
            "gripper_state": c.current_gripper_state,
        }
    finally:
        c.exit_motion()


@app.get("/graph/nearest")
async def get_nearest_node(joint_tolerance_deg: float = 10.0, rail_tolerance_mm: float = 2.0):
    """Nearest-node detection: which graph node best matches the
    controller's physical state right now?

    Useful after STOP / power-cycle / manual jog when current_node is
    None — the operator (or a workflow recovery step) can read this,
    eyeball the suggestion, and POST /control/graph/recover_to to
    re-pin. Returns 404 when no graph is loaded.
    """
    c = get_controller()
    if c.motion_graph is None:
        raise HTTPException(status_code=404, detail="motion_graph.yaml not loaded")
    match = c.suggest_current_node(
        joint_tolerance_deg=joint_tolerance_deg,
        rail_tolerance_mm=rail_tolerance_mm,
    )
    # The node's gripper leaves travel with the suggestion so the operator
    # can DECLARE one when the live stroke resolves to no catalog state
    # (e.g. a closed 71 mm against reach_72's 72 mm) — otherwise recovery
    # is impossible without force, which also skips the position check.
    allowed_states: list[str] = []
    if match.node_id is not None:
        try:
            allowed_states = list(c.motion_graph.node(match.node_id).gripper_states)
        except Exception:
            allowed_states = []

    return {
        "suggested_node": match.node_id,
        "arm_residual_deg": match.arm_residual,
        "rail_residual_mm": match.rail_residual,
        "gripper_state": match.gripper_state,
        "gripper_match": match.gripper_match,
        "within_tolerance": match.within_tolerance,
        "allowed_gripper_states": allowed_states,
        "gripper_stroke": c.last_gripper_position,
    }


@app.post("/control/graph/recover_to", dependencies=[Depends(require_claim)])
async def recover_to_node(request: GraphRecoverRequest):
    """Operator-declared re-pin to a known node after off-grid travel.

    Without ``force``, the nearest-node detector must agree with the
    requested node id (and be within tolerance) — otherwise returns
    422 with the detector's suggestion + residuals. With ``force=true``
    the check is skipped; the operator asserts the position is correct.

    On success returns the new graph snapshot.
    """
    c = get_controller()
    if c.motion_graph is None:
        raise HTTPException(status_code=404, detail="motion_graph.yaml not loaded")
    try:
        result = c.recover_to(
            request.node_id, force=request.force,
            gripper_state=request.gripper_state,
        )
    except UnknownNodeError as exc:
        raise HTTPException(status_code=409, detail=f"unknown node: {request.node_id!r}")
    except GripperTransitionError as exc:
        raise HTTPException(
            status_code=409,
            detail={
                "error": "gripper_state_not_allowed",
                "node": exc.node,
                "gripper_state": exc.to_state,
                "reason": exc.reason,
            },
        )
    except RecoveryMismatch as exc:
        raise HTTPException(
            status_code=422,
            detail={
                "error": "recovery_mismatch",
                "requested": exc.requested,
                "suggested": exc.suggested,
                "arm_residual_deg": exc.arm_residual,
                "rail_residual_mm": exc.rail_residual,
                "hint": "retry with force=true to override, or move the arm closer to the requested node",
            },
        )
    return result


@app.post("/control/graph/gripper", dependencies=[Depends(require_claim)])
async def set_gripper_state(request: GraphGripperRequest, background_tasks: BackgroundTasks):
    """Change the gripper state while parked at the current node.

    The only graph-sanctioned way to grip/release/narrow: the stroke is
    invariant during arm motion, so this is gated on the arm being
    stationary, a pinned current node, and the transition appearing in
    that node's whitelist (STRICT rejects violations with 409).

    After actuation the outcome is verified per the state's intent:
    grasp states must settle ABOVE the commanded stroke (reaching it
    exactly = nothing gripped), position states must REACH it (stalling
    early = blocked). Verification failure returns 500.
    """
    c = get_controller()
    if c.motion_graph is None:
        raise HTTPException(status_code=404, detail="motion_graph.yaml not loaded")
    if not c.motion_graph.has_gripper_state(request.state):
        raise HTTPException(
            status_code=422,
            detail=f"unknown gripper state: {request.state!r}",
        )

    try:
        success = await asyncio.to_thread(c.set_gripper_state, request.state)
    except GripperTransitionError as exc:
        raise HTTPException(
            status_code=409,
            detail={
                "error": "gripper_transition_not_allowed",
                "node": exc.node,
                "from_state": exc.from_state,
                "to_state": exc.to_state,
                "reason": exc.reason,
            },
        )

    background_tasks.add_task(broadcast_status_update)
    if not success:
        raise HTTPException(
            status_code=500,
            detail=(
                f"gripper transition to '{request.state}' failed "
                f"(actuation error or verification failure)"
            ),
        )
    return {
        "message": f"Gripper set to '{request.state}'",
        "gripper_state": c.current_gripper_state,
        "allowed_gripper_targets": c.allowed_gripper_targets(),
    }


@app.post("/control/graph/mode", dependencies=[Depends(require_claim)])
async def set_graph_mode(request: GraphModeRequest):
    """Switch the enforcement mode (off | advisory | strict).

    OFF: graph is not consulted; legacy behavior.
    ADVISORY: graph observes; off-whitelist moves log a warning but proceed.
    STRICT: edge.mode overrides preset format, edge.speed caps caller's
    speed, off-whitelist moves return HTTP 409.
    """
    c = get_controller()
    try:
        mode = GraphMode(request.mode)
    except ValueError:
        raise HTTPException(
            status_code=422,
            detail=f"mode must be one of: off, advisory, strict (got {request.mode!r})",
        )
    try:
        c.set_graph_mode(mode)
    except RuntimeError as exc:
        raise HTTPException(status_code=400, detail=str(exc))
    return {"graph_mode": c.graph_mode.value}


@app.post("/control/graph/record", dependencies=[Depends(require_claim)])
async def record_last_transition(request: GraphRecordRequest):
    """Append the most recent successful node-to-node transition to
    motion_graph.yaml as a new edge.

    The proposed edge is first validated against a candidate graph
    (existing edges + the new one) before being written; if validation
    fails (coherence rules, duplicates) the API returns 400 with the
    GraphError reason and the YAML is untouched. On success the in-
    memory graph is reloaded from disk.
    """
    c = get_controller()
    if c.motion_graph is None:
        raise HTTPException(status_code=404, detail="motion_graph.yaml not loaded")
    if c.last_transition is None:
        raise HTTPException(
            status_code=409,
            detail="no transition to record; perform a named move first",
        )

    transition = c.last_transition
    proposed = {
        "from": transition["from_node"],
        "to": transition["to_node"],
        "mode": request.mode or transition["mode"],
        "speed": request.speed if request.speed is not None else transition["speed"],
    }
    if request.comment:
        proposed["comment"] = request.comment
    if request.preconditions:
        proposed["preconditions"] = list(request.preconditions)

    try:
        new_graph = _append_edge_to_yaml(proposed)
    except GraphError as exc:
        raise HTTPException(
            status_code=400, detail=f"proposed edge failed validation: {exc}",
        )

    c.motion_graph = new_graph
    return {"recorded": proposed}


@app.post("/control/graph/edge", dependencies=[Depends(require_claim)])
async def update_graph_edge(request: GraphEdgeUpdateRequest):
    """In-place edit of an existing edge's mode and/or speed.

    Claim-gated like every mutating endpoint (the single-gate rule). The
    edge must already exist; node poses, grip/release actions, and
    topology are not editable here — those stay with the control panel
    and the record (append-by-demonstration) flow. The YAML is rewritten
    with a ruamel round-trip so comments survive, validated before the
    write, then the in-memory graph is hot-reloaded.
    """
    c = get_controller()
    if c.motion_graph is None:
        raise HTTPException(status_code=404, detail="motion_graph.yaml not loaded")
    if c.motion_graph.find_edge(request.from_node, request.to_node) is None:
        raise HTTPException(
            status_code=404,
            detail=f"no edge {request.from_node!r} -> {request.to_node!r}",
        )
    if request.mode is not None and request.mode not in ("joint", "linear"):
        raise HTTPException(
            status_code=422,
            detail=f"mode must be 'joint' or 'linear' (got {request.mode!r})",
        )
    try:
        new_graph = _update_edge_in_yaml(request)
    except GraphError as exc:
        raise HTTPException(status_code=400, detail=f"edit failed validation: {exc}")
    c.motion_graph = new_graph
    edge = new_graph.find_edge(request.from_node, request.to_node)
    return {
        "updated": {
            "from": edge.from_node, "to": edge.to_node,
            "mode": edge.mode.value, "speed": edge.speed,
        }
    }


def _update_edge_in_yaml(req: "GraphEdgeUpdateRequest") -> "MotionGraph":  # type: ignore[name-defined]
    """Edit an existing edge's mode/speed in motion_graph.yaml in place.

    Uses a ruamel.yaml round-trip so the file's comments (the "you hear a
    click" notes, the commented station templates) survive — a PyYAML
    re-dump would wipe them all. The candidate is validated with the real
    loader BEFORE the write, so a bad value never lands on disk, then the
    graph is reloaded from the file so memory matches disk.
    """
    import io
    from ruamel.yaml import YAML
    import yaml as _pyyaml
    try:
        from .motion_graph import MotionGraph, DEFAULT_PRECONDITIONS
    except ImportError:
        from core.motion_graph import MotionGraph, DEFAULT_PRECONDITIONS

    path = os.path.join("src", "settings", "motion_graph.yaml")
    yaml_rt = YAML()                 # round-trip mode (default)
    yaml_rt.preserve_quotes = True
    with open(path) as fh:
        data = yaml_rt.load(fh)

    # Locate the edge by (from, to) — the loader forbids duplicate pairs,
    # so this is unambiguous.
    target = next(
        (e for e in data.get("edges", [])
         if e.get("from") == req.from_node and e.get("to") == req.to_node),
        None,
    )
    if target is None:
        raise GraphError(f"no edge {req.from_node!r} -> {req.to_node!r}")
    if req.mode is not None:
        target["mode"] = req.mode
    if req.speed is not None:
        target["speed"] = req.speed

    # Serialize once; validate that exact text via the real loader before
    # committing it to disk (mode/speed don't touch the coherence rules,
    # but this re-confirms the file still loads — cheap insurance).
    buf = io.StringIO()
    yaml_rt.dump(data, buf)
    serialized = buf.getvalue()
    plain = _pyyaml.safe_load(serialized)
    MotionGraph.from_dict(plain, preconditions=DEFAULT_PRECONDITIONS)  # raises GraphError

    with open(path, "w") as fh:      # validated -> commit
        fh.write(serialized)
    return MotionGraph.from_yaml(path, preconditions=DEFAULT_PRECONDITIONS)


@app.post("/control/graph/edge/delete", dependencies=[Depends(require_claim)])
async def delete_graph_edge(request: GraphEdgeDeleteRequest):
    """Remove an existing edge (motion) from the graph.

    Claim-gated like every mutating endpoint (the single-gate rule). The edge
    must exist; only the edge is removed (both endpoint nodes stay). The YAML is
    rewritten with a ruamel round-trip so comments survive, validated before the
    write, then the in-memory graph is hot-reloaded.
    """
    c = get_controller()
    if c.motion_graph is None:
        raise HTTPException(status_code=404, detail="motion_graph.yaml not loaded")
    if c.motion_graph.find_edge(request.from_node, request.to_node) is None:
        raise HTTPException(
            status_code=404,
            detail=f"no edge {request.from_node!r} -> {request.to_node!r}",
        )
    try:
        new_graph = _delete_edge_in_yaml(request)
    except GraphError as exc:
        raise HTTPException(status_code=400, detail=f"delete failed validation: {exc}")
    c.motion_graph = new_graph
    return {"deleted": {"from": request.from_node, "to": request.to_node}}


def _delete_edge_in_yaml(req: "GraphEdgeDeleteRequest") -> "MotionGraph":  # type: ignore[name-defined]
    """Remove the edge for the ordered (from, to) pair from motion_graph.yaml.

    Uses a ruamel.yaml round-trip so the file's comments survive. The candidate
    is validated with the real loader BEFORE the write, so the file always
    stays loadable, then the graph is reloaded so memory matches disk.
    """
    import io
    from ruamel.yaml import YAML
    import yaml as _pyyaml
    try:
        from .motion_graph import MotionGraph, DEFAULT_PRECONDITIONS
    except ImportError:
        from core.motion_graph import MotionGraph, DEFAULT_PRECONDITIONS

    path = os.path.join("src", "settings", "motion_graph.yaml")
    yaml_rt = YAML()                 # round-trip mode (default)
    yaml_rt.preserve_quotes = True
    with open(path) as fh:
        data = yaml_rt.load(fh)

    edges = data.get("edges", []) or []
    idx = next(
        (i for i, e in enumerate(edges)
         if e.get("from") == req.from_node and e.get("to") == req.to_node),
        None,
    )
    if idx is None:
        raise GraphError(f"no edge {req.from_node!r} -> {req.to_node!r}")
    del edges[idx]

    buf = io.StringIO()
    yaml_rt.dump(data, buf)
    serialized = buf.getvalue()
    plain = _pyyaml.safe_load(serialized)
    MotionGraph.from_dict(plain, preconditions=DEFAULT_PRECONDITIONS)  # raises GraphError

    with open(path, "w") as fh:      # validated -> commit
        fh.write(serialized)
    return MotionGraph.from_yaml(path, preconditions=DEFAULT_PRECONDITIONS)


@app.post("/control/graph/edge/create", dependencies=[Depends(require_claim)])
async def create_graph_edge(request: GraphEdgeCreateRequest):
    """Add a new edge (motion) between two existing nodes.

    Claim-gated (single-gate rule). Both endpoints must exist and there must
    be no existing edge for that ordered pair. Edges never change the
    gripper — the loader also rejects edges whose endpoints share no
    gripper state (untraversable). Comments in the file are preserved
    (ruamel) and the graph hot-reloads.
    """
    c = get_controller()
    if c.motion_graph is None:
        raise HTTPException(status_code=404, detail="motion_graph.yaml not loaded")
    if not c.motion_graph.has_node(request.from_node):
        raise HTTPException(status_code=409, detail=f"unknown node: {request.from_node!r}")
    if not c.motion_graph.has_node(request.to_node):
        raise HTTPException(status_code=409, detail=f"unknown node: {request.to_node!r}")
    if request.from_node == request.to_node:
        raise HTTPException(status_code=422, detail="from_node and to_node must differ")
    if c.motion_graph.find_edge(request.from_node, request.to_node) is not None:
        raise HTTPException(
            status_code=409,
            detail=f"edge {request.from_node!r} -> {request.to_node!r} already exists",
        )
    if request.mode not in ("joint", "linear"):
        raise HTTPException(status_code=422, detail=f"mode must be 'joint' or 'linear' (got {request.mode!r})")

    edge = {"from": request.from_node, "to": request.to_node, "mode": request.mode}
    if request.speed is not None:
        edge["speed"] = request.speed
    if request.preconditions:
        edge["preconditions"] = list(request.preconditions)
    if request.comment:
        edge["comment"] = request.comment

    try:
        new_graph = _append_edge_via_ruamel(edge)
    except GraphError as exc:
        raise HTTPException(status_code=400, detail=f"edge failed validation: {exc}")
    c.motion_graph = new_graph
    e = new_graph.find_edge(request.from_node, request.to_node)
    return {
        "created": {
            "from": e.from_node, "to": e.to_node, "mode": e.mode.value, "speed": e.speed,
            "preconditions": list(e.preconditions), "comment": e.comment,
        }
    }


def _append_edge_via_ruamel(edge: dict) -> "MotionGraph":  # type: ignore[name-defined]
    """Append a fully-formed edge dict (may include a nested action block) to
    motion_graph.yaml via a ruamel round-trip, validate, then hot-reload.

    Unlike record's text-append helper, this serialises the whole edge with
    ruamel so a nested grip/release `action:` survives — and comments are
    preserved. Validation runs on the candidate before the write."""
    import io
    from ruamel.yaml import YAML
    import yaml as _pyyaml
    try:
        from .motion_graph import MotionGraph, DEFAULT_PRECONDITIONS
    except ImportError:
        from core.motion_graph import MotionGraph, DEFAULT_PRECONDITIONS

    path = os.path.join("src", "settings", "motion_graph.yaml")
    yaml_rt = YAML()
    yaml_rt.preserve_quotes = True
    with open(path) as fh:
        data = yaml_rt.load(fh)
    data.setdefault("edges", []).append(edge)

    buf = io.StringIO()
    yaml_rt.dump(data, buf)
    serialized = buf.getvalue()
    plain = _pyyaml.safe_load(serialized)
    MotionGraph.from_dict(plain, preconditions=DEFAULT_PRECONDITIONS)  # raises GraphError

    with open(path, "w") as fh:      # validated -> commit
        fh.write(serialized)
    return MotionGraph.from_yaml(path, preconditions=DEFAULT_PRECONDITIONS)


@app.post("/control/graph/node", dependencies=[Depends(require_claim)])
async def create_graph_node(request: GraphNodeCreateRequest):
    """Add a new node (arm position) to motion_graph.yaml.

    Claim-gated like every mutating endpoint (the single-gate rule). The
    node id must be unique and the result must satisfy the loader's
    coherence rules (unique (arm, rail), states in the catalog,
    transitions within the node's own states) — otherwise 400. Comments
    in the file are preserved (ruamel round-trip), the candidate is
    validated before the write, and the in-memory graph is hot-reloaded.
    """
    c = get_controller()
    if c.motion_graph is None:
        raise HTTPException(status_code=404, detail="motion_graph.yaml not loaded")
    if c.motion_graph.has_node(request.id):
        raise HTTPException(status_code=409, detail=f"node {request.id!r} already exists")
    try:
        new_graph = _append_node_to_yaml(request)
    except GraphError as exc:
        raise HTTPException(status_code=400, detail=f"node failed validation: {exc}")
    c.motion_graph = new_graph
    n = new_graph.node(request.id)
    return {
        "created": {
            "id": n.id, "arm": n.arm, "rail": n.rail,
            "gripper_states": list(n.gripper_states),
            "gripper_transitions": [list(t) for t in n.gripper_transitions],
            "tags": list(n.tags),
        }
    }


def _append_node_to_yaml(req: "GraphNodeCreateRequest") -> "MotionGraph":  # type: ignore[name-defined]
    """Append a new node to motion_graph.yaml (ruamel round-trip), validate
    the candidate with the real loader, then hot-reload.

    Comments survive because ruamel preserves them; the candidate is fully
    validated (unknown gripper states, duplicate (arm, rail), coherence
    rules) before the write, so a bad node never lands on disk."""
    import io
    from ruamel.yaml import YAML
    import yaml as _pyyaml
    try:
        from .motion_graph import MotionGraph, DEFAULT_PRECONDITIONS
    except ImportError:
        from core.motion_graph import MotionGraph, DEFAULT_PRECONDITIONS

    path = os.path.join("src", "settings", "motion_graph.yaml")
    yaml_rt = YAML()
    yaml_rt.preserve_quotes = True
    with open(path) as fh:
        data = yaml_rt.load(fh)

    node = {"id": req.id, "arm": req.arm, "rail": req.rail}
    if req.gripper_states:
        node["gripper_states"] = list(req.gripper_states)
    if req.gripper_transitions:
        node["gripper_transitions"] = [list(t) for t in req.gripper_transitions]
    if req.tags:
        node["tags"] = list(req.tags)
    data.setdefault("nodes", []).append(node)

    # Serialize once; validate that exact text via the real loader before
    # committing (catches unknown gripper/payload + coherence violations).
    buf = io.StringIO()
    yaml_rt.dump(data, buf)
    serialized = buf.getvalue()
    plain = _pyyaml.safe_load(serialized)
    MotionGraph.from_dict(plain, preconditions=DEFAULT_PRECONDITIONS)  # raises GraphError

    with open(path, "w") as fh:      # validated -> commit
        fh.write(serialized)
    return MotionGraph.from_yaml(path, preconditions=DEFAULT_PRECONDITIONS)


def _append_edge_to_yaml(proposed: dict) -> "MotionGraph":  # type: ignore[name-defined]
    """Text-append a new edge to motion_graph.yaml and reload the graph.

    Comments are preserved because the file is not parsed-and-rewritten;
    we just append a YAML edge block to the end of the file. Validation
    runs against a candidate graph before the write to avoid leaving the
    file in a state the loader would reject.
    """
    try:
        from .motion_graph import MotionGraph, DEFAULT_PRECONDITIONS
    except ImportError:
        from core.motion_graph import MotionGraph, DEFAULT_PRECONDITIONS

    path = os.path.join("src", "settings", "motion_graph.yaml")
    raw = open(path).read()
    # Build the candidate by parsing the current YAML, appending the
    # edge in-memory, and constructing a MotionGraph to run validation.
    import yaml as _yaml
    data = _yaml.safe_load(raw) or {}
    data.setdefault("edges", []).append(proposed)
    candidate = MotionGraph.from_dict(data, preconditions=DEFAULT_PRECONDITIONS)
    # Validation passed; write the appended edge as raw YAML text.
    block = _format_edge_yaml(proposed)
    if not raw.endswith("\n"):
        raw += "\n"
    with open(path, "w") as fh:
        fh.write(raw + block)
    # Reload the graph from disk so the in-memory object matches the file.
    return MotionGraph.from_yaml(path, preconditions=DEFAULT_PRECONDITIONS)


def _format_edge_yaml(edge: dict) -> str:
    """Format an edge as a YAML list item at column 0, matching the
    formatting of the existing edges in motion_graph.yaml.

    Hand-formats rather than using yaml.dump to control field order
    (matching the worked sample) and keep diffs small.
    """
    lines = [f"- from: {edge['from']}", f"  to: {edge['to']}"]
    lines.append(f"  mode: {edge['mode']}")
    if edge.get("speed") is not None:
        lines.append(f"  speed: {edge['speed']}")
    if edge.get("preconditions"):
        lines.append(f"  preconditions: {list(edge['preconditions'])}")
    if edge.get("comment"):
        # Quote the comment to handle any special characters.
        safe = str(edge["comment"]).replace('"', '\\"')
        lines.append(f'  comment: "{safe}"')
    return "\n".join(lines) + "\n"


# Test endpoint for log streaming
@app.post("/test/log")
async def test_log():
    """Test endpoint to generate log messages for debugging."""
    logger.info("Test info message from API")
    logger.warning("Test warning message from API") 
    logger.error("Test error message from API")
    return {"message": "Test logs sent"}

# WebSocket endpoint for real-time updates
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time status updates"""
    await manager.connect(websocket)
    try:
        # Send initial status on connect
        await broadcast_status_update()
        while True:
            # Keep connection alive, listen for messages if needed
            data = await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        manager.disconnect(websocket)

# Development server
if __name__ == "__main__":
    # Get host and port from environment variables or use defaults
    host = os.environ.get("XARM_API_HOST", "0.0.0.0")
    port = int(os.environ.get("XARM_API_PORT", 8000))
    
    logger.info(f"Starting server on {host}:{port}")
    
    # Example of how to connect automatically on startup (optional)
    # This can be useful for development or dedicated server setups
    # Note: In a real production scenario, you might want to handle
    # connection via API calls for better control.
    
    # async def startup_connect():
    #     global controller
    #     logger.info("Attempting to auto-connect on startup...")
    #     try:
    #         controller = XArmController(auto_enable=True)
    #         if not controller.initialize():
    #             logger.error("Auto-connect failed during initialization.")
    #             controller = None
    #         else:
    #             logger.info("Auto-connect successful.")
    #     except Exception as e:
    #         logger.error(f"Auto-connect failed with exception: {e}")
    #         controller = None

    # app.add_event_handler("startup", startup_connect)
    
    uvicorn.run(app, host=host, port=port) 