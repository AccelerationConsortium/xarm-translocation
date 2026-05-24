#!/usr/bin/env python3
"""
FastAPI Server for xArm Translocation Control

This module provides a REST API wrapper around the XArmController class
to enable web-based control and monitoring of xArm robots.
"""

# TODO: planning to implement DI/DO for safety light and additional e-stop

import asyncio
import json
import logging
import os
from contextlib import asynccontextmanager
from typing import Dict, List, Optional, Any
from datetime import datetime, timezone
from fastapi import FastAPI, HTTPException, BackgroundTasks, Header, Response, WebSocket, WebSocketDisconnect
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
    )
    from .motion_graph import (
        EdgeNotAllowedError, GraphError, GraphMode, RecoveryMismatch,
        UnknownNodeError,
    )
    from .claims import ClaimConflict, InvalidClaimToken
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
    )
    from core.motion_graph import (
        EdgeNotAllowedError, GraphError, GraphMode, RecoveryMismatch,
        UnknownNodeError,
    )
    from core.claims import ClaimConflict, InvalidClaimToken

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

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
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception as e:
                logger.error(f"Error broadcasting message: {e}")

manager = ConnectionManager()

# Custom logging handler to broadcast logs to WebSocket clients
class WebSocketLogHandler(logging.Handler):
    def __init__(self):
        super().__init__()
        self.setLevel(logging.INFO)
        formatter = logging.Formatter('%(levelname)s: %(message)s')
        self.setFormatter(formatter)

    def emit(self, record):
        try:
            msg = self.format(record)
            log_type = 'error' if record.levelno >= logging.ERROR else 'warning' if record.levelno >= logging.WARNING else 'info'
            
            # Create log message for WebSocket
            log_data = {
                'type': 'log',
                'log_message': msg,
                'log_type': log_type,
                'timestamp': record.created
            }
            
            # Store log data for WebSocket broadcast
            # Use a different approach - store in a queue for periodic broadcast
            if not hasattr(self, 'log_queue'):
                self.log_queue = []
            self.log_queue.append(log_data)
            
            # For debugging: print to console to verify handler is working
            print(f"LOG HANDLER: {log_type.upper()} - {msg}")
            
        except Exception as e:
            # For debugging: print to console if WebSocket broadcast fails
            print(f"WebSocket log handler failed: {e}")
            pass  # Don't let logging errors break the app

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
    location_name: str = Field(description="Name of the location defined in position_config.yaml")
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
    target_location: str = Field(description="Name of the target location from position_config.yaml")
    num_steps: int = Field(default=1, ge=1, le=100, description="Number of interpolation steps (1-100)")
    speed: Optional[float] = Field(default=None, description="Movement speed (validated by safety level)")
    wait_between_steps: float = Field(default=0.1, ge=0.0, le=5.0, description="Delay between steps in seconds (0-5)")


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


class GraphMoveToRequest(BaseModel):
    """Body of POST /control/graph/move_to.

    Wraps /move/location for graph-aware callers: takes a node id and
    looks up the underlying arm-pose preset name. Convenient for the
    web UI's reachable-node buttons since node ids and preset names
    can differ (e.g. node 'uplc_draw_approach' has arm 'uplc_draw_home').
    """
    node_id: str = Field(description="Graph node id to move to")
    speed: Optional[float] = Field(default=None, description="Movement speed (may be capped by edge.speed in STRICT)")

# Application lifespan management
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    logger.info("Starting xArm API Server")
    
    # Start background tasks
    log_task = asyncio.create_task(broadcast_logs())
    
    yield
    
    # Shutdown
    log_task.cancel()
    global controller
    if controller:
        logger.info("Disconnecting from robot...")
        controller.disconnect()
    logger.info("xArm API Server shutdown complete")

# Create FastAPI app
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
            if hasattr(ws_handler, 'log_queue') and ws_handler.log_queue:
                # Broadcast all queued logs
                logs_to_send = ws_handler.log_queue.copy()
                ws_handler.log_queue.clear()
                
                for log_data in logs_to_send:
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

def create_error_response(message: str, status_code: int = 500) -> JSONResponse:
    """Create standardized error response"""
    return JSONResponse(
        status_code=status_code,
        content={"error": message, "timestamp": datetime.now().isoformat()}
    )

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


@app.post("/connect")
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

        if controller.initialize():
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
            controller = None
            raise HTTPException(status_code=500, detail="Failed to initialize robot connection. Check logs for details.")
            
    except Exception as e:
        controller = None
        logger.error(f"Connection failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"An unexpected error occurred during connection: {e}")

@app.post("/disconnect")
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
            controller.disconnect()
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

    # Joints
    joints_raw = c.get_current_joints()
    joints = joints_raw[:c.num_joints] if joints_raw else None

    # Cartesian pose
    cart_raw = c.get_current_position()
    if cart_raw and len(cart_raw) >= 6:
        cartesian = {"x": cart_raw[0], "y": cart_raw[1], "z": cart_raw[2],
                     "roll": cart_raw[3], "pitch": cart_raw[4], "yaw": cart_raw[5]}
    else:
        cartesian = None

    # Linear track
    if c.has_track():
        track_pos = c.get_track_position()
        track = {"available": True, "position": track_pos}
    else:
        track = {"available": False, "position": None}

    # Gripper
    gripper_pos = c.get_gripper_position()
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
            os.path.join('src', 'settings', 'position_config.yaml'),
            os.path.join('settings', 'position_config.yaml'),
            os.path.join(os.path.dirname(__file__), '..', 'settings', 'position_config.yaml')
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
            logger.warning("position_config.yaml not found in any expected location, returning empty list.")
            locations = []
            positions = {}
        
        return {"locations": locations, "positions": positions}
    except Exception as e:
        logger.error(f"Get arm positions failed: {e}")
        raise HTTPException(status_code=500, detail=f"Get arm positions failed: {str(e)}")

# Movement endpoints
@app.post("/move/position")
async def move_to_position(request: PositionRequest, background_tasks: BackgroundTasks):
    """Move the robot to a specific Cartesian position."""
    c = get_controller()
    
    async def move_task():
        # Run the blocking SDK call in a worker thread so the event loop
        # stays free to handle STOP and status polls while the arm moves.
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
        await broadcast_status_update()

    background_tasks.add_task(move_task)
    return {"message": "Move to position command accepted."}

@app.post("/move/joints")
async def move_joints(request: JointRequest, background_tasks: BackgroundTasks):
    """Move the robot to a specific joint configuration."""
    c = get_controller()

    async def move_task():
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
        await broadcast_status_update()
    
    background_tasks.add_task(move_task)
    return {"message": "Move joints command accepted."}

@app.post("/move/relative")
async def move_relative(request: RelativeRequest, background_tasks: BackgroundTasks):
    """Move the robot relative to its current position."""
    c = get_controller()
    
    async def move_task():
        success = await asyncio.to_thread(
            c.move_relative,
            dx=request.dx, dy=request.dy, dz=request.dz,
            droll=request.droll, dpitch=request.dpitch, dyaw=request.dyaw,
            speed=request.speed,
        )
        if not success:
            logger.error("Failed to move relative.")
        await broadcast_status_update()

    background_tasks.add_task(move_task)
    return {"message": "Move relative command accepted."}

@app.post("/move/location")
async def move_to_location(request: LocationRequest, background_tasks: BackgroundTasks):
    """Move the robot to a pre-defined named location.

    Runs synchronously via to_thread so a STRICT-mode rejection surfaces
    as HTTP 409 (rather than a silent background failure) and the caller
    learns the actual outcome. STOP remains responsive because the move
    runs in a worker thread, not on the event loop.
    """
    c = get_controller()

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

    background_tasks.add_task(broadcast_status_update)
    if not success:
        logger.error(f"Failed to move to named location: {request.location_name}")
        raise HTTPException(
            status_code=500,
            detail=f"Move to '{request.location_name}' failed",
        )
    return {"message": f"Moved to '{request.location_name}'."}

@app.post("/move/home")
async def move_home(background_tasks: BackgroundTasks):
    """Move robot to home position"""
    ctrl = get_controller()
    
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
            
    except Exception as e:
        logger.error(f"Home movement failed: {e}")
        raise HTTPException(status_code=500, detail=f"Home movement failed: {str(e)}")

@app.post("/move/stop")
async def stop_movement(background_tasks: BackgroundTasks):
    """Stop all robot motion immediately."""
    c = get_controller()

    try:
        # Execute stop immediately (not in background) for fastest response.
        # Run via to_thread so the SDK call doesn't block the event loop
        # if the underlying TCP/USB layer stalls.
        stopped = await asyncio.to_thread(c.stop_motion)
        if not stopped:
            raise HTTPException(status_code=500, detail="Stop command failed.")
        logger.info("Stop command issued immediately.")
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

@app.post("/clear/errors")
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

@app.post("/robot/enable")
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

@app.post("/component/enable")
async def enable_component(request: ComponentRequest):
    """Enable a specific component (gripper, track, or force_torque)."""
    c = get_controller()
    component = request.component.lower()
    success = False
    if component == 'gripper':
        success = c.enable_gripper_component()
    elif component == 'track':
        success = c.enable_track_component()
    elif component == 'force_torque':
        success = c.enable_force_torque_sensor()
    else:
        raise HTTPException(status_code=400, detail="Invalid component specified. Use 'gripper', 'track', or 'force_torque'.")
    
    await broadcast_status_update()
    if success:
        return {"message": f"Component '{component}' enabled successfully."}
    else:
        raise HTTPException(status_code=500, detail=f"Failed to enable component '{component}'.")

@app.post("/component/disable")
async def disable_component(request: ComponentRequest):
    """Disable a specific component (gripper, track, or force_torque)."""
    c = get_controller()
    component = request.component.lower()
    success = False
    if component == 'gripper':
        success = c.disable_gripper_component()
    elif component == 'track':
        success = c.disable_track_component()
    elif component == 'force_torque':
        success = c.disable_force_torque_sensor()
    else:
        raise HTTPException(status_code=400, detail="Invalid component specified. Use 'gripper', 'track', or 'force_torque'.")

    await broadcast_status_update()
    if success:
        return {"message": f"Component '{component}' disabled successfully."}
    else:
        raise HTTPException(status_code=500, detail=f"Failed to disable component '{component}'.")

@app.post("/velocity/cartesian")
async def set_cartesian_velocity(request: VelocityRequest):
    """Set the Cartesian velocity of the robot arm."""
    c = get_controller()
    velocities = [request.vx, request.vy, request.vz, request.vroll, request.vpitch, request.vyaw]
    
    if not c.set_cartesian_velocity(velocities):
        raise HTTPException(status_code=500, detail="Failed to set Cartesian velocity.")
    
    return {"message": "Cartesian velocity set successfully."}

# Gripper endpoints
@app.post("/gripper/open")
async def open_gripper(request: Optional[GripperRequest] = None):
    """Open the attached gripper."""
    c = get_controller()
    request = request or GripperRequest()

    try:
        success = c.open_gripper(speed=request.speed, force=request.force, wait=request.wait)
        await broadcast_status_update()
        if not success:
            raise HTTPException(status_code=500, detail="Failed to open gripper.")
        return {"message": "Open gripper command completed."}
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Open gripper failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Open gripper failed: {str(e)}")

@app.post("/gripper/close")
async def close_gripper(request: Optional[GripperRequest] = None):
    """Close the attached gripper."""
    c = get_controller()
    request = request or GripperRequest()

    try:
        success = c.close_gripper(speed=request.speed, force=request.force, wait=request.wait)
        await broadcast_status_update()
        if not success:
            raise HTTPException(status_code=500, detail="Failed to close gripper.")
        return {"message": "Close gripper command completed."}
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Close gripper failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Close gripper failed: {str(e)}")

@app.post("/gripper/move/stroke")
async def move_gripper_stroke(request: GripperStrokeRequest):
    """Move gripper to a specific stroke position."""
    c = get_controller()

    try:
        success = c.move_gripper_to_stroke(
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

@app.post("/gripper/force")
async def set_gripper_force(request: GripperForceRequest):
    """Set gripping force for grippers that support force control."""
    c = get_controller()
    if not c.set_gripper_force(request.force):
        raise HTTPException(status_code=500, detail="Failed to set gripper force.")
    await broadcast_status_update()
    return {"message": f"Gripper force set to {request.force}."}

@app.get("/gripper/position")
async def get_gripper_position():
    """Get gripper stroke/position when supported by the installed gripper."""
    c = get_controller()
    position = c.get_gripper_position()
    if position is None:
        raise HTTPException(status_code=404, detail="Gripper position is not available.")
    return {"position": position}

# Linear track endpoints
@app.post("/track/move")
async def move_track(request: TrackRequest, background_tasks: BackgroundTasks):
    """Move the linear track to a specific position."""
    c = get_controller()

    async def track_task():
        success = await asyncio.to_thread(
            c.move_track_to_position,
            position=request.position, speed=request.speed, wait=request.wait,
        )
        if not success:
            logger.error("Failed to move linear track.")
        await broadcast_status_update()

    background_tasks.add_task(track_task)
    return {"message": "Move track command accepted."}

@app.post("/track/move/location")
async def move_track_to_location(request: TrackLocationRequest, background_tasks: BackgroundTasks):
    """Move the linear track to a pre-configured named location.

    Synchronous via to_thread (same rationale as /move/location).
    """
    c = get_controller()

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
    return {"position": c.get_track_position()}

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
@app.post("/force-torque/enable")
async def enable_force_torque_sensor():
    """Enable the 6-axis force torque sensor."""
    c = get_controller()
    
    if not c.has_force_torque_sensor():
        raise HTTPException(status_code=400, detail="Force torque sensor is not available or disabled in configuration.")
    
    success = c.enable_force_torque_sensor()
    await broadcast_status_update()
    
    if success:
        return {"message": "Force torque sensor enabled successfully."}
    else:
        raise HTTPException(status_code=500, detail="Failed to enable force torque sensor.")

@app.post("/force-torque/disable")
async def disable_force_torque_sensor():
    """Disable the 6-axis force torque sensor."""
    c = get_controller()
    
    success = c.disable_force_torque_sensor()
    await broadcast_status_update()
    
    if success:
        return {"message": "Force torque sensor disabled successfully."}
    else:
        raise HTTPException(status_code=500, detail="Failed to disable force torque sensor.")

@app.post("/force-torque/calibrate")
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
    
    data = c.get_force_torque_data()
    if data is None:
        raise HTTPException(status_code=500, detail="Failed to get force torque data.")
    
    return {
        "data": data,
        "magnitude": c.get_force_torque_magnitude(),
        "direction": c.get_force_torque_direction(),
        "calibrated": c.force_torque_calibrated
    }

@app.get("/force-torque/status")
async def get_force_torque_status():
    """Get comprehensive force torque sensor status."""
    c = get_controller()
    
    return c.get_force_torque_status()

@app.post("/force-torque/check-safety")
async def check_force_torque_safety():
    """Check if force/torque exceeds safety thresholds and trigger alerts."""
    c = get_controller()
    
    if not c.is_component_enabled('force_torque'):
        raise HTTPException(status_code=400, detail="Force torque sensor is not enabled.")
    
    violation_detected = c.check_force_torque_safety()
    
    return {
        "violation_detected": violation_detected,
        "message": "Safety check completed."
    }

@app.post("/force-torque/move-until-force")
async def move_until_force(request: ForceTorqueMovementRequest, background_tasks: BackgroundTasks):
    """Move in a linear direction until a force threshold is reached."""
    c = get_controller()

    async def force_movement_task():
        success = await asyncio.to_thread(
            c.move_until_force,
            direction=request.direction,
            force_threshold=request.force_threshold,
            speed=request.speed,
            timeout=request.timeout,
        )
        if not success:
            logger.error("Force-controlled movement failed or timed out.")
        await broadcast_status_update()

    background_tasks.add_task(force_movement_task)
    return {"message": "Force-controlled movement started."}

@app.post("/force-torque/move-joint-until-torque")
async def move_joint_until_torque(request: JointTorqueMovementRequest, background_tasks: BackgroundTasks):
    """Move a specific joint until a torque threshold is reached."""
    c = get_controller()

    async def torque_movement_task():
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
        await broadcast_status_update()

    background_tasks.add_task(torque_movement_task)
    return {"message": "Torque-controlled joint movement started."}

@app.post("/move/plate_linear")
async def move_plate_linear(request: PlateLinearRequest, background_tasks: BackgroundTasks):
    """Move linearly from current position to target with constant tool orientation."""
    c = get_controller()
    
    async def plate_linear_task():
        success = await asyncio.to_thread(
            c.move_plate_linear,
            target_location=request.target_location,
            num_steps=request.num_steps,
            speed=request.speed,
            wait_between_steps=request.wait_between_steps,
        )
        if not success:
            logger.error(f"Failed to move linearly to {request.target_location}")
        await broadcast_status_update()
    
    background_tasks.add_task(plate_linear_task)
    return {"message": f"Linear movement to '{request.target_location}' command accepted."}

# =============================================================================
# CLAIM PROTOCOL (STATUS_SPEC v1.1, Phase 3 — advisory)
# =============================================================================

@app.post("/control/claim", responses={409: {"model": ClaimRejection}})
async def acquire_claim(request: ClaimRequest):
    """Take the cooperative claim on this device.

    Returns 200 + ClaimResponse on success. Returns 409 + ClaimRejection
    when another active session holds the claim. Idempotent for the same
    session_id (token is rotated, TTL refreshed).
    """
    c = get_controller()
    try:
        record = c.claim_manager.acquire(
            owner=request.owner,
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


# =============================================================================
# MOTION GRAPH (Phase 2)
# =============================================================================

@app.get("/graph")
async def get_graph_state():
    """Snapshot of the motion-graph layer's current state.

    Useful for the web UI, debugging, and tests. Returns 404 when no
    graph is loaded (legacy config). The same data also rides in
    /status.details.motion_graph, but is exposed here too for direct
    polling without parsing the full status envelope.
    """
    c = get_controller()
    if c.motion_graph is None:
        raise HTTPException(status_code=404, detail="motion_graph.yaml not loaded")
    return {
        "graph_mode": c.graph_mode.value,
        "current_node": c.current_node,
        "reachable_nodes": c.reachable_node_ids(),
        "declared_payload": c.declared_payload,
        "arm_pose_name": c.last_arm_pose_name,
        "rail_location_name": c.last_rail_location_name,
        "last_transition": c.last_transition,
        "adjacency": c.motion_graph.adjacency_summary(),
    }


@app.post("/control/graph/move_to")
async def graph_move_to(request: GraphMoveToRequest, background_tasks: BackgroundTasks):
    """Move to a graph node by id.

    Looks up the node's arm-pose preset name and dispatches to
    move_to_named_location. Returns 409 (edge_not_allowed) when STRICT
    mode refuses the transition.
    """
    c = get_controller()
    if c.motion_graph is None:
        raise HTTPException(status_code=404, detail="motion_graph.yaml not loaded")
    try:
        node = c.motion_graph.node(request.node_id)
    except UnknownNodeError:
        raise HTTPException(status_code=409, detail=f"unknown node: {request.node_id!r}")

    try:
        success = await asyncio.to_thread(
            c.move_to_named_location,
            location_name=node.arm,
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

    background_tasks.add_task(broadcast_status_update)
    if not success:
        raise HTTPException(
            status_code=500,
            detail=f"Move to node '{request.node_id}' failed",
        )
    return {"message": f"Moved to node '{request.node_id}'", "current_node": c.current_node}


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
    return {
        "suggested_node": match.node_id,
        "arm_residual_deg": match.arm_residual,
        "rail_residual_mm": match.rail_residual,
        "gripper_match": match.gripper_match,
        "payload_match": match.payload_match,
        "within_tolerance": match.within_tolerance,
    }


@app.post("/control/graph/recover_to")
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
        result = c.recover_to(request.node_id, force=request.force)
    except UnknownNodeError as exc:
        raise HTTPException(status_code=409, detail=f"unknown node: {request.node_id!r}")
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


@app.post("/control/graph/mode")
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


@app.post("/control/graph/record")
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
    """Format an edge as a YAML list item with two-space indentation.

    Hand-formats rather than using yaml.dump to control field order
    (matching the worked sample) and keep diffs small.
    """
    lines = [f"  - from: {edge['from']}", f"    to:   {edge['to']}"]
    lines.append(f"    mode: {edge['mode']}")
    if edge.get("speed") is not None:
        lines.append(f"    speed: {edge['speed']}")
    if edge.get("preconditions"):
        lines.append(f"    preconditions: {list(edge['preconditions'])}")
    if edge.get("comment"):
        # Quote the comment to handle any special characters.
        safe = str(edge["comment"]).replace('"', '\\"')
        lines.append(f'    comment: "{safe}"')
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