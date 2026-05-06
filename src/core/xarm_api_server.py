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
from datetime import datetime
from fastapi import FastAPI, HTTPException, BackgroundTasks, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
import uvicorn

try:
    from .xarm_controller import XArmController, SafetyLevel, ComponentState
    from .xarm_utils import load_config
    from .models import (
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
except ImportError:
    from core.xarm_controller import XArmController, SafetyLevel, ComponentState
    from core.xarm_utils import load_config
    from core.models import (
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
    simulation_mode: bool = Field(default=False, description="Enable software simulation mode (no hardware required).")
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
    wait: bool = Field(default=True, description="Wait for operation to complete.")

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
        try:
            full_config = load_config(config_path)
            profiles = full_config.get('profiles', {})
            return sorted(list(profiles.keys()))
        except FileNotFoundError:
            continue
        except Exception as e:
            logger.error(f"Failed to read profiles from {config_path}: {e}")
            continue
    
    raise HTTPException(status_code=404, detail="Main xarm_config.yaml not found in any expected location.")


@app.post("/connect")
async def connect_robot(request: ConnectionRequest, background_tasks: BackgroundTasks):
    """
    Connect to the robot controller.

    This endpoint initializes the `XArmController`, allowing you to choose
    between hardware and simulation modes, and set the initial safety level.
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
            simulation_mode=request.simulation_mode,
            safety_level=request.get_safety_level_enum()
        )
        
        if controller.initialize():
            background_tasks.add_task(broadcast_status_update)
            return {
                "message": f"Successfully connected in {'Simulation' if request.simulation_mode else 'Hardware'} mode.",
                "connection_details": {
                    "host": controller.host,
                    "port": controller.xarm_config.get('port', 18333),
                    "profile_name": request.profile_name or 'custom',
                    "simulation_mode": request.simulation_mode
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

@app.get("/status/performance")
async def get_performance_status():
    """Get detailed performance and maintenance status (hardware only)."""
    c = get_controller()
    if c.simulation_mode:
        raise HTTPException(status_code=400, detail="Performance monitoring is not available in simulation mode.")
    return {
        "performance_metrics": c.get_performance_metrics(),
        "maintenance_status": c.get_maintenance_status(),
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
        success = c.move_to_position(
            x=request.x, y=request.y, z=request.z,
            roll=request.roll, pitch=request.pitch, yaw=request.yaw,
            speed=request.speed,
            check_collision=request.check_collision,
            wait=request.wait
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
        success = c.move_joints(
            angles=request.angles,
            speed=request.speed,
            acceleration=request.acceleration,
            check_collision=request.check_collision,
            wait=request.wait
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
        success = c.move_relative(
            dx=request.dx, dy=request.dy, dz=request.dz,
            droll=request.droll, dpitch=request.dpitch, dyaw=request.dyaw,
            speed=request.speed
        )
        if not success:
            logger.error("Failed to move relative.")
        await broadcast_status_update()

    background_tasks.add_task(move_task)
    return {"message": "Move relative command accepted."}

@app.post("/move/location")
async def move_to_location(request: LocationRequest, background_tasks: BackgroundTasks):
    """Move the robot to a pre-defined named location."""
    c = get_controller()
    
    async def move_task():
        success = c.move_to_named_location(
            location_name=request.location_name,
            speed=request.speed
        )
        if not success:
            logger.error(f"Failed to move to named location: {request.location_name}")
        await broadcast_status_update()
    
    background_tasks.add_task(move_task)
    return {"message": f"Move to location '{request.location_name}' command accepted."}

@app.post("/move/home")
async def move_home(background_tasks: BackgroundTasks):
    """Move robot to home position"""
    ctrl = get_controller()
    
    try:
        result = ctrl.go_home()
        
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
    
    # Execute stop immediately (not in background) for fastest response
    c.stop_motion()
    logger.info("Stop command issued immediately.")
    
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
        result = ctrl.clear_errors()
        
        if result:
            background_tasks.add_task(broadcast_status_update)
            return {
                "message": "All errors and warnings cleared successfully",
                "timestamp": datetime.now().isoformat()
            }
        else:
            raise HTTPException(status_code=500, detail="Failed to clear all errors")
            
    except Exception as e:
        logger.error(f"Clear errors failed: {e}")
        raise HTTPException(status_code=500, detail=f"Clear errors failed: {str(e)}")

@app.post("/robot/enable")
async def enable_robot():
    """Re-enable robot motion after emergency stop."""
    c = get_controller()
    
    if c.simulation_mode:
        logger.info("Simulation mode: Robot motion re-enabled")
        c.alive = True
        c.states['arm'] = ComponentState.ENABLED
    else:
        # Clear any errors first
        if hasattr(c.arm, 'clean_error'):
            c.arm.clean_error()
        if hasattr(c.arm, 'clean_warn'):
            c.arm.clean_warn()
        
        # Re-enable motion
        if hasattr(c.arm, 'motion_enable'):
            result = c.arm.motion_enable(enable=True)
            if result != 0 and result is not None:
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
async def open_gripper(request: GripperRequest, background_tasks: BackgroundTasks):
    """Open the attached gripper."""
    c = get_controller()

    async def gripper_task():
        success = c.open_gripper(speed=request.speed, wait=request.wait)
        if not success:
            logger.error("Failed to open gripper.")
        await broadcast_status_update()

    background_tasks.add_task(gripper_task)
    return {"message": "Open gripper command accepted."}

@app.post("/gripper/close")
async def close_gripper(request: GripperRequest, background_tasks: BackgroundTasks):
    """Close the attached gripper."""
    c = get_controller()

    async def gripper_task():
        success = c.close_gripper(speed=request.speed, wait=request.wait)
        if not success:
            logger.error("Failed to close gripper.")
        await broadcast_status_update()
    
    background_tasks.add_task(gripper_task)
    return {"message": "Close gripper command accepted."}

@app.post("/gripper/move/stroke")
async def move_gripper_stroke(request: dict, background_tasks: BackgroundTasks):
    """Move gripper to specific stroke position (for non-bio grippers)."""
    c = get_controller()
    
    stroke = request.get('stroke')
    if stroke is None:
        raise HTTPException(status_code=400, detail="Stroke value is required")

    async def gripper_task():
        success = c.move_gripper_to_stroke(stroke=stroke)
        if not success:
            logger.error(f"Failed to move gripper to stroke {stroke}.")
        await broadcast_status_update()
    
    background_tasks.add_task(gripper_task)
    return {"message": f"Move gripper to stroke {stroke} command accepted."}

# Linear track endpoints
@app.post("/track/move")
async def move_track(request: TrackRequest, background_tasks: BackgroundTasks):
    """Move the linear track to a specific position."""
    c = get_controller()

    async def track_task():
        success = c.move_track_to_position(position=request.position, speed=request.speed, wait=request.wait)
        if not success:
            logger.error("Failed to move linear track.")
        await broadcast_status_update()

    background_tasks.add_task(track_task)
    return {"message": "Move track command accepted."}

@app.post("/track/move/location")
async def move_track_to_location(request: TrackLocationRequest, background_tasks: BackgroundTasks):
    """Move the linear track to a pre-configured named location."""
    try:
        c = get_controller()

        async def track_task():
            try:
                success = c.move_track_to_named_location(
                    location_name=request.location_name,
            speed=request.speed,
            wait=request.wait
        )
                if not success:
                    logger.error(f"Failed to move track to named location: {request.location_name}")
                await broadcast_status_update()
            except Exception as e:
                logger.error(f"Exception in track move task: {e}", exc_info=True)

        background_tasks.add_task(track_task)
        return {"message": f"Move track to location '{request.location_name}' command accepted."}
    except Exception as e:
        logger.error(f"Track move location failed: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Track move location failed: {str(e)}")

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
        success = c.calibrate_force_torque_sensor(
            samples=request.samples,
            delay=request.delay
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
        success = c.move_until_force(
            direction=request.direction,
            force_threshold=request.force_threshold,
            speed=request.speed,
            timeout=request.timeout
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
        success = c.move_joint_until_torque(
            joint_id=request.joint_id,
            target_angle=request.target_angle,
            torque_threshold=request.torque_threshold,
            speed=request.speed,
            timeout=request.timeout
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
        success = c.move_plate_linear(
            target_location=request.target_location,
            num_steps=request.num_steps,
            speed=request.speed,
            wait_between_steps=request.wait_between_steps
        )
        if not success:
            logger.error(f"Failed to move linearly to {request.target_location}")
        await broadcast_status_update()
    
    background_tasks.add_task(plate_linear_task)
    return {"message": f"Linear movement to '{request.target_location}' command accepted."}

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