# API Reference

This document provides a detailed reference for the PyxArm project's RESTful API. 

The API allows for comprehensive control over the xArm robot, its components, and the simulation environment.

**Installation:** First install PyxArm in development mode:
```bash
conda run -n sdl2-robots pip install -e .
```

Start the server using the PyxArm CLI:

```bash
# Start web interface and API server
pyxarm web

# Or specify custom host/port
pyxarm web --host 0.0.0.0 --port 8080

# Alternative method (without installing package)
conda run -n sdl2-robots python -m src.cli.main web
```

The API server runs on `http://127.0.0.1:6001` by default.

**Access Points:**
- 🌐 **Web UI**: http://localhost:6001/web/
- 📖 **API Docs**: http://localhost:6001/docs  
- 📡 **REST API**: http://localhost:6001

---

## Table of Contents

- [Server & Connection](#server--connection-management)
- [Position Snapshot](#get-positions)
- [Robot Arm Movement](#robot-arm-movement)
- [Components](#components)
  - [Gripper](#gripper)
    - [Open / Close](#post-gripperopen)
    - [Stroke control (Gen2)](#post-gripperstroke)
    - [Force control (Gen2)](#post-gripperforce)
    - [Position readback (Gen2)](#get-gripperposition)
  - [Linear Track](#linear-track)
- [System & Safety](#system--safety)
- [WebSocket Interface](#websocket-interface)

---

## Server & Connection Management

These endpoints manage the connection to the robot (real or simulated) and provide status information.

### `GET /api/configurations`

Retrieves a list of available connection configuration files from the `src/settings/` directory. These can be used with the `/connect` endpoint.

**Response `200 OK`**

```json
[
  "xarm5_docker_local.yaml",
  "xarm5_docker_server.yaml",
  "xarm5_config.yaml"
]
```

**Example**

```bash
curl -X GET "http://127.0.0.1:6001/api/configurations"
```

### `POST /connect`

Initializes the connection to the xArm controller. This is the first command that must be sent to interact with the robot. The connection can be configured in several ways.

**Request Body**

```json
{
  "host": "string",
  "model": "integer",
  "config_name": "string",
  "simulation_mode": "boolean",
  "safety_level": "string"
}
```

*   `host` (optional): IP address of the robot or simulator.
*   `model` (optional): The xArm model number (e.g., 5, 6, 7).
*   `config_name` (optional): The name of a configuration file (e.g., `"xarm5_docker_local.yaml"`) to load settings from.
*   `simulation_mode` (optional, default: `false`): Set to `true` to use the built-in software simulator without any hardware or Docker.
*   `safety_level` (optional, default: `"medium"`): Sets the initial safety level. Options are `"low"`, `"medium"`, `"high"`.

**Response `200 OK`**

```json
{
  "status": "success",
  "message": "Controller initialized successfully for xArm6 at 127.0.0.1"
}
```

**Examples**

1.  **Connect using a configuration file:**
    ```bash
    curl -X POST "http://127.0.0.1:6001/connect" -H "Content-Type: application/json" -d '{
      "config_name": "xarm5_docker_local.yaml"
    }'
    ```

2.  **Connect to a remote Docker simulator:**
    ```bash
    curl -X POST "http://127.0.0.1:6001/connect" -H "Content-Type: application/json" -d '{
      "host": "100.64.254.50",
      "model": 6
    }'
    ```

3.  **Connect using the built-in software simulation:**
    ```bash
    curl -X POST "http://127.0.0.1:6001/connect" -H "Content-Type: application/json" -d '{
      "model": 7,
      "simulation_mode": true
    }'
    ```

### `POST /disconnect`

Disconnects from the robot and shuts down the controller gracefully.

**Response `200 OK`**
```json
{
  "status": "success",
  "message": "Disconnected from the robot."
}
```

**Example**
```bash
curl -X POST "http://127.0.0.1:6001/disconnect"
```

### `GET /status`

Retrieves the current status of the robot and controller.

**Response `200 OK`**
```json
{
    "connected": true,
    "running": true,
    "error_code": 0,
    "safety_level": "medium",
    "model": 6,
    "components": {
        "gripper": { "connected": true, "enabled": true },
        "linear_track": { "connected": false, "enabled": false }
    }
}
```

**Example**
```bash
curl -X GET "http://127.0.0.1:6001/status"
```

### `GET /positions`

Returns a read-only snapshot of **all position sensors** — joint angles, Cartesian pose, linear track, and gripper — in a single call. No movement is performed.

**Response `200 OK`**
```json
{
    "joints": [0.0, -30.2, 12.4, 0.0, 17.8, 0.0],
    "cartesian": {
        "x": 305.1, "y": 0.0, "z": 210.4,
        "roll": 180.0, "pitch": 0.0, "yaw": 0.0
    },
    "track": { "available": true, "position": 452.0 },
    "gripper": { "available": true, "position": 110 }
}
```
*   `joints`: List of current joint angles in degrees (length matches the robot model: 5, 6, or 7).
*   `cartesian`: End-effector pose in mm and degrees.
*   `track.available`: `false` if no linear track is configured; `position` will be `null`.
*   `gripper.available`: `false` for grippers without position feedback (e.g. BioGripper Gen1); `position` will be `null`.

**Example**
```bash
curl -X GET "http://127.0.0.1:6001/positions"
```

---
## Robot Arm Movement

Endpoints for controlling the physical movement of the xArm.

### `POST /move/cartesian`

Moves the robot's end-effector to a specific Cartesian position (X, Y, Z) and orientation (roll, pitch, yaw).

**Request Body**
```json
{
    "x": "number",
    "y": "number",
    "z": "number",
    "roll": "number",
    "pitch": "number",
    "yaw": "number",
    "speed": "integer",
    "mvacc": "integer"
}
```
*   `speed` (optional, default: 100 mm/s)
*   `mvacc` (optional, default: 1000 mm/s²)

**Response `200 OK`**
```json
{ "status": "success", "message": "Move command sent" }
```

**Example**
```bash
curl -X POST "http://127.0.0.1:6001/move/cartesian" -H "Content-Type: application/json" -d '{
    "x": 300, "y": 0, "z": 250, "roll": 180, "pitch": 0, "yaw": 0
}'
```

### `POST /move/joints`

Moves the robot to a specific pose by setting the angle for each joint.

**Request Body**
```json
{
    "j1": "number",
    "j2": "number",
    "j3": "number",
    "j4": "number",
    "j5": "number",
    "j6": "number",
    "j7": "number",
    "speed": "integer",
    "mvacc": "integer"
}
```
*   Provide angles for the joints available on your model (e.g., `j1` to `j5` for xArm5).
*   `speed` (optional, default: 20 deg/s)
*   `mvacc` (optional, default: 200 deg/s²)

**Response `200 OK`**
```json
{ "status": "success", "message": "Move command sent" }
```

**Example**
```bash
curl -X POST "http://127.0.0.1:6001/move/joints" -H "Content-Type: application/json" -d '{
    "j1": 45, "j2": -30, "j3": 0, "j4": 0, "j5": 30, "j6": 0
}'
```

### `GET /position/cartesian`

Retrieves the current Cartesian position and orientation of the end-effector.

**Response `200 OK`**
```json
{
    "x": 300.1, "y": 0.5, "z": 250.2, "roll": 179.9, "pitch": 0.1, "yaw": -0.2
}
```

**Example**
```bash
curl -X GET "http://127.0.0.1:6001/position/cartesian"
```

### `GET /position/joints`

Retrieves the current angles of all robot joints.

**Response `200 OK`**
```json
{
    "j1": 45.1, "j2": -30.0, "j3": 0.2, "j4": 0.1, "j5": 29.8, "j6": -0.1, "j7": 0.0
}
```

**Example**
```bash
curl -X GET "http://127.0.0.1:6001/position/joints"
```

---
## Components

These endpoints control attached components like the gripper and linear track.

### Gripper

All gripper endpoints accept an optional JSON body. Omitting the body uses config defaults.

#### `POST /gripper/open`

Opens the gripper to its fully open position.

**Request Body** (optional)
```json
{
    "speed": 1000,
    "force": 50,
    "wait": true
}
```
*   `speed` (optional): Movement speed. BioGripper Gen2 range: 0–4000. Default: 1000.
*   `force` (optional): Gripping force %. BioGripper Gen2 range: 1–100. Default: 50.
*   `wait` (optional, default `true`): Wait for motion to complete before returning.

**Response `200 OK`**
```json
{ "message": "Open gripper command completed." }
```
**Example**
```bash
curl -X POST "http://127.0.0.1:6001/gripper/open"
# With explicit speed and force:
curl -X POST "http://127.0.0.1:6001/gripper/open" -H "Content-Type: application/json" -d '{
    "speed": 800, "force": 30
}'
```

#### `POST /gripper/close`

Closes the gripper to its fully closed position.

**Request Body** (optional) — same fields as `/gripper/open`.

**Response `200 OK`**
```json
{ "message": "Close gripper command completed." }
```
**Example**
```bash
curl -X POST "http://127.0.0.1:6001/gripper/close"
```

#### `POST /gripper/stroke`

*BioGripper Gen2 / Standard / RobotIQ only.* Moves the gripper to a specific absolute stroke position.

**BioGripper Gen2 position range: 71 (fully closed) – 150 (fully open).**

**Request Body**
```json
{
    "stroke": 110,
    "speed": 1000,
    "force": 50,
    "wait": true
}
```
*   `stroke` (required): Target position in SDK units.
*   `speed`, `force`, `wait`: same as open/close.

**Response `200 OK`**
```json
{ "message": "Gripper moved to stroke 110." }
```
**Example**
```bash
curl -X POST "http://127.0.0.1:6001/gripper/stroke" -H "Content-Type: application/json" -d '{
    "stroke": 110
}'
```

#### `POST /gripper/force`

*BioGripper Gen2 only.* Sets the gripping force independently of any position command.

**Request Body**
```json
{ "force": 60 }
```
*   `force` (required): Force percentage, range 1–100.

**Response `200 OK`**
```json
{ "message": "Gripper force set to 60." }
```
**Example**
```bash
curl -X POST "http://127.0.0.1:6001/gripper/force" -H "Content-Type: application/json" -d '{"force": 60}'
```

#### `GET /gripper/position`

*BioGripper Gen2 / Standard only.* Returns the current gripper stroke position without moving.

**Response `200 OK`**
```json
{ "position": 110 }
```
**Response `404`** if the installed gripper does not support position readback.

**Example**
```bash
curl -X GET "http://127.0.0.1:6001/gripper/position"
```

### Linear Track

#### `POST /track/move`

Moves the linear track to an absolute position.

**Request Body**
```json
{
    "position": "number",
    "speed": "integer"
}
```
*   `speed` (optional, default: 100 mm/s)

**Response `200 OK`**
```json
{ "status": "success", "message": "Track move command sent" }
```
**Example**
```bash
curl -X POST "http://127.0.0.1:6001/track/move" -H "Content-Type: application/json" -d '{
    "position": 500
}'
```

#### `POST /track/move/location`

Moves the linear track to a pre-defined named location from `location_config.yaml`.

**Request Body**
```json
{ "location_name": "string" }
```

**Response `200 OK`**
```json
{ "status": "success", "message": "Track moving to location: start" }
```
**Example**
```bash
curl -X POST "http://127.0.0.1:6001/track/move/location" -H "Content-Type: application/json" -d '{
    "location_name": "start"
}'
```

#### `GET /track/status`

Retrieves the current status of the linear track.

**Response `200 OK`**
```json
{ "position": 500.1 }
```
**Example**
```bash
curl -X GET "http://127.0.0.1:6001/track/status"
```

### Force Torque Sensor

The 6-axis force torque sensor provides three main functionalities:
1. **Safety monitoring** - Alert when force/torque exceeds thresholds
2. **Linear force-controlled movement** - Move until force threshold is reached (for button pressing, drawer pulling)
3. **Joint torque-controlled movement** - Move joint until torque threshold is reached

#### `POST /force-torque/enable`

Enables the 6-axis force torque sensor.

**Response `200 OK`**
```json
{ "message": "Force torque sensor enabled successfully." }
```

**Example**
```bash
curl -X POST "http://127.0.0.1:6001/force-torque/enable"
```

#### `POST /force-torque/disable`

Disables the 6-axis force torque sensor.

**Response `200 OK`**
```json
{ "message": "Force torque sensor disabled successfully." }
```

**Example**
```bash
curl -X POST "http://127.0.0.1:6001/force-torque/disable"
```

#### `POST /force-torque/calibrate`

Calibrates the force torque sensor to zero.

**Request Body**
```json
{
    "samples": 100,
    "delay": 0.1
}
```
*   `samples` (optional): Number of calibration samples (default: 100)
*   `delay` (optional): Delay between samples in seconds (default: 0.1)

**Response `200 OK`**
```json
{ "message": "Force torque sensor calibration started." }
```

**Example**
```bash
curl -X POST "http://127.0.0.1:6001/force-torque/calibrate" -H "Content-Type: application/json" -d '{
    "samples": 100,
    "delay": 0.1
}'
```

#### `GET /force-torque/data`

Gets current force torque sensor data.

**Response `200 OK`**
```json
{
    "data": [1.2, -0.5, 15.3, 0.1, 0.2, -0.3],
    "magnitude": {
        "force_magnitude": 15.4,
        "torque_magnitude": 0.37,
        "total_magnitude": 15.4
    },
    "direction": {
        "force_direction": [0.078, -0.032, 0.994],
        "torque_direction": [0.270, 0.541, -0.811],
        "force_magnitude": 15.4,
        "torque_magnitude": 0.37
    },
    "calibrated": true
}
```
*   `data`: [fx, fy, fz, tx, ty, tz] in Newtons and Nm
*   `magnitude`: Magnitude of force and torque vectors
*   `direction`: Normalized direction vectors (if above dead zone)
*   `calibrated`: Whether sensor has been calibrated

**Example**
```bash
curl -X GET "http://127.0.0.1:6001/force-torque/data"
```

#### `GET /force-torque/status`

Gets comprehensive force torque sensor status.

**Response `200 OK`**
```json
{
    "enabled": true,
    "calibrated": true,
    "last_reading": [1.2, -0.5, 15.3, 0.1, 0.2, -0.3],
    "zero_point": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "history_length": 150,
    "alerts_active": false,
    "magnitude": {
        "force_magnitude": 15.4,
        "torque_magnitude": 0.37,
        "total_magnitude": 15.4
    },
    "direction": {
        "force_direction": [0.078, -0.032, 0.994],
        "torque_direction": [0.270, 0.541, -0.811],
        "force_magnitude": 15.4,
        "torque_magnitude": 0.37
    }
}
```

**Example**
```bash
curl -X GET "http://127.0.0.1:6001/force-torque/status"
```

#### `POST /force-torque/check-safety`

Checks if force/torque exceeds safety thresholds and triggers alerts.

**Response `200 OK`**
```json
{
    "violation_detected": false,
    "message": "Safety check completed."
}
```

**Example**
```bash
curl -X POST "http://127.0.0.1:6001/force-torque/check-safety"
```

#### `POST /force-torque/move-until-force`

Moves in a linear direction until a force threshold is reached.

**Request Body**
```json
{
    "direction": [0, 0, -1],
    "force_threshold": 20.0,
    "speed": 50,
    "timeout": 30.0
}
```
*   `direction`: Direction vector [x, y, z] (normalized)
*   `force_threshold` (optional): Force threshold in Newtons (default from config)
*   `speed` (optional): Movement speed in mm/s (default from config)
*   `timeout`: Maximum time to wait in seconds

**Response `200 OK`**
```json
{ "message": "Force-controlled movement started." }
```

**Example**
```bash
curl -X POST "http://127.0.0.1:6001/force-torque/move-until-force" -H "Content-Type: application/json" -d '{
    "direction": [0, 0, -1],
    "force_threshold": 20.0,
    "speed": 50,
    "timeout": 30.0
}'
```

#### `POST /force-torque/move-joint-until-torque`

Moves a specific joint until a torque threshold is reached.

**Request Body**
```json
{
    "joint_id": 5,
    "target_angle": 45.0,
    "torque_threshold": 2.0,
    "speed": 10,
    "timeout": 30.0
}
```
*   `joint_id`: Joint number (1-7)
*   `target_angle`: Target angle in degrees
*   `torque_threshold` (optional): Torque threshold in Nm (default from config)
*   `speed` (optional): Movement speed in deg/s (default from config)
*   `timeout`: Maximum time to wait in seconds

**Response `200 OK`**
```json
{ "message": "Torque-controlled joint movement started." }
```

**Example**
```bash
curl -X POST "http://127.0.0.1:6001/force-torque/move-joint-until-torque" -H "Content-Type: application/json" -d '{
    "joint_id": 5,
    "target_angle": 45.0,
    "torque_threshold": 2.0,
    "speed": 10,
    "timeout": 30.0
}'
```

---
## System & Safety

Endpoints for system-level configuration and monitoring.

### `POST /safety/level`

Sets the system's safety level, which can affect speed, acceleration, and collision sensitivity.

**Request Body**
```json
{ "level": "string" }
```
*   `level`: `"low"`, `"medium"`, or `"high"`

**Response `200 OK`**
```json
{ "status": "success", "message": "Safety level set to high" }
```
**Example**
```bash
curl -X POST "http://127.0.0.1:6001/safety/level" -H "Content-Type: application/json" -d '{
    "level": "high"
}'
```

### `GET /performance`

Retrieves performance statistics from the controller, such as API latency and command processing time.

**Response `200 OK`**
```json
{
    "api_latency_ms": 1.5,
    "command_rate_hz": 50.2,
    "cpu_usage_percent": 15.7
}
```
**Example**
```bash
curl -X GET "http://127.0.0.1:6001/performance"
```

---
## WebSocket Interface

A WebSocket is available for receiving real-time status updates from the controller.

### `GET /ws`

Establishes a WebSocket connection. Once connected, the server will push status updates automatically at a regular interval. This is the same data that drives the web UI.

**Connection URL**: `ws://127.0.0.1:6001/ws`

**Example Message (from server)**
```json
{
    "connected": true,
    "running": true,
    "error_code": 0,
    "mode": 0,
    "state": 4,
    "safety_level": "medium",
    "position_cartesian": [300.1, 0.5, 250.2, 179.9, 0.1, -0.2],
    "position_joints": [45.1, -30.0, 0.2, 0.1, 29.8, -0.1, 0.0],
    "components": {
        "gripper": { "connected": true, "position": 850.0 },
        "linear_track": { "connected": true, "position": 500.1 }
    }
}
``` 