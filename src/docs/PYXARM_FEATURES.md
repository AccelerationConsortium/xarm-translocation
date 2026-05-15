# Key Features

The `PyxArm` package provides a robust, feature-rich interface for controlling UFACTORY xArm robots, with a strong emphasis on safety, reliability, and ease of use. It supports multiple robot models, grippers, linear track, and force torque sensor, all managed through a single, unified API.

The system is designed to operate in three distinct stages, from pure software simulation to real hardware, ensuring code can be developed and tested safely and efficiently.

---

## Core Features

| Feature                 | Description                                                                                                                              |
| ----------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| **Multi-Model Support** | Natively supports **xArm5, xArm6, xArm7, and the 850 model**. The controller automatically adjusts joint limits and behavior.                |
| **Unified API**         | A single, consistent set of methods (`move_to_position`, `open_gripper`, `move_until_force`, etc.) control the arm, gripper, linear track, and force torque sensor.                  |
| **Component System**    | The arm, gripper, track, and force torque sensor are treated as components that can be enabled, disabled, and monitored independently (`enable_gripper_component`, `enable_force_torque_sensor`). |
| **Flexible Config**     | All settings are managed through clear `.yaml` files (`xarm_config.yaml`, `safety.yaml`, `force_torque_config.yaml`, etc.), separating configuration from code.        |
| **Named Locations**     | Pre-define and move to named locations (e.g., "home", "pickup_station") stored in `location_config.yaml` for repeatable operations.        |

---

## Simulation & Testing

The controller offers two distinct simulation modes, allowing for a comprehensive, three-stage testing workflow that moves from logic validation to full-physics simulation before deploying to hardware.

| Simulation Mode           | Description                                                                                                                                    |
| ------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| **1. Software Simulation**  | A lightweight, built-in mode (`simulation_mode=True`) that requires no external software. Ideal for fast development cycles and CI/CD.         |
| **2. Docker Simulation**    | Uses the official UFACTORY simulator for high-fidelity physics, realistic dynamics, and precise, mesh-based collision detection.                   |

*For a detailed guide, see the [**Testing Guide**](./SIMULATION_TESTING.md).*

---

## Safety & Error Handling

Safety is a primary design principle, with multiple layers of validation and intelligent error recovery.

| Safety Feature                  | Description                                                                                                                                                       |
| ------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Multi-Level Safety**          | Configure the `safety_level` (Strict, Medium, Loose) to automatically adjust speed, acceleration, and validation strictness.                                      |
| **Built-in Collision Checking** | All movement methods (`move_to_position`, `move_joints`) automatically perform collision checks against joint limits and workspace boundaries.                      |
| **Configurable Safety Zones**   | Define custom "keep-out" zones in `safety.yaml` to prevent the robot from entering restricted areas.                                                               |
| **Intelligent Error Recovery**  | The controller can automatically attempt to recover from common, non-critical errors. State and error history are tracked for diagnostics.                            |
| **Emergency Stop**              | A universal `stop_motion()` method provides an immediate halt for all robot, gripper, and track movements.                                                            |

---

## Force Torque Sensor

PyxArm includes comprehensive support for the 6-axis force torque sensor with three main functionalities:

| Force Torque Feature          | Description                                                                                                                                                            |
| ----------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Safety Monitoring**         | Real-time monitoring with configurable thresholds for force and torque in all axes. Automatic alerts when safety limits are exceeded.                                |
| **Force-Controlled Movement** | Linear movement until force threshold is reached (`move_until_force`) - perfect for button pressing, drawer pulling, and contact-based operations.                   |
| **Torque-Controlled Joints**  | Joint movement until torque threshold is reached (`move_joint_until_torque`) - ideal for screw turning, valve operations, and torque-sensitive tasks.               |
| **Direction Detection**       | Real-time force and torque direction analysis with configurable dead zones and smoothing for precise feedback.                                                        |
| **Auto Calibration**          | Automatic sensor calibration on startup with configurable sample count and delay for accurate zero-point reference.                                                  |

---

## Gripper & Linear Track

| Component Feature              | Description                                                                                                                                                               |
| ------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Universal Gripper API**      | `open_gripper()` and `close_gripper()` work seamlessly with any configured gripper type (**BioGripper Gen1, BioGripper Gen2, Standard, or RobotIQ**).                     |
| **BioGripper Gen2 Support**    | Full stroke and force control via `move_gripper_to_stroke()` and `set_gripper_force()`. Official SDK parameter ranges: Position 71–150, Speed 0–4000, Force 1–100 %. Position 71 = fully closed; 150 = fully open. Default speed: 1000. |
| **Input Validation**           | Stroke and force values are validated against configured ranges (`stroke_range`, `force_range` in `gripper_config.yaml`) before any SDK call is made.                     |
| **Integer Coercion**           | Float inputs for position, speed, force, and timeout are automatically rounded to integers before being passed to the xArm SDK (which requires integer arguments).        |
| **Linear Track Control**       | If `enable_track=True`, the controller provides validated motion control for the linear track, including position and speed limits defined in `linear_track_config.yaml`.  |
| **Position Snapshot**          | `GET /positions` returns joint angles, Cartesian pose, track position, and gripper position in a single read-only call — no movement triggered.                           |

---

## Command Line Interface

PyxArm includes a professional command-line interface for easy operation:

| CLI Feature               | Description                                                                                                                                 |
| ------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| **Web Interface Launcher** | Simple `pyxarm web` command to start the web interface and API server with customizable host and port options.                           |
| **Version Management**     | Built-in version checking with `pyxarm --version` for package information and compatibility verification.                                   |
| **Development Mode**       | Alternative execution method `python -m src.cli.main web` for development without package installation.                                    |
| **Flexible Configuration** | Command-line options for host binding (`--host`) and port selection (`--port`) to accommodate different network setups.                  |

---

## Monitoring & Performance

The controller includes systems for monitoring the robot's health and performance in real-time when connected to physical hardware.

| Monitoring Feature           | Description                                                                                                                                 |
| ---------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| **Performance Tracking**     | Automatically logs key metrics like cycle times and command success rates, helping to diagnose bottlenecks and optimize robot programs.           |
| **Predictive Maintenance**   | Monitors joint temperatures and torque (if available) to provide early warnings for potential hardware issues.                                |
| **Real-time State Callbacks**| Register custom callback functions to react to events like state changes or errors, enabling more dynamic and responsive applications.          | 