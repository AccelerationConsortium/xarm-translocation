# PyxArm Simulation Modes & Testing Strategy

This guide provides a comprehensive overview of the simulation modes and a three-stage testing strategy for the PyxArm project. Following this strategy ensures robust, safe, and reliable robot applications, progressing from pure software simulation to full physics simulation and finally to real hardware validation.

## 🔧 Understanding the `simulation_mode` Parameter

**IMPORTANT**: The `simulation_mode` parameter in the `XArmController` class can be confusing because there are actually **TWO different types of simulation**:

### `simulation_mode=True` → **Software Simulation**
- **What it is**: Pure Python simulation with **NO network connection**
- **Use case**: Fast development, testing logic, CI/CD
- **Physics**: Basic collision detection only
- **Connection**: No network connection made at all
- **Example**:
```python
# This runs entirely in Python memory - no network connection
controller = XArmController(
    profile_name='docker',
    simulation_mode=True  # ← Pure software simulation
)
```

### `simulation_mode=False` → **Hardware/Simulator Connection**
- **What it is**: Real network connection to either:
  - Real xArm robot hardware, OR
  - UFACTORY Studio simulator (Docker/desktop)
- **Use case**: Realistic testing with physics, visual verification
- **Physics**: Full physics simulation (if connecting to simulator)
- **Connection**: TCP/IP network connection to specified IP:port
- **Examples**:
```python
# This connects to Docker simulator via network (127.0.0.1:18333)
controller = XArmController(
    profile_name='docker',
    simulation_mode=False  # ← Network connection to simulator
)

# This connects to real robot hardware via network (192.168.1.237:18333)
controller = XArmController(
    profile_name='robot',
    simulation_mode=False  # ← Network connection to real robot
)
```

### Summary Table

| `simulation_mode` | Network Connection | Target | Physics Engine | Use Case |
|-------------------|-------------------|--------|----------------|----------|
| `True` | ❌ None | Python memory | Basic collision detection | Fast development, CI/CD |
| `False` + `docker` profile | ✅ Yes | Docker simulator (127.0.0.1) | Full physics simulation | Realistic testing |
| `False` + `robot` profile | ✅ Yes | Real robot (192.168.1.237) | Real hardware | Production |

**Key Point**: `simulation_mode=False` does NOT mean "real hardware only" - it means "make a network connection" (which could be to a simulator OR real hardware, depending on the profile).

---

## Three-Stage Testing Strategy

A professional testing methodology progressing from safe software simulation to real hardware deployment.

### Testing Flow Diagram

```
🚀 Start Development
         ↓
┌─────────────────────┐
│  Stage 1: Software  │
│     Simulation      │
│  ✅ Logic Testing   │
│  ✅ Error Handling  │
│  ✅ Boundary Check  │
└─────────────────────┘
         ↓
┌─────────────────────┐
│  Stage 2: Docker    │
│     Simulation      │
│  ✅ Physics Valid   │
│  ✅ Visual Verify   │
│  ✅ Real-time Test  │
└─────────────────────┘
         ↓
┌─────────────────────┐
│  Stage 3: Real      │
│     Hardware        │
│  ✅ Integration     │
│  ✅ Safety Systems  │
│  ✅ Production      │
└─────────────────────┘
         ↓
🎯 Production Ready
```

---

## Stage 1: Software Simulation (`simulation_mode=True`)

The controller has a built-in software simulation mode that does not require Docker or any external dependencies. It works by mocking the robot's hardware interface, allowing you to test your code's logic, parameter validation, and error handling.

This mode includes **logic-based collision detection**:
*   **Joint Limits**: Prevents movements that would exceed the specific model's joint angle limits.
*   **Workspace Boundaries**: Enforces Cartesian workspace limits to prevent the arm from trying to move outside its reachable area.
*   **Basic self-collision detection**

### How to Use

To activate the built-in simulation mode, simply instantiate the `XArmController` with the `simulation_mode=True` parameter.

```python
from src.core.xarm_controller import XArmController

# Enable software simulation mode
controller = XArmController(
    model=6,
    simulation_mode=True
)

# Initialize the controller (no hardware connection is made)
if controller.initialize():
    print("Software simulation initialized successfully.")
    # All controller commands can be used here
    controller.move_to_position(x=300, y=0, z=300)
    controller.disconnect()
```

### What to Test:
- **Algorithm Logic**: Verify movement sequences and decision trees
- **Error Handling**: Test exception handling and recovery mechanisms
- **Boundary Conditions**: Validate joint limits and workspace boundaries
- **Parameter Validation**: Check input sanitization and validation
- **State Management**: Verify robot state tracking and transitions
- **Configuration Loading**: Test different robot models and configurations

### Benefits:
- ✅ **Zero hardware risk** - No chance of damaging equipment
- ✅ **Rapid iteration** - Instant feedback and debugging
- ✅ **Comprehensive testing** - Test dangerous scenarios safely
- ✅ **Logic validation** - Focus on algorithm correctness
- ✅ **CI/CD integration** - Automated testing in pipelines

---

## Stage 2: Docker-Based Simulation (`simulation_mode=False`)

For higher-fidelity testing, this project supports the official UFACTORY xArm simulator, which runs in a Docker container. This provides a realistic physics environment, a 3D web interface, and accurate collision detection based on the robot's 3D model. The controller connects to this simulator via a network socket, just as it would with a real robot.

### How to Use

#### Step 1: Start the Docker Container

Run the following command in your terminal to download the image and start the container. This only needs to be done once.

```bash
docker run -d --name uf_software \
    -p 18333:18333 \
    -p 502:502 \
    -p 503:503 \
    -p 504:504 \
    -p 30000:30000 \
    -p 30001:30001 \
    -p 30002:30002 \
    -p 30003:30003 \
    danielwang123321/uf-ubuntu-docker
```
*Note: This Docker image is a community-provided one. Please check the official UFACTORY website or GitHub for the latest recommended image.*

You can check if the container is running with `docker ps`.

#### Step 2: Start the xArm Firmware

Once the container is running, start the firmware simulation inside it. The command below is for an **xArm6**. Replace `6` with `5` or `7` for other models.

```bash
docker exec uf_software /xarm_scripts/xarm_start.sh 6 6
```

The simulator is now running and will be accessible to your controller at `127.0.0.1`.

#### Step 3: Connect from your Code

Your controller will connect to the simulator automatically if the `host` in your configuration is set to `127.0.0.1` (which is the default). Ensure `simulation_mode` is `False`.

```python
from src.core.xarm_controller import XArmController

# Connect to the Docker simulator (ensure model matches the simulator)
controller = XArmController(
    profile_name='docker', # Connects to 127.0.0.1
    simulation_mode=False
)

# Initialize the controller (connects to 127.0.0.1)
if controller.initialize():
    print("Connected to Docker simulator successfully.")
    controller.move_to_position(x=300, y=0, z=300)
    controller.disconnect()
```

#### Step 4: (Optional) Access the Web UI

You can view a 3D visualization of the robot by navigating to **[http://localhost:18333](http://localhost:18333)** in your browser.

#### Step 5: Stopping the Simulator

To stop and remove the container when you are finished, run:

```bash
docker stop uf_software
docker rm uf_software
```

### Connecting to a Remote Simulator (e.g., via VPN/Tailscale)

You can run the Docker simulator on a separate, more powerful machine and connect to it over a secure network.

**On the Remote Server:**
1.  Start the Docker container and firmware as described above. The `-p` flags will expose the simulator's ports to the server's network interface.
2.  Ensure your client machine can ping the server's IP.

**On the Client Machine:**
Simply specify the server's IP address as the `host` when connecting.

```python
controller = XArmController(
    host='<REMOTE_IP_ADDRESS>',
    model=6,
    simulation_mode=False
)
controller.initialize()
```

### What to Test:
- **Motion Dynamics**: Verify realistic acceleration and deceleration
- **Collision Detection**: Test precise mesh-based collision detection
- **Trajectory Planning**: Validate smooth path execution
- **Visual Verification**: Confirm movements look correct in 3D
- **Timing Validation**: Test real-time execution timing
- **Complex Scenarios**: Multi-axis coordinated movements

### Benefits:
- ✅ **Official UFactory environment** - Exact robot behavior
- ✅ **Full physics simulation** - Realistic dynamics and forces
- ✅ **Precise collision detection** - Mesh-based collision checking
- ✅ **3D visualization** - Visual verification of movements
- ✅ **Real-time execution** - Proper timing validation

---

## Stage 3: Real Hardware Testing

The final stage is to validate your code on a physical xArm. At this stage, your code's logic and motion paths should already be well-tested from the simulation stages. Testing on hardware is focused on verifying real-world performance, sensor integration, and safety.

### How to Use

1.  Ensure the robot is powered on and connected to the network.
2.  Update your configuration or pass the robot's IP address directly to the controller.
3.  Run your code. **Always be prepared to trigger the emergency stop.**

```python
from src.core.xarm_controller import XArmController

# Connect to the real robot
controller = XArmController(
    host='192.168.1.XXX', # Replace with your robot's IP
    model=6,
    simulation_mode=False
)

if controller.initialize():
    # Run pre-validated, safe movements first
    # ...
    controller.disconnect()
```

### Safety First

*   Always have the emergency stop button within reach.
*   Start with slow speeds and simple, non-obstructed movements.
*   Define and test safety boundaries and collision sensitivity settings before running complex routines.

### What to Test:
- **Hardware Integration**: Verify sensor feedback and actuator response
- **Environmental Factors**: Test with real-world conditions
- **Safety Systems**: Validate emergency stops and safety protocols
- **Performance Optimization**: Fine-tune for actual hardware characteristics
- **End-to-End Workflows**: Complete system integration testing

### Risks:
- ⚠️ **Hardware damage potential** - Requires careful testing
- ⚠️ **Safety concerns** - Need proper safety protocols
- ⚠️ **Time intensive** - Slower iteration cycles
- ⚠️ **Cost implications** - Hardware wear and potential repairs 