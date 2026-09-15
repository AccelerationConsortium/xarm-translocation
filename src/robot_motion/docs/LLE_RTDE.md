# RTDE integration from automated-lle

The receive adapter in `drivers/lle_rtde.py` adapts `URArm` from
`automated-lle/components/robot_move/ur5_rtde_gripper.py`, author Xiaoman Guo,
source revision `83a5169630793e45499de7b0043706b34abe2f90`. The source project's
metadata declares Apache-2.0; that license is retained in APACHE-2.0.md. The
surrounding robot-motion project retains its existing license.

Preserved conventions: `get_joints()` / `joint_positions` in degrees, and
`get_tcp_pose()` in millimetres and Euler xyz degrees. The original UR RTDE TCP
pose uses metres and a rotation vector in radians; orientation must go through
`Rotation.from_rotvec(...).as_euler('xyz', degrees=True)`, not component-wise
radians-to-degrees scaling. The raw rotation-vector pose is also returned in
telemetry so no information is lost through Euler singularities.

## Deliberate changes

- Construction has no I/O; explicit connection opens RTDEReceiveInterface only.
- No RTDEControlInterface, RTDEIOInterface, script upload, brake release,
  power, gripper connection, digital output, or motion command is used.
- Samples require six finite coordinates and advancing controller timestamps.
  Failed/stalled sessions disconnect and retry receive-only on the next poll.
- Shutdown waits for an in-flight read before closing the receiver. The sample
  freshness wait is bounded by timeout_s; the vendor SDK's constructor and
  disconnect calls have their own native timeouts, not an application deadline.
- No LLE positions, workspace settings, global exception handlers, component
  manager, homing sequences, gripper calibration, or workflow code is imported.

The original wrapper's control constructor is **not passive**: ur_rtde uploads
a controller script by default. See the [SDK interface documentation](https://sdurobotics.gitlab.io/ur_rtde/introduction/introduction.html).
Its movement defaults also need review: movej selects the linear default
velocity and the provided max-velocity arguments are not enforced. These
control paths have not been copied into the observation service.

## Enable receive-only observation

Install the existing `robot-motion[ur]` extra. In the machine-local config, keep
driver=ur, model=ur5e, observe=true, the existing robot_host and
control_enabled=false, then select `"ur_transport": "rtde"`. No other device
settings need to change. The default remains `dashboard` for existing installs.
Restart only this observer during an approved maintenance window; never start
the LLE demo or instantiate its original URArm to test a passive connection.

The service maintains a 50 Hz receive stream but publishes snapshots at its
configured poll interval. Browser values update on status refresh and show the
sample time. These are operator observations, not synchronous control feedback.
The existing Dashboard read queries remain responsible for controller, safety,
and program-state interpretation; failed telemetry does not manufacture a
hardware fault. /status is still cached, allowed_actions remains empty, and the
authenticated dashboard proxy needs no new routes or permissions.

## Remaining before Direct Drive

LLE moveJ/moveL primitives are not yet commissioned behind this application's
SDK/claim and authentication boundary. Browser motion remains disabled.
Commissioning needs UR5e-specific tool/payload and workspace validation,
motion/speed limits, ownership, stop/failure behavior, verified feedback and
human-supervised execution through the lab's approved control workflow.
