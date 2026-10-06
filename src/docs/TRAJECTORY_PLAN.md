# Coordinated rail + arm trajectories

Status (2026-10-06): feasibility assessed, validation-only endpoint shipped,
rail added to the stop path. The execute endpoint is **not built**.

Requested by the digital-twin planner: execute time-stamped waypoints that
carry the absolute rail position and all arm joints on one shared timeline,
because running the rail and the arm sequentially changes the tool and
payload path and so cannot reproduce a validated trajectory.

## 1. Can the controller do it? No.

The UFACTORY controller and `xarm-python-sdk` 1.18.4 have no coordinated
rail-and-arm motion. The two are independent systems that happen to share a
control box.

- **The rail is not a robot axis.** It is a Modbus-RTU servo on the control
  box RS485 port (`xarm/x3/linear_motor.py`). The SDK writes one target
  position register (`pos * 2000`) and one speed register (`speed * 6.667`,
  so 0.15 mm/s resolution), then polls a status bit every 100 ms until the
  move ends. There is no streaming, timed, or velocity-profile command, and
  the rail never enters the arm's planner or kinematics.
- **The arm can be streamed.** Servo mode 1 (`set_servo_angle_j`) accepts
  joint targets at up to 250 Hz over TCP. This repo does not use it yet;
  every arm move today is a mode-0 point-to-point `set_servo_angle`.
- **Nothing in this repo moves both at once.** Cross-rail graph edges run the
  arm, re-check the sash, then run the rail. Overlapping commands are refused
  with `409 motion_in_progress` by the single motion slot.
- **The stop path did not reach the rail** until this change: `stop_motion`
  only issued the arm's `emergency_stop` (state 4), which the Modbus servo
  does not see. It now also writes the rail's stop register.

Firing the two existing freehand commands concurrently would not meet the
requirement either: the rail would run at one constant speed to one target
while the arm ran its own blended profile, with no feedback between them.

## 2. What is achievable: rail-indexed arm streaming

Let the rail define the timeline and make the arm follow it.

1. Per waypoint segment, write the rail target and a per-segment speed
   `|Δrail| / Δt` so the rail arrives on schedule.
2. Meanwhile stream arm joints at 100 to 250 Hz, interpolating along the
   planned trajectory **indexed by the rail's measured position**, not by
   wall-clock time.
3. When the rail's timing drifts (fixed servo ramps, Modbus latency), the arm
   slows or speeds up with it, so the **path shape is preserved** even though
   the schedule is not exact.

### Guarantees

| Quantity | Bound |
|---|---|
| Arm joint command timing | under 10 ms |
| Rail position readback latency | 10 to 30 ms per Modbus round trip |
| Rail speed resolution | 0.15 mm/s register steps |
| Rail acceleration / deceleration | fixed in the servo, not commandable per segment |
| Rail arrival error per segment | tens of ms, worst case 100 to 200 ms near segment boundaries |
| Coordinated stop skew | 10 to 50 ms between rail stop register and arm state 4 |

### Limitations the planner must accept

- Rail velocity is **piecewise-constant per segment**. A trajectory that
  needs smooth rail acceleration will be approximated by a stair-step speed
  profile. Short segments make this worse because the servo's fixed ramp
  dominates; segments under one rail status poll (100 ms) are rejected.
- A moving rail needs at least the servo's minimum commandable speed
  (1 mm/s). Segments that would imply a slower rail are rejected; park the
  rail (zero displacement) or move it faster.
- The coordinated stop is two back-to-back commands, not one. The rail coasts
  under its own deceleration while the arm is already halting.
- The **Docker simulator has no rail**: every track call returns success
  without moving (`@xarm_is_not_simulation_mode`). Simulated acceptance tests
  can cover validation, scheduling, interpolation and stop ordering. Rail
  timing can only be measured on hardware.
- Servo mode 1 leaves the arm in a state the rest of the server does not
  expect. The executor must restore mode 0 on every exit path, including
  faults, and must hold the motion slot until the last streamed command has
  settled and the rail status bit has cleared.

## 3. Request contract

Units are fixed and are not negotiable per request.

| Field | Meaning |
|---|---|
| `t` | seconds from trajectory start; first waypoint at 0, strictly increasing |
| `rail_mm` | absolute rail position in mm from the homed origin (0 = Home, 700 = Cytation) |
| `joints_deg` | arm joint angles in degrees, base to wrist (J1..J5 on the xArm5); exactly `num_joints` entries |
| `start_tolerance.joint_deg` | per-joint tolerance against the measured start, default 1.0 |
| `start_tolerance.rail_mm` | rail tolerance against the measured start, default 2.0 |

### Validation (shipped)

`POST /control/trajectory/validate` moves nothing. It needs a login but
neither a claim nor a lowered graph mode, and it never touches the motion
slot. It collects **every** violation rather than the first, so the planner
fixes everything in one round trip.

Checks, with the `errors[].code` emitted: `too_few_waypoints`, `non_finite`,
`joint_count`, `time_not_from_zero`, `time_not_increasing`,
`segment_too_short`, `duration_exceeded`, `joint_out_of_range`,
`rail_out_of_range`, `rail_danger_zone`, `rail_speed_too_high`,
`rail_speed_too_low`, `joint_speed_too_high`, `start_state_unavailable`,
`start_joint_mismatch`, `start_rail_mismatch`.

Limits come from the same places the move paths use: model joint limits
from `safety.yaml`, the safety-level-scaled `max_joint_speed`, the 0 to
700 mm rail stroke, 1 to 1000 mm/s rail speed, and configured danger zones.

Response: `200` with `valid: true` and the report, or `422
trajectory_invalid` with the same report under `detail.report`. The report
carries `summary.segments` (implied per-segment rail and joint speeds, which
are exactly what the executor would command), `start_state` (measured joints
and rail with the errors against the first waypoint) and `execution_model`
(the guarantees table above, machine-readable).

### Execution (not built)

`POST /control/trajectory/execute` would be a **freehand** action: it
bypasses the motion graph, so it is refused in STRICT (`409
graph_mode_strict`), requires a claim, passes the sash interlock guard, and
holds the single motion slot as one operation for its whole duration.
Progress would be reported through `/status`, and `/control/stop` would halt
both axes with the skew above.

## 4. Graph mode and simultaneous motion

Simultaneous arm and rail motion is not banned by graph mode. It is banned
everywhere by the single motion slot, which refuses a second command while
one is in flight. Graph mode is a separate rule: STRICT refuses anything that
bypasses the motion graph, and both freehand rail and freehand joint commands
do. A trajectory executor would therefore be impossible in STRICT regardless
of how it is scheduled, unless a later design makes trajectories graph-aware
(start and end at verified nodes with a whitelisted edge).

## 5. Acceptance plan

1. Mock-SDK tests (done for validation): request rejections, per-segment
   speed computation, start-state rejection, slot untouched, no motion calls.
2. Mock-SDK tests for the executor (when built): slot held to completion,
   mode 0 restored on every exit, stop issues rail then arm, interpolation
   against a scripted rail readback.
3. Hardware timing characterization under separate authorization: measure
   rail arrival error versus segment length and speed, and stop skew.
