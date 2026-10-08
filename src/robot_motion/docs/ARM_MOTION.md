# UR arm motion — offline-tested, not commissioned

`drivers/ur_motion.py` moves the arm for a human operator in the shared panel:
absolute joint moves and single-joint jogs (`moveJ`), Cartesian jogs and
absolute TCP poses (`moveL`). Everything runs inside commissioned limits
from the local config. Manual (teach) mode lets the operator guide the arm by
hand instead. It replaces the single-step `joint_step` mode on a
deployment; a config names one or the other, never both.

None of this is a safety-rated function. The controller's safety
configuration (joint limits, safety planes, force and speed limits), the
pendant and the e-stop remain the safety system. The limits here stop
mistakes. They do not replace those.

## Configuration

A `motion` block inside `control` (with `control_enabled: true`, RTDE
observation and `authorized_operators`). Every field without a default is
required; nothing is guessed:

```json
"motion": {
  "commissioning_id": "UR5E-ARM-2026-10-xx",
  "joint_lower_deg": [j1, j2, j3, j4, j5, j6],
  "joint_upper_deg": [j1, j2, j3, j4, j5, j6],
  "workspace": {"x_mm": [min, max], "y_mm": [min, max], "z_mm": [min, max]},
  "max_joint_speed_deg_s": 15,
  "joint_accel_deg_s2": 30,
  "max_linear_speed_mm_s": 100,
  "linear_accel_mm_s2": 500,
  "stop_joint_decel_deg_s2": 90,
  "stop_linear_decel_mm_s2": 1000,
  "force_guard_n": 30
}
```

- **Joint envelope:** every measured joint must be inside it before a move.
  Every planned target, and every sample along a line, must be inside it too.
- **Workspace box:** base-frame limits for the TCP *point*, in millimetres.
  The tool body, the fingers and the arm links can still reach outside it.
  It is not collision checking.
- **Speed caps:** a request above a cap is refused with 422, never clamped. A
  request with no speed uses `default_*_speed`, or a quarter of the cap.
  Ceilings in code: 60 deg/s and 250 mm/s.
- **Accelerations:** every move uses the configured acceleration. The stop
  decelerations must be at least as strong.
- **Jogs:** `max_jog_joint_deg` (default 10) and `max_jog_mm` (default 50).
- **Force guard:** `force_guard_n`, optional. A move aborts when the measured
  TCP force changes by more than this from its start. It compares against the
  start, so a sensor offset does not trip it, but a wrong payload can. The
  controller's own force limit applies regardless.
- **Tolerances:** completion within 0.05 deg / 0.5 mm / 0.2 deg (defaults).
  While moving: joints within 0.5 deg of their start-to-target segment, and
  the TCP within 2 mm of the planned line. A request already within the
  completion tolerance is answered `moved: false` and not sent.
- **Pendant speed slider:** moves are refused below `min_speed_fraction`
  (default 10 %). Above it, the deadline stretches by the slider.

## Routes

Every route needs the edge identity and a listed operator. The moves, reset
and force zero also need the held claim. Connect and Disconnect follow the
xArm order (Connect, then Take Control): they need no claim while nobody holds
one, but once someone does, only that holder may call them (423 otherwise).
The paths are the ones the shared panel already calls:

| Route | Body | Motion |
|---|---|---|
| `POST /connect` | — | open the session (uploads ur_rtde's control script) |
| `POST /disconnect` | — | close it (refused 409 while a move runs) |
| `POST /control/freehand/joints` | `{angles[6], speed?}` deg, deg/s | moveJ to absolute joints |
| `POST /control/freehand/joint_jog` | `{joint, delta, speed?}` | moveJ one joint by delta |
| `POST /control/freehand/relative` | `{dx, dy, dz, speed?}` mm, mm/s | moveL by a base-frame offset, orientation kept |
| `POST /control/freehand/linear` | `{x, y, z, roll, pitch, yaw, speed?}` | moveL to an absolute pose (`current_position` convention) |
| `POST /robot/manual` (alias `/control/manual`) | `{enable}` | manual (teach) mode on or off |
| `POST /control/reset` (alias `/clear/errors`) | — | clear a stop/fault latch |
| `POST /control/force/zero` | — | zero the flange force/torque sensor |
| `POST /control/stop` (alias `/move/stop`) | — | identity only: stop the move (stopJ, or stopL during a linear move) |

Each move takes an optional `request_id` (UUID). A request id never runs
twice.

### What a move does

1. **Gates:** claim, operator, open session, control link and a running
   control script, no latch, no move running. Feedback must be fresh and
   RUNNING/NORMAL, with the speed slider high enough, the arm inside the
   envelope and box, and still across two packets.
2. **Planning, using the controller's kinematics with the active TCP;
   nothing moves:**
   - joint moves: the target must be inside the envelope and the
     controller's joint safety limits. Forward kinematics every
     `path_check_step_deg` (2°) along the joint-interpolated path must keep
     the TCP inside the box.
   - linear moves: the target must be inside the box and the controller's
     pose safety limits. Inverse kinematics every `path_check_step_mm`
     (10 mm) or 2° of turn along the line must have a solution, checked first
     because URScript's IK ends the control script when there is none.
     Every sample must stay inside the envelope, with no joint jumping more
     than `max_ik_jump_deg` between samples. A jump means a wrist flip or a
     near-singular stretch, so the move is refused; use a joint move instead.
     Each joint's estimated speed along the line must also stay within
     `max_joint_speed_deg_s`. A modest TCP speed can still spin J1 far from
     the base, or a wrist near a singularity; the 422 says what TCP speed
     would fit.
3. **Re-measure:** the plan is used only from the pose it was made for. The
   last cancel check and the moveJ/moveL call share a lock with STOP, so a
   STOP either prevents the dispatch or lands after it and stops the move.
4. **Dispatch:** asynchronous moveJ/moveL, then monitoring at the feedback
   rate. A move fails if any of these happens:
   - the claim or authorization is lost;
   - a stop is requested;
   - feedback goes stale;
   - the controller leaves RUNNING/NORMAL (for example a protective stop);
   - a joint leaves the envelope, or leaves its start-to-target segment on a
     joint move;
   - the TCP leaves the box or strays from the line;
   - any joint turns faster than 1.25 × `max_joint_speed_deg_s` + 1 deg/s;
   - the force guard trips;
   - the deadline passes: 1.5 × the trapezoidal duration + 2–3 s.
5. **Completion:** at the target within tolerance, with the arm still for at
   least 3 packets and 0.1 s. The controller accepting the move is not
   completion.

### Responses

- **200:** `{ok, action, message, moved, measured_joints_deg,
  measured_tcp_mm_rpy_deg, peak_force_change_n, elapsed_s, ...}`.
  `moved: false` means the arm was already at the target.
- **409:** no session, or a move is already running. Commands are never
  queued. Disconnect is refused while a move runs, and is atomic with a move
  starting.
- **412:** a state precondition, using the same helper as `allowed_actions`:
  - `motion_latched`
  - `control_link_down`
  - `control_script_stopped`: after a protective stop; Disconnect and
    Connect
  - `feedback_stale`
  - `robot_not_ready`
  - `speed_slider_low`
  - `robot_moving`
  - `outside_envelope` / `outside_workspace`: move the arm in with the
    pendant
  - `remote_control_off`: on connect
  - `manual_mode`: a move, reset or force zero while manual mode is on
- **422:** the request cannot run under the limits:
  - `above_commissioned_limit`
  - `jog_too_large`
  - `target_outside_envelope` / `target_outside_workspace`
  - `path_leaves_workspace`
  - `path_outside_envelope`
  - `path_configuration_change`
  - `joint_speed_exceeded`
  - `unreachable`
  - `outside_controller_limits`
  - `request_replayed`
- **502:** `planning_failed`: a controller kinematics query failed during
  planning. Nothing was sent.
- **500:** the move was sent and failed. A stop was sent; the body carries
  `stop_attempted`, `stop_error` and `latched: true`. Check the arm, then
  reset.

### Latch and reset

A STOP (even with the arm idle), or any failure after dispatch, latches the
session: moves answer 412 `motion_latched`, and `allowed_actions` offers
`control.reset`. Request ids that already ran stay remembered across a reset.
Reset (the panel's Clear errors) needs a running control script,
RUNNING/NORMAL and a still arm. After a protective stop the controller ends
the script. Clear the stop on the pendant, then Disconnect and Connect.

Disconnect ends ur_rtde's control script (`stopScript`) before it closes the
link. Until 2026-10-08 it only closed the link, which left the script running
on the controller, holding the program slot, until the communication
watchdog stopped it.

Stopping the service stops an in-flight move and closes the session before
anything else shuts down. This needs time: open connections get at most 3 s,
and NSSM must allow the shutdown to finish (`AppStopMethodConsole` 15000 ms;
the 1.5 s default killed the process mid-shutdown on 2026-10-08). A process
killed with a session open leaves the stop to the controller's watchdog, and
that left the arm in a protective stop. Disconnect before a planned restart.

## Manual (teach) mode

`POST /robot/manual {"enable": true}` puts the arm in teach mode (ur_rtde
`teachMode`, URScript `teach_mode()`): it can be pushed by hand. The pendant's
freedrive button does not work in Remote Control, so this is the way to
hand-guide the arm while the service owns it.

- **On** needs the claim, an open session with a running control script,
  fresh feedback, RUNNING/NORMAL, a still arm and no latch. There is no
  envelope, box or speed-slider check: guiding the arm by hand is how one
  that is outside them gets back in. The controller's own joint limits,
  safety planes and speed and force limits still apply.
- The arm holds itself up using the **payload** and TCP set on the pendant.
  A wrong payload makes it drift up or down, so keep a hand on it.
- **While on:** every move, reset and force zero is refused with 412
  `manual_mode`; `allowed_actions` offers `arm.manual_mode` (to turn it off),
  `disconnect` and `control.stop`.
- **Off** (`{"enable": false}`) returns to position control and does not
  latch.
- **It ends by itself, and latches,** within about 0.1 s when:
  - the claim is released or expires;
  - the operator or session is no longer authorized;
  - the control link or script is lost;
  - feedback goes stale, or the controller leaves RUNNING/NORMAL (a
    protective stop).

  STOP ends it and latches too; Disconnect ends it.
- If the controller does not confirm the end, the route answers 500, the
  session latches and manual mode stays reported on, so moves stay refused.
  Clear errors is refused then. Use STOP, Disconnect (which ends the control
  script, and teach mode with it) or the pendant.

## Status

- `allowed_actions`, which lists exactly what a POST would honor (STATUS_SPEC
  section 6.2):
  - `connect`: only when idle and in Remote Control;
  - `disconnect`;
  - `arm.move_joints`, `arm.jog_joint`, `arm.move_linear`, `arm.jog_linear`;
  - `arm.manual_mode`: to turn it on (arm ready and still) or off;
  - `control.reset`: only when latched;
  - `arm.zero_force_sensor`;
  - the gripper actions.
- `details.control_session` contains:
  - `mode: "motion"`
  - `open`, `busy`, `active_move`, `latched`, `manual_mode` (also at
    `details.manual_mode`, the field the panel's Manual switch reads)
  - `watchdog_ok`
  - `limits`: the envelope, box, caps and jog limits an operator or agent
    plans within
  - `tcp_offset_mm_rpy_deg`: the active TCP read at connect; verify it is
    the real tool
  - `last_event`
- `details.telemetry` (RTDE transport, every `telemetry_interval_s`,
  default 0.5 s):
  - `joints_deg`
  - `tcp_mm_rpy_deg`
  - `tcp_force`: `force_n`, `torque_nm` and `force_magnitude_n` in the base
    frame, payload-compensated by the controller
  - `payload`: `mass_kg` and `cog_mm` as configured on the pendant
  - The panel's `/ws` push follows the same interval.
- `details.remote_control` and `details.operational_mode`, from the Dashboard
  (e-Series).

## Panel

For UR the panel's Connect and Disconnect follow `allowed_actions` instead of
"controller ready". Neither needs Take Control, but both are disabled while
someone else holds control. Move Joints and the XYZ jog buttons are enabled
only while the moves are offered, and Clear errors only while latched.
The Manual switch follows `arm.manual_mode`, for the claim holder only, and
shows "Manual (drag)" while on. Named-location moves do not exist here and
stay disabled. The rail controls
read N/A and stay disabled (there is no rail), as do Safety level and Connect
to: the speed caps come from the config, not the panel. The joint inputs take
absolute degrees; the jog step is in mm; the speed boxes are deg/s and mm/s,
refused above the caps.

## Before the first live move

1. Pendant: Remote Control on, and nothing else attached to the controller.
2. Verify the **TCP** and the **payload** on the pendant: mass, and centre of
   gravity for the gripper plus any held part. The 2026-10-07 read showed
   1.0 kg with the CoG at the flange (0, 0, 0) mm, which is wrong for a
   2F-140. Zero the force sensor with the arm still: the same read showed
   about 64 N at rest with nothing held.
3. Decide the joint envelope, the workspace box (with margin from the bench,
   the hood and the camera mounts) and the speed caps. Start slow, for
   example 10 deg/s and 50 mm/s.
4. Check the controller's own safety configuration: joint limits, planes,
   reduced mode.
5. With the operator at the pendant, e-stop in reach:
   - Connect;
   - a small joint jog;
   - a small Cartesian jog;
   - STOP mid-move, then Clear errors;
   - the watchdog test: kill the service mid-move and confirm the
     controller stops.

Offline tests (`tests_robot_motion/test_motion.py`) cover the cases above
against a simulated arm. No physical motion was performed as part of this
implementation.
