# Robotiq gripper — offline-tested, not commissioned

`drivers/robotiq.py` drives a Robotiq adaptive gripper (2F-85, 2F-140 or
Hand-E) through the Robotiq Gripper URCap's socket server on the UR
controller, TCP 63352. It needs neither the arm's RTDE control script nor the
controller's program slot, so it can be commissioned before arm motion.

## Configuration

A top-level `gripper` block in the local config:

```json
"gripper": {
  "model": "robotiq_2f140",
  "default_speed_pct": 50,
  "default_force_pct": 20,
  "max_speed_pct": 100,
  "max_force_pct": 50
}
```

- **Monitoring:** any observed UR robot with this block gets read-only
  status (GET only, 1 Hz by default, `poll_interval_s`). It shows as
  `components.gripper` (`enabled`, `disabled`, `activating`, `fault`,
  `unknown`), `details.gripper`, and the shared panel's
  `connection_details.gripper_type` / `gripper_config`.
- **Commands:** they exist only with `control_enabled: true` and a `control`
  block. `control.joint_step` is optional, so a gripper-only deployment has
  claims and gripper routes but no `/connect` or arm motion.
- **Speed and force** are percent of the gripper's range (Robotiq SPE/FOR
  0..255). A request above `max_*_pct` is refused (422), never clamped.
  For the 2F-140, 100 % force is roughly 125 N and 100 % speed roughly
  250 mm/s.
- **Opening:** the stroke in mm is a linear estimate from the position
  register using `open_raw` / `closed_raw` (defaults 0 / 255). Measure the
  real fingers before trusting it for anything other than display.

## Routes

These are the paths the shared panel already calls for the xArm's gripper.
Every command needs the edge identity, a listed operator and the held claim
(`X-Claim-Token`):

| Route | Body | Effect |
|---|---|---|
| `POST /gripper/open` | `{speed?, force?}` | move to `open_raw` |
| `POST /gripper/close` | `{speed?, force?}` | move to `closed_raw` |
| `POST /control/freehand/gripper/stroke` (alias `/gripper/move/stroke`) | `{stroke, speed?, force?}` | move to an opening in mm |
| `POST /control/freehand/gripper/force` (alias `/gripper/force`) | `{force}` | set the force for later moves that name none; moves nothing |
| `POST /component/enable` | `{"component": "gripper"}` | activate; no-op when already active |
| `GET /gripper/position` | — | cached status, no claim |

- **Moves wait** for the fingers to stop. The response reports the final
  position, the opening and the object state: `at_requested_position`,
  `contact_closing` or `contact_opening`. Contact means something was gripped
  or blocked the fingers.
- **One command at a time:** a second request while one runs is 409, never
  queued.
- **Activation moves the fingers:** a 2F gripper sweeps through its full
  stroke to calibrate. Make sure nothing is between the fingers. Activation
  runs to completion once started; a stop does not interrupt it.
- **Refusals (412)**, from one helper that also decides `allowed_actions`
  (`gripper.open`, `gripper.close`, `gripper.move`, `gripper.activate`):
  - no fresh gripper status;
  - no fresh robot observation;
  - robot mode not RUNNING, or safety not NORMAL;
  - a program PLAYING or PAUSED that is not this service's own control
    session (a pendant program may be using the gripper);
  - a gripper fault, or the gripper not activated.
- **Failure (500)**, after the request was sent: timeout, fault during the
  move, lost status, or a cancel. `GTO 0` is sent to stop the fingers and the
  body says whether that worked. Check any held part before the next command.
- **STOP:** `POST /control/stop` (identity only) also cancels an in-flight
  gripper move. It never touches an idle gripper, so a stop cannot release a
  held part.

Every command, refusal and failure is an audit event (`control.audit_file`).

## Before the first live command

1. Confirm the model and that the fingers and any fingertip pads are clear.
2. Pick `max_force_pct` for what the gripper will hold; start low.
3. With the user present, run: claim, open, close on nothing, a stroke move,
   and a close on a soft test object, watching the fingers. Confirm STOP
   during a slow close cancels it.
4. Record the measured open/closed register values and set `open_raw` /
   `closed_raw` if the mm display should be accurate.

No physical gripper motion was performed as part of this implementation.
