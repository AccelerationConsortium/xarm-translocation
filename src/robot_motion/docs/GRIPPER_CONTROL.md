# Robotiq gripper — commissioned on ligand_ur5e (2F-140, 2026-10-07)

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
  real fingers before trusting it for anything other than display. A 2F
  gripper stops short of the register ends (the UR5e's opens to 3) and
  still reports "at requested position".

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

## Commissioning a gripper

1. Confirm the model and that the fingers and any fingertip pads are clear.
2. Pick `max_force_pct` for what the gripper will hold; start low.
3. With the user present, run: claim, open, close on nothing, a stroke move,
   and a close on a soft test object, watching the fingers. Confirm STOP
   during a slow close cancels it.
4. Record the measured open/closed register values and set `open_raw` /
   `closed_raw` if the mm display should be accurate.

## ligand_ur5e commissioning record (2026-10-07)

Supervised by the operator at the robot, through the dashboard edge, speed
30 %, force 10 % (cap 20 %), with a 25 ms GET-only register log alongside:

- Every `SET` was acknowledged; open, close, a 70 mm stroke move and re-open
  all completed (full stroke about 1.6-1.8 s at 30 % speed, roughly 170 raw
  counts per second).
- STOP during a 1 %-speed close cancelled it 1 s in; `GTO 0` was acknowledged
  and the close answered 500 "move cancelled by stop" about 13 ms after the
  stop request.
- Contact: three closes on a held object at 10 % force stopped at raw 99-105
  with `OBJ 2` (`contact_closing`); re-closing on an already held object
  returns after the 0.3 s persistence window without moving.
- The URCap's status registers change about every 100 ms. `OBJ` stayed 0
  throughout every logged move and turned non-zero only once the fingers had
  stopped, so a move is not reported finished while the fingers still travel.
- Open settles at raw 3. Two closes on nothing reported raw 227 and 255 (both
  `OBJ 3`); `closed_raw` stays at the 255 default until a measured close on
  nothing confirms the real end.

Stroke calibration (2026-10-08, operator at the robot, 30 % speed, 10 % force,
25 ms GET-only register log):

- Five closes on nothing all stopped at raw 227-228 with `OBJ 3`, and every
  open settled at raw 3. The single 255 from 2026-10-07 did not recur.
- A 70 mm stroke move under the default mapping (0 / 255) went to raw 128,
  and the operator measured a 62 mm finger gap. A linear map with raw 3 as
  140 mm and raw 228 as 0 mm predicts 62.2 mm there.
- The pc-05 config now sets `open_raw: 3` and `closed_raw: 228`, so Open and
  Close command those registers, the panel reads 140 / 0 mm at the ends, and
  a 70 mm move goes to raw 116 (69.7 mm predicted). Backup:
  `.state/robot-motion.local.json.bak-20261008-pre-gripper-cal`.
