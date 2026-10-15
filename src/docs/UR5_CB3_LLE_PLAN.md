# UR5 CB3 (Process Chemistry) under robot-motion

The Process Chemistry bench's UR5 CB3 (dashboard id `lle_ur5_arm`) runs a
third robot-motion instance, the same way as the UR5e (`ligand_ur5e`) and the
Gibbie UR3e (`ur3e_gibbie`).

## Facts (probed 2026-10-13, Dashboard Server, read-only)

| | |
|---|---|
| Robot | UR5 (CB3), serial 2016352495, PolyScope 3.15.8 |
| Address | 192.168.254.16 on the lab switch (LLE PC: 192.168.254.5) |
| Ports open | 29999 Dashboard, 30002, 30004 RTDE, 502, 63352 Robotiq URCap |
| Gripper | Robotiq Hand-E, on the URCap socket. Catalog stroke is 50 mm; this arm measured 52.76 mm fully open (raw 3) and 0 mm closed (raw 249). Set `gripper.stroke_mm` (and those raw ends) in the local config. |
| Safety | a safety-stop button on the safeguard-stop input (not the e-stop) |
| Host PC | sdl2-pc-00-lle (`SDL2Win02`, Tailnet 100.64.254.13, ssh `lle-pc`) |

## Deployment

- Branch `debug-ur5cb3`; checkout `C:\Users\sdl2\Projects\robot-motion`,
  `.venv` with extras `ur` + `dev`, config `.state\robot-motion.local.json`.
- NSSM `robot-motion-ur5cb3` (`C:\SDL_Tools\nssm.exe`), binds
  100.64.254.13:8075, logs `.state\logs`, `AppStopMethodConsole 15000`,
  `ROBOT_MOTION_EDGE_SHARED_SECRET` in the service environment (the edge holds
  the same value as `ROBOT_MOTION_UR5CB3_EDGE_SHARED_SECRET`).
- Edge: `/ur5cb3/` on the single edge, forward_auth
  `/auth/verify?equipment=lle_ur5_arm`. Who may command the arm is the
  central roster's decision; the device keeps no operator list.
- The process-chem monitor's Dashboard-only probe of this arm (:8070) is no
  longer what the dashboard polls.

## CB3 differences handled in code

- No Remote Control or operational mode: the Dashboard queries for them are
  e-Series only and are not sent; `remote_control` reads null and does not
  gate /connect.
- No payload / payload_cog RTDE outputs (e-Series only); not requested.
- No flange F/T sensor: `actual_TCP_force` is estimated from joint currents
  and is noisy, so the motion block carries no force guard, and
  `arm.zero_force_sensor` is never offered (412 `no_force_sensor`).
- RTDE and servoJ run at 125 Hz at most; the config refuses a higher
  feedback or trajectory rate.
- The pendant's active TCP stays all zeros (the bare flange): automated-lle's
  `moveL` / `getActualTCPPose` poses are flange poses, so setting a Hand-E
  TCP on the pendant would shift every one of them. FK with an all-zero
  offset ends the control script, so robot-motion verifies FK with a 1 um
  tool-Z offset instead (`FLANGE_TCP_OFFSET`); joint moves need it for the
  workspace check. robot-motion's linear moves are therefore flange moves.
- A safeguard stop reads `requires_init` (the tile's "Needs Init"), not an
  error, with `required_actions` `disconnect` then `connect` if a session is
  open, otherwise `connect`. /connect stays refused until safety is NORMAL
  again. After the stop (or if the control program is stopped on the
  controller) a session with `watchdog_ok` false stays `requires_init` until
  disconnect and connect; it does not fall back to `ready`.
- Dashboard `programState` stays STOPPED while robot-motion's ur_rtde control
  script is running. `/status` reports that script as PLAYING
  (`program_state_source: control_session`) when the watchdog is acknowledged.
  `activity` stays `idle` while the arm is merely connected; it is `running`
  only during a move, an active trajectory, or a gripper move.

## automated-lle

automated-lle (`C:\Users\sdl2\Projects\LLE_SDL2_V2`) drives this arm over
ur_rtde's control interface during runs, so a robot-motion arm session and
an LLE run exclude each other. /connect refuses while a program is PLAYING,
but an LLE run between moves may show STOPPED. Rule until an interlock
exists: no INIT while an LLE run is scheduled or in progress. automated-lle's
code is not changed by this work.

In practice the IvoryOS server (`implementations/SDL2/LLE_IvoryOS.py`, conda
env `ur_robot`) keeps a ur_rtde control session open between runs (seen
2026-10-13: up since 2026-10-06, holding 29999/30003/30004). Its session
holds the RTDE input registers, so /connect gets ur_rtde's "RTDE input
registers are already in use" and answers 409 `arm_held_by_another_client`.
That refusal is the de facto interlock: do not work around it (e.g. with the
upper register range), since a second control script would replace
IvoryOS's on the controller. To use the arm, stop IvoryOS on the LLE PC,
connect, and Disconnect before the LLE team restarts it.

## Commissioning

Placeholder limits, as on the other two arms, until real ones are surveyed:
commissioning id `UR5CB3-ARM-2026-10-13-OPEN-PLACEHOLDER`, joints ±360°,
box ±1000 mm, 10 deg/s, 50 mm/s, no force guard, trajectories off, watchdog
2 Hz. Gripper: Hand-E, measured fully open 52.76 mm at raw 3 and closed
0 mm at raw 249 (`stroke_mm`, `open_raw`, `closed_raw` in the local config);
speed 30/100 %, force 10/20 % (default/cap).

Live tests need the user at the arm: connect, small J6 move, a +Z 10 mm jog,
STOP mid-move then Clear errors, the safety-stop button mid-move (expect the
tile to read Needs Init and the move to fail), manual (teach) mode on/off,
gripper activate / open / close and the stroke calibration.
