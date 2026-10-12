# UR3e on the Gibbie platform: bringing it under robot-motion

Started 2026-10-12. The Gibbie sample-prep bench has a UR3e that the
dashboard only watches (`gibbie_ur_arm`, a Dashboard-Server probe in
`sdl2-gibbie-server`). The goal is the same service the Ligand UR5e has
(`ligand_ur5e`, this repo's `robot_motion`): RTDE observation, the shared
panel, the Robotiq gripper, claim-gated arm motion and joint trajectories,
staged the way the UR5e was (`UR5E_TRAJECTORY_PLAN.md`,
`robot_motion/docs/ARM_MOTION.md`, `GRIPPER_CONTROL.md`).

## What was found (read-only recon, 2026-10-12)

Controller, from the Gibbie PC over the Dashboard Server (29999):

| | UR3e (Gibbie) | UR5e (Ligand), for comparison |
|---|---|---|
| Address | 192.168.254.89 (lab switch) | 192.168.254.49 |
| Model / serial | UR3 e-Series, 20245300979 | UR5 e-Series, 20245501504 |
| PolyScope | 5.17.3.128850 | 5.17.3.128850 (same) |
| State at recon | RUNNING / NORMAL, program STOPPED `<unnamed>` | same |
| Remote Control | **on** (operational mode NONE) | had to be switched on at the pendant |
| Open ports | 30002, 30004 (RTDE), 502, **63352 (Robotiq URCap)** | same set |
| Gripper | **Robotiq 2F-85** (85 mm stroke; user, 2026-10-12). URCap registers ACT 1, STA 3, FLT 00, POS 207, SPE 128, FOR 128, OBJ 3 | 2F-140 |

The driver already knows the model (`gripper.model: robotiq_2f85`,
`drivers/robotiq.py`). POS 207 under the default 0/255 map is about 16 mm
open, so the fingers were nearly closed at recon; the real open/closed raw
values are measured in stage 2 as they were on the UR5e (3/228 there).

The Gibbie PC (`sdl2-pc-04`, Tailnet 100.64.254.17, lab switch
192.168.254.79, ssh alias `gibbie-pc`, cmd.exe shell, elevated): git 2.53,
uv 0.10.5 at `C:\SDL_Tools\uv.exe`, GitHub reachable over https, 155 GB
free. Services: `gibbie-server` :8070 (the monitor), `mt-xpr-balance`
:8081, `sdl-camera-server` 127.0.0.1:8071, `sdl-lab-hostops` :8060. Port
8075 is free and has no firewall rule.

**The bench workflow owns the arm during runs.** `sdl2_solid_dosing_XG`
(`C:\Users\sdl2\SamPrepPlatfrom`, the `ot_py312` conda env; its UI bridge
is `app.py` on 127.0.0.1:8000) drives the UR3e with ur_rtde's
`RTDEControlInterface` (moveJ/moveL between named positions such as
`safe_rack`, `home_prep_bal`, `vial_stock_prep`, at 20–100 mm/s) and the
Robotiq gripper through the URCap socket. A UR controller runs one RTDE
control script at a time, so robot-motion's arm session and a Gibbie run
are mutually exclusive; observation (RTDE receive) and the Dashboard-Server
polls coexist with either. The `gibbie_workflow` tile shows a run as
`busy`; at recon its last run had failed (Flex over-pressure) and the arm
was idle.

## Stages

**Stage 1, observer (deployed 2026-10-12, see below).** A second
robot-motion instance on the Gibbie PC, read-only: `observe` over RTDE,
`control_enabled: false`, Tailnet bind only. Dashboard entry `gibbie_ur3e`
with the observation tile, an edge block `/ur3e/` for the read-only panel,
host-ops whitelist and probe port. The old `gibbie_ur_arm` tile stays until
the user retires it.

**Stage 2, gripper.** The 2F-85: `gripper.model: robotiq_2f85` in the
config, calibrate `open_raw`/`closed_raw` as for the UR5e (closes on
nothing, opens, five times each), live test with the user present.
Re-activating it sweeps the fingers, and the bench workflow activates it
too, so agree the moment with the workflow's owner. Needs `control_enabled` and the
edge secret in the service environment (`ROBOT_MOTION_UR3E_EDGE_SHARED_SECRET`,
its own, in Caddy's and the dashboard API's env files too).

**Stage 3, arm session and motion.** Before it: TCP offset and payload
verified on the pendant (the UR5e's were wrong); a `control.motion` block
with a real envelope, workspace box and speed caps, which the user supplies
(the workflow's named positions are a starting survey of where the arm
works); the interlock with the bench: robot-motion must refuse `/connect`
while a program is PLAYING on the controller and should also consult the
`gibbie_workflow` state, and the workflow side must know not to start a
run while a robot-motion session is open (an `allowed_actions`/`busy`
read of `gibbie_ur3e`, to be agreed with the workflow's owner). The service
account question (LocalSystem with control on, DEVICE_PC_SETUP §5) applies
here as it does on pc-05.

**Stage 4, joint trajectories.** The UR5e's stages 0a/0b/3 repeated on the
UR3e (servoJ timing, rate, lag), then `control.trajectory.enabled` with the
watchdog rule, and the `trajectory.*` skills for `gibbie_ur3e`.

## Stage 1 deployment record

Service `robot-motion-ur3e` on `sdl2-pc-04`, checkout
`C:\Users\sdl2\Projects\robot-motion` on branch `debug-ur3e` (cut from
`main` on 2026-10-12), managed Python 3.12.13 under `.state\python`,
extras `ur` + `dev`, `--link-mode copy` (this PC's uv needs it). Config
`.state\robot-motion.local.json`:

```json
{"equipment_id": "gibbie_ur3e", "equipment_name": "UR3e Arm (Gibbie)",
 "driver": "ur", "model": "ur3e", "robot_host": "192.168.254.89",
 "observe": true, "poll_interval_s": 15, "timeout_s": 2,
 "control_enabled": false, "ur_transport": "rtde"}
```

Binds `--host 100.64.254.17 --port 8075`; firewall rule "Robot Motion
UR3e 8075 (Tailnet only)"; logs under `.state\logs`; LocalSystem under
the read-only exception, like the UR5e observer was. Install scripts are
kept in `.state\install-stage*.ps1` on the PC. Rollback: stop and remove
that service and its firewall rule; nothing else on the PC was touched.

## Stage 2/3 commissioning record (2026-10-12, user at the arm)

Control enabled on `robot-motion-ur3e` at 02:21 UTC with
`.state\install-stage3-enable-control.ps1` (the previous observer config is
kept next to it as `robot-motion.local.json.bak-*-pre-control`): operator
list, audit `control-audit.jsonl`, `gripper.model robotiq_2f85` (speed
30 % default / 100 % cap, force 10 % / 20 %), `control.motion` with
commissioning id `UR3E-ARM-2026-10-12-OPEN-PLACEHOLDER` (joints ±360°, box
±800 mm, 10 deg/s, 50 mm/s, 30 N force guard, stop decelerations 90 deg/s²
and 1000 mm/s²), trajectories off, `watchdog_hz 2`, and
`ROBOT_MOTION_EDGE_SHARED_SECRET` in the service environment (its own
secret; the edge and the dashboard API still had none, so the panel at
`/ur3e/web/` stayed read-only). LocalSystem, as on pc-05 (DEVICE_PC_SETUP
§5 exception still open). Gates checked before any move: `/connect` 401
without identity, gripper 423 without a claim, claim 403 for an operator
not in the list. The arm had been moved by hand away from the balance
first; the Gibbie workflow was idle (its last run had failed on the Flex).

Live results, all through the device API with the edge identity and a
claim (the claim lives 30 s and needs a heartbeat every 10 s; two of the
first attempts expired between commands):

| Step | Result |
|---|---|
| connect | session open in one try; TCP offset (0, 0, 195) mm, payload 0.82 kg CoG (-6, 2, 65) mm already set on the pendant |
| force zero | 14.6 N at rest → 0.18 N |
| J6 +5° / −5° at 5 deg/s | 1.42 s each, peak force change 0.5–0.8 N, joints back to start; a second move sent in the same second was refused 412 `robot_moving` |
| +Z / −Z 10 mm at 10 mm/s | 1.30 s each, TCP z 408.7 → 418.7 → 408.7 mm |
| STOP mid-move (+Z 40 mm at 5 mm/s, stop at 2.5 s) | stop round trip 19 ms; 11.6 mm travelled; the move returned 500 `arm_move_failed` latched; a move while latched 412 `motion_latched`; `/control/reset` cleared it; the return jog brought z back to 408.7 |
| gripper open ×2 | raw 3 (requested 0), 1.7–1.9 s |
| gripper close on nothing ×2 | raw 228 (requested 255), 1.8 s, `at_requested_position` |
| stroke 40 mm (default 0/255 map) | raw 135 in 0.9 s; under a linear 3/228 map that is ≈35 mm, to be confirmed with calipers before `open_raw 3` / `closed_raw 228` go into the config |
| disconnect | closed with no error at 02:29:05 UTC; RUNNING / NORMAL, program STOPPED for the following 100 s, no protective stop |

Still to do from this record: the gripper calibration above; manual
(teach) mode and its STOP latch were not exercised; the real limits for
the motion block; the interlock contract with the workflow's owner; the
edge and API secrets plus the control tile on the dashboard.

## Open with the user

- Who may hold the arm when: the interlock contract with the Gibbie
  workflow's owner (stage 3).
- Whether `gibbie_ur_arm` (the monitor's probe) is retired once
  `gibbie_ur3e` is on the dashboard.
- The real limits for the motion block (stage 3), and a date for the
  supervised first session.
