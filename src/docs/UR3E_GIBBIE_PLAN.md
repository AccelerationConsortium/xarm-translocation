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
| Gripper URCap registers | ACT 1, STA 3, FLT 00, POS 207, SPE 128, FOR 128, OBJ 3 | 2F-140 |

The gripper model is not known yet (the URCap does not say; POS 207 with
OBJ 3 at rest is a closed or nearly closed gripper). Read it from the
pendant's Robotiq screen before any gripper work.

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

**Stage 2, gripper.** Identify the model on the pendant, add it to the
config (`gripper.model`), calibrate `open_raw`/`closed_raw` as for the
UR5e, live test with the user present. Needs `control_enabled` and the
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

## Open with the user

- The gripper model (pendant).
- Who may hold the arm when: the interlock contract with the Gibbie
  workflow's owner (stage 3).
- Whether `gibbie_ur_arm` (the monitor's probe) is retired once
  `gibbie_ur3e` is on the dashboard.
- The real limits for the motion block (stage 3), and a date for the
  supervised first session.
