# Joint trajectories on the UR5e (ligand_ur5e) — plan

Status (2026-10-11): **plan, not built.** The xArm5 version is built,
tested on the arm and deployed (see
[`SERVOJ_TRAJECTORY_PLAN.md`](SERVOJ_TRAJECTORY_PLAN.md) and the client
guide [`TRAJECTORY_API.md`](TRAJECTORY_API.md)). This plan ports it to the
UR5e behind the **same HTTP contract**, so a client written for the xArm
(Astra's digital twin) works on the UR5e by changing the base URL and the
joint count.

---

## 1. What carries over unchanged

These are robot-independent and already tested:
- **Wire format:** `t`, `joints_deg`, optional `velocities_deg_s` and
  `accelerations_deg_s2`, with chunks, `final` and idempotent uploads.
- **Routes:** validate, limits, create, chunks (PUT or POST), start,
  status, cancel and log, under `/control/freehand/trajectory`, with the
  same errors and states.
- **The pure maths** (`joint_trajectory.py`): quintic and cubic Hermite
  interpolation, validation on the interpolated path, the lead-in, and the
  stop ramp.
- **The session and executor logic** (`trajectory_executor.py`):
  - the stop generation, never cleared, and the `starting` state;
  - the re-measure after prepare;
  - speed and acceleration guards on every sample;
  - try/finally finalisation;
  - sessions owned by a claim;
  - the delay-rather-than-burst rule;
  - GC paused while streaming.
- **The dashboard skills:** the seven `trajectory.*` skills, registered for
  `ligand_ur5e` as well.

## 2. What is new

### 2.1 A UR backend (ur_rtde `servoJ`)

| Backend call | UR5e implementation |
|---|---|
| `prepare` | Refuse unless the robot mode is RUNNING, the safety mode is NORMAL (no protective stop), Remote Control is on, teach mode is off, the RTDE control script is running, and the pendant speed slider is at 100 % (it scales servoJ). No mode switch is needed. |
| `send(q)` | `servoJ(q_rad, 0, 0, dt, lookahead_time, gain)`. ur_rtde ignores speed and acceleration here. The controller's servoj smooths over `lookahead_time` (0.03–0.2 s) with `gain` (100–2000). Proposed: 0.1 s and 300, to tune in stage 0b. |
| `finish` | `servoStop(decel)`, then wait until the arm is stationary |
| hard stop | `servoStop` at the commissioned stop deceleration, then `stopJ`, the same as the service's STOP today. A protective stop by the controller also counts |
| `read_joints`, health | From the RTDE **receive** interface, which the service already runs: `actual_q`, robot mode, safety mode and runtime state, at up to 500 Hz. This is better feedback than the xArm's 50 ms report bursts. |

The UR controller runs at **500 Hz**, and ur_rtde has `initPeriod()` and
`waitPeriod()` for pacing. The service runs Python on Windows (pc-05). So
start at **125 Hz** with lookahead smoothing, and raise the rate only if
stage 0a supports it.

### 2.2 UR-specific constraints

- **Limits come from the commissioned `control.motion` block,** not the
  xArm's safety level. **Today that block is the open placeholder**
  `UR5E-ARM-2026-10-08-OPEN-PLACEHOLDER`: joints ±360°, **10 deg/s, 30
  deg/s²**, stop deceleration 90 deg/s². Validation and `GET /limits` must
  report those. Real limits, which you deferred, are needed before
  trajectories are useful at speed.
- **Six joints,** J1..J6 base to wrist. The joint-count checks already use
  `num_joints`.
- **The RTDE watchdog** (`setWatchdog`, C207A0 on timeout) already guards
  the control link. Streaming must keep it fed, so a stalled executor
  protective-stops instead of the arm coasting.
- **The control interface is not thread-safe.** The service already
  serialises it. The executor holds it for the whole run, and STOP must
  still reach it within one tick: the same lock-and-flag pattern as on the
  xArm.
- **Existing UR gates still apply:**
  - the claim;
  - refusal in manual mode, with 412 `manual_mode`;
  - `allowed_actions`;
  - the force guard (`force_guard_n`), checked on every tick from the
    receive interface;
  - the envelope and workspace checks, run on the interpolated path in
    joint space.

### 2.3 Code layout

The shared pieces move into the unified package, as
`src/robot_motion/trajectory/` with interpolation, validation, sessions and
the executor. Then:
- the xArm service (`src/core`) imports them from there;
- the UR backend lives in `src/robot_motion/drivers/ur_trajectory.py`;
- the UR routes go into the UR service.

The xArm's behaviour stays identical, and its 1000+ tests guard the move.

---

## 3. Stages (the same gates as the xArm)

0. **0a, no motion, on pc-05:**
   - the RTDE receive rate and age;
   - the jitter of a 125, 250 and 500 Hz absolute-deadline loop under the
     service's normal load;
   - the controller's answer time for a harmless control call.

   This sets the rate.
1. **Offline:**
   - extract the shared modules (the xArm tests must stay green);
   - add the UR backend with a fake RTDE pair;
   - add the routes and tests to the UR service.
2. **0b, supervised motion probe, with you at the robot:**
   - a small slow wiggle (J1 ±5°, J6 ±10°, within the placeholder's 10
     deg/s) at 125 Hz, then 250 Hz;
   - tune lookahead and gain;
   - one cancel and one STOP at mid-swing.
3. **Supervised API test on the arm:**
   - deploy to `debug-ur5e` (pc-05 runs the checkout's Python directly, so
     no service-command change);
   - run the stage 3 check script against port 8075.

## 4. Decisions needed from you

1. **Real motion limits** for `control.motion`: joint envelope, max joint
   speed and acceleration. Without them, trajectories are capped at the
   placeholder's 10 deg/s.
2. **Starting rate:** 125 Hz is proposed; the measurement can raise it.
3. **Cartesian moves:** confirm the UR5e version is joint-space only, like
   the xArm's.
4. **Dashboard:** whether the UR5e trajectory skills should appear on the
   dashboard straight away, or only after stage 3.
