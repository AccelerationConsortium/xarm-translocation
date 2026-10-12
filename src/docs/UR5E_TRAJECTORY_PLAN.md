# Joint trajectories on the UR5e (ligand_ur5e) — plan

Status (2026-10-11): **built; stages 0a, 0b and 3 passed on pc-05.** See
[section 5](#5-what-is-built-stage-1). The xArm5 version is built,
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
- the UR backend lives in `src/robot_motion/trajectory/ur_backend.py`;
- the UR routes live in `src/robot_motion/trajectory/routes.py`, installed
  by the UR service (`control.py`).

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

---

## 5. What is built (stage 1)

On branch `feature/ur5e-trajectory`, offline only:
- **Shared package.** `src/robot_motion/trajectory/` holds
  `joint_trajectory.py` and `executor.py`; `src/core/joint_trajectory.py`
  and `src/core/trajectory_executor.py` are alias shims, so the xArm
  service is unchanged (its trajectory tests pass).
- **Backend** (`ur_backend.py`, `URServoJBackend`):
  - `servoJ(q_rad, 0, 0, 1/rate, lookahead, gain)`; `servoStop` at the
    end; `servoStop` then `stopJ` on a hard stop;
  - `prepare` refuses without fresh feedback, RUNNING/NORMAL, the script
    running, the slider at 100 %, the arm still, and the commissioned
    envelope and box;
  - every tick holds the feedback to the envelope (plus
    `path_tolerance_deg`), the workspace box (plus `path_tolerance_mm`),
    1.25 × the speed cap, and the force guard: any is a hard stop;
  - `path_errors` checks the planned path against the box and the
    controller's safety limits with controller forward kinematics, every
    `path_check_step_deg` (2°) of joint travel, before validate passes and
    before start.
- **Routes** (`routes.py`): the xArm contract, six joints:
  - `GET /limits` (from `control.motion`, plus `workspace_mm`,
    `force_guard_n`, `commissioning_id`);
  - `POST /validate` (needs an open arm session for the measured pose and
    the kinematics; no claim);
  - create, chunks (PUT or POST), start, cancel, status and log.
- **Gates.** Start uses the same `arm_block` as a moveJ (latch, link,
  script, feedback, RUNNING/NORMAL, manual mode, slider, still, envelope,
  box), then reserves the motion executor's move slot for the whole run.
  While it runs, every move, manual mode, Clear errors, force zero and
  Disconnect answer 409. The run watches the claim, the operator list, the
  session and its RTDE watchdog every 100 ms (a constrained stop).
- **STOP.** `/control/stop` bumps the trajectory stop generation first,
  then latches the motion executor and sends `stopJ`; nothing is streamed
  after it. Clear errors re-enables moves, as after any STOP. Service
  shutdown stops a run the same way.
- **Status.** `details.trajectory` in `/status`, as on the xArm.
- **Tests.** `tests_robot_motion/test_ur_trajectory_backend.py` (14) and
  `test_ur_trajectory_routes.py` (8): a full 125 Hz run, refusals in manual
  mode and outside the box, moves and Disconnect refused while running,
  STOP sends nothing more, and the force guard.

**To enable it** on pc-05, add to the `control` block of the local config
(off by default; the `motion` block's limits apply):

```json
"trajectory": {"enabled": true, "rate_hz": 125, "lookahead_s": 0.1, "gain": 300}
```

Other keys, all optional: `max_rate_hz` (125), `start_tolerance_deg`
(0.5), `max_duration_s` (600), `max_points` (60000),
`max_points_per_chunk` (5000), `path_check_step_deg` (2), `max_fk_checks`
(3000), `log_dir` (`logs/trajectory`, beside the config).

## 6. Stage 0a result (2026-10-11, pc-05, no motion)

`tools/ur_servo_timing_probe.py`, receive-only (no control interface), with
the service up and its arm session open. Python 3.12.13 on Windows 10.

- **RTDE receive:** 500 Hz; gaps between packets p50 2.05 ms, max 2.67 ms.
- **Absolute-deadline loop** (the executor's pacing, GC off, 10 s each):

| Rate | Late p50 | p99 | max | Late > 1 period |
|---|---|---|---|---|
| 125 Hz | 0.07 ms | 0.99 ms | 1.15 ms | 0 |
| 250 Hz | 0.06 ms | 0.94 ms | 1.04 ms | 0 |
| 500 Hz | 0.10 ms | 1.04 ms | 1.17 ms | 0 |

Start stage 0b at 125 Hz and try 250 Hz; leave 500 Hz off (the worst case
is over half its 2 ms period). The probe ran in its own process, so 0b
measures the timing inside the service.

## 7. Stage 0b result (2026-10-11, pc-05, on the arm)

`tools/ur_servo_probe.py`: the production code path without HTTP (the
service's ControlSession, the shared executor, URServoJBackend with the
commissioned limits as guards). Wiggle J1 ±5°, J6 ±10°, min-jerk, peak 8
deg/s, 10.6 deg/s², 9.4 s; placeholder limits (10 deg/s, 30 deg/s²).

| Rate | Lookahead / gain | Lag | Tracking rms / max (lag removed) | Following error max |
|---|---|---|---|---|
| 125 Hz | 0.1 / 300 | 110 ms | 0.028° / 0.12° | 0.95° |
| 125 Hz | 0.05 / 600 | 60 ms | 0.029° / 0.12° | — |
| 125 Hz | 0.03 / 1000 | 42 ms | 0.028° / 0.14° | — |
| 250 Hz | 0.1 / 300 | 106 ms | 0.029° / 0.13° | 0.93° |
| 250 Hz | 0.05 / 600 | 56 ms | 0.026° / 0.12° | 0.54° |
| 250 Hz | 0.03 / 1000 | 36 ms | 0.027° / 0.11° | 0.37° |

- **Timing:** zero late ticks in all 10 runs; lateness max 1.4 ms; a servoJ
  call returns in at most 0.25 ms. Force change at most 2.1 N.
- **Lag** is about the lookahead plus 6–12 ms; tracking does not change, so
  the defaults are now **lookahead 0.03 s, gain 1000, 125 Hz** (250 Hz
  allowed: only ~6 ms better).
- **Cancel** at peak speed slowed to rest on the path. **STOP** at peak
  speed stopped with ~0.8° overshoot on J6 (lag plus braking at 90 deg/s²);
  nothing was sent after it.
- **Found and fixed:**
  - ur_rtde `getForwardKinematics(q)` returns a wrong pose, and with an
    all-zero TCP offset it ends the script (protective stop). FK now always
    passes the TCP offset verified at Connect (1550499).
  - A STOP during a run ended the control script; the pendant said
    "Another thread is already controlling the robot". ur_rtde runs
    servoJ in its own script thread, so a `stopJ` is a second thread
    commanding motion. While servoing, every stop is now a `servoStop`.
    `servoStop` waits while the arm brakes under the SDK lock, so with
    trajectories enabled the config requires `watchdog_hz` below
    1 / (2 × max speed / stop deceleration): 4.5 Hz for the placeholder;
    pc-05 runs 2 Hz. Other stops are sent without waiting.

- **STOP re-test with servoStop (fd841ed):** the control script and the
  watchdog were still up 0.5 s and 2 s after a STOP at peak speed; the arm
  drove back and a full run followed on the same session (lag 42 ms,
  tracking 0.025° rms, 0 late ticks, lateness max 0.9 ms). **Stage 0b
  passed.**

## 8. Stage 3 result (2026-10-11, on the arm, through the edge)

`tools/ur_trajectory_http_check.py` on pc-05, as a remote client: emailed
sign-in at the edge, then every call through `http://100.64.254.6/ur5e/`
(Caddy checks the cookie and adds the operator identity and the device
edge secret; the client's `X-Claim-Token` passes through). Claim,
heartbeats, `GET /limits`, validate (with the workspace check), create,
chunk upload (POST), start, status, cancel, STOP, Clear errors, log,
release. Wiggle as in stage 0b, 125 Hz, 0.03 / 1000.

| Run | Result | Lag | Tracking rms | Late ticks |
|---|---|---|---|---|
| full | completed, 0.014° from the target | 40 ms | 0.027° | 0 |
| cancel at 50 % (twice) | cancelled, on the path | 40–42 ms | 0.029–0.030° | 0 |
| STOP at 50 % | stopped; session open, watchdog OK, moves latched until Clear errors | 40 ms | 0.029° | 0 |

During a run, status `measured` was live (2.7–13 ms old), `/status` showed
the session running and the arm busy, and `arm.move_joints` was not
offered. A moveJ sent 0.1 s after a cancel was refused as `robot_moving`
while the arm settled; the backend now waits until the arm is still before
freeing the move slot. **Stage 3 passed.**

The dashboard API proxy (`/api/equipment/ligand_ur5e/control/*`) is not
the client route for the UR: its registry entry has no `edge_secret_env`,
so it sends the global edge secret, which the UR refuses (401). The edge's
`/ur5e/` route is the one the panel and clients use.

**Still to do:** the real motion limits. The UR column in
`TRAJECTORY_API.md` was added on 2026-10-11; the dashboard skills
(`trajectory.*` for `ligand_ur5e`, with the device advertising the same
names in `allowed_actions`) on 2026-10-12.
