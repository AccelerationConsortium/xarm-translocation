# Chunked joint trajectories executed with ServoJ (or online planning) — plan

Status (2026-10-09): **version 1 implemented on branch
`feature/xarm-joint-trajectory`.** The executor has run on the arm through
the stage 0b probe (§2); the HTTP routes have not yet run on the arm.
It is disabled by default (`src/settings/trajectory.yaml`). See §13 for what
was built and where it differs from this plan. **Test first:** whether
execution uses ServoJ (mode 1) or joint online planning (mode 6) is decided
by measurements on the real arm before either is built (§2, §10 stage 0).

**Astra's answers (2026-10-09):**
- **Synchronisation:** the goal is to stay in sync with Astra's digital
  twin. The twin runs on a workstation in the Sandford Fleming building
  and reaches this server over HTTP through SSH.
- **Two-way (closed-loop) sync** is not possible over that link, and is
  out of scope.
- **Upload:** sending the **whole trajectory, then one-way sync**, is
  acceptable. The arm runs the twin's timed plan, and the twin aligns
  to the start time and progress the server reports (§4, one-way sync).

This sets the scope:
- **Version 1** is the whole-trajectory path: create, one chunk with
  `final: true`, validate, start.
- **Chunked append during execution** (§10 stage 2) is **deferred** until
  a use needs it. The contract below still covers it, so adding it later
  changes no existing field.
- Because exact plan timing is the point, the backend choice **leans to
  mode 1**, subject to the stage 0b motion test.

Requested by Astra: upload a joint trajectory over HTTP, either whole or as
consecutive chunks, and have the device buffer, interpolate and execute it
at a fixed rate with UFACTORY `set_servo_angle_j()` (ServoJ). Today every
arm move is one mode-0 target, and a second request while one runs gets
`409 motion_in_progress`, so an externally planned continuous trajectory
cannot be executed.

Scope: **arm joints only on the xArm5.** No rail, and no rail-arm
synchronisation. That case stays in [`TRAJECTORY_PLAN.md`](TRAJECTORY_PLAN.md),
and this executor is the arm half it would need. Work happens on a
feature branch off `main`. Physical acceptance happens on `debug-xarm5`
after `main` is merged into it, under separate authorisation (§10 stage 3).

---

## 1. What the SDK and controller give us (xarm-python-sdk 1.18.4)

Checked in the SDK source. These facts shape every decision below.

- **Servo mode executes only the latest target.** `set_servo_angle_j` needs
  `set_mode(1)` plus `set_state(0)`. The controller moves straight to each
  target, with no planning or blending. `speed`, `mvacc` and `mvtime` are
  documented as *reserved*, so they are ignored. The executor alone shapes
  the motion: whatever it sends at each tick is the profile.
- **Each command is a request/response round trip.** `move_servoj` goes
  through `set_nfp32`, which sends, then waits for the reply under the
  SDK's single command lock, with `SET_TIMEOUT` = 2000 ms. The round-trip
  time therefore limits the achievable rate. It must be measured on the
  real LAN (§10, stage 0).
- **STOP shares that lock.** `emergency_stop()` is `set_state(4)` on the
  same channel, so a STOP waits behind any in-flight servo command. With the
  default 2 s timeout, a controller that stops answering could delay STOP
  by up to 2 s. The executor shortens the command timeout for the session
  (§7.3).
- **Every other SDK call contends too.** `/status` and `/positions` read
  joints with `get_servo_angle()`, a command round trip on the same lock.
  Rail and gripper reads are Modbus through the control box, at 10 to 30 ms
  each. Any of these during a session adds jitter or stalls a tick, so a
  running session must serve them from cached report data (§7.4).
- **The SDK checks joint range on every call** and pads to 7 joints. On the
  xArm5 it zeroes J6 and J7, so the client sends exactly 5.
- **Feedback.** The controller connects with the SDK default report stream,
  `rich` on port 30002, which pushes at 5 Hz. The real-time stream on
  30003 pushes at 100 Hz, in batches about every 50 ms (stage 0a, §2).
  Commanded-versus-measured records (§8) use the real-time stream and
  never issue extra read commands.

Host: the xArm control box is connected to the Cytation PC directly by
Ethernet, on its own subnet (PC 192.168.1.100, box 192.168.1.237, on-link).
The xArm service runs on that PC under Python 3.14.4. There,
`time.perf_counter()` and `time.monotonic()` are both high-resolution, and
`time.sleep()` uses a high-resolution waitable timer. The PC also hosts the
plateloc, cytation, two OT-2 gateway, shaker, biostack and dashboard API
services, so CPU contention is real and the timing must be measured, not
assumed.

---

## 2. Two ways to execute, chosen by test: ServoJ (mode 1) or online planning (mode 6)

UFACTORY's [servo mode guide](https://docs.supportarticle.ufactory.cc/support_articles/developer/ufactory-servo-mode-guide.html)
sets host conditions for mode 1:
- a direct cable;
- network latency under 0.5 ms;
- "Linux system with PREEMPT_RT patch. Windows has higher latency and is
  not recommended."

It recommends 50–200 Hz (250 Hz maximum; below 50 Hz "motion may not be
continuous"). It adds: "If conditions do not allow, you can first try
online planning modes (modes 6 and 7)."

This cell against those conditions:

| Condition | xArm cell |
|---|---|
| Direct cable | **Met:** the control box is on the Cytation PC's Ethernet port |
| Latency under 0.5 ms | **Met for typical commands.** Stage 0a (below): read round trip p50 0.24 ms, p99 0.36 ms. Rare stalls reach 27 ms |
| Linux with PREEMPT_RT | **Not met:** Windows 10, Python, a PC shared with about 8 other services |

### Stage 0a results (2026-10-09 01:21–01:22 UTC, no motion)

Measured by `tools/servo_timing_probe.py`, run on the Cytation PC. The
setup:
- Python 3.14.4 on Windows 10 19045; `perf_counter` is
  QueryPerformanceCounter, at 0.1 µs resolution;
- the controller on firmware v2.8.2;
- the `xarm` service running as usual, with the arm idle and unclaimed.

The probe used its own read-only SDK connection. It never sent a set,
mode, state or move command, and the arm's status was unchanged
afterwards.

| Measurement | Result |
|---|---|
| Read command round trip (`get_servo_angle`, 3000 back to back) | p50 0.24 ms, p99 0.36 ms, p99.9 0.74 ms, max 26.9 ms; 2 of 3000 over 5 ms |
| Sleep loop alone, 50 / 100 / 200 / 250 Hz (10 s each) | wake lateness p99 under 1 ms and max 1.3 ms at every rate; no interval over 1.5 periods |
| One read per tick at 100 Hz (1000 ticks) | interval p50 10.0 ms, p99 10.7 ms, max 25.8 ms; read p99 4.8 ms; **2 intervals over 1.5 periods**, 3 reads longer than a period |
| One read per tick at 200 Hz (2000 ticks) | interval p50 5.0 ms, p99 5.7 ms, max 24.3 ms; **9 intervals over 1.5 periods**, 10 reads longer than a period |
| Report streams | normal (30001) 5.4 Hz and rich (30002, the stream the service uses) 5.2 Hz; real-time (30003) 100.6 Hz on average, but delivered in **bursts about every 47–52 ms** |

What the results say:
- **The cable and the PC are fine.** Typical latency meets UFACTORY's
  0.5 ms condition, and Windows with Python 3.14 holds its schedule to
  about 1 ms at up to 250 Hz.
- **The weak point is rare command stalls of 15–27 ms.** They hit about 1
  tick in 400 at 100 Hz and 1 in 200 at 200 Hz. Their source isn't known
  yet. It may be the SDK's hand-off from its receive thread, the service's
  own traffic to the controller, or the controller itself.
  - **Mode 1 at 100 Hz:** expect a 2–3 period hiccup every few seconds.
    Whether that is visible on the arm is what stage 0b must show.
  - **Mode 6 at 20–50 Hz:** a 25 ms stall is shorter than one command
    period, so it would not matter.
- **Feedback.** The cached state the service reads today updates only at
  5 Hz, so session records (§8) must use the real-time stream. Its 50 ms
  batching limits how fresh "arrived" can be: about 50 ms.

**Rerun with the `xarm` service stopped** (2026-10-09 02:46–02:47 UTC; the
service was restarted straight after). Nothing else changed:

| Measurement | Service running | Service stopped |
|---|---|---|
| Read round trip (3000) | p99 0.36 ms, p99.9 0.74 ms, **max 26.9 ms** | p99 0.33 ms, p99.9 0.45 ms, **max 0.55 ms** |
| 100 Hz, one read per tick | 2 late intervals, read max 25.8 ms | **0 late intervals**, read p99 1.0 ms, max 1.1 ms |
| 200 Hz, one read per tick | 9 late intervals, read max 24.3 ms | **0 late intervals**, read p99 1.0 ms, max 1.1 ms |
| Sleep loop alone | max lateness 1.3 ms | max lateness 1.0 ms |
| Real-time report | 100 Hz, about 50 ms bursts | the same (99.8 Hz, bursts to 62 ms) |

**The stalls come from the service's own traffic to the controller, not
from Windows, the cable or the SDK.** The service polls in the background:
- joints, pose and the rail at 10 Hz (`telemetry_loop`, with
  `XARM_TELEMETRY_HZ` defaulting to 10);
- the gripper at 1 Hz (`gripper_status_loop`).

The rail and gripper reads go over the control box's RS485 link. The
likely cause is one of those Modbus reads holding up the controller's
command handling, but that isn't confirmed.

So the §7.4 rule, that a session suspends every other SDK traffic
including these two loops, is **required, and by this evidence
sufficient**, for clean timing. With that traffic gone, a 100–200 Hz
mode 1 loop on this Windows PC showed no late tick in 3000. Mode 1
therefore looks viable. Whether the motion is smooth on the arm itself is
still for stage 0b to show.

So the design keeps two backends behind one interface, and stage 0b picks
one.

### Stage 0b results (2026-10-09 13:39–14:29 UTC, on the arm)

Run with `tools/servo_mode_probe.py` on the Cytation PC, with the user at
the robot and the `xarm` service disconnected from the arm. The arm started
at home, `[0, -45, 0, 45, 90]`. Each run was J1 ±5° and J5 ±10°, two cycles,
peak 20°/s, 7.5 s. Records are in `logs/servo_mode_probe/` on this machine.

| | ServoJ 100 Hz | ServoJ 200 Hz | Mode 6, 20 Hz | Mode 6, 50 Hz |
|---|---|---|---|---|
| Send lateness, max | 1.1 ms | 1.2 ms | 1.0 ms | 1.1 ms |
| Command round trip, max | 3.2 ms | 3.1 ms | 3.2 ms | 3.3 ms |
| Lag behind the commands (upper bound; includes report delivery) | 15 ms | 10 ms | 95 ms | 65–70 ms |
| Tracking error after that lag (max) | 0.10–0.11° | 0.07–0.11° | 0.76° | 0.73–0.75° |
| End: error to the last command | <0.001° | <0.001° | 0.19° | 0.16° |
| Smoothness over 50 ms (acc p99) | 93 deg/s² | 64–73 deg/s² | 97 deg/s² | 88 deg/s² |

**Stops at 20°/s, mid-swing:**
- **Cancel (constrained stop):** rest along the path in 50–60 ms, averaging
  about 400 deg/s², under the 500 deg/s² limit.
- **STOP:** nothing was sent after it. The arm reported state 4 26–41 ms
  after the last command. ServoJ stopped exactly at the last commanded
  pose; mode 6 stopped 1° short of it, because of its lag.
- **Recovery** (as Clear errors does) worked every time.
- The user saw all runs as smooth.

**Found on the arm, and fixed the same day:**
- **Lead-in starved the stop.** A lead-in sized to the full acceleration
  limit left no budget for a constrained stop, which then fell back to a
  2 s ramp. Lead-ins are now sized at half the limits, and the stop rate
  follows the phase the arm is in.
- **Garbage-collection stalls.** Two 29–40 ms command stalls in one run,
  with nothing else on the controller; most likely a garbage-collection
  pause. The collector now pauses while streaming. The confirmation run
  had none (worst round trip 3.2 ms over six runs).

**Behaviour to know:**
- **State 5 at mode changes.** The arm reports state 5 around every mode
  change, before `set_state(0)`. Because reports arrive in bursts, one such
  frame can arrive after streaming has begun. The executor ignores report
  frames until it has seen the streaming mode in a healthy state, so this
  caused no false fault.
- **States 1 and 2 alternate in ServoJ.** The reported state flips between
  1 (moving) and 2 (ready) while streaming, including long stretches of 2
  while the arm follows. Both are treated as healthy.
- **Mode 6 overruns.** It keeps moving for 0.3–0.5 s after the last
  command, and ends 0.15–0.2° short.
- **ServoJ holds.** 1–4 single 10 ms report frames per run show a held
  joint (10 during the cancel run), which isn't visible.

**Decision: mode 1 (ServoJ), at 100 Hz by default.** It follows the plan to
about 0.1° with a constant 10–15 ms lag, which suits Astra's one-way sync.
Mode 6 is roughly seven times less accurate and lags 65–95 ms. 200 Hz was
slightly tighter than 100 Hz, and stays available within
`max_servo_rate_hz`.

| | Mode 1, ServoJ (`set_servo_angle_j`) | Mode 6, joint online planning (`set_servo_angle`, `wait=False`) |
|---|---|---|
| Who shapes the motion | The executor. The controller moves at full speed (180 deg/s) to the newest target; speed and acceleration are ignored | The controller. Each new target interrupts the current move, and the controller replans from where the arm is, using the speed and acceleration the SDK sends with every call |
| Command rate | Fixed 100–200 Hz; the cap is set by measurement | Lower, proposed 20–50 Hz; set by test |
| Timing | Exact to the tick, when commands arrive on time | Approximate: the controller's planner decides the arrival times |
| Path | Follows the interpolant point for point | Close to it when targets are dense; the deviation must be measured |
| Windows timing jitter | Hurts: a late tick shows as a stall then a resume (the executor delays rather than skips, §5) | Tolerated: the controller keeps moving between commands |
| Commands stop arriving | The executor must run the constrained stop (§6) | Gentle by construction: the arm decelerates to the last target within its own limits |
| Speed and acceleration limits enforced by | Our executor only | The controller, with our values |
| Documented | Mode, rates and host conditions | Interrupt-and-replan behaviour. **Not documented:** whether velocity carries over smoothly at each interruption, so it must be tested on our firmware (v2.8.2) |

**The same for both:**
- the wire format and interpolation (§3);
- the session and chunk API (§4);
- validation (§9);
- the control gates (§7);
- the records (§8).

The SDK lock, timeout and no-other-traffic rules (§1, §7.3, §7.4) also
apply to both, because `set_servo_angle` is a request/response too. Only
the code that sends the commands differs. It is selected in the
deployment config (`trajectory.backend: servoj | online_planning`), not
per request, and is reported in the status and the `execution_model`.

**Mode 6 specifics:**
- At each command tick, send the interpolant at `τ + lookahead`
  (proposed: two command periods). The arm must never reach its current
  target before the next one arrives, or it slows for every target and
  moves stop-and-go.
- The speed sent with each target is the joint speed that reaches it on
  schedule, `max |Δq| / Δt`, clamped to the safety-scaled
  `max_joint_speed`. The acceleration sent is the streaming acceleration
  limit (§12).
- An underrun still ends the session (§6). The last target sent is a point
  on the buffered path, so the arm comes to rest on the path.
- Mode 0 is restored on every exit, the same as for mode 1 (§5).

If exact timing turns out to be required (Astra's answer) and mode 1 is
not smooth on this Windows PC, the fallback is the setup UFACTORY
recommends: a small dedicated Linux PREEMPT_RT computer on the control
box's subnet, running only the executor. That is a deployment change and
would be planned separately.

---

## 3. Wire format

Units and conventions are fixed and not negotiable per request.

| Field | Meaning |
|---|---|
| `t` | seconds from the **trajectory** start (not the chunk start). The first point of chunk 0 is at 0; `t` strictly increases across chunks |
| `joints_deg` | joint angles in degrees, base to wrist, J1..J5 on the xArm5; exactly `num_joints` entries (5) |
| `velocities_deg_s` | optional, the same shape |
| `accelerations_deg_s2` | optional, the same shape; only with velocities |

A chunk is:

```json
{"seq": 3, "final": false, "points": [{"t": 1.20, "joints_deg": [...], "velocities_deg_s": [...]}]}
```

Rules:
- `seq` starts at 0 and increases by 1. `final: true` marks the last chunk.
  Without it, running out of data is an underrun, not a normal end.
- Points closer together than one servo period are rejected.
- The trajectory must start and end at rest. The first point and the final
  point have zero velocity and acceleration; when the client omits them,
  the server assumes zero there.
- Limits per chunk and per session (proposed): at most 5000 points per
  chunk, and at most 600 s per session (the existing `max_duration_s`).

### Interpolation rule

The executor samples a **local Hermite interpolant** at the servo rate.
Each segment depends only on the state at its two end points, so a chunk
boundary is mathematically the same as any other point. This is what
removes artificial pauses at chunk joins: a global spline would change
already-executed segments when new data arrived.

- With velocities and accelerations: **quintic Hermite**. Position,
  velocity and acceleration are continuous everywhere, including across
  chunks. This is the recommended mode, since the planner already has
  these values.
- With velocities only: **cubic Hermite**. Position and velocity are
  continuous; acceleration can step at points, and each step is checked
  against the limit.
- Positions only: velocities come from central differences (cubic). This
  needs one point of lookahead, so the last segment of a chunk can't run
  until the next chunk's first point arrives or `final` is set. The buffer
  accounting in §5 includes this.

---

## 4. API

These are freehand routes, because they bypass the motion graph. They
therefore live under `/control/freehand/*`, and `strict_graph_guard`
refuses them in STRICT, like every other route there.

| Route | Gate | Does |
|---|---|---|
| `POST /control/freehand/trajectory/validate` | login | Validates a whole trajectory without moving anything, and returns the full report. This is the arm-only sibling of `/control/trajectory/validate` |
| `POST /control/freehand/trajectory` | claim | Creates a session: `num_joints`, `servo_rate_hz`, `interpolation`, `start_tolerance`. Returns `session_id`. One open session per controller. The session is bound to the creating claim token |
| `PUT /control/freehand/trajectory/{id}/chunks/{seq}` | claim (same token) | Uploads or appends a chunk, before or during execution. Returns the chunk's validation report and the session status |
| `POST /control/freehand/trajectory/{id}/start` | claim, guards §7.1 | Starts execution. `min_start_buffer_s` (default 1.0 s) must be buffered, unless the final chunk is already in |
| `POST /control/freehand/trajectory/{id}/cancel` | claim | Constrained stop along the path (§6), ending in `cancelled` |
| `GET /control/freehand/trajectory/{id}` | login | Status (below) |
| `GET /control/freehand/trajectory/{id}/log` | login | The session's timing and feedback record (§8) |

Uploading a whole trajectory is simply create, then one chunk with
`final: true`, then start. That is the first deliverable (§10).

### Chunk upload outcomes

| Case | Response |
|---|---|
| next expected `seq`, valid | `200`, `accepted` |
| same `seq`, byte-identical content (sha256 of the canonical JSON) | `200`, `duplicate: true`; never executed twice |
| same `seq`, different content | `409 chunk_conflict` |
| `seq` beyond the next expected | `409 chunk_out_of_order` with `expected_seq`. v1 does not hold gaps |
| first `t` not after the last received `t` | `422 trajectory_invalid` (`time_not_increasing`) |
| first point due before the executor's lookahead (already too late to run smoothly) | `409 chunk_too_late` |
| session finished, cancelled or failed, or the final chunk is already in | `409 session_closed` |
| content invalid (shape, limits, continuity) | `422 trajectory_invalid` with the full report |

### Status

```text
state            created | ready | running | stopping | completed | underrun_stopped
                 | cancelled | stopped | failed
reason           set in terminal states (e.g. sdk_error, claim_lost, graph_mode_strict)
received         last_seq, points, final_received, t_end_received
executed         t_exec (trajectory time reached), point_index, fraction (when final is known)
buffered_s       received time still ahead of t_exec
executable_s     buffered_s minus the lookahead the interpolation needs
servo            rate_hz_configured; period_ms {mean, p50, p99, max}; overruns;
                 round_trip_ms {p50, p99, max}; sdk_errors
tracking         max_error_deg, last_error_deg (commanded versus reported)
final            on exit: measured joints, error to the last target, settle time, mode restored
started_at_utc   wall-clock time of τ = 0 (the first trajectory sample sent), from the PC clock
measured         latest joints from the real-time report, with their receive time (UTC)
```

**One-way sync.** `started_at_utc` and `executed.t_exec` let the twin
play its copy of the plan aligned with the real arm. `measured` lets it
draw the real arm beside it. The twin polls `GET …/{id}` (10–20 Hz is
plenty), and the network delay only makes its picture slightly late.
`started_at_utc` comes from the Cytation PC's clock. If the twin's clock
is not synchronised with it, the twin should align to `t_exec` instead,
which needs no shared clock. Afterwards, the session log (§8) gives the
exact sent-versus-arrived timeline for comparing the plan with reality.

The figures under `servo` are **measured** values, never the configured
rate.

---

## 5. Execution model

Written for mode 1. With mode 6, the thread, the clock and the exits are
the same, but it enters `set_mode(6)`, sends `set_servo_angle(..., wait=False)`
at the lower rate with the lookahead and the per-target speed, and
doesn't need the lead-in, because the controller plans the first move
itself (§2).

- **A dedicated executor thread**, not the asyncio loop. The thread keeps
  absolute deadlines `t0 + k / rate` on `time.perf_counter()`, so lateness
  never accumulates as drift. It sleeps to just before each deadline, then
  sends the sample for trajectory time `τ_k = k / rate`.
- **The trajectory clock is the servo clock.** HTTP arrival times never
  move `τ`; chunks only extend the buffer. That is what makes the rhythm
  independent of the network while the buffer is adequate.
- **Rate.** `servo_rate_hz` is configurable from 100 to 250 Hz (the
  vendor's recommended range); the default is 100 Hz. A rate is accepted
  only up to the highest value that stage 0 and stage 3 measurements
  support on this PC (a config cap). The configured rate is never reported
  as achieved.
- **Overruns.** A tick that starts late still sends the sample for its
  scheduled `τ`; it does not catch up by skipping. Its lateness is
  recorded. More than N consecutive late ticks, or one tick later than a
  hard bound (proposed: 5 periods), ends the session with a constrained
  stop and `reason: timing`.
- **Entering servo mode.** Check §7.1. Read the measured joints from the
  report and compare them with the first point (`start_tolerance`, proposed
  0.5 deg). Insert a **lead-in** quintic segment from the measured pose to
  the first point, sized by the acceleration limit, because even a 0.5 deg
  jump in one tick would be an extreme acceleration. Then `set_mode(1)`,
  `set_state(0)`, and confirm mode 1 in the report before the first send.
- **Leaving servo mode, on every exit path.** After the last command, wait
  until the reported joints settle within tolerance of the final target (or
  a timeout), then `set_mode(0)` and `set_state(0)`, then release the
  motion slot. The exception is STOP (§7.2): state 4 stays until Clear
  errors, which already restores mode 0 and state 0.

---

## 6. Constrained stop (underrun, cancel, faults)

The controller holds the last target when commands stop. If that target
was mid-motion, holding it means an instant stop at full speed, which is
exactly what the request rules out. So the executor never just stops
sending.

**Stop along the path, not off it.** The executor slows the trajectory
clock from rate 1 to 0, so `τ` advances more and more slowly. The
commanded joints therefore stay on the planned, collision-checked path
while they decelerate. The ramp is sized so that commanded joint
accelerations stay within the limit (the path's own acceleration plus the
velocity times the clock deceleration).

**Trigger for underrun:** at every tick,

```text
stop_horizon  = path time the ramp needs from the current speed + margin (proposed 0.2 s)
if not final_received and executable_s <= stop_horizon: begin the underrun stop
```

The buffer must therefore always hold more than the stop horizon, and
`min_start_buffer_s` must exceed it with margin. The status reports both
numbers, so a client can see how close it is running.

**v1: an underrun stop is final.** The session ends in `underrun_stopped`
at the reached `t_exec`. Chunks that arrive during the ramp get
`409 chunk_too_late` / `session_closed`. To continue, the client starts a
new session from the stopped pose. Resuming in place is a later option
(§12).

| Event | Behaviour | Terminal state |
|---|---|---|
| buffer runs short (no final) | constrained stop along the buffered path | `underrun_stopped` |
| client disconnects | HTTP has no connection to lose, so a disconnect shows up as an underrun or a claim expiry, whichever comes first | `underrun_stopped` / `failed: claim_lost` |
| `POST …/cancel` | constrained stop | `cancelled` |
| claim released, expired or taken over (checked every 100 ms) | constrained stop | `failed: claim_lost` |
| graph mode back to STRICT (the bounded override expired) | constrained stop | `failed: graph_mode_strict` |
| timing fault (§5) | constrained stop | `failed: timing` |
| SDK error code on a servo send, controller error or warning in the report, arm not in mode 1 | **no** soft ramp, since the controller may not be following: stop sending, `emergency_stop()` | `failed: sdk_error` (code recorded) |
| `/control/stop`, the sash watchdog or a controller STOP | the existing STOP path (`emergency_stop`, arm then rail); the executor sends nothing after it | `stopped` |
| connection lost | stop sending; the controller holds or faults on its own; the result is reported on reconnect | `failed: disconnected` |

---

## 7. Keeping the existing control constraints

### 7.1 Start gates (in this order, all existing helpers)

1. `require_claim`: the session's claim token must match the current
   holder.
2. `box_sim_guard("trajectory")`.
3. `strict_graph_guard("trajectory.servoj")`, which refuses in STRICT. When
   the full trajectory is known (final received), also refuse if the
   bounded mode override would expire before the planned end plus a
   margin.
4. `interlock_freehand_guard("trajectory.servoj")`: the sash gate (412).
5. `reserve_motion()`: the single motion slot, `409 motion_in_progress`.
   The slot is held for the **whole session**, as one motion, until mode
   0 is restored. `activity` reads `running` throughout.
6. The arm state must be ready and the rail not moving. The start-state
   check is done at start, not at upload, because the arm may have moved
   in between.

### 7.2 During the session

- Every other motion request gets `409 motion_in_progress` from the held
  slot. Chunk append and cancel are not motions; they are bound to the
  session and its token.
- Gripper actions must refuse with `409 motion_in_progress` while a
  session runs. **This is a new gate.** Today only the graph-routed gripper
  path (STRICT) checks that the arm is still. The plain `/gripper/*`
  commands used in ADVISORY and OFF, the only modes a session can run in,
  do not. They would also put Modbus traffic on the command lock (§7.4).
- Graph bookkeeping: like every freehand move, the start clears the
  pinned pose (`last_arm_pose_name = None`). Re-pin afterwards with
  `/control/graph/recover_to`.
- STOP keeps its meaning: state 4, recovery required. A cancel is the soft
  stop; STOP is the hard one.

### 7.3 SDK command timeout

For the session, set the SDK command timeout to a short value (proposed
100 ms, at least several round-trip p99s) with `set_timeout`, and restore
it on exit. This bounds how long a STOP can wait behind a stuck servo
command. The executor also checks the stop flag before each send, so a
STOP gets the lock within one round trip.

### 7.4 No other SDK traffic while running

While a session runs:
- `/status`, `/positions` and the WebSocket broadcast serve joints, pose
  and state from report data cached by the SDK report thread, never from
  `get_servo_angle` and the like;
- rail and gripper reads return their last cached values, marked stale;
- the events exporter and the camera tracker issue no SDK commands.

A unit test asserts that the fake SDK sees **only** servo commands (and
the stop path) between the first and last send.

---

## 8. Records (sent versus arrived)

Each session writes `logs/trajectory/<session_id>.json`, and
`GET …/log` serves it.

- **Per tick:** `k`, scheduled time, actual send start, round-trip time,
  interval since the previous send, `τ`, the commanded joints,
  `buffered_s`, and the SDK code.
- **Per report sample:** receive time and reported joints. Commanded and
  measured are separate series, so "command sent" is never confused with
  "arm arrived".
- **Events:** chunk accepted, duplicate or rejected (with reason), start,
  underrun trigger, cancel, stop, mode changes, exit.
- **Summary:**
  - period statistics, overruns and round-trip statistics;
  - the maximum tracking error (commanded minus measured, with the
    report's delay noted);
  - the settle time after the last command;
  - the final error to the last target.

---

## 9. Validation

On each chunk, and on the whole trajectory for `…/validate`:
- shape, finiteness, joint count, and time from zero and strictly
  increasing;
- the minimum point spacing (one servo period), the session duration cap,
  and joint limits (`safety.yaml`, model 5);
- velocity and acceleration of the **interpolant**, sampled at the servo
  rate: velocity against the safety-scaled `max_joint_speed`, and
  acceleration against a new `servo.max_joint_acc_deg_s2`. That limit is
  proposed at 500 deg/s², the profile's `angle_acc`, and must be confirmed;
- across chunk boundaries, the joining segment is evaluated the same way,
  so continuity is checked rather than assumed;
- at start: the measured start against the first point.

It collects every error, as the shipped validator does, and reuses its
error codes. New codes: `segment_too_short` (below one period),
`joint_acc_too_high`, `not_at_rest` (first or final point),
`chunk_conflict`, `chunk_out_of_order`, `chunk_too_late`.

**Collision coverage, stated plainly:** validation checks joint limits
and the interpolant's speed and acceleration. Nothing else is in v1:
- **no** check of the robot's links against each other or the
  environment;
- **no** Cartesian check, beyond an optional check of the tool tip (TCP)
  against the workspace box at the waypoints, if a local forward-kinematics
  computation is added.

Passing validation is **not** a collision-safety proof. Collision checking
is the planner's responsibility, and the report says so in its
`execution_model`.

---

## 10. Implementation order

0. **Test first, before building either backend.** Nothing in stages 1–3
   starts until this stage has picked the backend.
   - **0a. No motion** (with the user's agreement, on the deployed xArm):
     - the read-command round trip from the Cytation PC (`get_servo_angle`
       p50, p99, max over a few thousand calls, timed with `perf_counter`);
     - the jitter of a 50 / 100 / 200 Hz loop of absolute-deadline sleeps
       on that PC, under its normal service load;
     - the report stream's rate.

     This also answers the 0.5 ms latency question of §2.
   - **0b. Supervised motion comparison** (the user at the robot, with an
     explicit go-ahead). Run it from a standalone probe script
     (`tools/servo_mode_probe.py`), not the service. Stop the `xarm`
     service first, so that only one client commands the arm.

     Use the same small, slow trajectory for every run: start at a known
     clear graph node; move J1 ±5 deg and J5 ±10 deg; peak joint speed
     about 20 deg/s; about 10 s long. Run it:
     - in mode 1 at 100 Hz, and at 200 Hz if 0a supports it;
     - in mode 6 at 20 Hz and at 50 Hz, with the §2 lookahead.

     Each run records:
     - commanded and reported joints;
     - send intervals and round trips;
     - controller errors and warnings.

     From those it derives:
     - the tracking error;
     - the timing error against the plan;
     - the path deviation;
     - the smoothness (acceleration and jerk from the reported joints).

     Each mode also gets one deliberate stop-sending mid-move and one STOP
     mid-move, to see the real behaviour.
   - **Decision gate.** Choose the backend from 0a, 0b and Astra's answer
     on whether exact timing is needed:
     - **Exact timing needed, mode 1 smooth at 100 Hz or more on this PC**
       (send period p99 under 1.5 periods, no visible stalls): mode 1.
     - **Exact timing needed, mode 1 not smooth:** stop. Plan the dedicated
       Linux executor of §2 instead.
     - **Smooth motion through the points is enough:** mode 6, if its path
       deviation is acceptable to Astra.

     Record the numbers and the decision in this document.
1. **Whole trajectory, offline:**
   - a pure module `core/joint_trajectory.py` (interpolants, validator,
     constrained-stop ramp; no SDK);
   - the session and executor (`core/trajectory_executor.py`, with the
     chosen backend) against an injected clock and a fake SDK;
   - the routes with a single final chunk.

   Then test against the Docker simulator. It accepts the commands but
   proves nothing about timing.
2. **Chunked append:** append during execution, idempotency,
   out-of-order, too-late and closed handling, the underrun stop, and claim
   loss.
3. **Hardware acceptance**, under separate authorisation, on `debug-xarm5`
   after `main` is merged into it. That merge brings the unified package,
   so the `xarm` service command must become `run --extra xarm pyxarm web`
   in the same supervised session. Start with a slow, small-amplitude
   trajectory at the rate chosen in stage 0, then:
   - the same trajectory whole and chunked;
   - an underrun on purpose;
   - a cancel, a STOP mid-trajectory, and a claim release;
   - only then a higher rate, if stage 0 supports it.

Offline tests that must exist before stage 3:
- **identical streams:** the commanded sample stream of one trajectory
  uploaded whole equals the stream from the same trajectory in N chunks,
  bit for bit;
- **idempotency:** duplicate, conflict, out-of-order and too-late chunks
  each give their documented result, and nothing is executed twice;
- **underrun:** the commanded velocity ramps to zero within the
  acceleration limit, and the last command is never held while the arm is
  moving;
- **exits:** STOP, cancel, claim loss, an SDK error and a disconnect each
  end in their documented state, restoring mode 0 where §5 says so;
- **gates:** the slot is held to the end; other moves get 409; STRICT gets
  409; the sash gate gets 412;
- **traffic:** no non-servo SDK traffic while running (§7.4);
- **records:** the log separates sent from reported.

---

## 11. Acceptance criteria mapping

| Requirement | Where |
|---|---|
| Whole and chunked uploads run continuously, no pause at chunk joins | §3 local interpolation, §5 servo clock, §10 identical-stream test |
| Agreed rhythm under the agreed network delay and jitter, given enough buffer | §5, §10 stages 0 and 3; delay and jitter bounds to agree with Astra |
| Verifiable outcomes for duplicate, out-of-order and missing chunks, disconnect, cancel, SDK fault | §4 table, §6 table, §10 tests |
| Timing, buffer and joint-feedback records, sent kept apart from arrived | §8 |
| Mock and offline first; physical acceptance authorised separately | §10 stages 1–2, then 3; the stage 0 mode test is also supervised and authorised separately |

---

## 12. Open decisions

1. **Backend:** decided 2026-10-09 by stage 0b: mode 1 (ServoJ). It tracks
   the plan to about 0.1° with a 10–15 ms lag, against mode 6's 0.75° and
   65–95 ms. Mode 6 stays available as a setting.
2. **Rate:** the default rate and the cap. For mode 1, 100 Hz is
   proposed; for mode 6, 20–50 Hz. Both are settled by measurement.
3. **Acceleration limit:** for streaming (500 deg/s² proposed). Also
   whether to bound jerk.
4. **Start:** the start tolerance (0.5 deg proposed) and the maximum
   lead-in length.
5. **Interpolation:** whether to require velocities and accelerations from
   the planner (quintic only), or also accept positions only.
6. **Network and buffer:** not needed for version 1. With a whole
   trajectory uploaded before start, the network plays no part in
   execution. The bounds come back only if chunked append is built.
7. **Resume:** whether an underrun may resume in place in a later version,
   instead of ending the session.
8. **Records:** how long session logs are kept.

The session and chunk contract, the interpolants and the stop ramp are
written so they don't depend on the robot. A later UR5e executor could
reuse them with `ur_rtde`'s `servoJ`, and only the backend that sends the
commands would change.

---

## 13. Version 1 as built (2026-10-09)

**Files:**
- `src/core/joint_trajectory.py`: pure interpolation, validation, lead-in
  and stop-ramp maths.
- `src/core/trajectory_executor.py`: settings, the two xArm backends, the
  real-time report reader, sessions and the executor thread.
- The routes and guards in `src/core/xarm_api_server.py`, and the STOP and
  disconnect hooks in `src/core/xarm_controller.py`.
- `details.trajectory` in `/status`.
- `src/settings/trajectory.yaml`, with `enabled: false`.
- `tools/servo_mode_probe.py` for stage 0b.

**Tests:** 79 new offline tests, against a fake SDK backend and a fake
clock, stable across 15 repeats. The full suites pass: 999 xArm and 160 UR,
plus 15 UR tests skipped here because `scipy` isn't installed.

**Review.** An independent safety review on 2026-10-09 found one critical
race and several high and medium issues. All are fixed, and each has a
regression test:

| Finding | Fix |
|---|---|
| A STOP during start-up was lost: the executor's own `set_state(0)` then cleared it, and the arm ran the whole trajectory | Stops are counted (a stop generation) and never cleared. A start captures the count before its gates and refuses (409 `stopped_during_start`) if it changed. The run checks it before and after `set_mode`, before `set_state(0)`, before every send, before settling and around the mode restore. A STOP racing the run's own `set_state(0)` is re-issued |
| A STOP during settle or the mode restore was undone | As above: no settle or restore after a STOP |
| A cancel during start-up was overwritten | New `starting` state; a cancel then refuses the start (409 `session_cancelled`) |
| Arm reads and gripper actuation not paused during a run | Also guarded: `/graph/nearest`, `/control/graph/recover_to`, `/control/graph/gripper`, `/control/graph/pose`, `/connect` and the log route. A force-torque tare in progress refuses a start |
| A paused arm (state 3) was not a fault | Only states 1 (moving) and 2 (ready) are healthy, in both the SDK cache and the real-time report |
| The start pose was stale; the first sample went unchecked | The arm is re-measured after entering the mode, and the lead-in is planned from that. The run fails before moving if the arm is out of tolerance. The first step is checked, and every sample also gets an acceleration bound. Start needs the arm idle (state 2) with no error or warning |
| The log route blocked the event loop | Encoded off the loop, and refused while a run streams |
| The probe lost its results; `--yes` auto-recovered real faults | It waits for the executor thread. `--yes` is removed. It stops after any unplanned outcome, and checks the return move |
| Finalisation could leave a session active forever | Finalisation now runs in try/finally: the slot is always released and a terminal state always set |
| Sleep wasn't interruptible | Sleeps run in 5 ms slices, re-checking for a STOP right before each send |
| A claim token rotation orphaned the session; with enforcement off, nothing ended a run if the client vanished | Sessions are bound to the claim's `session_id`, and trajectories require hard claim enforcement (412 `claim_enforcement_off`) |
| Upload size and validation memory | Pydantic length caps; errors capped while collected; setting types and ranges checked |
| A NaN from the real-time report | Frames with non-finite or absurd angles are dropped |

**Routes:** all under `/control/freehand/trajectory`.

| Route | Gate |
|---|---|
| `POST /validate` | login. Works while disabled and in any graph mode |
| `POST` (create) | claim with enforcement on; refused in STRICT; 412 `trajectory_disabled` unless enabled |
| `PUT /{id}/chunks/{seq}` | the creating claim; refused in STRICT |
| `POST /{id}/start` | the creating claim, then the §7.1 gates. Also: arm idle with no error or warning, and no force-torque tare running |
| `POST /{id}/cancel` | the creating claim. Never refused for graph mode or a disabled feature |
| `GET /{id}` | login |
| `GET /{id}/log` | login; refused while a run streams |

**Other differences from the plan above:**
- **Complete trajectories only.** Start is refused until the final chunk is
  in (409 `trajectory_incomplete`). An append while running gets 409
  `session_running`. So there is no underrun path yet; the §6 underrun
  trigger is not built.
- **Late ticks delay rather than burst.** A late tick re-anchors the
  schedule, so the arm never catches up in a burst. The lost time is
  reported as `servo.cumulative_delay_ms`; for one-way sync, align to
  `executed.t_exec`.
- **Any controller warning is a hard fault** (emergency stop). This may
  prove too strict on the arm.
- **Traffic guard.** 35 route paths answer 409 `trajectory_running` while a run
  is active, from start to the release of the slot. The telemetry and
  gripper polling loops pause. `/control/stop` is never gated.
- **Order at the end of a run:** the mode is restored, then the slot is
  released, then the terminal state is reported, then the record is
  written (`log_available` follows).

**Known limits of version 1:**
- **Sash watchdog.** It decides "inside the hood region" from the graph
  pins, which any freehand move clears. During a trajectory it therefore
  may not treat the arm as inside. That's the same as every other freehand
  move today; the sash gate applies at start only.
- **No panel controls.**
- **Tracking error includes report latency.** Reports arrive in about 50 ms
  bursts, so up to about 50 ms of latency is mixed in.

**Stage 0b with the probe.** Run it with the `xarm` service stopped, or
disconnected from the arm in the panel, and someone at the robot. It drives
the same executor through the real backends, with no claim and no graph or
sash interlock. Without `--execute` it only reads the pose and validates
every run. With `--execute` it always asks for a typed YES, then runs:
- ServoJ at 100 and 200 Hz;
- mode 6 at 20 and 50 Hz;
- one cancel and one STOP per mode.

Each run is J1 ±5° and J5 ±10° at 20°/s around the current pose. Recovery
after the planned STOP needs another typed YES. Any other failure ends the
probe without clearing it. It writes the full records and reports, for each
run:
- send timing;
- path deviation;
- timing drift;
- peak acceleration;
- stalls.
