# Small joint-step executor — offline-tested, not commissioned

> Superseded for operator use by the `motion` mode (ARM_MOTION.md). A config
> names `joint_step` or `motion`, never both. Connect precondition refusals
> are now 412, and `allowed_actions` offers `connect` only when it would
> succeed.

`drivers/joint_step.py` implements the control primitive for one finite,
single-joint `moveJ`, followed by measured completion checks. The executor
itself has no robot address, SDK constructor, script upload, linear move,
gripper, power, brake-release, or automatic homing operation. It is reached
only through the config-gated routes described under "Service integration";
a deployment without `control_enabled` has no such routes and stays
receive-only.

## Service integration (config-gated, not commissioned)

`control.py` installs the routes only when the local config sets
`control_enabled: true` together with a complete `control` block
(`authorized_operators`, every `joint_step` limit including the six joint
bounds, the stop deceleration and `commissioning_id`) on a service that
already observes the same UR robot over RTDE. A lone flag is rejected.

- Identity: requests must carry the dashboard edge's `X-Auth-User` and an
  `X-Edge-Auth` that matches `ROBOT_MOTION_EDGE_SHARED_SECRET`. Without that
  secret every control route answers 503; unverified callers get 401 and
  verified callers outside `authorized_operators` get 403. The verified
  e-mail, never the client's `owner` field, becomes the claim holder.
- Claims: `POST /control/claim`, `/control/heartbeat`, `/control/release`
  reuse `core.claims.ClaimManager` under hard enforcement.
  `/control/joint_step` requires the held `X-Claim-Token` (423 otherwise,
  including when nobody holds a claim). `/connect` and `/disconnect` follow
  the xArm order: any listed operator may call them while nobody holds a
  claim, but once someone does, only with that holder's token (423).
- `POST /connect` first requires a fresh Dashboard observation showing the
  robot idle: controller RUNNING, safety NORMAL, program STOPPED. PLAYING or
  PAUSED means another program or client (LLE demo, pendant, other ur_rtde
  user) owns the robot and the request is refused 409 `robot_not_idle`; a
  stale or failed observation is refused 409 `observation_unavailable`.
  It is then the one place that constructs `RTDEControlInterface`, which
  uploads ur_rtde's control script and takes the controller's program slot,
  plus a dedicated receive stream (`actual_q`, `actual_qd`, `robot_mode`,
  `safety_mode`, default 125 Hz) stamped on packet arrival and served one new
  packet per read. `POST /disconnect` closes both.
- The session arms ur_rtde's communication watchdog (`watchdog_hz`, default
  10 Hz) and kicks it at twice that rate from a dedicated thread. If this
  service freezes or dies, the controller itself halts the control script.
  A refused kick marks the interface faulted: steps are refused, status
  shows `watchdog_ok: false` with the error, and kicking stops so the
  controller completes the shutdown. A controller that refuses the watchdog
  never gets a session (502).
- Every control event (claim, connect, connect_refused, joint_step,
  joint_step_refused, joint_step_failed, stop, disconnect) is appended as a
  JSON line to `control.audit_file` when configured (path relative to the
  config file), besides the process log and `details.control_session.last_event`.
- `POST /control/joint_step` runs the executor once; refusals are 412 with
  the executor's reason, a failed dispatch is 500 with `stop_attempted`,
  `stop_confirmed: false`, `stop_error` and `latched: true`. A latch clears
  only through an explicit, identified `/disconnect` then `/connect`.
- `POST /control/stop` (alias `/move/stop`) needs identity but no claim: it
  wakes the executor's cancel latch and issues `stopJ` through the session's
  SDK lock. It is a software request, never confirmed by measurement and not
  a safety-rated stop.
- `/status` reports `details.claimed_by`, `details.control_session` (open,
  latched reason, commissioning id, last event) and `allowed_actions`
  (`control.stop` always; `control.joint_step` while open and unlatched;
  `connect` while closed).

## Implemented

- Requests identify J1–J6 and a signed change in degrees, with a unique UUID.
  The default step cap is 0.1 degrees; the hard software ceiling is 0.5 degrees.
  Steps smaller than 0.02 degrees are rejected so the default measurement
  tolerance cannot report an unchanged start position as completion.
- Speed defaults to 0.5 deg/s (ceiling 1 deg/s), acceleration to 1 deg/s²
  (ceiling 2 deg/s²). These are engineering software caps, **not validated safe
  operating values for this robot**.
- All six absolute joint bounds, a commissioning reference, and stop
  deceleration are mandatory inputs. No LLE poses or UR5e bounds are guessed.
- Hard claim enforcement and a mandatory SDK/session/plan authorization callback
  are checked before dispatch and throughout execution. Authorization is bound
  to the request, exact target, and commissioning reference. A callback returning
  anything other than True refuses. The initial gate passes target=None to
  verify session authority before reading feedback.
- Two fresh, advancing, stationary samples are required before planning.
  The target changes exactly one joint; no angle wrapping, clamping, or linear
  path is substituted. Feedback must be at most 0.2 seconds old and contain
  all six joint positions and velocities, controller state and safety state.
- Degree inputs are converted to radians/rad/s/rad/s² for RTDE moveJ, using the
  same conventions as the LLE driver. Dispatch is asynchronous to permit stopJ.
- One command at a time; concurrent requests are refused, never queued.
  UUIDs are never replayed. A session has a cumulative absolute travel budget
  (default 0.5 degrees, ceiling 1 degree), including ambiguous attempts.
- Completion requires all joints at target, all velocities near zero, and
  multiple advancing samples settled for at least 0.1 seconds. An SDK
  acknowledgement alone is not completion. The default move deadline is 5 s.
- Claim/auth loss, unsafe state, stale feedback, unexpected joint travel,
  cancellation or timeout after dispatch requests stopJ and latches failure.
  A lost dispatch reply is ambiguous, not a reason to retry. Stop failure is
  preserved separately; stopJ returning is not claimed as verified stopping.
  Pre-dispatch refusals never issue a stop against unrelated robot activity.

The SDK is not thread-safe: only the execution thread calls moveJ/stopJ;
request_stop sets a latched event consumed by that thread. See the
[SDK control interface source](https://gitlab.com/sdurobotics/ur_rtde/-/blob/master/include/ur_rtde/rtde_control_interface.h)
and [asynchronous move documentation](https://sdurobotics.gitlab.io/ur_rtde/pages/examples/basic_motion/move_async_example.html).

## Required before live commissioning

The routes above are a control building block, **not a complete safety system
or permission to execute hardware**. Still outstanding before a first
supervised step on the UR5e:

1. A human-approved, main-merged commissioned plan through lab-skills. The
   service checks identity, the operator allowlist and the configured
   `commissioning_id`; it does not verify that a plan exists or was approved.
   Do not replace the authorization callback with a bypass.
2. Exclusive controller-program ownership in practice: `/connect` refuses
   while the Dashboard shows a program PLAYING/PAUSED, but a client that has
   connected without running a program is invisible to it. Confirm on the
   pendant that nothing else (LLE demo, another ur_rtde client) is attached
   before connecting. Never treat `/connect` as passive.
3. Feedback measured on the live UR5e (2026-10-07, receive-only, sdl2-pc-05):
   packets advance every ~16 ms (Windows timer granularity), sample age at
   read <= 16 ms (bound 200 ms), modes decode as RUNNING/NORMAL, and joint
   noise at rest reaches 0.0068 deg peak-to-peak. Commission with
   `position_tolerance_deg: 0.01` (the cap); the 0.005 default refuses on
   noise alone. Re-run `.state/measure_control_feedback.py` on the day.
   Validation of the feedback stamp: the reader stamps a packet when ur_rtde's
   cached timestamp advances, polled at twice the stream rate. Measure the
   real stamp error on the deployment PC (Windows timer granularity is ~15 ms)
   before trusting `feedback_max_age_s` as a bound.
4. Verify tool/TCP, payload, joint limits, workspace clearance, speed and stop
   deceleration on the actual UR5e. Joint-space bounds do not establish Cartesian
   clearance or collision safety, even for a small rotation.
5. Test the controller-side watchdog and the operator stop procedure on the
   real controller: confirm on the pendant that the control script halts
   when the service is killed mid-step, and measure how long that takes at
   the configured `watchdog_hz`. Python callbacks, SDK calls, OS scheduling,
   or a crashed process can still block the polling loop: its software
   timeout and cancellation are not a safety-rated stop or a hard real-time
   deadline. Stop may not be deliverable after a disconnect. Use established
   hardware/operator stops.
6. Record handling and reconciliation: the JSON-lines audit file is a local
   trail, not the lab's record system. Human reconciliation after a latch is
   an identified `/disconnect` + `/connect`; nothing verifies the arm's physical
   state for the operator. The dashboard proxy allowlist still exposes none of
   these routes; control traffic must come through the authenticated edge.
   No endpoint can enable control at runtime: it is a local config change plus
   a service restart during an authorized window.

Offline tests exercise success, unit conversion, bounds, claims, stale data,
unexpected motion, request replay, concurrency, ambiguous dispatch, stop failure,
and latch behavior. They do not commission the real robot. No physical motion
was performed as part of this implementation.
