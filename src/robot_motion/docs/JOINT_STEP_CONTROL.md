# Small joint-step executor — offline-tested, not commissioned

`drivers/joint_step.py` implements the control primitive for one finite,
single-joint `moveJ`, followed by measured completion checks. It is **not loaded
by the live application**. It has no robot address, SDK constructor, script
upload, server endpoint, UI handler, linear move, gripper, power, brake-release,
or automatic homing operation. The deployed service remains receive-only.

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

This library is a control building block, **not a complete safety system or
permission to execute hardware**. A reviewed application integration must:

1. Provide a human-approved, main-merged commissioned plan through lab-skills,
   hard claims, and authenticated equipment authorization. A profile's text ID
   does not establish any of these. Do not replace the callback with a bypass.
2. Establish exclusive controller-program ownership. RTDEControlInterface can
   upload a script on construction; the executor deliberately does not create
   it. Never treat this as the passive RTDE receive connection.
3. Supply a dedicated bounded, fresh RTDE reader including actual_qd and safety
   state. The UI's polling snapshots are too old for control. Host timestamps
   must reflect packet receipt, not merely the time cached getters are called.
4. Verify tool/TCP, payload, joint limits, workspace clearance, speed and stop
   deceleration on the actual UR5e. Joint-space bounds do not establish Cartesian
   clearance or collision safety, even for a small rotation.
5. Provide and test an independent controller-side watchdog and operator stop
   procedure. Python callbacks, SDK calls, OS scheduling, or a crashed process
   can block the polling loop: its software timeout and cancellation are not a
   safety-rated stop or a hard real-time deadline. Stop may not be deliverable
   after a disconnect. Use established hardware/operator stops.
6. Integrate structured 412/423 refusals, truthful activity/last_error and
   allowed_actions, authenticated SDK routes, audit/record handling and human
   reconciliation. Keep the existing read-only proxy allowlist unchanged until
   that complete integration is reviewed. No endpoint can enable this executor
   in the current deployment.

Offline tests exercise success, unit conversion, bounds, claims, stale data,
unexpected motion, request replay, concurrency, ambiguous dispatch, stop failure,
and latch behavior. They do not commission the real robot. No physical motion
was performed as part of this implementation.
