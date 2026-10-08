# Robot Motion — operator and agent guide

Version 0.4.0a1 is an observation and topology-preview prototype, not a new
hardware-control deployment. The repository's shared web UI is served at /web/.
Importing the package and listing drivers never connect to equipment.

## Implementation status

- xArm5: existing xArm application preserved through pyxarm and the explicit
  robot-motion legacy-xarm command. Its API, claims and graph interlocks remain
  unchanged. No migration of an existing service is implied.
- UR3e, UR5e, UR5-CB3: read-only Dashboard status plus opt-in RTDE joint, TCP
  and TCP-force observation using the adapted automated-lle URArm wrapper (see
  LLE_RTDE.md). With RTDE the telemetry refreshes every telemetry_interval_s
  (0.5 s) while the Dashboard poll stays slow. On e-Series the Dashboard also
  reports remote_control and operational_mode.
  By default no RTDE control interface, program upload, motion, recovery,
  power, brake, gripper or IO command is instantiated or exposed.
- UR control: a local config may enable config-gated, identity-checked,
  hard-claimed arm motion in one of two modes. `motion`: joint moves and jogs
  and Cartesian moves and jogs, inside a commissioned joint envelope, TCP
  workspace box and speed caps, with an optional force guard (ARM_MOTION.md).
  `joint_step`: single-joint steps of 0.1 deg by default, 0.5 deg ceiling
  (JOINT_STEP_CONTROL.md). Neither is commissioned on the UR5e yet.
- UR gripper: an optional `gripper` block reads a Robotiq gripper's status
  through its URCap (read-only); under control_enabled the same identity and
  claim gates allow open, close, stroke, force and activation. See
  GRIPPER_CONTROL.md. Commissioned on the UR5e (2F-140) on 2026-10-07.
- MG400: reserved optional extra and model metadata only; no hardware driver yet.

## Contract and safety

Without control_enabled the service conforms to STATUS_SPEC v1.2's read-only
profile: the /control surface, claims, and mutation-refusal semantics are N/A
and allowed_actions is empty. With control enabled, /control/claim,
/control/heartbeat and /control/release hold a hard-enforced single claim;
/connect, /disconnect, the arm moves and the gripper commands refuse 423
without its token, 412 when a precondition fails, 422 when a request exceeds
the commissioned limits, and /control/stop needs identity only.
allowed_actions then lists exactly what a POST will honor.
Primary operation for UR observation means the controller program is PLAYING;
it does not mean the physical arm is moving. Failed/stale observations are unknown,
not device faults. Reported safety faults take precedence over readiness.

Transport documentation does not authorize hardware execution. Follow the lab's
binding AGENTIC_LAB_DESIGN.md Part I and STATUS_SPEC.md. Future hardware execution
must use lab-skills, claims, interlocks, and validated human-approved plans.

## Configuration

Use a gitignored *.local.json config, passed with --config. Robot addresses,
deployment paths and calibration stay local. Observation requires observe=true,
driver=ur, an explicit model, and robot_host. control_enabled defaults to false
and is rejected unless a complete control block (authorized_operators plus
joint_step or motion limits, or a gripper) accompanies it on an RTDE-observed robot;
the edge secret comes from ROBOT_MOTION_EDGE_SHARED_SECRET in the service
environment, never from the config file. Optional control settings:
watchdog_hz (controller-side watchdog rate, default 10) and audit_file
(JSON-lines control event trail, relative to the config file). Enabling control is a local config
change plus a restart in an authorized window; no request can switch it on.
Do not point a second controller at a robot owned by another workflow.
ur_transport defaults to dashboard; rtde adds receive-only telemetry and
requires the existing [ur] extra. It does not by itself enable physical control.

The process polls only fixed read commands in the background. /status reads its
cache and never reconnects, enables hardware, resets a fault, or runs a program.
Automatic polling resumes after a read failure; it retries observations only.
A read failure is logged and returned as unknown. Stop the dedicated service to
stop polling. No activity history or scientific run data is written by this prototype.

## Motion graph

/graph/validate and /graph/preview are offline calculations only. Preview reuses
the existing xArm breadth-first path planner through an arm-only topology view.
Legacy xArm gripper/rail schema and physical interlocks are not changed.

Validation checks graph structure, finite numbers and model-specific joint count.
It does NOT validate reachability, collisions, physical limits, TCP calibration,
payload, gripper state, rail clearance, or physical execution authority.
Never transfer coordinates from one physical robot to another.
Graphs submitted in the browser are not persisted or sent to hardware.
UR hardware control still requires implementation and commissioning.

## Shared web UI

/web/ serves the repository's shared xArm web UI (src/web) byte-for-byte from
a fixed file allowlist: index.html, graph.html and their bundled scripts and
stylesheets. server.py, Python source and every other path answer 404. This
service has no UI of its own; UI work happens in src/web for every robot.

The page is written against the xArm API and renders here as a read-only
panel. /status fills current_joints and current_position only from a valid
RTDE receive sample (otherwise null, never zero), num_joints from the model
profile, and connection_details when observation is enabled. /ws pushes the
same cached envelope at the telemetry cadence (RTDE) or the poll cadence and
acts on no browser message. The
page's other load-time reads (/graph/layout, /locations, /track/locations,
/interlocks/sash, /auth/config, /auth/me, /assistant/status,
/api/configurations) answer that the feature is absent, so the page renders
without inventing state. /camera/config and /realsense/cameras are real (see
Cameras). Nothing from the browser is persisted.

Without control enabled, Take Control, Connect, STOP, recovery, gripper, rail,
graph-edit and motion buttons send their requests to routes this service does
not have and receive 404; allowed_actions stays empty regardless of what the
page shows. With control enabled, Take Control, Connect and STOP reach the
claim, /connect and /control/stop routes (identity permitting). In motion
mode, Move Joints, the XYZ jog buttons and Clear errors reach the arm routes,
and the panel gates Connect, Disconnect and those buttons on allowed_actions.
The graph and named-location buttons still have no routes here. In every case the
displayed STOP button is a software request, not a safety-rated stop: use the
established operator controls.

The canonical operator URL is the lab edge's /ur5e/web/ (dashboard repo,
deploy/Caddyfile.single-edge): forward_auth signs the human in, the edge
strips only the /ur5e prefix and injects X-Auth-User plus X-Edge-Auth. The
page derives that prefix from its URL for its API and /ws calls. /auth/config
and /auth/me report the edge identity when ROBOT_MOTION_EDGE_SHARED_SECRET
matches; otherwise they report no identity, and the panel still renders
read-only. The earlier dashboard-side CSP proxy and its /utils/robot_motion
page are retired; the direct port stays reachable only on the Tailnet.

## Cameras

The panel's camera tile has two optional views, each configured by a local
config block and neither loading a camera SDK in this process. Looking is not
arm actuation: no camera route is claim-gated, touches the robot, or changes
equipment_status.

- Tapo Camera (lab_camera): the bench's network PTZ camera as registered on
  the lab dashboard (dashboard_base_url, camera_id, optional lens). The xArm's
  CameraTracker reads its live state from the dashboard's open /api/equipment
  snapshot; GET /camera/config reports configured, available, reason, lenses
  (with ptz_capable), presets and the go2rtc stream source. The video itself
  uses the dashboard's authenticated viewing sessions on the shared page
  origin, so it plays only through the lab edge. POST /camera/ptz (verbatim
  {direction, speed, duration_ms} or stop {pan, tilt, zoom}) and POST
  /camera/preset ({preset_id}) forward to the dashboard's audited
  /api/equipment/<camera_id>/control/* passthrough carrying the caller's own
  credential: their X-Api-Key if present, else only their ac_auth_session
  cookie. This service stores no camera credential; the dashboard authorizes
  and audits the real person. With no motion graph runner there is no "Follow
  arm": /camera/config says follow_supported false and POST /camera/follow
  answers 409.
- Stereo Camera (camera_service): RealSense cameras owned by the standalone
  SDL camera service on this PC, reached through the xArm's remote facade with
  the xArm's /realsense/<id>/* routes and payloads. service_file names the
  private {"url", "token", "cameras"} JSON (the xArm's
  XARM_CAMERA_SERVICE_CONFIG format), relative to the config file; cameras
  lists the panel metadata (id, label, short_label, mount) for exactly those
  ids. A rejected camera configuration is logged and reported as the
  /realsense/cameras reason; it never stops robot observation. /status gains
  components.realsense_<id> and details.realsense from cached telemetry.

Discovery, /camera/config, /realsense/<id>/status, /depth and /intrinsics are
open reads. Pixels (snapshot.jpg, depth.png, stream.mjpg), start/stop and PTZ
need the edge identity (ROBOT_MOTION_EDGE_SHARED_SECRET): 401 without it, 503
when the service has no secret.

## API discovery

- /: STATUS_SPEC probe
- /health: process liveness, not hardware readiness
- /status: cached robot observation
- /drivers: model and SDK inventory
- /graph: configured local topology, if any
- POST /graph/validate and /graph/preview: offline topology calculations
- /web/: the shared web UI, read-only here; /ws: cached status envelope push
- shared-ui tagged routes: absent-feature answers for the page's load-time reads
- cameras tagged routes: /cameras and /realsense/cameras discovery,
  /camera/config, /camera/ptz, /camera/preset, /camera/follow (409), and
  /realsense/<id>/{status,start,stop,snapshot.jpg,depth.png,stream.mjpg,depth,intrinsics}
- control tagged routes (only with control_enabled): /control/claim, heartbeat,
  release; /connect, /disconnect; /control/stop (/move/stop); joint_step mode:
  /control/joint_step
- arm tagged routes (motion mode): /control/freehand/joints, joint_jog,
  relative, linear; /control/reset (/clear/errors); /control/force/zero
- /docs, /openapi.json: generated API documentation
- /agent-docs/api-reference: generated readable route/schema reference
- /llms.txt: documentation index

The read-only routes have no authentication because they mutate nothing. The
control routes trust only the dashboard edge's shared-secret identity headers
and refuse entirely when that secret is unset. Serve the process only on the
loopback or explicitly firewalled Tailnet interface; never expose it publicly.
Commissioning arm motion on the physical robot remains outstanding; see the
checklist in ARM_MOTION.md.
