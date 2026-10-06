# Robot Motion — operator and agent guide

Version 0.4.0a1 is an observation and topology-preview prototype, not a new
hardware-control deployment. The repository's shared web UI is served at /web/.
Importing the package and listing drivers never connect to equipment.

## Implementation status

- xArm5: existing xArm application preserved through pyxarm and the explicit
  robot-motion legacy-xarm command. Its API, claims and graph interlocks remain
  unchanged. No migration of an existing service is implied.
- UR3e, UR5e, UR5-CB3: read-only Dashboard status plus opt-in RTDE joint/TCP
  observation using the adapted automated-lle URArm wrapper (see LLE_RTDE.md).
  No RTDE control interface, program uploads, motion, recovery, power, brakes,
  gripper or IO commands are instantiated or exposed by the prototype.
- MG400: reserved optional extra and model metadata only; no hardware driver yet.

An isolated small joint-step executor is available for offline development;
see JOINT_STEP_CONTROL.md. It is not wired into this service or UI and does
not change allowed_actions or the receive-only deployment configuration.

## Contract and safety

The prototype conforms to STATUS_SPEC v1.2's read-only profile: the /control
surface, claims, and mutation-refusal semantics are N/A. allowed_actions is empty.
Primary operation for UR observation means the controller program is PLAYING;
it does not mean the physical arm is moving. Failed/stale observations are unknown,
not device faults. Reported safety faults take precedence over readiness.

Transport documentation does not authorize hardware execution. Follow the lab's
binding AGENTIC_LAB_DESIGN.md Part I and STATUS_SPEC.md. Future hardware execution
must use lab-skills, claims, interlocks, and validated human-approved plans.

## Configuration

Use a gitignored *.local.json config, passed with --config. Robot addresses,
deployment paths and calibration stay local. Observation requires observe=true,
driver=ur, an explicit model, and robot_host. control_enabled accepts only false.
Do not point a second controller at a robot owned by another workflow.
ur_transport defaults to dashboard; rtde adds receive-only telemetry and
requires the existing [ur] extra. It does not enable physical control.

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
same cached envelope at the poll cadence and acts on no browser message. The
page's other load-time reads (/graph/layout, /locations, /track/locations,
/interlocks/sash, /auth/config, /auth/me, /camera/config, /assistant/status,
/api/configurations) answer that the feature is absent, so the page renders
without inventing state. Nothing from the browser is persisted.

Take Control, Connect, STOP, recovery, gripper, rail, graph-edit and motion
buttons send their requests to routes this service does not have and receive
404. In particular, the displayed STOP button cannot stop the robot: use the
established operator controls. allowed_actions stays empty regardless of what
the page shows, and no status response can change that.

Known limitation: the shared page uses an inline theme script and ?v= asset
queries, which the dashboard proxy's script-src 'self' CSP and fixed,
query-free asset paths refuse. Until the proxy allowlist or the shared UI
changes, open the panel directly on this service's port. That is UI and proxy
work, not a change to this service.

## API discovery

- /: STATUS_SPEC probe
- /health: process liveness, not hardware readiness
- /status: cached robot observation
- /drivers: model and SDK inventory
- /graph: configured local topology, if any
- POST /graph/validate and /graph/preview: offline topology calculations
- /web/: the shared web UI, read-only here; /ws: cached status envelope push
- shared-ui tagged routes: absent-feature answers for the page's load-time reads
- /docs, /openapi.json: generated API documentation
- /agent-docs/api-reference: generated readable route/schema reference
- /llms.txt: documentation index

The prototype has no authentication because it exposes no hardware mutations.
Serve it only on the loopback or explicitly firewalled Tailnet interface.
Do not expose it publicly. A future control service needs reviewed authentication,
hard claims, commissioning gates, safe stop handling and truthful completion checks.
