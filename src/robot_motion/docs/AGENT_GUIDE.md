# Robot Motion — operator and agent guide

Version 0.4.0a1 is an observation and topology-preview prototype, not a new
hardware-control deployment. The bundled UI is at /web/. Importing the package
and listing drivers never connect to equipment.

## Implementation status

- xArm5: existing xArm application preserved through pyxarm and the explicit
  robot-motion legacy-xarm command. Its API, claims and graph interlocks remain
  unchanged. No migration of an existing service is implied.
- UR3e, UR5e, UR5-CB3: explicit model profiles and read-only Dashboard observation.
  No RTDE control interface, program uploads, motion, recovery, power, brakes,
  gripper or IO commands are instantiated or exposed by the prototype.
- MG400: reserved optional extra and model metadata only; no hardware driver yet.

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

## Control workspace UI

The control page reuses the packaged pyxarm stylesheet byte-for-byte and its
two-column layout: connection tile and Direct Drive / Graph Control on the left,
camera placeholder, Gripper / Tool I/O, and status log on the right. The camera
and tool panels explicitly show unavailable state; no camera source is loaded.
Refresh Status only reads the service's cached /status response. It does not
connect or send commands to the robot. The page does not load the legacy xArm
command handlers, camera player, controller settings or API server.

Motion Graph opens the offline editor in a separate view on the same page.
Control Interface returns to the control view without discarding the draft.
The editor reuses the packaged Cytoscape library byte-for-byte.

Graph Workspace provides local node editing, directed joint/linear edges,
selection on the canvas or keyboard-accessible selectors, topology validation,
route highlighting, JSON import/export and a bounded 30-step graph undo history.
Deleting a node also removes its incident edges; undo restores the prior graph.
Coordinates must be entered explicitly; blank fields do not imply zero.
Changing draft model starts an empty graph rather than converting coordinates.
Linear arrivals require an explicit target TCP pose. Drafts and canvas layout
are in tab memory, not server state. Export before closing the tab. Unsaved
graph changes produce a browser leave-page warning where supported.

Blue graph highlighting denotes a proposed topology route, NOT the measured
robot position, physical trajectory, simulation or execution. Invalid or edited
JSON disables preview/export and hides the old diagram until validation succeeds.
Responses from older validation/preview requests cannot replace newer edits.
Import and offline requests are bounded to 256 KiB.

The Direct Drive pane is a non-operational layout preview. Joint/TCP telemetry
is explicitly "Not observed" because the Dashboard observer does not collect
it. Take Control, STOP, recovery, connection, freedrive, pose capture and motion
buttons are disabled and have no command handlers. In particular, the displayed
STOP button cannot stop the robot: use the established operator controls.
Neither an installed SDK nor a status response can enable these buttons.

Shared assets are exposed through a two-file allowlist under /web/pyxarm/;
arbitrary files, Python source, legacy control pages and command scripts are
not exposed there. The browser requests only status, driver inventory, configured
graph and the two offline graph-calculation routes. There are no camera streams
or robot WebSocket connections in this workspace.

## API discovery

- /: STATUS_SPEC probe
- /health: process liveness, not hardware readiness
- /status: cached robot observation
- /drivers: model and SDK inventory
- /graph: configured local topology, if any
- POST /graph/validate and /graph/preview: offline topology calculations
- /docs, /openapi.json: generated API documentation
- /agent-docs/api-reference: generated readable route/schema reference
- /llms.txt: documentation index

The prototype has no authentication because it exposes no hardware mutations.
Serve it only on the loopback or explicitly firewalled Tailnet interface.
Do not expose it publicly. A future control service needs reviewed authentication,
hard claims, commissioning gates, safe stop handling and truthful completion checks.
