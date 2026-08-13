# Changelog

All notable changes to this project are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and the project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added — gripper states in `allowed_actions` (`gripper.<state>`)

- **`/status.allowed_actions` now advertises the gripper.** Each catalog state
  reachable from the current node and current stroke is listed as
  `gripper.<state>` (e.g. `gripper.grip_120`), alongside the existing
  `move.<node_id>` targets. `POST /control/graph/gripper` has always honored
  these, so the previous list *understated* capability — the direction §6.2
  forbids, and it left the gripper unreachable for any client that treats
  `allowed_actions` as the authority (the dashboard's lab assistant does, so it
  could see `details.motion_graph.allowed_gripper_targets` but had no action to
  invoke).
- **Both surfaces read the same `allowed_gripper_targets()`**, so the advertised
  set is the endpoint's whitelist exactly — nothing added or dropped. Enumerating
  one action per legal state (rather than a single `gripper.set`) is what makes
  that mirror possible: the whitelist is per (node, current state), which one
  action name could not express.
- **Withheld under every gate the endpoint enforces**: while a motion is in
  flight (the stroke is invariant during arm motion, so the endpoint requires a
  parked arm), outside `graph_mode == STRICT` (ADVISORY/OFF don't constrain
  transitions, so a list would understate what is honored), on a Studio-Sim box
  (`box_sim_guard` 412s it), and when no gripper is attached.

### Changed — STATUS_SPEC v1.2 conformance (`activity`)

- **Wire-contract types come from `sdl-lab-contract` v1.2.0** instead of a
  vendored `models.py`; `protocol_version` is `"1.2"` on `/` and `/status`.
- **`activity` is observed, not derived.** It is read from the controller's
  motion flag — the one every motion primitive brackets its SDK call with —
  rather than computed from `equipment_status`, which spec §2.3 forbids
  because it would add no information. A `degraded` controller mid-move now
  reports `degraded` + `activity: "running"`; previously the activity axis
  could only echo the state word.
- **`activity_since` is the transition instant.** `XArmController._motion_in_progress`
  is now a property that latches the timestamp when the flag flips, so
  `/status` reports when the current activity *began*. It was previously
  `datetime.now()` on every call, which made an in-progress move's elapsed
  duration unrecoverable and changed the field on every poll.
- **`equipment_status: busy` is derived from `activity`, not alongside it.**
  §2.3 defines `busy` as healthy + running, so the invariants (`busy` ⇒
  `running`, `ready` ⇒ `idle`) now hold by construction. Two consequences:
  a move in flight while the controller is not fully alive reports
  `degraded` + `running` rather than `busy` (§2.3 forbids `busy` +
  `degraded`), and `busy` additionally covers motion the previous
  flag-only check missed.
- **`requires_init` ⇒ `idle`** on both the no-controller and
  connection-down envelopes, per the §2.3 invariant table. They previously
  reported `activity: "unknown"`, and a connection dropping mid-move could
  leave the flag latched — a move that can no longer be executing must not
  read as a run.
- **README** now documents what "primary operation" means for this device
  (v1.2 checklist requirement) and drops the stale "no `/control/*` claim
  surface yet" note.

### Added — simulation self-identification (`dry_run` + panel banner)

- **A docker-profile session now identifies itself on every surface.**
  `XArmController.is_simulated` (profile name contains `docker`) is the
  single predicate behind all simulation accommodations, replacing the two
  inline checks.
- **Envelope:** healthy states report `equipment_status: "dry_run"` — the
  spec's first-class simulation state, which the dashboard's reader-side v2
  projection already maps to `simulated: true` — with `activity` unchanged
  (§2.3 allows any activity for `dry_run`). Fault states keep their honest
  value so simulated failure paths stay testable. All states get a
  `[SIMULATION]` message prefix and `details.simulated: true`.
- **Panel:** a sticky amber banner ("SIMULATION — connected to the Docker
  simulator, not the real arm") whenever the envelope carries
  `details.simulated`. Controls stay live (`dry_run` counts as alive).
- **Events exporter is suppressed while simulated** — sim telemetry never
  reaches the lab history DB stamped as the real device, even with
  `XARM_INGEST_URL` configured.
- Consequence, intended: workflows gating on `equipment_status == "ready"`
  will not run against a sim-connected service by accident; running against
  the sim is an explicit opt-in.

### Added — one motion at a time (HTTP 409 `motion_in_progress`)

- **Concurrent motions are refused.** Every motion endpoint now reserves a
  single motion slot and returns **409** with
  `{"error": "motion_in_progress", ...}` when one is already in flight.
  Previously nothing stopped two overlapping commands from reaching the
  same arm. Covers `/move/{position,joints,relative,location,home,plate_linear}`,
  `/track/move{,/location}`, `/control/graph/{move_to,travel_to}`,
  `/assistant/execute`, and both `/force-torque/move-*` endpoints. `/move/stop`,
  `/clear/errors`, `/control/graph/recover_to`, and the gripper endpoints are
  deliberately not gated.
- **The slot is reserved at accept time, not inferred.** Several motion
  endpoints accept-and-return before the arm starts moving (the SDK call is
  a background task), so a caller firing two moves back to back would have
  found the controller still idle on the second. Reservation happens
  synchronously in the request handler, where check-and-set is atomic on the
  event loop thread.
- **`XArmController` motion state is a nesting counter** (`enter_motion()` /
  `exit_motion()`, `_motion_in_progress` now read-only). A composite move —
  a cross-rail edge's two sub-moves, a travel's N hops, an assistant step
  list — reads as one continuous motion instead of flickering to idle
  between parts, and the reservation is not released early by an inner
  primitive finishing. `exit_motion()` clamps at zero so an unbalanced
  release cannot wedge the arm into refusing every subsequent move.
- **`allowed_actions` mirrors the refusal** (§6.2): while `activity` is
  `running`, `move.<node_id>` targets are withheld and `stop` remains. A
  property test asserts the two surfaces cannot drift.
- **`allowed_actions` includes `connect`** on the no-controller envelope,
  matching the connection-down branch that reports the same
  `equipment_status`. `/connect` is honored in both.
- `/move/home` no longer double-wraps its own 500 into
  `"Home movement failed: 500: Home movement failed"`.

### Added — multi-hop travel (`/control/graph/travel_to`)

- **`MotionGraph.plan_path(from, to, gripper_state)`** — shortest hop path
  via BFS over the state-aware successor set (`allowed_targets_for_state`).
  The gripper state is held for the whole journey (it can only change while
  parked), so a fixed-state search is exact. Raises the new `NoPathError`
  (carries `from_node` / `to_node` / `gripper_state`) when no whitelisted
  corridor connects the endpoints. Companion `reachable_set(from, state)`
  returns every multi-hop-reachable node.
- **`XArmController.travel_to_node(node_id, speed)`** — plans once, then
  executes hop-by-hop through `move_to_node`, so every hop keeps per-edge
  STRICT validation, edge speed caps, cross-rail dispatch, and transition
  recording. Fail-fast: a mid-journey failure stops the loop with the arm
  parked at the last completed node, reported as
  `{success, path, completed, failed_hop}`.
- **`POST /control/graph/travel_to`** (claim-gated) — multi-hop counterpart
  of `move_to`. Blocks until the journey completes (repo convention; live
  progress rides the `/ws` stream). Errors: 409 `unknown node` / `no_path` /
  `edge_not_allowed`, 500 `travel_failed` with `failed_hop` + `completed` +
  the node the arm is parked at.
- **`travel_targets`** published in `/status.details.motion_graph` and
  `GET /graph` — the multi-hop reachable set for the current node + gripper
  state (superset of the one-hop `reachable_nodes`).
- **Web UI:** new "Travel" row in the Drive Arm card — destination picker
  fed by `travel_targets`, same claim/manual/moving gating as the one-hop
  Drive row; logs the executed hop path. The Motion graph viewer link moved
  from the page header into the Motion Graph card ("Open graph viewer ↗").

### Changed — gripper-leaf motion graph (schema 0.2)

**Breaking:** `motion_graph.yaml` schema bumped from 0.1 to 0.2. Graphs written
for 0.1 no longer load; node ids with gripper suffixes no longer exist.

- A motion-graph **node now represents an arm position only** (arm pose + rail
  location). The stacked `<pose>_empty` / `<pose>_grip_120` node pairs were
  collapsed into single nodes with bare pose ids (76 nodes → 46), and the
  loader rejects two nodes sharing the same (arm, rail).
- The gripper is modelled as **leaves on each node**, driven by a new global
  `gripper_states` catalog with a commanded stroke and verification intent:
  - `empty` — 150 mm, fully open, no verification.
  - `grip_120` — 120 mm, grasp: jaws must settle *above* 120 mm; reaching
    120 mm exactly means the tray was missed → error.
  - `reach_90` — 90 mm, position: jaws must *reach* 90 mm (collision-clearance
    narrowing); stalling early means blocked → error.
  - `grip_80` — 80 mm, grasp (tray held on the short side). No nodes reference
    `reach_90`/`grip_80` yet; the catalog and machinery are ready.
- Each node lists `gripper_states` (states you may occupy there, default
  `[empty]`) and `gripper_transitions` (state changes allowed *while parked*
  there). The recorded same-pose grip/release edges were migrated into
  transitions on `opentrons_2_low`, `deck_slot1_low`, and `deck_solid_low`.
- **The gripper never changes while the arm moves.** Edges carry no gripper
  actions; an edge is traversable with state G only if G is allowed at both
  endpoints, and the loader rejects edges whose endpoints share no state.
  Duplicated empty/held edge variants of the same motion were merged, keeping
  the held variant's mode and the lower speed.
- Controller: `move_to_node` is now a pure arm move. The new
  `set_gripper_state(state)` is the only way to grip/release/narrow — it
  refuses while the arm is in motion (moving interlock), requires a pinned
  node and a whitelisted transition (STRICT rejects, ADVISORY warns), then
  actuates and runs the intent-aware verification. STRICT edge gating and
  `reachable_node_ids()` now also enforce gripper-state occupancy at the
  target node.
- `recover_to` accepts an optional declared `gripper_state` (validated against
  the node's leaves); nearest-node detection matches on arm+rail only and
  reports the resolved gripper state instead of filtering by stroke.

### Added

- `POST /control/graph/gripper` `{state}` (claim-gated): change the gripper
  leaf at the current node. Returns 409 when the transition is not
  whitelisted (or the arm is moving / off-grid), 500 when actuation or
  verification fails.
- `GET /graph` now serializes the gripper-state catalog, per-node
  `gripper_states`/`gripper_transitions`, and live `gripper_state` +
  `allowed_gripper_targets`. The same fields ride in
  `/status.details.motion_graph`.
- Web UI, Drive Arm card: a Gripper row with the live state, a dropdown of
  transitions allowed at the current node, and a **Set Gripper** button
  (gated by claim/manual/moving like the rest of the card).
- Web UI, graph viewer: one node per pose with compact leaf badges
  (`▢` empty, `▣` grip, `◇` reach) and a `⇄` marker on nodes where
  grip/release is allowed; live holding highlight from `gripper_state`; the
  add-node form takes gripper-state checkboxes and an optional transitions
  field. Saved layout positions keyed by the old `<id>_empty` ids are reused
  for the collapsed ids.
- Web UI, control panel: Drive Arm card for graph-gated moves — a dropdown of
  reachable destinations (STRICT mode auto-ensured on send) and a live
  "Current" node readout.

### Fixed

- `POST /control/graph/node` and `POST /control/graph/edge/create` referenced
  request/node fields dropped in an earlier schema (`gripper`, `payload`,
  `grip`/`release` action blocks) and would crash when called; both now match
  the current model.
- The record-edge YAML append helper wrote list items at a different
  indentation than the file, which could produce unparseable YAML; it now
  matches the file's column-0 list style.

## [0.3.0] — earlier

Pre-changelog state: xArm 6 controller with BioGripper Gen2 and linear track,
FastAPI control server with STATUS_SPEC envelope and claim protocol, web
control panel and Cytoscape motion-graph viewer, motion-graph interlock
(schema 0.1) with OFF/ADVISORY/STRICT enforcement, nearest-node recovery, and
edge recording/editing.
