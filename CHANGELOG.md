# Changelog

All notable changes to this project are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and the project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

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
