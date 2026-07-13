# Changelog

All notable changes to this project are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and the project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

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

- Natural-language command layer (Claude Haiku). A new **Language Command** card
  below the Drive Arm card lets an operator type plain-English commands
  ("go to deck home", "pick up the tray from deck slot 1"). Claude Haiku
  interprets the utterance into a structured intent; a deterministic,
  graph-gated validator (`plan_from_intent` in `src/core/nl_command.py`) decides
  whether it can run; the operator confirms; execution reuses the STRICT
  graph-gated `move_to_node` / `set_gripper_state` paths. The LLM never drives
  the arm directly.
  - `GET /control/nl/status`: reports whether the LLM is configured.
  - `POST /control/nl/interpret` `{text}` (claim-gated): returns the interpreted
    intent + validated plan without moving. `503` when no API key is set.
  - `POST /control/nl/execute` `{steps}` (claim-gated): forces STRICT, re-validates
    each step against live state, then runs it. `409` on stale/invalid steps.
  - Config: `ANTHROPIC_API_KEY` (required to enable) and `XARM_LLM_MODEL`
    (optional, default `claude-haiku-4-5`); `anthropic` added as a dependency.
  - v1 is single-hop (direct neighbors / current-node gripper transitions only);
    the plan step list and per-step execution loop are the seam for future
    multi-hop path planning. See
    [PYXARM_LLM_COMMANDS.md](src/docs/PYXARM_LLM_COMMANDS.md).
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
