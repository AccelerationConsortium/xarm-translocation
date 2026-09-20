# Changelog

All notable changes to this project are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and the project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Changed — RealSense cameras are addressed by id (2026-09-20)

The camera layer was a process-wide singleton: one camera, one set of
`/realsense/*` routes, one `components.realsense_camera`. It now holds a
registry of cameras keyed by a **device-local id**, and the first camera is
`rs435i`. This is a breaking change to every `/realsense/*` path; there is one
deployment and it is updated with this commit, so no compatibility shim was
added — a permanent alias for a one-off migration is a cost that never ends.

- **Config.** `realsense.yaml` grows a `cameras:` list. Each entry carries an
  `id` matching `^[a-z0-9][a-z0-9_-]{0,31}$` (it lands in URLs and in capture
  paths, so it is validated at load time rather than sanitised at every use)
  and a `serial`, which is **required** once more than one camera is
  configured: "the first device librealsense enumerates" is not stable across
  reboots. A malformed, duplicate or serial-less entry is logged and skipped;
  nothing here can stop the service from booting.
- **Routes.** `GET /realsense/cameras` is the discovery endpoint — it never
  404s and returns an empty list plus a `reason` when nothing is configured —
  and everything else nests under the id: `/realsense/{camera_id}/status`,
  `…/snapshot.jpg`, `…/depth.png`, `…/stream.mjpg`, `…/depth`, `…/intrinsics`,
  `…/captures`, `POST /control/realsense/{camera_id}/capture`,
  `DELETE /control/realsense/{camera_id}/captures/{capture_id}`. Gating is
  unchanged. An unknown id answers **404** `camera_not_found` listing the ids
  that do exist, which is a different fact from **404**
  `realsense_not_configured`.
- **The skill-facing alias stays a fixed path.** `POST
  /control/realsense/capture` now takes an optional `camera` in the body. A
  SkillDef carries one fixed `endpoint` string that `lab_skills.plan.execute_plan`
  and the dashboard passthrough send verbatim — there is no path templating in
  that chain, so an agent plan cannot express a camera id as a path segment.
  Omitted `camera` resolves to the sole camera; with two or more it is
  **400** `camera_required` rather than a guess that files evidence under the
  wrong lens. The alias only resolves a name and delegates to the nested route.
- **Capture store.** Layout becomes
  `<root>/<camera_id>/<YYYY-MM-DD>/<capture_id>/` and `meta.json` records
  `camera_id`; `_scan` derives it from the directory, so the two captures
  written before this change describe themselves correctly once moved under
  `rs435i/`. Retention stays **one** budget across all cameras, oldest first
  regardless of camera — the bound that matters is the disk's, not any one
  lens's. `tools/replicate_captures.py` walks and lands the same three levels.
- **Status.** One `components.realsense_<camera_id>` per camera (they fail
  independently, and a merged entry would hide the working one), and
  `details.realsense` becomes `{default, cameras: {id: block}, captures}`.
  `realsense.capture` is advertised when *any* configured camera qualifies.

### Changed — RealSense streams to 1280×720 (2026-09-19)

Both stream profiles in `src/settings/realsense.yaml` move from 640×480 @ 30
to **1280×720 @ 30**. Resolution was the request; frame rate was allowed to
fall to 15 or 6 if the link refused, and it did not need to: librealsense
accepted 1280×720 @ 30 for depth + colour on the first attempt on this
machine's USB 3.2 link (D435i serial 050422071813, firmware 5.11.1.100), no
warnings, `fps_measured` 30.0–31.4 over 76 frames.

- **Depth is upsampled, not sharper.** The D435i depth module's native
  best-accuracy mode is 848×480; 1280×720 depth is produced by the ASIC and
  adds pixels, not information. It is requested so the aligned `depth.png`
  matches the colour frame pixel for pixel. Bench fill on the current scene
  was **70.1 % valid pixels** (min 69.8 %, max 70.4 % over 10 frames), range
  0.30–19.2 m, median 0.38 m — the gripper's own view of the bench.
- **Per-capture size ~253 KB** measured through the real `CaptureStore`
  (colour JPEG 111.9 KB at quality 80 + 16-bit depth PNG 145.1 KB + meta):
  about **4.6×** the 55 KB the 640×480 profile wrote, not the ~3× a pixel
  count would predict, because the PNG grows with depth texture. Retention
  bounds (`keep_days: 30`, `keep_max_gb: 20`) are unchanged; at this size
  20 GB is ~80k captures, so `keep_max_gb` binds before `keep_days` only
  above ~2,600 captures a day.
- Comments and docs that stated the old resolution or the "~0.5 MB per
  capture" figure as current fact were updated (`realsense.yaml`,
  `realsense_captures.py`, `docs/agent/API_REFERENCE.md`,
  `docs/REALSENSE_API_PLAN.md`). Earlier CHANGELOG entries are left as
  written: they record what was verified at the time.

### Added — Intel RealSense depth camera (`/realsense/*`)

The service can now own an Intel RealSense depth camera (D435i and any other
librealsense model) plugged into the device PC over USB 3. New module
`src/core/realsense_camera.py`, new config `src/settings/realsense.yaml`,
new optional extra `realsense` (`uv sync --extra realsense` →
`pyrealsense2`, `numpy`, `Pillow`), a **Depth Camera** panel card
(`src/web/realsense-card.js`), 93 new tests, and a reference doc at
`src/docs/REALSENSE_CAMERA.md`.

- **A second, unrelated camera.** The existing "Lab Camera" is a network
  PTZ camera driven through the dashboard; this one is local USB hardware
  the service talks to directly. It supplies what the PTZ cannot: **metric
  depth per pixel** in a known camera frame — the primitive an arrival
  check, a plate locator, or a hood obstacle check will build on. This
  release deliberately ships only the foundation: capture, health, images,
  and pixel → metres. No motion decision consults it yet.
- **Endpoints.** `GET /realsense/status` (enumeration + pipeline state, open,
  answers before `/connect` and reports `installed: false` instead of
  404ing when the extra is missing); `POST /realsense/{start,stop}`;
  `GET /realsense/snapshot.jpg?stream=color|depth`; `GET /realsense/depth.png`
  (raw 16-bit, `X-Depth-Scale-M` header); `GET /realsense/stream.mjpg` (MJPEG
  for an `<img>`, paced server-side); `GET /realsense/depth?x=&y=&window=5`
  (median-filtered distance + pinhole-deprojected `[X, Y, Z]`);
  `GET /realsense/intrinsics`. Failure bodies are
  `{"detail": {"error", "reason"}}` with `realsense_not_configured` 404,
  `realsense_unavailable` 503, `realsense_not_streaming` 409,
  `realsense_error` 502.
- **Gating.** Reads that cannot switch the camera on are open like
  `GET /status`; anything that starts the pipeline or ships video is
  login-gated, matching the authenticated viewing sessions the PTZ preview
  moved to. Nothing is claim-gated — looking is not arm actuation. The
  numeric `/realsense/depth` read is *not* allowed to start the pipeline
  (409 when stopped) precisely so an open read cannot turn the camera on.
- **`/status`.** `components.realsense_camera` (`connected`, `state` ∈
  `streaming | idle | disconnected | driver_missing | error`, one-line
  `message`) and its machine-readable twin `details.realsense`. Both are
  **absent** when `enabled: false`, so unmigrated deployments see an
  unchanged envelope, and both are present on the pre-`/connect`
  `requires_init` envelope too — the camera does not wait for the arm. **The camera never changes `equipment_status`**: arm
  motion does not depend on it, so an unplugged camera is a component fact,
  not a `degraded` arm — the same §2.2 reasoning as the sash interlock
  being blind. A feature that later makes a move depend on a reading owns
  that gate (412 + `allowed_actions` mirroring), not this layer.
- **Optional at every layer, by construction.** Missing extra,
  `enabled: false`, and no camera on the bus each yield a camera object
  whose `describe()` carries a `reason`; nothing raises into the control
  path and the service always boots. `pyrealsense2` is imported lazily and
  the backend is injectable (`rs_module` / `np_module`), so the whole
  lifecycle — enumeration, start/stop, the capture thread, encoding, depth
  lookup, camera-lost recovery, idle timeout — is unit-tested against a
  fake `pyrealsense2` with no hardware.
- **Process-wide, not per-connection.** The camera outlives `/connect` /
  `/disconnect` (operators want the bench view before the arm is up), so
  it is a shared instance the API server configures at import and
  `status_builder` reads through `realsense_camera.shared_camera()` — the
  two surfaces cannot disagree. Lifespan handles `autostart` and shutdown.
- **A daemon capture thread owns the pipeline.** `wait_for_frames` blocks,
  so handlers only read the latest frame bundle under a lock; the MJPEG
  generator waits on a condition variable rather than polling. Five
  consecutive frame failures (`max_consecutive_frame_failures`) mark the
  camera lost and release it; `idle_timeout_seconds` (default 120) stops it
  when nobody is watching. USB 2 links are surfaced as a `warning`.
- **No OpenCV.** Pillow encodes JPEG/PNG and librealsense's own colorizer
  renders the depth map, keeping the extra to three wheels.

**Operational notes.**

- `pyrealsense2` 2.58.4 installs and imports on the service venv's
  Python 3.14. Syncing the extra while the `xarm` service runs hits the
  DEVICE_PC_SETUP "own-service `.exe` lock" — use
  `uv sync --inexact --no-install-project --extra realsense` (the project is
  editable; nothing about it needed reinstalling) or stop the service
  first, and verify the three packages landed.
- The legacy `:6001` proxy (`src/web/server.py`) buffers whole response
  bodies, so `/realsense/stream.mjpg` never renders through it; snapshots
  do. Use the panel on the API port (`:8000/web/`). `/realsense` was added
  to its proxied prefixes anyway so JSON and snapshots work.
- The bench PC's D435i was **not enumerating** at the time of writing (no
  Intel VID on the USB bus — cable/port, not driver). The layer reports
  exactly that (`state: disconnected`, `reason: no RealSense device
  connected`); nothing here has been exercised against live frames yet.

### Added — fume hood sash interlock

Refuses arm motion into the fume hood / Opentrons region unless the **separate**
`fume_hood_actuator` device reports its sash parked at the required preset, and
stops the arm if the sash leaves that preset while the arm is already inside.
New module `src/core/sash_interlock.py`, new config
`src/settings/interlocks.yaml`, 113 new tests.

- **Two behaviours.** (1) A move whose target is a `hood`- or
  `opentrons`-tagged graph node is refused with **HTTP 412**
  (`error: "interlock_not_satisfied"`, carrying the observed position and a
  hint). (2) A watchdog thread polling at 0.5 s while the arm is inside the
  region calls `stop_motion()` if the sash leaves position — after which *all*
  motion is refused, egress included.
- **No automatic retreat, deliberately.** A blind retreat could drag the arm
  through a descending sash, and the interlock cannot see where the sash is
  along that path. So an arm caught inside stays put until an operator
  overrides. Consequently the override is load-bearing, not a debug flag:
  `POST /control/interlocks/sash/override {reason, ttl_seconds}` (claim- and
  login-gated, capped at 120 s, re-issuable, reason required and audited),
  surfaced as a button in the `/web/` banner. `POST .../override/clear`
  restores enforcement; `GET /interlocks/sash` (+`?refresh=true`) reads state
  without commanding a move and answers before `/connect`.
- **Gated where graph mode cannot switch it off.** The check sits in
  `_consult_graph_for_move` *above* the `GraphMode.OFF` early return: graph
  mode is a policy switch (the calibration escape hatch), while this is
  physics. It also covers `move_plate_linear`, which never consults the graph
  at all — and is exactly the motion used to descend into the hood — plus a
  re-check between the arm and rail halves of a cross-rail move, since the rail
  translation is what actually carries the arm in, seconds after the first
  check. `interlock_freehand_guard` refuses raw cartesian/joint/velocity/rail
  moves while the arm is inside the region.
- **Fails open on an unreachable fume hood device**, by decision — an outage
  must not halt arm work. Because that means the guard protects nothing during
  one, every bypass is logged at WARNING (on the `xarm.interlock` logger, which
  the panel's log stream now carries), counted in
  `details.interlocks.fume_hood_sash.moves_allowed_while_blind`, emitted as an
  `interlock_bypass` event, and shown as an amber `SASH INTERLOCK BLIND` banner
  with a `[SASH-BLIND]` prefix on `message`. Two carve-outs close what
  fail-open did *not* cover: a device **never reached since startup** is
  misconfiguration rather than an outage and fails closed
  (`require_initial_contact`), as does one answering with an unparseable body
  (`malformed_fails_closed`).
- **`equipment_status` is unchanged when the interlock is blind.** §2.2 scopes
  `degraded` to an unhealthy subsystem *of this device*; a neighbouring Pi
  being unreachable is not this arm's ill health, and under fail-open its
  capability is not reduced — only its supervision. `details.interlocks` plus
  the message prefix carry it instead.
- **§6.2 mirroring** via a single filter in `reachable_node_ids()`, which feeds
  `allowed_actions`, `details.motion_graph.reachable_nodes`, `GET /graph`, and
  the panel's Drive Arm buttons at once. It reads only the cached sash
  observation — `build_status` is side-effect-free and polled every 2-3 s, so
  it must never fetch.
- **Sash position is visible in the panel at all times**, not only when the
  interlock is unhappy. The `#sash-banner` above is an alarm and is absent
  while the sash is parked, which left the normal case with no readout — so an
  operator planning a hood move learned the position by being refused. The
  Motion Graph card ("interlock layer") now carries a persistent
  `Fume Hood Sash` row: position, what state it implies, and how stale the
  reading is (`/status` serves the watchdog-warmed cache, never a live probe).
  Hidden entirely when no interlock is configured. Its data comes from the
  `/status` poll already in flight — no extra request per second — via a new
  reading-scoped `details.interlocks.fume_hood_sash.sash_position`, which
  exists because `observed_position` is *decision*-scoped and therefore None
  exactly while an override is active, i.e. when an operator walking a stuck
  arm out most needs the number. Same field on `GET /interlocks/sash`.
- **Not gated, on purpose:** gripper actions (the hazard is the arm envelope
  against the glass; a jaw stroke does not extend it, and gating would trap
  plates for no safety gain), and freehand *entry* (no node id, so there is no
  way to know where such a move ends).
- `MotionGraph.has_tag()` / `nodes_with_tag()` added — no tag-query helper
  existed, every consumer did raw set arithmetic on `node.tags`.

**Operational notes.**

- *After a watchdog stop* the arm is off-grid mid-motion: re-pin it with
  `POST /control/graph/recover_to` (`force=true`). Same for a move aborted by
  the pre-rail re-check, which leaves the intermediate `(arm, rail)` state that
  is a non-node by design.
- *If the fume hood Pi is down*, hood/Opentrons moves keep working and the
  amber blind banner appears. If they are instead **refused** with
  `state: "blind"`, the service has never reached the device since startup —
  check `base_url` in `src/settings/interlocks.yaml` (or
  `XARM_SASH_STATUS_URL`) and that the fume hood service is up, then
  `GET /interlocks/sash?refresh=true`.
- *Unverified assumption:* whether the fume hood device's
  `metrics.sash_position` is a true readback or the last *commanded* preset is
  not yet known. Because of that the predicate is composite — it also requires
  `components.sash.{state,connected}`, `components.actuator.state == "idle"`
  and the device's `equipment_status == "ready"` — since a last-commanded
  metric reports the *target* for a whole trip. Settle it at the bench
  (command 5, move the sash by hand, re-read `/status`) before treating this
  as a hard collision guard.
- *`deck` is not gated*, though the `Hood` and `Deck` rail locations are the
  same 550 mm and `joint_config.yaml` notes the arm at Deck can reach the
  fumehood positions. If a bench check shows the arm intrudes into the sash
  envelope there, add `deck` to `gated_tags` — that is the whole change.

### Added — catalog skill names in `allowed_actions` (`graph.*`) + `/control/{stop,clear_errors}` aliases

Closes the ROADMAP "skill-name reconciliation" item on the device side.

- **`/status.allowed_actions` now advertises the lab-skills catalog names**
  (`graph.move_to`, `graph.gripper`, `graph.recover_to`, `graph.mode`,
  `graph.record`) alongside the existing per-target `move.<node_id>` /
  `gripper.<state>` enumeration. STATUS_SPEC defines `allowed_actions` as
  "skill names matching `Skill.name` from the SDK catalog"; because the device
  advertised only its own per-target spellings, `lab.skills()` computed every
  `robot_arm` skill unavailable, and the dashboard assistant had to carry a
  name-bridging resolver. Availability mirrors each endpoint's state gates
  (§6.2): `graph.move_to`/`graph.gripper` require ≥1 whitelisted target in
  STRICT (and are advertised unconditionally in ADVISORY/OFF, which honor any
  target — those modes previously advertised nothing but `stop`);
  `graph.recover_to`/`graph.mode` require a loaded graph; `graph.record`
  requires a real (non-simulated) last transition. The motion-in-flight,
  Studio-Sim-box, error, and requires_init withholdings apply unchanged.
- **`POST /control/stop` and `POST /control/clear_errors`** now exist as
  aliases of `/move/stop` and `/clear/errors` (same handlers, same
  login-only/no-claim safety-floor gating), so the URL a generic STATUS_SPEC
  client composes from the advertised action names resolves instead of 404ing.
  The advertised `connect` is deliberately not aliased: the dashboard
  registry's `do_not_call_connect` forbids generic clients from composing it,
  and operator paths use the root `/connect`.

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
