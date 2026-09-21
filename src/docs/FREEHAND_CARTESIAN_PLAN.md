# Freehand Cartesian control without breaking the motion graph — plan

**Status:** **Step 1 shipped 2026-09-21.** Steps 2 and 3 are still proposals.
Originally written 2026-09-20, when nothing here was implemented. Written because
two operators (Jiaru, Allan) asked for Cartesian control of the xArm and the
answer today is "switch the graph to ADVISORY and remember to switch it
back", which works and is the sanctioned escape hatch — but it hands out a
process-wide, unbounded, self-restoring-by-discipline-only relaxation of the
safety model. This plan makes freehand control **bounded, time-limited and
self-reverting**, and then makes the common case need no mode change at all.

## What exists today (no code change needed to use it)

| Surface | Gate | STRICT | ADVISORY / OFF |
|---|---|---|---|
| `POST /move/position` `{x,y,z,roll?,pitch?,yaw?,speed?}` | claim | 409 `graph_mode_strict` | honoured |
| `POST /move/relative` `{dx,dy,dz,droll,dpitch,dyaw,speed?}` | claim | 409 | honoured |
| `POST /velocity/cartesian` | claim | 409 | honoured |
| `POST /move/joints` | claim | 409 | honoured |
| `POST /control/graph/mode` `{mode}` | claim | — | the switch itself |

Every freehand endpoint still runs the controller's `workspace_limits` +
collision check (`safety_config`), `interlock_freehand_guard` (fume-hood
sash, 412, **every mode**) and `box_sim_guard`. So ADVISORY is not
"unguarded"; it is "not graph-guarded".

> **2026-09-21:** the sash interlock is disabled (`enabled: false`) pending
> the xyz safe/danger volume, so of those three only `workspace_limits` +
> collision and `box_sim_guard` are actually running. The table's paths have
> also gained `/control/freehand/*` spellings. The rest of this section is
> still accurate.

Two consequences the operators must know:

1. **The pin drops.** Any raw move sets `last_arm_pose_name = None`
   (`xarm_controller.py`, the "raw cartesian move invalidates any pinned named
   arm pose" comment), so `current_node` becomes `null` and every `graph.*`
   move is refused until the arm is re-pinned with `POST /move/location
   {location_name}` or `POST /control/graph/recover_to`.
2. **Mode is process-wide state** with no expiry. A forgotten ADVISORY is
   inherited by the next client, workflow or agent.

The working recipe (claim → `mode: advisory` → moves → `/move/location` to a
node → `mode: strict` → release) is in the 2026-09-20 report and needs no
change. Everything below is about removing the two consequences.

## Design rules

1. **The graph stays the authority in STRICT.** Freehand motion in STRICT is
   allowed only inside an envelope the graph itself declares, anchored to a
   node the arm is pinned at. Nothing here weakens named-move checking.
2. **Every relaxation is bounded in time and audited**, exactly like the
   sash override (`POST /control/interlocks/sash/override {reason,
   ttl_seconds}`, capped by `override_max_seconds`, logged at WARNING, one
   history row). Same body shape, same cap discipline, same UI banner.
3. **Reverting is automatic and unconditional**: on TTL expiry and on claim
   release/expiry. Discipline is a backstop, not the mechanism.
4. **`allowed_actions` mirrors every new refusal** (STATUS_SPEC §6.2). One
   predicate feeds the endpoint and the list.
5. **Testable without hardware** against the existing MagicMock controller +
   motion-graph fixtures.

## Step 1 — Time-limited mode override with auto-revert — **SHIPPED**

Landed 2026-09-21, close to as designed below. What actually shipped, and
where it diverged:

- `GraphModeRequest` gained `reason` and `ttl_seconds` as planned. Defaults
  and cap live in `motion_graph.yaml` at the proposed 300 s / 900 s (**D-1
  settled as proposed**).
- **Expiry is lazy in the `graph_mode` property itself**, not checked at each
  call site. `XArmController.graph_mode` is now a property whose getter runs
  the revert check, so every guard, every controller move path and every
  status build inherits it and none of them can be the one that forgot. This
  is stronger than the plan's "checked on every /status build and every
  motion endpoint" and removes a whole class of missed-call-site bug.
- Revert triggers are all four proposed (TTL, claim release *or* expiry by
  the lowering session, `/disconnect`, explicit STRICT). The claim trigger is
  evaluated by comparing the live holder's `session_id` against the one
  recorded at grant time, so it covers release and silent expiry with one
  check. It is skipped when no claim was held at grant time — otherwise a
  deployment with claims unenforced would revert on the first read.
- **D-2 settled: OFF stays reachable through the API**, no config gate. Raw
  `/track/move` and genuinely unguarded work need it, and a gate would only
  push people toward `enabled: false`, which is the outcome the plan warns
  about. It is logged more loudly and carries the same window.
- Added beyond the plan: `POST /control/graph/mode/restore`, for the same
  reason the sash interlock has `override/clear` — "put the guard back" is a
  distinct intent from "set the mode to this value", and a one-button UI
  control should not have to know which value to send.
- `message` takes `[GRAPH-ADVISORY]` / `[GRAPH-OFF]`, keyed on an *active
  window* rather than on the mode alone: a deployment with no graph sits at
  OFF permanently and prefixing every poll there would be noise.
- Tests: `test/test_graph_mode_override.py` (27, real controller + status
  envelope + the freehand guard end-to-end), plus 6 API-level ones in
  `test/test_motion_graph_api.py`.

The original proposal follows, unedited.

### Original proposal

Extend `GraphModeRequest`:

```python
class GraphModeRequest(BaseModel):
    mode: str                                   # off | advisory | strict
    reason: Optional[str] = None                # REQUIRED when lowering below strict
    ttl_seconds: Optional[float] = Field(None, ge=1.0)   # clamped to mode_override_max_seconds
```

Behaviour of `POST /control/graph/mode` when `mode != strict` **and the
current mode is strict**:

- `reason` required (422 without it). Logged at WARNING with the claim owner;
  one `graph_mode_override` row to `/api/ingest/events` with
  `{mode, reason, ttl_s, owner}`.
- The controller records `graph_mode_override_until` (monotonic) and
  `graph_mode_override_owner_session`. Default TTL and cap live in
  `motion_graph.yaml` (`mode_override_default_seconds: 300`,
  `mode_override_max_seconds: 900` — longer than the sash's 120 s because
  a calibration session is minutes, not one retreat).
- **Revert triggers**, all restoring STRICT and emitting
  `graph_mode_restored {trigger}`:
  - TTL expiry — checked lazily on every `/status` build and every motion
    endpoint (no timer thread), the same lazy-expiry idiom `ClaimManager` uses;
  - claim release (`POST /control/release`) or claim expiry by the session
    that lowered the mode;
  - `/disconnect`;
  - explicit `POST /control/graph/mode {mode: strict}` (clears the override).
- Raising to STRICT never needs a reason or TTL. Switching OFF is allowed
  but logged as a separate, louder line; consider refusing OFF unless a
  config flag `allow_mode_off: true` is set — OFF also disables
  ADVISORY's warnings, and nobody has asked for that.
- Re-issuing extends the TTL (like the sash override).

Surfaces:

- `details.motion_graph` gains `mode_override: {active, mode, reason,
  owner, expires_at}` (null when none). `message` gets a `[GRAPH-ADVISORY]`
  prefix while lowered, mirroring `[SASH-*]`.
- `allowed_actions`: unchanged semantics — in ADVISORY the family names are
  already advertised. Add nothing new; the point of this step is that
  ADVISORY cannot persist.
- Panel: the same amber banner component the sash override uses, with the
  countdown and a "Restore STRICT now" button.

Tests: reason required only when lowering; TTL clamp; lazy expiry restores
STRICT on the next `/status`; release/expiry/disconnect each restore;
re-issue extends; events emitted with the right trigger; `[GRAPH-ADVISORY]`
prefix appears and disappears.

Effort: ~½ day.

## Step 2 — Freehand zones in STRICT (the real fix)

Let the graph declare where Cartesian control is acceptable, per node:

```yaml
# motion_graph.yaml (schema 0.3, additive)
freehand_zones:
  bench_inspect:
    frame: base                     # xArm base frame, mm / degrees
    box: {x: [250, 450], y: [-150, 150], z: [120, 320]}
    orientation_tolerance_deg: 15   # roll/pitch/yaw may deviate this much from the node's pose
    max_step_mm: 40                 # per-request bound on |dx,dy,dz| for /move/relative
    max_speed: 60
    endpoints: [move.position, move.relative, velocity.cartesian]   # velocity optional
nodes:
- {id: robot_home, ..., freehand_zone: bench_inspect}
```

Rules in `strict_graph_guard` (rename to `freehand_guard`, one function fed
by both the endpoint and `allowed_actions`):

- Freehand is permitted in STRICT iff **all** of: the arm is pinned at a node
  with a `freehand_zone`; the current TCP pose (`last_position`) is inside
  the box; the requested target (absolute, or current + delta) is inside the
  box **and** within `orientation_tolerance_deg`; the per-request step and
  speed respect the zone's caps; no sash-gated tag on the node; a claim is
  held (already true via `require_claim`). Otherwise the existing 409
  `graph_mode_strict` is returned with a new structured body:
  `{error: graph_mode_strict, action, reason: no_zone | outside_zone |
  step_too_large | speed_capped, zone, box, target}`.
- **The pin is kept, not dropped.** Inside a zone the controller records
  `freehand_offset` (target minus the node's pose) instead of clearing
  `last_arm_pose_name`, so `current_node` stays valid and reads as
  `"robot_home (freehand, +12,-3,+40 mm)"` in `details.motion_graph`.
  Leaving the zone via a graph move (`move_to` / `travel_to` from that node)
  first returns to the node's canonical pose along a straight line, then
  runs the edge — the same "recover then move" the graph already does for
  `recover_to`.
- `/move/location` to the pinned node itself is always allowed from inside
  the zone (the way home), and `recover_to` clears the offset.
- Zones are validated at graph load: box must contain the node's own pose
  (else the node could never be "inside"), must not intersect any
  sash-gated node's approach volume by a configurable margin, and must lie
  within `workspace_limits`.

Surfaces:

- `allowed_actions` gains `freehand.move_position` / `freehand.move_relative`
  (advertised only when the predicate above passes, STRICT included; in
  ADVISORY/OFF they are advertised unconditionally as today's behaviour
  implies).
- `details.motion_graph.freehand = {zone, inside, offset_mm, remaining_mm:
  {x: [-, +], y: [...], z: [...]}}` so a UI can show how far the operator can
  still go.
- `GET /graph` includes the zone boxes so the web viewer can draw them.
- Panel: a small jog pad (±1/±5/±20 mm, speed slider capped by the zone)
  that only enables when `freehand.move_relative` is advertised. This is the
  operator surface Jiaru/Allan actually want; the API calls are what agents
  and scripts use.

Tests: zone load validation (box excludes node pose → GraphError; overlaps
hood → GraphError); inside/outside/edge-of-box targets; step and speed caps;
delta arithmetic uses the live `last_position`; pin retained and offset
reported; graph move from inside a zone returns to canonical pose first;
`allowed_actions` ⇔ predicate for a grid of poses (property-style, like the
plateloc temperature interlock test); sash-gated node with a zone is
refused at load.

Effort: ~1½ days + a bench session to size the first zone.

## Step 3 — Catalog and agents (outside this repo, later)

- `lab-skills` `robot_arm` catalog stays `graph.*` only for now. Whether
  `freehand.move_relative` becomes an agent skill is a policy question, not a
  plumbing one: the assistant's Control mode proposes per-hop graph moves
  precisely because they are enumerable and reviewable, and a jog is
  neither. Recommendation: **humans only** until a zone has been in use for
  a while; if agents need it, expose it with `requires_components:
  {freehand: inside_zone}` so availability is visible before the call.
- Dashboard: no change until Step 2 lands; then surface the
  `[GRAPH-ADVISORY]` banner and the zone status on `RobotArmTile`.

## Decisions needed

- ~~**D-1** Default and cap for the mode-override TTL (proposed 300 s / 900 s).~~
  **Settled 2026-09-21 as proposed**, in `motion_graph.yaml`.
- ~~**D-2** Keep OFF reachable through the API, or config-gate it?~~
  **Settled 2026-09-21: reachable, ungated.** See the Step 1 notes.
- **D-3** First zone: which node and how big? Proposal: `robot_home` with a
  200 × 300 × 200 mm box above the bench, no rail motion — enough for
  camera inspection and gripper checks, nowhere near the hood.
- **D-4** Should `velocity.cartesian` be zone-eligible? It is the one
  endpoint whose target is not known at request time (it runs until stopped);
  allowing it means checking the pose in the telemetry loop and stopping
  at the boundary. Recommendation: no in the first cut.

## Ordering

Step 1 first: it is small, removes the "forgot to restore STRICT" failure
today, and its banner/TTL/event plumbing is reused by Step 2. Step 2 makes
the mode switch unnecessary for the normal case. Step 3 waits for evidence.
