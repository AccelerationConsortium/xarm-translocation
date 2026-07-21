# xArm Integration + Measurement-Readiness Audit & Plan

**Status:** re-baselined against shipped state. **Read-only audit — no code was changed, no services restarted.**
**Original:** 2026-06-03 (workspace root). **Re-baselined:** 2026-07-19, and moved into this repo (`xarm-translocation/docs/`) now that the xArm is a live v1.1 device.
**Scope:** (A) integrate an xArm with state-machine observability; (B) determine whether the current event log can produce the methods-paper metrics (C1–C5); (C) smallest-change-first reversible plan.

> Ground-truth contracts used: `ac-organic-lab/docs/STATUS_SPEC.md` (v1.1 claims, `allowed_actions`, `details.claimed_by`), `ac-organic-lab/docs/OBSERVABILITY.md` (SQLite schema + event_type registry), `ac-organic-lab/docs/INTERLOCKS.md` (four-layer safety), `ac-organic-lab/docs/ROADMAP.md` (v0.4 `execute_plan`/MCP, per-device migration state). This repo's own source is under `src/core/`. Where code contradicts docs, the contradiction is flagged — never bypassed.

---

## Progress since the 2026-06-03 baseline (what shipped)

The original audit predated a wave of xArm + SDK work. Verified done as of this re-baseline:

- **xArm is a live STATUS_SPEC v1.1 device.** `ac-organic-lab/equipment.yaml:103` is now `protocol: "1.1"` (was `1.0` + a stale "read-only" comment); the comment now correctly describes the claim-gated control surface. `do_not_call_connect: true` is retained by design (`equipment.yaml:115`). This repo reports `protocol_version == "1.1"` (`src/core/models.py:PROTOCOL_VERSION`, asserted in `test/test_claims_api.py:271`).
- **Full claim protocol + hard enforcement** (`src/core/xarm_api_server.py`): `/control/claim` (1898), `/control/heartbeat` (1958), `/control/release` (1971), `/control/claim/enforce` (1980); `X-Claim-Token` enforced on control (`src/core/claims.py`, `test/test_claim_enforcement.py`).
- **Motion-graph control surface** (`xarm_api_server.py`): `/control/graph/{move_to (2126), travel_to (2174, multi-hop — new), recover_to (2272), gripper (2318), mode (2371), record (2395), edge (2438), edge/delete (2527), edge/create (2594), node (2676)}`, plus reads `/graph` (2022), `/graph/layout` (2086/2108), `/graph/nearest` (2245).
- **Controller state → `EquipmentState` mapping is implemented** in `src/core/status_builder.py:149-173` (`requires_init`/`error`/`busy`/`ready`/`degraded`; `e_stop` recognised at :325), with graph position surfaced in `details.motion_graph`. This is the L1/L2 device-side mapping the original plan's A.4b only proposed.
- **Device-push events exporter shipped** (`src/core/events_exporter.py`, wired at `src/core/xarm_controller.py:370,466-467`): fine-grained `state_transition` / `error` rows pushed from `register_state_changed_callback` / `register_error_warn_changed_callback` to `POST /api/ingest/events`, `extra:{xarm_state, error_code, warn_code, graph_node}`; disabled unless `XARM_INGEST_URL` is set.
- **`execute_plan` shipped** (lab-skills v0.4): `ac-organic-lab/skills/src/lab_skills/plan.py:311`, sync façade `sync.py:161`, MCP tool `mcp.py:243`. The runtime veto chokepoint the original plan called "deferred to v0.4" now exists.
- **OBSERVABILITY event_type registry landed** — `agent_observation` is now documented (was "undocumented"), `error`/`startup`/`shutdown` are attributed to the xArm exporter, and a new `alert_emitted` type exists.

Net effect on the plan below: **Steps 0, 3, 4 are essentially done; Step 5 is half-done (mechanism yes, decision-journaling no).** The paper-specific instrumentation (Steps 1, 2, 6, 7, 8) is still ahead. Part B's core verdicts are largely unchanged: the metrics still can't be computed because the paper's event vocabulary is not emitted anywhere (verified: zero hits for `recovery_attempt`, `transition_proposed`, `transition_decision`, `vision_verification`, `run_started/finished/failed`).

---

## TL;DR

1. **The adapter contract is tiny and already generic**, and the xArm now uses it end-to-end. `adapter: http` + `protocol: "1.1"` drives the device; the control passthrough forwards `/control/graph/*` with the full claim dance. **No aggregator API changes are needed** — the `equipment.yaml` flip has happened.
2. **State is modeled explicitly, and transition recording is now two-tier.** The device owns its state machine and pushes fine-grained `state_transition`/`error` rows via the exporter; the aggregator's 60 s poll (`ac-organic-lab/api/app/main.py`) remains the coarse backstop. Sub-60 s transitions that were invisible in the baseline are now captured by the device-push path.
3. **The event log can still produce almost none of the paper's metrics** — not because the schema is wrong, but because the *paper's* events (recovery attempts, proposal/veto decisions, vision verdicts, general run lifecycle) **aren't emitted yet.** Executed actions are logged; proposed-and-vetoed actions and vision verdicts are not.
4. **The single highest-leverage remaining change is an event vocabulary, not a schema change.** `/api/ingest/events` accepts a free-string `event` + an `extra: dict` blob, and both PyPoe and the xArm exporter already write through it. Defining ~6 new `event_type` strings + a small set of `extra` keys, emitted from the right layer, unlocks C1, C2, C3, C4 **with zero DDL change**.

---

# PART A — Architecture Audit

## A.1 Device interface — the exact adapter contract

**Abstract contract** (`ac-organic-lab/skills/src/lab_skills/status_adapters/base.py:32-41`):

```python
class EquipmentAdapter(ABC):
    def __init__(self, entry: EquipmentEntry) -> None:
        self.entry = entry
    @abstractmethod
    async def fetch(self, client: httpx.AsyncClient) -> AdapterResult:
        """Return the latest equipment status, or a synthetic `unknown` envelope."""
```

`AdapterResult` (`base.py:22-29`) = `{status: EquipmentStatus, fetched_at, latency_ms, error}`. **Adapters must never raise from `fetch()`** — failures become a synthetic `unknown` envelope via `self.fail()`.

**Factory dispatch on the `adapter:` yaml field** (`status_adapters/factory.py:34-46`): `mock` → `MockAdapter`; `http` → `HttpStatusAdapter`; `legacy_http` → per-id translator or `HttpStatusAdapter` fallback.

**What this repo implements to be wrapped (all present):** `GET /` (`ProbeResponse`, `xarm_api_server.py:905`), `GET /health` (915), `GET /status` (1185) returning the `EquipmentStatus` envelope; `POST /control/*` incl. `/control/{claim,heartbeat,release}`, `allowed_actions`, `details.claimed_by`. **No Python is written on the aggregator side** — `adapter: http` covers it, and the `equipment.yaml` entry (`ac-organic-lab/equipment.yaml:99-115`) is the only registry change (already made).

## A.2 State model — explicit state, two-tier transition recording

- **State is an explicit closed enum**, owned by the device: `EquipmentState = ready|busy|requires_init|degraded|dry_run|error|e_stop|unknown` (`ac-organic-lab/skills/src/lab_skills/models.py:51-60`), computed here in `src/core/status_builder.py:149-173`. The aggregator does **not** re-derive state — it reads the device's value.
- **In-memory current state, SQLite append-only history — confirmed.** The aggregator caches only the state string + reachability bool in module-level dicts (`ac-organic-lab/api/app/main.py:53-56`); the full snapshot is not cached.
- **Transitions are now recorded on two paths:**
  1. **Coarse (aggregator):** `_uptime_poll_loop` (`api/app/main.py:65-198`) writes a `state_transition` row when its **60 s** poll observes an `equipment_status` change. Timing resolution ±60 s; a transition that begins and ends inside one window is invisible on this path.
  2. **Fine-grained (device-push):** `src/core/events_exporter.py` pushes `state_transition`/`error` rows from the xArm SDK callbacks the instant they fire — closing the sub-60 s and latch-and-clear gaps the original audit flagged. Disabled unless `XARM_INGEST_URL` is set (so dev/CI emit nothing).

## A.3 Event ingest — schema + handler + context/extra

**Table DDL** (`ac-organic-lab/api/app/db.py:49-59`): `equipment_events(id, ts, device_id, event_type, from_state, to_state, message, payload)`, index on `(device_id, ts)`. `event_type` is **TEXT with no CHECK/enum** — any string is accepted. `payload` is a JSON string.

**Ingest handler** (`api/app/history.py`): `message = rec.message or rec.context`; the handler pulls known keys then `payload.update(rec.extra)` folds in everything else, so **arbitrary keys in `extra` are persisted**. The model has no `extra="allow"`, so *top-level* unknown keys 422 — the supported escape hatch is the explicit `extra` dict. **New event types/fields ride through with no api/ change as long as they go in `event` + `extra`.**

**`control_action` audit** (`api/app/control.py:455-500`): every dashboard passthrough writes one row, `event_type="control_action"`, `payload={action, method, status_code, outcome, owner[, detail≤500]}`. Outcomes: `ok|refused|claim_denied|timeout|transport_error`. **The request body is still *not* captured** (verified `control.py:473-481`) — so for `graph/move_to`/`recover_to` the `node_id` is absent from the audit row (`action` carries the verb path, not the argument). This is the one A.3 gap the original audit flagged that is **still open** (Step 2).

**Event types actually emitted vs documented (updated):**

| event_type | Emitted? | Where |
|---|---|---|
| `state_transition` | ✅ | `api/app/main.py` (60 s poll) **and** device-push (`src/core/events_exporter.py`) |
| `control_action` | ✅ | `api/app/control.py` |
| `agent_observation` | ✅ (now documented) | PyPoe `append_observation` → `/api/ingest/events` |
| `error`, `startup`, `shutdown` | ✅ **now emitted** by the xArm exporter | `src/core/events_exporter.py` |
| `alert_emitted` | ✅ (new since baseline) | `ac-organic-lab/api/app/alert_notifier.py` |
| `calibration`, `claim_acquired` | ❌ reserved, not yet emitted | — |

**Read path:** `GET /api/history/events/{device_id}?limit=&event_type=` now supports an `event_type` filter (was client-side only in the baseline).

## A.4 xArm — wrapped, translating, emitting (all shipped)

**(a) Wrapped as STATUS_SPEC adapter — done.** `equipment.yaml:99-115` is `adapter: http`, `protocol: "1.1"`. The control passthrough runs acquire→attach `X-Claim-Token`→release because `entry.protocol == "1.1"` (`api/app/control.py`). `do_not_call_connect: true` stays (claims are only issuable while the controller is connected; the SDK must never auto-connect a robot arm). **Open reconciliation:** the catalog registers 4 graph SkillDefs — `graph.{move_to,recover_to,record,mode}` (`skill_catalog/robot_arm.py:75-106`) — but the device now exposes more verbs (`travel_to`, `gripper`, `edge`/`edge.create`/`edge.delete`, `node`). `RobotArmClient`/`RobotArmTile` remain read-only (no `graph.*` buttons yet). See Step 4-remainder.

**(b) Controller state/error codes → our vocabulary — done.** Implemented in `src/core/status_builder.py` (not connected → `requires_init`; collision/e-stop → `e_stop`; non-zero `error_code` → `error`; warn/reduced-capability → `degraded`; in motion → `busy`; cleared & ready → `ready`), with graph position in `details.motion_graph`.

**(c) State-transition events — done (fine-grained).** `src/core/events_exporter.py` pushes `state_transition`/`error` to `/api/ingest/events` with `extra:{xarm_state, error_code, warn_code, graph_node}`. No aggregator schema change (rides the existing ingest path); 60 s poll remains the coarse backstop.

## A.5 Interlocks — layers the arm joins, and the veto path for C2

Per `ac-organic-lab/docs/INTERLOCKS.md` the arm participates in all four layers: L1 firmware limits/collision/e-stop (this repo); L2 graph `mode` (off/advisory/strict) + 409/412 on illegal graph transitions (`/control/graph/mode`, `xarm_api_server.py:2371`); L3 skill preconditions on the `robot_arm` SkillDefs; L4 project plan interlocks (INTERLOCKS.md's worked example is literally an xArm rule).

**Deterministic veto path for C2 — status updated:**
- **Offline: yes.** `validate_plan(plan)` runs L3 + L4 and returns a `PlanReport` with `violations`.
- **At runtime: mechanism now exists.** `execute_plan` (`skills/src/lab_skills/plan.py:311`) re-runs L3 + L4 immediately before each step — the execution-time veto chokepoint the baseline said was "deferred." **But no layer persists the veto decision** — `PlanReport` is in-memory and no `transition_decision` event is emitted. So C2's "agent proposes, deterministic layer vetoes, with measurable veto rate" now has the **runtime gate** but still lacks the **audit trail**.

---

# PART B — Measurement-Readiness Audit (re-baselined)

Format per metric: **[metric] → [computable now?] → [missing] → [where emitted]**. New event types/fields ride the existing `equipment_events` table via `event` + `extra` (no DDL change) unless noted.

## C1 — Recoverability model (cost-chosen recovery target)

- **recovery-success rate** → **Partial** → attempts are countable from `control_action` rows where `action LIKE 'graph/recover_to%'`, but the **node identity is still missing** (request body not captured, `control.py:473-481`) and "success" = HTTP 2xx, not sensor-verified → emit a **`recovery_attempt`** event with `extra:{recovery_id, target_node, outcome, verified_by}` from the recovery planner.
- **unsafe-recovery rate vs. naive nearest-node** → **N** → needs per recovery: candidate set + each candidate's `(reversible, sensor_verifiable, interlock_safe)`, selection `policy` (`cost`|`nearest`), `chosen_cost`, counterfactual `nearest_node`, and a post-hoc `unsafe` verdict → `recovery_attempt.extra:{candidates[], policy, chosen_cost, nearest_node, unsafe}`. The device already exposes `/graph/nearest` (`xarm_api_server.py:2245`) and `recover_to` has a `force` flag, so the counterfactual is cheap to compute at selection time. **Where:** a `lab-skills` recovery module or the project repo.

## C2 — Interlock-gated agent navigation (propose → deterministic veto)

> **Updated:** the runtime veto gate now exists (`execute_plan`), but it does **not journal** proposal/veto decisions, and PyPoe still has no proposal logging (it's read-only journaling). `validate_plan`'s `PlanReport` is in-memory.

- **proposed-transition count** → **N** → emit **`transition_proposed`** `{extra:{proposer, proposed_node|action, run_id, guardrails_mode}}` at the agent boundary → PyPoe MCP (a new journaling tool) and/or the workflow submitting the plan.
- **vetoed-transition count / veto rate** → **N** → emit **`transition_decision`** `{extra:{verdict: allowed|vetoed, vetoing_layer: L3|L4, reason, proposed_node}}` **at the `execute_plan` gate** (the mechanism is now there; only the emit call is missing). Veto rate = vetoed / proposed.
- **unsafe-action incidence (guardrails on vs off)** → **N** → ablation tag `extra.guardrails_mode ∈ {off, advisory, strict}` (maps to the device's `graph.mode` knob, `xarm_api_server.py:2371`) + `extra.unsafe_executed: bool`. **Where:** same gate, tagged by run configuration.

## C3 — Vision-gated state verification (independent CV confirms recovery node)

- **false-resume prevention rate** → **N** (no CV exists anywhere) → emit **`vision_verification`** `{extra:{recovery_id, telemetry_state, telemetry_would_resume, vision_verdict: confirmed|rejected, vision_confidence, resumed}}`. Denominator = recoveries where telemetry believed the node was reached; numerator = of those, how many vision rejected. Correlate to C1 via `recovery_id`. `graph.recover_to`'s `force` flag makes vision the natural non-`force` evidence source. **Where:** a new CV verification component invoked before the resume `graph.move_to`.

## C4 — End-to-end integration: completion rate + MTBF

- **autonomous-run completion rate** → **Partial** → `runs` has `status ∈ {in_progress, complete, failed, aborted}` + timestamps (`ac-organic-lab/docs/OBSERVABILITY.md §4`), so completion rate is computable **for dosing runs**. `runs` is dosing-shaped (`plate_id, compound_id, target_mg, n_wells, n_converged`); a general multi-device campaign (e.g. an xArm translocation workflow) is not modeled → generalize `runs` **or** emit `run_started`/`run_finished`/`run_failed` with a shared `run_id` + `extra.failure_cause`. **Where:** the workflow runner (project repo).
- **MTBF** →
  - *Infra reachability MTBF* → **Y** → `service_uptime` (written by the 60 s poll) covers it; overlaps Uptime Kuma (which sees transport/service liveness only, not device-internal faults).
  - *Operational (device-fault) MTBF* → **Partial (improved)** → the xArm exporter now emits `error` events with `extra:{error_code, warn_code}` (was "never emitted" in the baseline), so device-fault inter-arrival is computable **for the xArm**. Run-failure events for the workflow layer are still missing. **Where:** device exporter (done for the arm) + workflow runner (run failures, still to do).

## C5 — Workflow-level composability

- **per-workflow diff** → **Y, VCS-measured** → adapters dispatch on a generic `adapter:` field; the control passthrough is generic (`{action:path}`); the skill catalog is per-kind not per-device; project repos depend only on `lab-skills`. A workflow composed from the existing catalog should touch only the project repo (+ optional `equipment.yaml` role binding). **Metric = git diff scope per workflow PR** (target = 0 files in `skills/` adapters, `api/`, device drivers).
- **Boundedness caveat:** C5 holds only for workflows expressible with the existing catalog. The capability inventory is now **~10 kinds / ~80 SkillDefs** (`SKILL_REGISTRY`; verified 11 kind modules / 81 `SkillDef(` in `skill_catalog/`) — up from the baseline's "8 kinds / ~50". A workflow needing a new device capability breaks C5 by definition; measure candidate workflows against the published inventory. Optional CI guard: assert project repos don't import device repos / `api`.

### Summary table (re-baselined)

| Claim | Metric | Computable now | Missing / change since baseline |
|---|---|---|---|
| C1 | recovery-success rate | Partial | `recovery_attempt` event + `target_node`, `verified_by` (node id still absent from `control_action`) |
| C1 | unsafe vs nearest-node | **N** | candidate set, classifications, `policy`, `chosen_cost`, `nearest_node`, `unsafe` |
| C2 | proposed count | **N** | `transition_proposed` event |
| C2 | vetoed count / veto rate | **N** | `transition_decision` emit at the `execute_plan` gate (gate now exists; emit missing) |
| C2 | unsafe incidence on/off | **N** | `guardrails_mode`, `unsafe_executed` tags |
| C3 | false-resume prevention | **N** (no CV) | `vision_verification` + fields |
| C4 | run completion rate | Partial (dosing only) | general run model OR `run_started/finished/failed` + `failure_cause` |
| C4 | MTBF (infra) | **Y** (`service_uptime`) | — |
| C4 | MTBF (operational) | **Partial** (was N/weak) | xArm `error` events now emitted; workflow run-failure events still missing |
| C5 | per-workflow diff | **Y** (VCS-measured) | capability inventory now ~10 kinds/~80 SkillDefs; optional CI import guard |

---

# PART C — Proposed Plan (smallest-change-first, reversible)

Principles: **aggregator stays single source of truth**; **reuse the existing SQLite event log** (new `event_type` strings + `extra` keys only, no new DB); **honor STATUS_SPEC v1.1 + INTERLOCKS** (never bypass a veto; the device remains the authority). Steps ordered so instrumentation that unlocks the most claims comes first; each independently revertible.

### Step 0 — Documentation reconciliation — ✅ DONE
`ac-organic-lab/docs/OBSERVABILITY.md` now carries the event_type registry (records `state_transition`, `control_action`, `agent_observation`, `alert_emitted`, and the xArm-exporter `error`/`startup`/`shutdown`; `calibration`/`claim_acquired` marked reserved). The stale xArm `equipment.yaml` comment is fixed and `protocol` is `1.1`.

### Step 1 — Define the paper event vocabulary (high-leverage; unlocks C1–C4) — ▢ TODO
Specify in `ac-organic-lab/docs/OBSERVABILITY.md` the new `event_type` strings + standard `extra` keys: `recovery_attempt`, `transition_proposed`, `transition_decision` (verdict allowed|vetoed), `vision_verification`, `run_started`/`run_finished`/`run_failed`. All persist via the existing `equipment_events.payload` JSON. Optionally add a typed emitter helper (`lab_skills.observability.emit_event(device_id, event, **extra)`). **No `api/` change.** Verified not yet present (zero hits for any of these strings). **Risk:** low (additive). **Reversible:** delete helper; events are just rows.

### Step 2 — Capture control request bodies in the audit row (unlocks C1 attempt-keying) — ▢ TODO (still open)
In `ac-organic-lab/api/app/control.py:455-500`, add a size-bounded, secret-stripped `node_id`/`mode` (or bounded `request_body`) to the `control_action` payload so every executed `graph/move_to`/`recover_to` is self-describing. **Contracts:** STATUS_SPEC §8 (redact; ≤500 chars). **Risk:** low; audit-only. **Reversible:** revert the field. *(Verified still absent as of 2026-07-19.)* **Per the Step 4 architecture decision, this covers only the SDK/dashboard-mediated path; human control driven from the device-hosted panel bypasses the dashboard passthrough entirely and must be audited device-side (Step 4c). The two paths should write the same `control_action` payload shape from different layers.**

### Step 3 — xArm device-side fine-grained transitions + error events — ✅ DONE (this repo)
`src/core/events_exporter.py` pushes `state_transition`/`error` from the SDK callbacks (wired in `src/core/xarm_controller.py:370,466-467`) with `extra:{xarm_state, error_code, warn_code, graph_node}`; controller-code→`EquipmentState` mapping in `src/core/status_builder.py`; `details.motion_graph` surfaced. Disabled unless `XARM_INGEST_URL` set. **Deploy check:** confirm `XARM_INGEST_URL` is set in the device PC's NSSM service env (per ROADMAP's outstanding deploy note).

### Step 3b — Motion-graph correctness & safety guards — ◑ PARTIAL (this repo)
Hardening of the graph the arm is actually driven over (schema 0.2), added on `drive-with-motion-graph`:

- ✅ **Cross-rail safety rule** (`src/core/motion_graph.py::_validate_topology`): a rail (cross-location) edge is a load-time `GraphError` unless *both* endpoints are transit gateways. Gateways are tagged `global_home` (the single rail-0 safe pose, `robot_home`) or `transit_home` (each station's front-door pose). This makes the invariant structural — the rail only translates home-to-home, and each station is reached by a pure arm move from its local home. (Tags renamed from `home`/`transit` for readability across the YAML, `_CROSS_RAIL_TAGS`, and the `src/web/graph.js` anchor picker.) Unit-tested: cross-rail→untagged rejected, →transit accepted, same-rail→untagged allowed.
- ✅ **Connectivity guard** (`test/test_motion_graph.py::test_no_unexpected_unreachable_nodes` + `WIP_UNREACHABLE_FROM_HOME`): nothing outside a documented allowlist may be unreachable *from* `robot_home`. Freezes the 21 not-yet-wired station orphans (cytation / plateloc / opentrons 4·6 / all `_press` grip poses / uplc plate) as explicit WIP and fails if a wired node silently loses its edges — which STRICT would turn into a stranded arm. A companion test rejects stale allowlist ids.
- ✅ **One-way UPLC-drawer trap — fixed (2026-07-21):** the return edge `uplc_draw_open_max → uplc_draw_home` was added to `motion_graph.yaml`, closing the pocket. Verified by the new reverse-reachability guard below.
- ✅ **Reverse-reachability guard — shipped (2026-07-21):** `MotionGraph.nodes_without_return_to(target)` + `test/test_motion_graph.py::test_all_connected_nodes_can_return_home` assert every node reachable from `robot_home` also has a path back ("the arm can always retreat"). **Optional by design** — the requirement assumes this arm is the cell's only mover; a deployment with a second gantry/arm can set `enforce_return_to_home: false` in `motion_graph.yaml` (guard skips) or allowlist individual one-way poses in `WIP_NO_RETURN_TO_HOME`.
- ▢ **Gripper verification fails *open*** (`src/core/xarm_controller.py:1900`): when the hardware position can't be read, `_verify_gripper` warns and returns `True`. For a `grasp` intent (confirming an object is held) this should arguably fail *closed* — a silent "couldn't check → assume success" can drop a plate. **Decision needed:** fail-closed vs. keep-warn.
- Note — **STRICT is not the default** (`graph_mode` boots ADVISORY): the graph advises but does not yet interlock off-whitelist moves. The connectivity/one-way gaps that gated the STRICT flip are now closed (orphans commented out of the YAML rather than wired — `WIP_UNREACHABLE_FROM_HOME` is empty; drawer return edge added; reverse guard green), so the flip is unblocked for the currently-wired subgraph. Re-wiring the commented-out stations remains ahead.

**Manual wiring in progress (2026-07-20).** The orphaned stations and the missing drawer return edge are being connected by hand (directly in `src/settings/motion_graph.yaml`; see the edit checklist in that file's header). The guards are deliberately edit-friendly and do **not** lock manual editing:
- the connectivity guard is a *subset* check — wiring a node up simply makes it pass, so no test edit is forced (only a brand-new, not-yet-wired orphan turns it red);
- the loader degrades gracefully — an invalid graph disables the interlock and logs the reason rather than crashing, so the arm stays drivable (raw moves) while iterating;
- the graph-edit API stays claim-gated, but that governs only `/control/graph/*` calls, never direct YAML edits.

As each station is wired, shrink `WIP_UNREACHABLE_FROM_HOME` in `test/test_motion_graph.py` to match. Once the one-way (drawer return edge) and reverse-reachability gaps close, the graph is ready to flip to STRICT.

**Risk:** low (shipped items are validation + tests; the drawer fix is one YAML edge). **Reversible:** each item is independent and additive.

### Step 4 — Flip xArm to v1.1 + expose the control surface — ◑ PARTIAL
✅ Registry flipped to `protocol: "1.1"` (claim dance on); `do_not_call_connect` retained.

**Architecture decision (2026-07-20): the control interface and its API stay on the device (the PC that connects to and runs the arm); the dashboard *displays* that interface rather than reimplementing it. Auth still routes through the central server, but is optional — so the arm is fully usable standalone.** Rationale: a device that owns its own control surface can be installed and operated *without* the dashboard (clone → run → open `/web/`), which the lab wants; the dashboard becomes one consumer of that surface, not a prerequisite. **This supersedes the earlier plan 4a ("add native `graph.*` buttons to `RobotArmTile`").**

Consequences (the device-side half is already shipped on `drive-with-motion-graph`):
- **Three auth modes, config-selected** (`src/core/xarm_api_server.py`): (i) *standalone* — no `AUTH_SIDECAR_URL`/`XARM_EDGE_SHARED_SECRET`, so `require_login` no-ops (`:490`) and `/web/` is open behind the tailnet ACL; (ii) *direct + on-host login* — cookie/email one-time-code or `X-Api-Key` → sidecar; (iii) *edge-fronted SSO* — `XARM_EDGE_SHARED_SECRET` set, so the central Caddy edge authenticates once (`forward_auth → ac_auth`) and injects `X-Auth-User` + `X-Edge-Auth`, which `_edge_identity` (`:729`) trusts only on a constant-time secret match (a directly-reachable device never trusts client-supplied identity headers). ✅ shipped.
- **The panel holds its own claim** (`src/web/graph.js` `ensureClaim`, heartbeated) so a human driving it mutually excludes with an SDK workflow — the device stays the single authority. ✅ shipped.
- **A dashboard-only user (no direct route to the arm) controls it via the mirrored UI *through the edge*.** The dashboard must embed the panel at the **single edge origin under a subpath** (the panel is already base-pathed for this, commit `054e430`), **not** the raw device URL. Same origin ⇒ the user's central-login cookie reaches the edge ⇒ the edge vouches (injects identity) ⇒ the arm trusts it; the user needs no arm-side network access and no second sign-in, and the claim + audit attribute to them. Embedding the raw `…:8000/web/` breaks this (login cookie not sent cross-origin, http-in-https mixed content, no injected identity) — **so the mirror must go through the edge.** ▢ TODO.

▢ Remaining (human control surface):
- **(a) Dashboard embeds the edge-fronted panel** in `RobotArmTile` (iframe/link to the edge subpath), replacing the read-only summary + raw deep-link. **File:** `ac-organic-lab/web/src/components/RobotArmTile.tsx`.
- **(b) Edge config (central repo):** a Caddy `forward_auth → ac_auth` route for the xArm origin that injects `X-Auth-User`/`X-Auth-Role`/`X-Edge-Auth`; set `XARM_EDGE_SHARED_SECRET` on the device NSSM env to match. **Files:** `ac-organic-lab/deploy/` Caddy config; device PC env.
- **(c) Device-side control audit** (ties to Step 2): because panel-driven control reaches the arm directly, bypassing the dashboard passthrough, the arm must emit its own `control_action` — reuse the events exporter to POST `{action, node_id, outcome, owner}` (owner = the edge-injected identity) to `/api/ingest/events`. Today it only `logger.info`s the actor (`xarm_api_server.py:1419`), which rotates away in journald. **Files:** `src/core/events_exporter.py`, control handlers.

▢ Remaining (programmatic control — a *separate* track from the human UI, for SDK/agent driving via `execute_plan` / `lab.skills()`):
- **(d)** Reconcile the catalog (`graph.{move_to,recover_to,record,mode}`) with the device's larger verb set (`travel_to`, `gripper`, `edge*`, `node`) and its advertised `allowed_actions` (currently `move.<node_id>`, STRICT-only). **(e)** Mirror the SkillDefs in `typed_clients/robot_arm.py`. **Files:** `skill_catalog/robot_arm.py`, `typed_clients/robot_arm.py`.

**Risk:** medium (live control) — verify tokenless `/control/graph/*` → 423 and edge-secret-mismatch → ignored first. **Reversible:** remove the embed; unset `XARM_EDGE_SHARED_SECRET`; flip `protocol` back.

### Step 5 — Deterministic veto chokepoint + decision journaling (unlocks C2) — ◑ PARTIAL
✅ `execute_plan` shipped (`skills/src/lab_skills/plan.py:311`, sync `sync.py:161`, MCP `mcp.py:243`): re-runs L3 + L4 before each step. ▢ Remaining: at the gate, emit `transition_proposed` (on submit) + `transition_decision` (`verdict`, `vetoing_layer`, `reason`, `guardrails_mode`); tag runs `guardrails_mode ∈ {off, advisory, strict}`; give PyPoe MCP a *propose/journal* tool (still **no** `control_action` tool — proposals route to the gate, not the device). **Files:** `plan.py` emit calls; PyPoe MCP. **Risk:** low (the execution path exists; this is additive journaling). **Reversible:** journaling is additive.

### Step 6 — Recovery planner with cost model + counterfactual logging (unlocks C1 fully) — ▢ TODO
Node classification `(reversible, sensor_verifiable, interlock_safe)` + cost-based target selection; on each recovery emit `recovery_attempt` with full `extra` (candidates, policy, chosen_cost, **nearest_node counterfactual** — use `/graph/nearest`, unsafe). Keep selection in a `lab-skills` recovery module or the project repo — not in adapters/aggregator. **Reversible:** module is opt-in.

### Step 7 — Vision verification step (unlocks C3) — ▢ TODO
A CV check confirming the arm physically reached the recovery node before resume; emit `vision_verification` correlated by `recovery_id`; gate the resume `graph.move_to` on `vision_verdict=confirmed` (non-`force` path). **Risk:** isolated; fail-closed (reject → no resume). **Reversible:** disable verifier → falls back to telemetry/`force`.

### Step 8 — Run/campaign model generalization (completes C4) — ▢ TODO
Generalize `runs` to non-dosing workflows (nullable dosing fields + a `kind`/`workflow` column) **or** standardize `run_started/finished/failed` events with a shared `run_id` + `failure_cause`. Prefer the event route first (no DDL). **Reversible:** events are additive; a column add is backward-compatible.

### Step 9 — C5 measurement scaffolding (no runtime change) — ▢ TODO
Publish the capability inventory (derive from `SKILL_REGISTRY`, ~10 kinds/~80 SkillDefs); add an optional CI guard asserting project repos import only `lab-skills`. Measure per-workflow diff scope from VCS.

### Prioritization rationale (updated)
- **Step 1 first** (unblocked substrate for the paper), then **Step 2** (cheap, and the last remaining `control_action` gap).
- **Step 5-journaling** is now the keystone for C2 — the execution gate already exists, so only the emit calls + ablation tagging remain.
- **Steps 6–7** complete C1/C3; **Step 8** finishes C4; **Steps 4-remainder + 9** finish the safe control surface and C5 measurement.
- Steps 0, 3, and the registry half of 4 are done. Nothing in the plan bypasses an interlock or the device's authority; the aggregator remains the single journal; no sidecar DB is introduced.
