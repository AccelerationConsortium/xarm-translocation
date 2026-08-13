# Dashboard assistant — whole-motion-graph access (plan)

> **Where the work lands:** every code change in this plan is in the
> **`ac-organic-lab`** repo (the dashboard). **This repo is not modified.** The
> plan lives here because it is entirely about this device's motion graph, and
> because the device-side facts it depends on (`travel_targets`,
> `/control/graph/travel_to`, `GET /graph`) are documented here.
>
> Status: **planned, not implemented.** Drafted 2026-08-12.

## Context

The dashboard lab assistant (Control mode) can only walk the xArm one node at a
time: "move to the UPLC" needs the operator to name each intermediate node, turn
by turn. The on-device assistant on `sdl2-pc-03-cytation`
(`src/core/assistant_llm.py` + `assistant_actions.py`) does it in one shot.

The cause is not model quality — it is the toolset. Three things stack up:

1. This device's `allowed_actions` advertises **1-hop neighbours only**
   (`src/core/status_builder.py:517` → `controller.reachable_node_ids()` →
   `allowed_targets_for_state`). Correct per STATUS_SPEC §6.2, because
   `/control/graph/move_to` genuinely 409s on a non-adjacent node.
2. The dashboard's `_propose_action` hard-gates on `action ∈
   status.allowed_actions` (`api/app/assistant_control.py:308`), so only one-hop
   moves are proposable.
3. Its `_resolve` (`assistant_control.py:157`) maps exactly one pattern,
   `move.<node_id>` → `graph.move_to`, and the SDK catalog has no multi-hop verb.

The assistant also has **no tool that shows it the graph at all** — no nodes, no
tags, no edges — so it cannot reason about routes or resolve "slot 1" to a node.

Outcome wanted: the assistant can see the whole topology, explain routes, and
propose **one** multi-hop travel action that the operator authorizes with one
click. Scope confirmed with the user: **travel + graph reasoning only** (no
pick/place sequences), and **dashboard repo only** (no device redeploy).

## What this device already provides — do not rebuild

The device side needs no changes. It already exposes everything required:

| Need | Already here |
|---|---|
| Multi-hop execution | `POST /control/graph/travel_to` — plans the shortest path and runs it hop-by-hop under one motion reservation, each hop STRICT-validated (`src/core/xarm_api_server.py:2523`, `src/core/xarm_controller.py:2272`) |
| Multi-hop reachability | `details.motion_graph.travel_targets` — *"every node reachable in >= 1 hops with the current gripper state"* (`src/core/status_builder.py:598`), **already on every `/status` poll** |
| Full topology | `GET /graph` — nodes with tags, edges, adjacency, gripper catalog. Un-gated read (`src/core/xarm_api_server.py:2363`) |
| Human place names | `GET /assistant/status` → `places: [{key,label}]`, derived from graph tags (`src/core/xarm_api_server.py:2639`) |

Critically, **`travel_targets` is device-computed** from this device's own graph
and current gripper state — exactly like `reachable_nodes`. Gating on it keeps
the "device is the authority" property that §6.2 gives `allowed_actions`. The
plan does not move path planning into the dashboard; it hands the model a bigger
map and one verb that lets this device do the planning.

## The change (all in `ac-organic-lab`)

### 1. Catalog: add the multi-hop skill

`skills/src/lab_skills/skill_catalog/robot_arm.py` — add alongside the existing
four SkillDefs (same file, same `register("robot_arm", [...])` block):

```python
class GraphTravelToArgs(BaseModel):
    node_id: str = Field(description="Any node reachable through the graph, not just adjacent.")
    speed: float | None = Field(default=None, description="Per-hop speed; each hop may be capped by its edge speed.")
```

`SkillDef(name="graph.travel_to", endpoint="/control/graph/travel_to",
requires_states=["ready"], estimated_duration_s=45.0)`. Update the module
docstring's control-surface list. Export `GraphTravelToArgs` in `__all__`.

`_passthrough_action()` already turns `/control/graph/travel_to` into
`graph/travel_to`, so the browser's Authorize click reaches the existing
passthrough with **no routing change**.

### 2. `api/app/assistant_control.py`: sight, then the travel verb

**a. A device read helper.** Add `_device_get(entry, path, timeout=5.0)` — a
plain `httpx.AsyncClient` GET against `entry.base_url + path`. Reads are un-gated
on this device, same as the aggregator's own poll. Fail closed: any error becomes
a refusal the model relays.

**b. New MCP tool `get_motion_graph(equipment_id)`.** This is the "reason through
it" half. Compose from two sources, deliberately kept separate:

- **Live state** from `/status.details.motion_graph` (via the existing
  `_read_status`) — `current_node`, `gripper_state`, `graph_mode`,
  `reachable_nodes`, `travel_targets`, `allowed_gripper_targets`.
- **Static topology** from `GET /graph` — trimmed hard to `nodes: [{id, tags}]`
  and `edges: [[from, to]]`. Drop `arm`/`rail`/`gripper_transitions`: the model
  does not need preset names, and the untrimmed payload is ~20 KB of tokens per
  call. Cache per `equipment_id` for ~60 s (topology only changes on a graph edit).
- **Optional** `places` from `GET /assistant/status`, best-effort, omitted on any
  failure. Node `tags` already encode station+slot (`[opentrons, slot2]`), so
  naming works without it — this just makes "slot 1" resolution reliable.

Refuse with `unsupported_kind` for any kind other than `robot_arm`.

**c. Gate + resolve `graph.travel_to`.** In `_resolve`, add a branch before the
`move.` one for `action == "graph.travel_to"`, taking `node_id` from `args`.

The gate in `_propose_action` becomes: if the action is `graph.travel_to`, skip
the `action ∈ allowed_actions` check and instead require **both**:

1. **`any(a.startswith("move.") for a in allowed_actions)`** — the device is in a
   state where graph moves are honoured. This one condition reuses this device's
   own composite gate: `_build_allowed_actions` only emits `move.*` when
   connected, not `requires_init`, no active error, `activity != "running"`, not
   Studio-Sim, and `graph_mode == "strict"`. Do **not** reimplement those checks
   dashboard-side.
2. **`node_id ∈ details["motion_graph"]["travel_targets"]`** — device-computed
   multi-hop reachability under the current gripper state.

Missing `details.motion_graph` → refuse (`graph_unavailable`). Everything else
keeps today's `allowed_actions` gate untouched.

**d. Discoverability.** `_list_available_actions` must surface `graph.travel_to`,
or the prompt's "list, then propose" flow never finds it. Append a synthetic
entry when the gate holds, carrying its `args_schema` and the current
`travel_targets`. Tag every entry with `"source": "device"` vs
`"source": "dashboard_gated"` so the surface stays honest about which list an
action came from.

**e. Card context.** For `robot_arm`, add `current_node` and `graph_mode` to the
proposal's `device_state`, so the operator sees *from → to* rather than a bare
destination. This matters more for travel than for a single hop.

### 3. `api/app/control.py`: per-action timeout

**Required, or this ships broken.** `travel_to` blocks until the whole journey
completes, but the dashboard's shared control client has
`_CONTROL_TIMEOUT_SECONDS = 15.0` (`control.py:46`, applied at `:111`). A
multi-hop journey exceeds it, and the operator gets a 504 while the arm is still
moving.

Add `_action_timeout(kind, action) -> httpx.Timeout | None` and pass it to
`client.post(...)` inside `_proxy`'s `_send`. Follow the existing precedent at
`control.py:985`, where the media route already overrides per request. Map
`("robot_arm", "graph/travel_to") -> read=180.0`; return `None` (client default)
for everything else.

### 4. Prompt + card

- `api/app/assistant.py` — extend `CONTROL_PROMPT_ADDENDUM`: `get_motion_graph`
  exists, `graph.travel_to` moves to **any** reachable node in one action, prefer
  it over chaining `move.*`, and never invent a `node_id` outside
  `travel_targets`. Replace the now-wrong line *"Only robot_arm move targets are
  proposable"*.
- `web/src/components/AssistantBubble.tsx` — `Proposal["device_state"]` gains
  optional `current_node`; `ProposalCard` renders a **From** row when present
  (the card's `dl` is a fixed shape today, `:763`). No other UI change — Authorize
  already POSTs `passthrough_action` verbatim.

## Files (all in `ac-organic-lab`)

| File | Change |
|---|---|
| `skills/src/lab_skills/skill_catalog/robot_arm.py` | `GraphTravelToArgs` + `graph.travel_to` SkillDef |
| `api/app/assistant_control.py` | `_device_get`, `get_motion_graph` tool, travel gate in `_propose_action`, `_resolve` branch, synthetic listing entry, `device_state` extras |
| `api/app/control.py` | `_action_timeout` + per-request timeout in `_proxy._send` |
| `api/app/assistant.py` | control prompt addendum |
| `web/src/components/AssistantBubble.tsx` | optional From row |
| `api/tests/test_assistant_control.py` | new cases (below) |
| `docs/UI_DESIGN.md` §5 | record the travel extension |
| `docs/ROADMAP.md` | xArm sub-tasks: note the dashboard half of the skill-name gap |

## Verification

**Unit** — `uv run pytest api/tests skills/tests -q`. Extend the existing
`api/tests/test_assistant_control.py` (already covers refusals) with:

- travel proposed when `move.*` present **and** `node_id ∈ travel_targets`
- refused when `travel_targets` lacks the node (`not_allowed`)
- refused when no `move.*` is advertised — cover `requires_init`, `activity:
  "running"`, and non-STRICT `graph_mode`, since all three collapse to that one
  condition
- refused when `details.motion_graph` is absent (`graph_unavailable`)
- `get_motion_graph` trims to `{id, tags}` / `[from, to]` and survives a `GET
  /graph` failure and a missing `/assistant/status`
- existing single-hop `move.<node>` behaviour unchanged (regression)

**Web** — `pnpm test && pnpm typecheck` in `web/`.

**Live** (from the dashboard host; arm connected and in STRICT — otherwise
nothing lights up, by design):

1. `curl http://sdl2-pc-03-cytation.tail6a1dd7.ts.net:8000/graph` — confirm it
   answers un-authenticated from the dashboard host.
2. `curl .../status | jq '.details.motion_graph.travel_targets'` — non-empty.
3. Bubble → **Control** mode → *"where can the arm go from here, and how would it
   get to the UPLC drawer?"* — expect a route explanation with no proposal.
4. *"take the arm to the UPLC drawer"* — expect **one** confirm card showing
   From → To, one Authorize click, arm completes the whole journey.
5. Audit: an `assistant_proposal` event plus a `control_action` row with
   `origin: assistant` and a `duration_s` matching the real journey.

## Risks and non-goals

- **Long-journey claim expiry.** The dashboard passthrough claims → acts →
  releases with no heartbeat, so a journey longer than this device's claim TTL
  leaves a stale claim until it expires. Not a safety hole: `require_claim` is
  checked at request entry, and `reserve_motion()` refuses any concurrent move
  with 409 `motion_in_progress`. Release is already best-effort. Confirm the
  observed TTL in step 5.
- **The card shows destination, not the hop list.** Computing a preview path
  would mean a second path planner in the dashboard that can disagree with this
  device's — the exact duplication this plan avoids. The model can narrate its
  expected route from `get_motion_graph` (subordinate prose, per UI_DESIGN §5.3).
  If that proves insufficient, the right fix is a **device-side read-only
  path-preview endpoint** (a non-LLM sibling of `/assistant/plan` returning
  `plan_path()` output) — out of scope here, but this repo is where it would land.
- **Still one action per proposal.** Pick/place stays out (user-confirmed), so
  UI_DESIGN §5.3's "no batches, no sequences" and the unapproved §5.5 Step 2 are
  untouched. `graph.travel_to` is a single action that happens to be multi-hop
  *inside the device* — it fits the existing policy rather than bending it.
- **This repo untouched**, so the ROADMAP's `move.<node_id>` vs `graph.*`
  skill-name mismatch survives on the wire. The plan works around it in
  `_resolve` rather than fixing it. Advertising `graph.travel_to` in this
  device's own `allowed_actions` (a `status_builder.py` change) remains the
  cleaner long-term fix and would also close that ROADMAP item.
