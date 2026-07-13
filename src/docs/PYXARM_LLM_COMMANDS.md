# Natural-Language Commands (LLM)

This document describes the natural-language command layer that lets an
operator drive the arm with plain-English instructions such as:

- "Go to deck home"
- "Go to deck high"
- "Pick up the tray from deck slot 1"

It is powered by Anthropic's **Claude Haiku** and sits *on top of* the
existing motion-graph interlock — it never bypasses it. The LLM only
*interprets* language into a structured intent; a deterministic validator
gates that intent against the graph, and execution reuses the same
graph-gated control paths as the Drive Arm card.

---

## Table of Contents

- [Design principles](#design-principles)
- [Architecture](#architecture)
- [The two-phase flow](#the-two-phase-flow)
- [Intent and plan model](#intent-and-plan-model)
- [Configuration](#configuration)
- [API reference](#api-reference)
- [Web UI](#web-ui)
- [v1 limitations](#v1-limitations)
- [Extending it](#extending-it-future-work)
- [Files](#files)

---

## Design principles

1. **The LLM never commands the robot directly.** It returns a structured
   *intent* (which node to move to, or which gripper state to set). It has
   no ability to move the arm.
2. **The graph is the source of truth for safety.** A deterministic,
   LLM-free validator (`plan_from_intent`) decides whether an intent is
   executable. If the graph says a move isn't allowed, no prompt wording
   can override it.
3. **Human-in-the-loop.** Interpretation and execution are separate
   requests. The operator sees the interpreted plan and clicks **Confirm**
   before anything moves.
4. **Re-validated at execution.** The plan is checked again against live
   state immediately before each step runs, so a stale plan (the arm moved
   since interpretation) is rejected rather than executed blindly.
5. **Graceful degradation.** With no `ANTHROPIC_API_KEY` set, the feature
   reports itself unavailable and the rest of the system is unaffected.

---

## Architecture

```
 operator text
      │
      ▼
 POST /control/nl/interpret
      │
      ├─►  NLCommandInterpreter  ──►  Claude Haiku (forced tool-use)
      │         (LLM)                   returns a structured NLIntent
      │
      ├─►  plan_from_intent  (deterministic, graph-validated)  ──►  NLPlan
      │
      ▼
 UI preview  ──(operator clicks Confirm)──►  POST /control/nl/execute
                                                   │
                                                   ├─ force STRICT mode
                                                   ├─ re-validate each step
                                                   └─ move_to_node / set_gripper_state
```

Key modules:

- `src/core/nl_command.py` — interpreter, intent/plan types, validator.
- `src/core/xarm_api_server.py` — the `/control/nl/*` endpoints.
- `src/web/index.html` + `src/web/main.js` — the Language Command card.

---

## The two-phase flow

The feature is deliberately split into **interpret** and **execute** so a
human confirms before motion:

1. **Interpret** (`POST /control/nl/interpret`): the utterance goes to the
   LLM, which returns an intent. The validator turns it into a plan and
   reports whether it's executable and why. No motion happens.
2. **Confirm & execute** (`POST /control/nl/execute`): the operator sends
   the validated plan back. The server forces STRICT enforcement,
   re-validates every step against the current state, and runs it.

---

## Intent and plan model

### `NLIntent`

The LLM's structured interpretation of one command:

| Field         | Meaning |
|---------------|---------|
| `action`      | `move_to`, `set_gripper`, or `none` |
| `target`      | node id (for `move_to`) or gripper-state name (for `set_gripper`); `null` for `none` |
| `explanation` | one-sentence, operator-facing description of the interpretation |
| `error`       | why the command couldn't be mapped (only when `action` is `none`) |

The LLM is constrained by a **forced tool-use schema** (`robot_command`),
so the response is always structured JSON, never free-form prose. The
prompt is seeded with the full node catalog (ids + tags) and the
gripper-state catalog so it can fuzzy-match loose phrasing like "deck slot
1" to a real node id such as `deck_slot1_high`.

### `NLPlan`

The validated, executable result:

| Field         | Meaning |
|---------------|---------|
| `steps`       | ordered list of `{type, target}` steps |
| `explanation` | carried over from the intent |
| `valid`       | whether the plan can run now |
| `reason`      | human-readable rejection/summary text |

`steps` is a **list** even though v1 always produces exactly one step —
this is the extension point for multi-hop planning (see below).

### Validation rules (v1)

`plan_from_intent` accepts a step only when it is **directly available**
from the current state:

- **`move_to`**: the target must be a real node **and** appear in the
  controller's `reachable_node_ids()` (a direct outgoing edge traversable
  with the current gripper state).
- **`set_gripper`**: the target must be a real catalog state **and** appear
  in `allowed_gripper_targets()` (a whitelisted transition at the current
  node).
- Anything else (unknown target, off-grid, already-there, not directly
  reachable) is returned with `valid: false` and an explanatory `reason`.

---

## Configuration

Configured entirely through environment variables on the **server**:

| Variable            | Required | Default            | Purpose |
|---------------------|----------|--------------------|---------|
| `ANTHROPIC_API_KEY` | Yes      | —                  | Anthropic API key. Without it, the feature is unavailable. |
| `XARM_LLM_MODEL`    | No       | `claude-haiku-4-5` | Model id/alias to use. |

The `anthropic` Python package is a project dependency (`pyproject.toml`).
Install with the rest of the package:

```bash
pip install -e .
```

Example:

```bash
export ANTHROPIC_API_KEY="sk-ant-..."
# optional
export XARM_LLM_MODEL="claude-haiku-4-5"
pyxarm web
```

If the key is missing the server still starts normally; the Language
Command card shows "LLM not configured" and the `/control/nl/*` interpret
endpoint returns `503`.

---

## API reference

All three endpoints live under the existing control surface. `interpret`
and `execute` are **claim-gated** (require `X-Claim-Token`) like every
other control endpoint; `status` is not.

### `GET /control/nl/status`

Advertises availability so the UI can adapt.

```json
{ "available": true, "model": "claude-haiku-4-5" }
```

### `POST /control/nl/interpret`

Interpret a command into a validated plan. No motion.

Request:

```json
{ "text": "pick up the tray from deck slot 1" }
```

Response (`200`):

```json
{
  "intent": {
    "action": "set_gripper",
    "target": "grip_120",
    "explanation": "Gripping the tray at deck slot 1.",
    "error": null
  },
  "plan": {
    "steps": [ { "type": "set_gripper", "target": "grip_120" } ],
    "explanation": "Gripping the tray at deck slot 1.",
    "valid": true,
    "reason": "Set gripper to 'grip_120' at 'deck_slot1_low'."
  },
  "valid": true,
  "reason": "Set gripper to 'grip_120' at 'deck_slot1_low'."
}
```

Status codes:

- `200` — interpreted (may still be `valid: false` with a `reason` when the
  command can't run right now).
- `404` — no motion graph loaded.
- `423` — claim not held.
- `503` — LLM not configured (`{"error": "llm_unavailable"}`).
- `502` — the LLM call failed (`{"error": "llm_error"}`).

### `POST /control/nl/execute`

Execute a plan previously returned by `interpret`. Forces STRICT mode,
re-validates each step against live state, then runs it.

Request:

```json
{ "steps": [ { "type": "move_to", "target": "deck_high" } ] }
```

Response (`200`):

```json
{
  "executed": [ { "type": "move_to", "target": "deck_high", "ok": true } ],
  "current_node": "deck_high",
  "gripper_state": "empty"
}
```

Status codes:

- `200` — all steps executed.
- `409` — a step is no longer valid (`{"error": "plan_invalid", ...}`), or
  an edge/gripper transition was rejected by the graph.
- `422` — empty plan or unsupported step type.
- `423` — claim not held.
- `500` — a step's actuation/verification failed.

---

## Web UI

The **Language Command** card sits directly below the Drive Arm card. It
appears whenever a motion graph is loaded and is gated exactly like the
Drive Arm card (connected, claim held, not in manual mode, not moving),
plus a live check that the LLM is configured.

Flow:

1. Type a command and click **Interpret** (or press Enter).
2. The card shows the interpretation and the resolved plan. A valid plan
   reveals **Confirm & Run** / **Cancel**; an invalid one shows the reason.
3. **Confirm & Run** executes the plan; **Cancel** discards it.

A pending plan is automatically discarded if the current graph node changes
underneath it (the interpretation would be stale).

---

## v1 limitations

- **Single hop only.** A command must resolve to a node that is *directly*
  reachable, or a gripper transition allowed *at the current node*. For
  example, from `robot_home` the command "pick up the tray on deck slot 1"
  is understood but reported not directly reachable, because it requires
  traversing `deck_home → deck_high → deck_slot1_high → deck_slot1_low`,
  gripping, and returning.
- **One action per command.** No compound instructions ("do X then Y").
- **No task-level memory.** Each command is interpreted against the current
  state only.

---

## Extending it (future work)

The seams are deliberate:

- **Multi-hop path planning.** Replace the "must be a direct neighbor"
  check in `plan_from_intent` with a BFS/Dijkstra search over the graph
  (respecting gripper-state occupancy on edges). Because `NLPlan.steps` is
  already a list and `/control/nl/execute` already loops over and
  re-validates each step, neither the API nor the UI needs to change.
  Example target flow for "pick up the tray on deck slot 1" from
  `robot_home`:
  `deck_home → deck_high → deck_slot1_high → deck_slot1_low`,
  `set_gripper grip_120`, then `deck_slot1_low → deck_slot1_high`.
- **Workflow macros.** Add higher-level intents (e.g. `dose_solid`) that
  expand into multi-step plans, still validated by the same graph gate.
- **Richer intent types.** The `action` enum and the tool schema in
  `nl_command.py` are the single place to add new verbs.
- **Batch confirmation / dry-run preview** of multi-step plans in the UI.

---

## Files

| File | Role |
|------|------|
| `src/core/nl_command.py` | `NLIntent`, `NLStep`, `NLPlan`, `NLCommandInterpreter`, `build_context`, `plan_from_intent` |
| `src/core/xarm_api_server.py` | `/control/nl/status`, `/control/nl/interpret`, `/control/nl/execute` |
| `src/web/index.html` | Language Command card markup |
| `src/web/main.js` | interpret/confirm logic and gating |
| `test/test_nl_command.py` | validator + interpreter (mocked LLM) tests |
