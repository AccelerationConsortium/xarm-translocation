# Motion-Graph Web Viewer + Edge Editor — Implementation Plan

**Status:** plan, not yet built. Author on the device PC.
**Scope decided 2026-06-04.**

A new **read-only graph viewer** that shows the motion graph, highlights
where the arm currently is (live), and shows whether it's holding labware —
plus the ability to **edit an existing edge's `mode` (joint/linear) and
`speed`** from the canvas. Nothing else is editable here.

---

## 1. Scope

**In scope**
- Visual canvas of `motion_graph.yaml`: nodes grouped by station, directional edges.
- Live "where is the robot": highlight `current_node`; flash the edge as it traverses.
- Labware indicator: badge a node when its `payload != empty` (you watch it flip across a grip edge).
- Edit an **existing** edge's `mode` and `speed`; save → validate → write YAML → hot-reload.

**Out of scope (stays where it is)**
- Editing joint values / poses → keep in the control panel ("capture from live arm").
- Adding/deleting nodes or edges → keep the existing `record` (append-by-demonstration) flow.
- Editing `preconditions`, `grip`/`release` actions → optional later extension (see §7).

**Why a separate page (not in the control panel):** editing only edge
attributes needs no jog controls, so the canvas wants the full viewport.
The one mutation (edge `mode`/`speed`) needs only a claim token, which the
page can acquire on its own.

---

## 2. Architecture

```
browser ──GET /graph.html, /graph.js, cytoscape.min.js (static)──► web/server.py :6001
        ──GET /graph, /ws, POST /control/graph/edge (proxied)────► xarm_api_server :8000
```

- `src/web/server.py` is a static file server + dumb proxy to `:8000`.
- New page is just another static file it already serves; only the **proxy
  allowlist** needs `/graph` added (it already lists `/ws`).
- Cytoscape.js is **vendored locally** (lab PC may have no/limited internet on
  the Tailnet) — do NOT rely on a CDN.

---

## 3. Backend changes (`src/core/xarm_api_server.py`)

### 3.1 Extend `GET /graph` to include full edge data
`get_graph_state()` (`:1494`) currently returns `adjacency` (node → target ids)
but not edge attributes. Add an `edges` list and node detail so the viewer can
render + edit without a second call:

```python
return {
    "graph_mode": c.graph_mode.value,
    "current_node": c.current_node,
    "reachable_nodes": c.reachable_node_ids(),
    "declared_payload": c.declared_payload,
    "arm_pose_name": c.last_arm_pose_name,
    "rail_location_name": c.last_rail_location_name,
    "last_transition": c.last_transition,
    "adjacency": c.motion_graph.adjacency_summary(),
    # NEW:
    "nodes": [
        {"id": n.id, "arm": n.arm, "rail": n.rail,
         "gripper": n.gripper, "payload": n.payload, "tags": list(n.tags)}
        for n in c.motion_graph.nodes
    ],
    "edges": [
        {"from": e.from_node, "to": e.to_node, "mode": e.mode.value,
         "speed": e.speed, "grips": e.grip is not None,
         "releases": e.release is not None,
         "preconditions": list(e.preconditions), "comment": e.comment}
        for e in c.motion_graph.edges
    ],
}
```
(`MotionGraph.nodes` / `.edges` properties already exist — `motion_graph.py:387,391`.)

### 3.2 New claim-gated edit endpoint
Mirror the `record` pattern but for an **in-place** edit of an existing edge.

```python
class GraphEdgeUpdateRequest(BaseModel):
    from_node: str
    to_node: str
    mode: Optional[str] = None   # "joint" | "linear"; None = leave unchanged
    speed: Optional[float] = Field(default=None, gt=0)

@app.post("/control/graph/edge", dependencies=[Depends(require_claim)])
async def update_graph_edge(request: GraphEdgeUpdateRequest):
    c = get_controller()
    if c.motion_graph is None:
        raise HTTPException(404, "motion_graph.yaml not loaded")
    if c.motion_graph.find_edge(request.from_node, request.to_node) is None:
        raise HTTPException(404, f"no edge {request.from_node!r} -> {request.to_node!r}")
    try:
        new_graph = _update_edge_in_yaml(request)   # see 3.3
    except GraphError as exc:
        raise HTTPException(400, f"edit failed validation: {exc}")
    c.motion_graph = new_graph
    edge = new_graph.find_edge(request.from_node, request.to_node)
    return {"updated": {"from": edge.from_node, "to": edge.to_node,
                        "mode": edge.mode.value, "speed": edge.speed}}
```

- `find_edge` already exists (`motion_graph.py:436`).
- Claim-gating is automatic via `Depends(require_claim)` — same as every other
  mutating endpoint (the single-gate rule).

### 3.3 Comment-preserving writer with `ruamel.yaml`
`record`'s `_append_edge_to_yaml` (`:1687`) preserves comments only because it
*appends text*. An in-place edit must rewrite a value mid-file. **`PyYAML`
re-dump would wipe every comment** in `motion_graph.yaml` (the "you hear a
click" notes, the commented station templates). Use `ruamel.yaml` round-trip:

```python
def _update_edge_in_yaml(req) -> "MotionGraph":
    from ruamel.yaml import YAML
    yaml_rt = YAML()                      # round-trip mode (default)
    yaml_rt.preserve_quotes = True
    path = os.path.join("src", "settings", "motion_graph.yaml")
    with open(path) as fh:
        data = yaml_rt.load(fh)

    # locate the edge by (from, to) — unique per loader invariant
    target = next((e for e in data["edges"]
                   if e["from"] == req.from_node and e["to"] == req.to_node), None)
    if target is None:
        raise GraphError(f"no edge {req.from_node!r} -> {req.to_node!r}")
    if req.mode is not None:
        target["mode"] = req.mode
    if req.speed is not None:
        target["speed"] = req.speed

    # validate a plain-dict candidate BEFORE writing (reuse the loader's rules)
    import yaml as _pyyaml
    from .motion_graph import MotionGraph, DEFAULT_PRECONDITIONS
    plain = _pyyaml.safe_load(_dump_to_str(yaml_rt, data))
    MotionGraph.from_dict(plain, preconditions=DEFAULT_PRECONDITIONS)  # raises GraphError

    with open(path, "w") as fh:           # validated → commit
        yaml_rt.dump(data, fh)
    return MotionGraph.from_yaml(path, preconditions=DEFAULT_PRECONDITIONS)
```
> `mode`/`speed` don't touch the three coherence rules (those are payload/grip
> only), so validation mainly catches a bad `mode` string via the `MoveMode`
> enum and re-confirms the file still loads. Validate anyway — cheap insurance.

### 3.4 Add the dependency
`pyproject.toml` (`:34`, alongside `PyYAML>=6.0`):
```
"ruamel.yaml>=0.18",
```
Then `uv sync` (or the device's install path per DEVICE_PC_SETUP.md §3.3).

---

## 4. Proxy fix (`src/web/server.py`)
Add `/graph` to the `api_paths` allowlist (`:27`) so both `GET /graph` and
`POST /control/graph/edge` proxy through to `:8000`:

```python
api_paths = [
    '/api', '/status', '/locations', '/track',
    '/connect', '/disconnect', '/move', '/clear', '/gripper', '/ws',
    '/graph', '/control',           # <-- add
]
```
(`/control` also covers `/control/graph/edge`, `/control/claim`, etc.)

---

## 5. Frontend (new files under `src/web/`)

| File | Purpose |
|------|---------|
| `graph.html` | Standalone page: `<div id="cy">` canvas + edge-edit side panel + claim chip. |
| `graph.js`   | Fetch `/graph`, build Cytoscape elements, subscribe to `/ws`, handle edge edits. |
| `cytoscape.min.js` | **Vendored** (download once, commit). No CDN. |
| `graph.css` (or reuse `style.css`) | Canvas + panel styling. |

Add a link from `index.html` header (e.g. next to the claim chip) → `graph.html`,
and a "← Control panel" link back.

### 5.1 Render
- `cytoscape({ container: #cy, layout: 'breadthfirst' or 'dagre', ... })` —
  `breadthfirst`/`dagre` suit the per-station ladders. (dagre needs the
  `cytoscape-dagre` + `dagre` libs vendored too; `breadthfirst` is built-in —
  **start with `breadthfirst`** to avoid extra vendoring.)
- Group/color nodes by first `tag` (station). Node label = id.
- Edge label = `mode @ speed` (e.g. `linear @ 15`). Style `grips`/`releases`
  edges distinctly (thicker / colored) — read from the new edge fields.

### 5.2 Live state
- Open the existing `/ws` socket. Its status payload carries
  `details.motion_graph` (per `get_graph_state` docstring). On each message:
  - add class `current` to the node == `current_node` (pulse/emerald).
  - if `declared_payload != "empty"`, show a plate badge on that node.
  - optionally flash the edge between previous and new `current_node`.
- Fallback: poll `GET /graph` every ~1 s if the socket drops.

### 5.3 Edit an edge
- On edge tap → side panel: `mode` `<select>` (joint/linear), `speed` `<input number>`, Save.
- Save needs a claim. Reuse the control panel's claim flow (the page can call
  `POST /control/claim` to get an `X-Claim-Token`, or read an existing one).
  **Decision:** acquire a short-lived claim on first edit, attach
  `X-Claim-Token` to `POST /control/graph/edge`, keep it for the session.
  (If a workflow/operator holds the claim elsewhere you'll get 423 — surface
  "device is controlled by <owner>", same as the control panel.)
- On 200, update the in-memory elements + edge label; on 400/404/423 show the error.

---

## 6. Phasing (build/verify in this order)

- **Phase A — viewer only (no writes).** §3.1 (extend `/graph`), §4 (proxy),
  §5.1–5.2 (render + live). Ship this first; it's useful immediately and
  needs no claim, no ruamel, no new mutating endpoint. De-risks the canvas.
- **Phase B — edge editing.** §3.2–3.4 (endpoint + ruamel + dep), §5.3 (panel).

---

## 7. Open decisions (defaults chosen; change if you disagree)

1. **Save timing:** immediate per-edit (matches `record`'s hot-reload). Alt:
   batch + one "Save graph" button. *Default: immediate.*
2. **Edit `preconditions` too?** Out for v1 (needs name-validation against the
   registered set). Easy to add later — the endpoint already round-trips the
   whole edge dict. *Default: mode + speed only.*
3. **Layout persistence (drag positions):** `breadthfirst` auto-layout means no
   manual drag needed. If you later want to drag + keep positions, store them
   in a **sidecar** `src/web/graph.layout.json` (keep `motion_graph.yaml`
   data-clean — do not add cosmetic x/y to the data file). *Default: auto-layout,
   no persistence.*

---

## 8. Testing

- Backend: unit-test `_update_edge_in_yaml` — round-trips a fixture, changes
  `speed`, asserts (a) value changed, (b) a known comment survived, (c) bad
  `mode` raises `GraphError` and leaves the file untouched.
- `GET /graph` shape test: asserts `nodes` + `edges` present with `mode`/`speed`.
- Manual: open `graph.html`, jog the arm via the control panel in another tab,
  confirm `current_node` + payload badge track live; edit an edge speed, confirm
  the YAML changed and the comment block above it is intact.

## 9. Watch-outs

- **Single-gate rule:** the edit endpoint MUST stay behind `require_claim`.
  Don't add an unauthenticated edit path.
- **`/web/` side-door:** this adds another mutation surface on the device's own
  page (already flagged un-audited in `ac-organic-lab` ROADMAP). Edge `mode`/
  `speed` edits are low-blast-radius (can't make the graph incoherent), but note
  it when the audit/edge work lands.
- **Don't re-dump with PyYAML.** Only `ruamel` round-trip preserves the
  comments in `motion_graph.yaml`.
- **`find_edge` / unique edges:** the loader already forbids duplicate
  `(from,to)` pairs, so locating the edge to edit is unambiguous.
