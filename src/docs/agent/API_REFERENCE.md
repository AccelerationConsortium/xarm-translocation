# xArm translocation — API reference

Every route this service exposes, with its gate, body and refusal codes. Gates
are defined in the [agent guide](agent-docs): **open** needs nothing, **login**
needs a session cookie or `X-Api-Key`, **claim** needs `X-Claim-Token`.

Base URL: `http://sdl2-pc-03-cytation.tail6a1dd7.ts.net:8000`.

## Status and discovery

| Method | Path | Gate | Returns |
|---|---|---|---|
| GET | `/status` | open | STATUS_SPEC v1.1 envelope: `equipment_status`, `activity`, `allowed_actions`, `components`, `details` |
| GET | `/openapi.json` | open | OpenAPI schema |
| GET | `/agent-docs` | open | the agent guide, Markdown |
| GET | `/agent-docs/api-reference` | open | this document, Markdown |
| GET | `/llms.txt` | open | discovery index |

## Claims

| Method | Path | Gate | Body | Notes |
|---|---|---|---|---|
| POST | `/control/claim` | open | `{owner, session_id?, ttl_s?}` | **409** when held by someone else; returns the claim token |
| POST | `/control/heartbeat` | open | `{session_id}` | renews the TTL |
| POST | `/control/release` | open | `{session_id}` | drops the claim |

## Connection and safety

| Method | Path | Gate | Notes |
|---|---|---|---|
| POST | `/connect` | login | instantiates the controller; required out of `requires_init` |
| POST | `/disconnect` | login | |
| POST | `/control/stop` | login | safety floor, always available while reachable |
| POST | `/control/clear_errors` | login | |

## Motion graph

All claim-gated. Targets come from `allowed_actions` as `move.<node_id>`.

| Method | Path | Body | Refusals |
|---|---|---|---|
| POST | `/control/graph/move_to` | `{node_id, ...}` | **409** motion in flight · **412** sash interlock · **422** edge not whitelisted |
| POST | `/control/graph/travel_to` | `{node_id}` | multi-hop; same refusals |
| POST | `/control/graph/gripper` | `{state}` | **409** arm must be stationary · **422** state not reachable from here |
| POST | `/control/graph/recover_to` | `{node_id, force?}` | **412** when vision verification rejects |
| POST | `/control/graph/mode` | `{mode}` | `off` · `advisory` · `strict` |
| POST | `/control/graph/record` | | **412** on any simulator · **409** with no last transition |
| GET | `/graph` | open | nodes, edges, current node, reachable targets |

## RealSense — transient reads

| Method | Path | Gate | Notes |
|---|---|---|---|
| GET | `/realsense/status` | open | device list, pipeline state, stream profiles; answers before `/connect` |
| GET | `/realsense/intrinsics` | open | pinhole intrinsics per stream plus depth scale; populated while streaming |
| GET | `/realsense/depth?x=&y=&window=` | open | metres plus a camera-frame 3-D point. `window` is an odd median patch, default 5. **409** when the pipeline is stopped — this read never starts it |
| POST | `/realsense/start` | login | **503** when the extra or hardware is missing |
| POST | `/realsense/stop` | login | |
| GET | `/realsense/snapshot.jpg?stream=` | login | `stream` is `color` (default) or `depth` for the colourised map |
| GET | `/realsense/depth.png` | login | raw 16-bit depth, lossless |
| GET | `/realsense/stream.mjpg?stream=&fps=` | login | multipart MJPEG; keep it opt-in, it costs campus bandwidth |

## RealSense — capture records

| Method | Path | Gate | Notes |
|---|---|---|---|
| POST | `/control/realsense/capture` | claim | body below; starts the pipeline on demand |
| GET | `/realsense/captures` | open | `?limit=&node_id=&label=&since=` — newest first, metadata only |
| GET | `/realsense/captures/{id}` | open | that capture's `meta.json` |
| GET | `/realsense/captures/{id}/color.jpg` | login | the colour frame |
| GET | `/realsense/captures/{id}/depth.png` | login | the raw 16-bit depth frame |
| DELETE | `/control/realsense/captures/{id}` | claim | removes it, including a protected one |

### POST /control/realsense/capture

```json
{
  "label": "plate-arrival-check",
  "node_id": "hood_deck_1",
  "tags": ["reference", "plate"],
  "protected": false
}
```

All fields optional. `node_id` defaults to the arm's current graph node.
`protected: true` exempts the capture from both retention bounds.

Response:

```json
{
  "capture_id": "20260920T014233Z-9f3c1a20",
  "urls": {
    "meta": "/realsense/captures/20260920T014233Z-9f3c1a20",
    "color": "/realsense/captures/20260920T014233Z-9f3c1a20/color.jpg",
    "depth": "/realsense/captures/20260920T014233Z-9f3c1a20/depth.png"
  },
  "meta": { "...": "see below" }
}
```

`meta.json` carries `capture_id`, `captured_at` (UTC), `label`, `tags`,
`protected`, `requested_by`, `files` (per-file size and SHA-256), `camera`
(label, device serial and firmware, library version, stream profiles),
`frame` (`frame_number`, `timestamp_ms`, `depth_scale_m`,
`aligned_depth_to_color`), `intrinsics`, and `arm` (`connected`, `node_id`,
`joints`, `position`, `track_position`, `gripper_state`).

Refusals: **404** captures disabled or camera not configured · **409** camera
stopped and `start_on_demand` is false · **423** no claim · **500** the store
could not write · **503** extra or hardware missing.

### Storage and retention

Captures live outside the repo at `C:\SDL_Data\xarm\realsense\<YYYY-MM-DD>\<capture_id>\`.

| Bound | Value |
|---|---|
| `keep_days` | 30 |
| `keep_max_gb` | 20 |

Both are enforced after every write, oldest first, age before size. A capture
is roughly 0.5 MB at 640x480. Captures marked `protected` are exempt from both.
New files replicate nightly to `/home/sdl2/storage/external/realsens_xarm/` on
the lab data server.

## Camera tracking (network PTZ camera, not the RealSense)

| Method | Path | Gate | Notes |
|---|---|---|---|
| GET | `/camera/config` | open | |
| POST | `/camera/follow` | login | pan the lab camera to follow the arm |
| POST | `/camera/ptz` | login | |
| POST | `/camera/preset` | login | |

## Refusal codes

| Code | Meaning |
|---|---|
| 401 | login required |
| 409 | motion in flight, or camera pipeline stopped |
| 412 | safety gate: sash not parked, simulator guard, vision rejected |
| 422 | target not whitelisted from the current node |
| 423 | claim required, or held by another session |
| 500 | capture store write failure |
| 503 | RealSense extra or hardware missing |
