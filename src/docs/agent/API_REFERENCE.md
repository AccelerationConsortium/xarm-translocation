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

## RealSense — discovering the cameras

Every camera on this device PC has a device-local id (`rs435i` is the first
one) and every route for it is nested under that id. Start here; the listing
hands back a ready-made URL per route, so nothing downstream has to build a
path by concatenation.

| Method | Path | Gate | Returns |
|---|---|---|---|
| GET | `/realsense/cameras` | open | `{cameras: [...], default, reason}` — never 404s; an unconfigured service answers with an empty list and a `reason` |

```json
{
  "cameras": [
    {
      "id": "rs435i",
      "label": "xArm depth camera (D435i, eye-in-hand)",
      "state": "off",
      "streaming": false,
      "start_on_demand": true,
      "device": null,
      "urls": {
        "status": "/realsense/rs435i/status",
        "snapshot": "/realsense/rs435i/snapshot.jpg",
        "depth_png": "/realsense/rs435i/depth.png",
        "stream": "/realsense/rs435i/stream.mjpg",
        "depth": "/realsense/rs435i/depth",
        "intrinsics": "/realsense/rs435i/intrinsics",
        "captures": "/realsense/rs435i/captures",
        "capture": "/control/realsense/rs435i/capture"
      }
    }
  ],
  "default": "rs435i",
  "reason": null
}
```

`default` is the id you may omit naming. It is the sole camera's id when
exactly one is configured, and `null` as soon as there are two — with two
lenses there is nothing sensible to assume.

Naming a camera that is not configured returns **404**
`camera_not_found` with the ids that do exist:

```json
{"detail": {"error": "camera_not_found", "camera_id": "nope",
            "cameras": ["overhead", "rs435i"]}}
```

A service with no camera configured at all answers **404**
`realsense_not_configured` with a `hint`, on every route below. The two are
different facts: the feature is off, versus you named the wrong lens.

## RealSense — transient reads

All nested under the camera id. `{id}` below is a camera id, not a capture id.

| Method | Path | Gate | Notes |
|---|---|---|---|
| GET | `/realsense/{id}/status` | open | device list, pipeline state, stream profiles; answers before `/connect` |
| GET | `/realsense/{id}/intrinsics` | open | pinhole intrinsics per stream plus depth scale; populated while streaming |
| GET | `/realsense/{id}/depth?x=&y=&window=` | open | metres plus a camera-frame 3-D point. `window` is an odd median patch, default 5. **409** when the pipeline is stopped — this read never starts it |
| POST | `/realsense/{id}/start` | login | **503** when the extra or hardware is missing |
| POST | `/realsense/{id}/stop` | login | |
| GET | `/realsense/{id}/snapshot.jpg?stream=` | login | `stream` is `color` (default) or `depth` for the colourised map |
| GET | `/realsense/{id}/depth.png` | login | raw 16-bit depth, lossless |
| GET | `/realsense/{id}/stream.mjpg?stream=&fps=` | login | multipart MJPEG; keep it opt-in, it costs campus bandwidth |

## RealSense — capture records

| Method | Path | Gate | Notes |
|---|---|---|---|
| POST | `/control/realsense/{id}/capture` | claim | canonical; body below, starts the pipeline on demand |
| POST | `/control/realsense/capture` | claim | the alias: same body plus `camera`. See below |
| GET | `/realsense/captures` | open | `?limit=&node_id=&label=&since=` — every camera, newest first, metadata only; each record carries `camera_id` |
| GET | `/realsense/{id}/captures` | open | the same listing, narrowed to one camera |
| GET | `/realsense/{id}/captures/{capture_id}` | open | that capture's `meta.json` |
| GET | `/realsense/{id}/captures/{capture_id}/color.jpg` | login | the colour frame |
| GET | `/realsense/{id}/captures/{capture_id}/depth.png` | login | the raw 16-bit depth frame |
| DELETE | `/control/realsense/{id}/captures/{capture_id}` | claim | removes it, including a protected one |

### The fixed-path alias

`POST /control/realsense/capture` takes the camera in the **body** instead of
the path, and exists for one reason: a SkillDef carries a single fixed
`endpoint` string, and both the lab's skill executor
(`lab_skills.plan.execute_plan`) and the dashboard passthrough send it
verbatim — there is no path templating anywhere in that chain, so a plan
cannot express a camera id as a path segment. A fixed path with the camera in
the body can. Everything else (the panel, the URLs in capture responses, this
document) uses the nested route, which is canonical; the alias only resolves a
name and delegates to it.

| `camera` in the body | Result |
|---|---|
| omitted, one camera configured | that camera |
| omitted, two or more configured | **400** `{"error": "camera_required", "cameras": [...]}` |
| a configured id | that camera |
| an unknown id | **404** `{"error": "camera_not_found", "cameras": [...]}` |

On the nested route the path wins: a `camera` field in the body is ignored.

### POST /control/realsense/{id}/capture

```json
{
  "label": "plate-arrival-check",
  "node_id": "hood_deck_1",
  "tags": ["reference", "plate"],
  "protected": false
}
```

All fields optional. `node_id` defaults to the arm's current graph node.
`protected: true` exempts the capture from both retention bounds. `camera` is
accepted too, but only the alias reads it.

Response:

```json
{
  "capture_id": "20260920T014233Z-9f3c1a20",
  "camera_id": "rs435i",
  "urls": {
    "meta": "/realsense/rs435i/captures/20260920T014233Z-9f3c1a20",
    "color": "/realsense/rs435i/captures/20260920T014233Z-9f3c1a20/color.jpg",
    "depth": "/realsense/rs435i/captures/20260920T014233Z-9f3c1a20/depth.png"
  },
  "meta": { "...": "see below" }
}
```

`meta.json` carries `capture_id`, `camera_id`, `captured_at` (UTC), `label`, `tags`,
`protected`, `requested_by`, `files` (per-file size and SHA-256), `camera`
(id, label, device serial and firmware, library version, stream profiles),
`frame` (`frame_number`, `timestamp_ms`, `depth_scale_m`,
`aligned_depth_to_color`), `intrinsics`, and `arm` (`connected`, `node_id`,
`joints`, `position`, `track_position`, `gripper_state`).

Refusals: **400** `camera_required` on the alias with several cameras ·
**404** captures disabled, no camera configured, or an unknown camera id ·
**409** camera stopped and `start_on_demand` is false · **423** no claim ·
**500** the store could not write · **503** extra or hardware missing.

### Storage and retention

Captures live outside the repo at
`C:\SDL_Data\xarm\realsense\<camera_id>\<YYYY-MM-DD>\<capture_id>\`.
The camera is the top level, then the UTC day, then the capture.

| Bound | Value |
|---|---|
| `keep_days` | 30 |
| `keep_max_gb` | 20 |

Both are enforced after every write, oldest first, age before size — and
across **all** cameras together, not per camera: one root, one budget, because
the bound that matters is the disk's. A capture is roughly 250 KB at the
configured 1280x720 (colour JPEG ~110 KB + 16-bit depth PNG ~145 KB, measured
2026-09-19; scene-dependent), so 20 GB holds about 80k captures. Captures
marked `protected` are exempt from both.
New files replicate nightly to
`/home/sdl2/storage/external/realsens_xarm/<camera_id>/<YYYY-MM-DD>/` on the
lab data server, mirroring the source layout.

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
| 400 | the alias needs a `camera` and more than one is configured |
| 401 | login required |
| 404 | no camera configured, unknown camera id, or unknown capture |
| 409 | motion in flight, or camera pipeline stopped |
| 412 | safety gate: sash not parked, simulator guard, vision rejected |
| 422 | target not whitelisted from the current node |
| 423 | claim required, or held by another session |
| 500 | capture store write failure |
| 503 | RealSense extra or hardware missing |
