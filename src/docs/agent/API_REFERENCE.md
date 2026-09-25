# xArm translocation — API reference

Every route this service exposes, with its gate, body and refusal codes. Gates
are defined in the [agent guide](../agent-docs): **open** needs nothing, **login**
needs a session cookie or `X-Api-Key`, **claim** needs `X-Claim-Token`, and
**admin** needs a verified identity with role `admin` (no claim required).

Paths are relative to the configured service base, including its mount prefix.
OpenAPI is authoritative for request and response schemas. Transport
documentation does not authorize hardware execution; see the agent guide for
the binding lab contract and SDK boundary.

## Status and discovery

| Method | Path | Gate | Returns |
|---|---|---|---|
| GET | `/status` | open | STATUS_SPEC v1.1 envelope: `equipment_status`, `activity`, `allowed_actions`, `components`, `details` |
| GET | `/openapi.json` | open | OpenAPI schema |
| GET | `/agent-docs` | open | the agent guide, Markdown |
| GET | `/agent-docs/api-reference` | open | this document, Markdown |
| GET | `/llms.txt` | open | discovery index |

## Read-only controller configuration and kinematics

These endpoints are open reads (including the two POST calculations), require
an already connected controller, and never move, enable, clear errors, change
mode, or write configuration. They do not acquire a claim. SDK calls run in a
worker thread. Units are **mm and degrees**, pose order **XYZ, Roll, Pitch, Yaw**.

| Method | Path | Body / result |
|---|---|---|
| GET | `/positions` | Existing joints/TCP/rail/gripper snapshot |
| GET | `/kinematics/config` | Model, firmware, SDK version, report-cached TCP/world offsets and payload, raw DH parameters, reference-angle capability |
| GET | `/kinematics/limits` | Live reduced mode and raw ranges; ordinary effective limits explicitly unavailable |
| POST | `/kinematics/fk` | `{"joints":[0,-30,0,30,0]}` → controller TCP in `result.data` |
| POST | `/kinematics/ik` | `{"pose":[300,0,300,180,0,0],"reference_angles":[0,-30,0,30,0],"limited":false}` → controller joints and separate limit check |

Examples illustrate the schema, not validated motion targets. Arrays of joints
must contain exactly the connected robot's axis count (five for xArm5).
Omit `reference_angles` to sample current joints as the reference. `limited`
controls SDK ±180-degree normalization, not safety or joint-limit enforcement.
Reference IK requires firmware >=2.7.103 and a compatible SDK; **409** means
unsupported, and the reference is never silently discarded. **422** means
invalid dimensions/non-finite values; **400** means no controller instance;
**503** means the controller is disconnected.

SDK query results use `{available, code, data, reason}`. An HTTP 200 alone is
not query success: require `available:true` and `code:0`. IK's independent
`joint_limit_check.data` is `true` for exceeding limits, `false` for within
limits, and null/unavailable on query failure. `violating_joints` remains null:
the verified SDK query does not identify individual joints. Error codes are
preserved with SDK symbolic reasons; no detailed controller reason is invented.
A valid inverse solution is not collision checking or path approval.

Configuration offsets/payload come from the SDK report cache; `sampled_at`
records service read time, not controller measurement time. Reads are not an
atomic controller snapshot. DH values are returned in the controller's raw
7-slot order; parameter convention and calibration completeness are unverified.
RPY axes are X/Y/Z, but transform composition is explicitly unverified.
Ordinary-mode joint bounds are **not** replaced by SDK model defaults.
Reduced joint ranges retain raw SDK slot order; the observed xArm5 J4/J5
mapping discrepancy is unresolved, and disabled reduced limits are not active.

## Claims

| Method | Path | Gate | Body | Notes |
|---|---|---|---|---|
| POST | `/control/claim` | login when configured | see OpenAPI | **409** when held by someone else; returns the claim token |
| POST | `/control/heartbeat` | claim token | no body | renews the TTL using `X-Claim-Token` |
| POST | `/control/release` | claim token | no body | drops the claim using `X-Claim-Token` |

## Connection and safety

| Method | Path | Gate | Notes |
|---|---|---|---|
| POST | `/connect` | login | instantiates the controller; required out of `requires_init` |
| POST | `/disconnect` | login | |
| POST | `/control/stop` | login | safety floor, always available while reachable |
| POST | `/control/clear_errors` | login | |

## Motion graph

The ordinary control routes below are claim-gated. Targets come from
`allowed_actions` as `move.<node_id>`. Administrator routes are listed separately.

| Method | Path | Body | Refusals |
|---|---|---|---|
| POST | `/control/graph/move_to` | `{node_id, ...}` | **409** motion in flight · **412** sash interlock · **422** edge not whitelisted |
| POST | `/control/graph/travel_to` | `{node_id}` | multi-hop; same refusals |
| POST | `/control/graph/gripper` | `{state}` | **409** arm must be stationary · **422** state not reachable from here |
| POST | `/control/graph/recover_to` | `{node_id, force?}` | **412** when vision verification rejects |
| POST | `/control/graph/mode` | `{mode}` | only `strict` is accepted. **403** `admin_required` for `off` / `advisory`: lowering enforcement is administrator-only |
| POST | `/control/graph/mode/restore` | | restores `strict` now; idempotent |
| POST | `/control/graph/off` | | retired for claim holders: always **403** `admin_required`; use the administrator switch below |
| POST | `/control/graph/record` | | **412** on any simulator · **409** with no last transition |

`GET /graph` is an open read returning nodes, edges, the current node and
reachable targets.

### Administrator switch (no claim required)

`POST /control/admin/graph/off` enables persistent OFF;
`POST /control/admin/graph/restore` clears it and any ordinary timed override.
Both require verified administrator identity and neither requires a claim.
See [OpenAPI](../openapi.json) for the optional OFF audit reason, request
validation, response fields and refusal envelopes. Restore re-enables STRICT
when a graph is loaded; without a graph the mode remains OFF. Repeating
restore is idempotent.

The admin OFF switch has **no expiry**. It persists across claim release,
claim expiry, different users, disconnects and service restarts, until an
administrator restores it. All users may then use freehand with their own
valid motion claims, without graph path restrictions. Other device checks
continue to apply. These endpoints never acquire or release a user's claim.

Admin identity is verified using a session cookie, API key, or authenticated
edge headers. A claim token alone does not authorize admin actions. Verification
is mandatory even when the general login gate is disabled. Anonymous requests
return 401, non-admin identities 403, and transport failures during identity
verification 503. State storage failures return 503 without changing the mode.

`details.motion_graph.mode_override` reports `scope: admin`, `persistent: true`,
`claim_bound: false`, the administrator/reason, and null expiry/countdown.
Ordinary `/control/graph/mode`, `/control/graph/off` and
`/control/graph/mode/restore` requests with a valid claim return **409** with
`detail.error: admin_graph_off` while this switch is active; only the admin
restore endpoint clears it. `graph.mode` is withheld from `allowed_actions`.

### Lowering enforcement is administrator-only

The `/control/freehand/*` family (raw Cartesian, joints, jog, velocity,
rail) requires `advisory` or `off` mode, and only an administrator can put
the device there, with the switch above. Claim holders cannot lower
enforcement: `/control/graph/mode` below `strict` and `/control/graph/off`
return **403** `admin_required`. An agent that meets a STRICT refusal must
report it; it must not try to lower enforcement. A freehand move in STRICT
is refused with **409** `graph_mode_strict`.

## Freehand Cartesian and joint motion

The dashboard and `lab-skills` expose these xArm-specific catalog names:

| SDK action | Method | Device endpoint | OpenAPI request model |
|---|---|---|---|
| `freehand.position` | POST | `/control/freehand/position` | `PositionRequest` |
| `freehand.relative` | POST | `/control/freehand/relative` | `RelativeRequest` |
| `freehand.joints` | POST | `/control/freehand/joints` | `JointRequest` |

Use the SDK for agent actuation. Full argument schemas, units and defaults
are in [OpenAPI](../openapi.json) and the dashboard's catalog action cards.
These commands require a claim and graph mode OFF or ADVISORY. They are
advertised in `allowed_actions` only when available; STRICT returns 409,
as does a motion already in flight. Configured interlock and simulation
guards can return 412; invalid claims return 423. Existing workspace and
collision checks still apply. A successful HTTP response means accepted,
not completed: poll status for completion/errors. Raw moves clear the node
pin; recover to a verified node before resuming graph motion.

## RealSense — discovering the cameras

Every camera on this device PC has a device-local id (`rs435i`, the D435i
eye-in-hand, and `rs405`, the D405 facing down) and every route for it is
nested under that id. Each entry carries a descriptive `mount`
(`{location, facing}`, either may be null) that is also recorded under
`camera.mount` in every capture's `meta.json`; it never alters the frames. Start here; the listing
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
      "mount": {"location": "gripper (eye-in-hand)", "facing": null},
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
the bound that matters is the disk's. A capture is roughly 175-255 KB at the
configured 1280x720 (colour JPEG 55-110 KB + 16-bit depth PNG 119-145 KB,
measured 2026-09-19 and re-measured 2026-09-20; strongly scene-dependent, a
dim or flat scene compresses smaller), so 20 GB holds on the order of 100k
captures. Both streams run at 1280x720 by design, not by limitation: depth
is at its hardware maximum there, while colour could reach 1920x1080 but is
matched to depth so the two images stay pixel-for-pixel comparable and the
aligned depth map is not inflated with interpolated pixels — see the
[agent guide](../agent-docs). Captures
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
| 403 | verified identity does not have the required admin role |
| 404 | no camera configured, unknown camera id, or unknown capture |
| 409 | motion in flight, camera pipeline stopped, or `admin_graph_off` blocks ordinary mode changes |
| 412 | safety gate: sash not parked, simulator guard, vision rejected |
| 422 | target not whitelisted from the current node, invalid graph override reason, or unsupported admin OFF fields |
| 423 | claim required, or held by another session |
| 500 | capture store write failure |
| 503 | RealSense extra or hardware missing, admin identity verification unavailable, or admin graph state could not be persisted/cleared |
