# xArm translocation — agent guide

This service drives a UFactory xArm5 on a linear rail with a BioGripper Gen2.
The standalone SDL camera service owns the Intel RealSense depth cameras:
`rs435i` (D435i, eye-in-hand on that
gripper) and `rs405` (D405 close-range, mounted **facing down**). Either may be
unplugged at a given time; `GET /realsense/cameras` lists both, and because two
are configured every capture must name its `camera`. It
speaks STATUS_SPEC v1.1: read `GET /status` before acting, and treat
`allowed_actions` as the contract for what will be honoured right now.

Paths are relative to the configured service base, including its mount prefix.

The binding [lab contract, Part I](https://github.com/AccelerationConsortium/ac-organic-lab/blob/main/docs/AGENTIC_LAB_DESIGN.md#part-i--binding-rules-normative)
and [STATUS_SPEC](https://github.com/AccelerationConsortium/ac-organic-lab/blob/main/docs/STATUS_SPEC.md)
remain authoritative. Transport documentation does not authorize hardware
execution. Agent equipment use goes through the `lab-skills` SDK; a refusal
must be reported, not worked around by lowering enforcement or switching
endpoints. Ambiguous command outcomes require reconciliation before another
physical action; recovery belongs to the operator.

## Access gates

Check the route's gate before calling it to avoid authentication and claim
refusals.

| Tier | What it covers | How you pass it |
|---|---|---|
| Open | `GET /status`, depth numbers, capture metadata, this document | nothing |
| Login | anything that ships a frame or turns the camera on | session cookie, or `X-Api-Key` |
| Claim | anything that moves the arm or writes a record | `X-Claim-Token` from `POST /control/claim` |
| Admin | `/control/admin/graph/off` and `/control/admin/graph/restore` | verified admin session cookie, `X-Api-Key`, or authenticated edge identity; no claim needed |

Admin verification applies even when the general login gate is disabled.
A claim token alone is insufficient: missing identity returns **401**,
a non-admin identity **403**, and unavailable identity verification **503**.

A claim is cooperative and exclusive: one holder at a time, renewed with
`POST /control/heartbeat`, released with `POST /control/release`. Acting
without it returns **423** with the current holder in the body. Take the claim
first, do your work, release it. Do not hold one across a long idle stretch.

## Read before you act

`GET /status` returns the envelope. The fields that decide your next move:

- `equipment_status` — `ready`, `busy`, `degraded`, `dry_run`, `error`,
  `requires_init`. `requires_init` means the controller is not instantiated;
  `POST /connect` is the only thing that helps.
- `activity` — `idle` or `running`. While `running`, every move target and the
  capture verb disappear from `allowed_actions`, and the endpoints refuse with
  **409**. That is the same rule on two surfaces, by design.
- `allowed_actions` — the catalog names you may call now. Motion families are
  `graph.move_to`, `graph.travel_to`, `graph.gripper`, `graph.recover_to`,
  `graph.mode`, `graph.record`. In OFF or ADVISORY, idle motion also exposes
  `freehand.position`, `freehand.relative`, and `freehand.joints`. These are
  xArm-specific `lab-skills` catalog entries. The depth cameras add `realsense.capture`,
  which is offered when *any* configured camera could serve it.
- `details.motion_graph.current_node` — where the arm is on the graph, or
  `null` when it is off-grid after a stop or a raw move.
- `details.realsense` — `default`, a `cameras` map keyed by camera id with
  each camera's health, and `captures` for the store's occupancy (one store,
  shared by every camera).

A camera never changes `equipment_status`. An unplugged one shows up on its
own `components.realsense_<camera_id>` entry (for example
`components.realsense_rs435i`) and leaves the arm's state alone, because arm
motion does not depend on it. Two cameras are two components: they fail
independently, and one merged entry would hide the working one.

## Read-only kinematics

`GET /kinematics/config` reads controller offsets, payload, version and DH
parameters. `GET /kinematics/limits` reads reduced-mode settings and reports
ordinary effective limits as unavailable. `POST /kinematics/fk` and
`POST /kinematics/ik` calculate without motion or state changes; IK accepts
`reference_angles` or reads current joints when omitted. These routes require
an existing connection and no claim. Check SDK result codes and the separate
limit check; a solution does not establish a safe path. See the
[API reference](agent-docs/api-reference) for schemas and remaining limitations.

## Moving the arm

Nodes are named positions; edges are whitelisted transitions. In `strict`
mode only the edges out of the current
node are legal, and `allowed_actions` enumerates them as `move.<node_id>`.

A fume-hood sash interlock withholds hood and Opentrons targets while the sash
is not parked. Those targets vanish from `allowed_actions` and the endpoint
answers **412**. This is a safety floor, not an error. **It is switched off as
of 2026-09-21** pending an xyz safe/danger volume that replaces it, so no
target is currently withheld for this reason and no **412** will come from it
— do not read that silence as "the sash is open".

`POST /control/stop` is always available while the device is reachable.

### Cartesian work with the graph OFF

An administrator may enable persistent OFF using
`POST /control/admin/graph/off` (no body or claim required; verified admin
identity required). The optional audit reason and request/response schemas
are documented in [OpenAPI](openapi.json).

OFF persists for all users across claim release/expiry, changes of user,
disconnects and service restarts. Only an administrator can clear it using
`POST /control/admin/graph/restore`, with no body or claim required. Restore
re-enables STRICT when a graph is loaded; without a graph the mode remains
OFF. Repeated restoration is idempotent. State storage failures leave the
mode unchanged. Neither endpoint acquires or releases a user's claim.
An agent encountering a graph refusal must stop and report it; the admin
switch is not an alternate execution route.

Inspect `details.motion_graph.mode_override`: admin OFF reports
`scope: admin`, `persistent: true`, `claim_bound: false`, the admin's `owner`
and `reason`, and null `expires_at` / `remaining_seconds`. While active,
`graph.mode` is absent from `allowed_actions`. Ordinary graph mode, OFF and
restore requests with a valid claim return **409** `admin_graph_off`; an
administrator must restore enforcement. Request and response
details are in the [API reference](agent-docs/api-reference).

Use the `lab-skills` SDK and its claim/session handling for agent control.
The dashboard API Reference lists `freehand.position` (absolute TCP pose),
`freehand.relative` (TCP displacement), and `freehand.joints` (joint angles).
Coordinates and displacements are in mm; angles are in degrees. The live
OpenAPI document supplies the request schemas.

### Raw BioGripper Gen2 stroke when the graph is OFF

When an administrator has enabled graph OFF, use the claimed
`POST /control/freehand/gripper/stroke` route (also exposed as
`/gripper/move/stroke`) for an absolute jaw opening. For example,
`{"stroke":120,"force":50,"wait":true}` requests 120 **mm** and a force
**setting of 50%**, not 50 N. The gripper's range is 71 mm closed to
150 mm open. Choose the target from a calibrated plate grip; the example
does not establish that 120 mm is appropriate for a particular plate.
`POST /control/freehand/gripper/force` changes only the force setting and
does not move the jaws. The gripper force setting is separate from the wrist
force/torque sensor used when pressing on a surface.

`wait:true` waits for the SDK to report a stopped or object-detected jaw
move, up to the configured 5-second timeout. A successful raw stroke call
does **not** confirm a grasp or a measured force. Read the live opening from
`GET /gripper/position` after the move. If the jaws stop before the target,
there may be an object between them; if they reach the target, they may
have closed on empty space. Neither observation proves the plate is held.
The gripper status in `GET /status` is cached after the last successful jaw
move, not a live grip-force measurement. Inspect the plate before lifting;
after timeout or fault, reconcile the physical outcome before another move.
The detailed [API reference](agent-docs/api-reference#biogripper-gen2-raw-stroke-and-force)
gives the routes, units, and refusal codes.
This is a device API route, not a `lab-skills` catalog action: the current
robot-arm catalog has no raw-stroke skill. An agent workflow needs an approved
SDK binding before it can use this route under the lab contract above.

Freehand is available only while an administrator has turned enforcement
OFF (`details.motion_graph.mode_override.scope == "admin"`). Agents and
other claim holders cannot lower enforcement: `graph.mode` below `strict`
and `POST /control/graph/off` return **403** `admin_required`. Setting
`graph.mode` to `strict` is always allowed.

Freehand moves remain subject to claims, device safety checks, configured
interlocks, and the motion reservation. STRICT rejects them with 409.
They clear the named pose pin. Before resuming graph motion, recover to a
verified node. HTTP success acknowledges acceptance; poll status to observe
completion and errors before issuing another move. A disconnected device
must be connected by an authenticated operator; the SDK does not auto-connect.

## The depth cameras

The standalone SDL camera service owns the USB and RealSense devices, their
streams, and saved captures. This xArm service retains `/realsense/*` routes as
compatibility proxies with its existing login and claim gates. Camera clients
may instead use `GET /v1/cameras` and the returned URLs on the camera service
with a camera-scoped Bearer token. That service is bound to the device PC's
loopback interface; remote clients need the approved gateway or tunnel.
If the camera service is unavailable, xArm reports camera unavailability while
its robot status remains independent. xArm does not open a local camera.

### Stream profiles

The camera service defaults to **1280x720** for colour and depth. The current
Cytation RealSense profiles are configured at **15 fps**. Read live values
from `GET /realsense/{id}/status` or the camera service's
`GET /v1/cameras/{id}/status` before using a profile.

The D435i's ceilings are not the same on each sensor:

| Stream | Hardware maximum | Note |
|---|---|---|
| Colour | 1920x1080 @ 30 | native sensor resolution |
| Depth | 1280x720 @ 30 | anything above 848x480 is ASIC-upsampled from it |

Colour is deliberately **not** run at its 1920x1080 maximum. With
`align_depth_to_color` on, the depth map is resampled to the colour
resolution, so a 1920x1080 colour stream would produce a 1920x1080
`depth.png` carrying no more depth information than the 1280x720 stereo
stream behind it — roughly double the bytes per capture for pixels rather
than measurements (reviewed and settled 2026-09-20).

Two consequences that matter when you consume a capture:

- **A pixel means the same thing in both images.** `color.jpg` and
  `depth.png` share dimensions and come from one frameset, so the pixel you
  pick in the colour frame is the pixel you read in depth — which is exactly
  what `GET /realsense/{id}/depth?x=&y=` relies on.
- **Depth above 848x480 is interpolated.** 1280x720 depth is upsampled from
  the module's native stereo resolution. Treat fine-grained depth detail as
  smoothed, not resolved. The values are still metric and still trustworthy
  at the scale an arm works at.

Unmeasured depth pixels read `0`; `65535` is the 16-bit saturation marker,
not a 65 m reading. Expect roughly 60-75% valid pixels on a normal bench
scene.

**Find them first.** Each camera has a device-local id — the first is
`rs435i` — and every route for it is nested under that id:

```
GET /realsense/cameras
```

That returns one entry per camera with a ready-made `urls` block, plus
`default` (the id you may omit naming; `null` once there is more than one
camera) and `reason` when the list is empty. Follow those URLs rather than
building paths: the ids are device configuration, not something to hard-code.
An unknown id answers **404** `camera_not_found` and lists the ids that do
exist; a service with no camera at all answers **404**
`realsense_not_configured`.

Two kinds of read, and the difference matters:

- **Transient** — `GET /realsense/{id}/snapshot.jpg`, `depth.png`,
  `stream.mjpg`, `depth?x=&y=`, `intrinsics`. Nothing survives the response.
  Use these to look.
- **Durable** — `POST /control/realsense/{id}/capture`. Writes one aligned
  frameset to disk as `color.jpg` + `depth.png` + `meta.json` and returns a
  `capture_id`. Use this when the frame is evidence.

`GET /realsense/{id}/depth?x=&y=` returns metres and a camera-frame 3-D point
for one pixel, median-filtered over a `window` patch. It is an open read and
it will **not** start the pipeline: a stopped camera answers **409** rather
than letting an anonymous read switch hardware on. If you want numbers from a
cold camera, take the claim and capture, or `POST /realsense/{id}/start` with
a login.

## Captures

A capture is the unit later vision work consumes, which is why `meta.json`
carries the arm state the frames were taken from. A depth map without its pose
is a picture, not a measurement.

`meta.json` holds the capture id, the `camera_id` that took it, the UTC
timestamp, the camera's serial, firmware and stream profiles, the intrinsics
and depth scale, the arm block (`node_id`, joints, TCP position, rail
position, gripper state, connected), who requested it, your label and tags,
and a SHA-256 for each file.

Body fields, all optional: `label`, `node_id` (defaults to the arm's current
node), `tags`, and `protected`. Set `protected: true` only for records that
must outlive retention, such as a reference frame for a node — it exempts the
capture from both retention bounds.

**The fixed-path alias.** `POST /control/realsense/capture` takes the camera
in the body (`{"camera": "rs435i"}`) instead of the path. It exists because a
SkillDef carries one fixed `endpoint` string that the skill executor and the
dashboard passthrough send verbatim, with no path templating: a plan cannot
put a camera id in a path, but it can put one in a body. Omit `camera` and the
default camera is used; with two or more cameras configured there is no
default and the call is refused with **400** `camera_required` listing the
ids, because guessing would file the evidence under the wrong lens. The nested
route stays canonical everywhere else.

One store holds every camera's captures, under
`<root>/<camera_id>/<YYYY-MM-DD>/<capture_id>/`. `GET /realsense/captures`
lists all cameras newest first (each record carries its `camera_id`);
`GET /realsense/{id}/captures` narrows it to one.

Retention is enforced by the camera service after every write: 30 days and
10 GB on the current Cytation configuration, oldest first, age
before size, and shared across all cameras rather than split between them. Do
not build anything that assumes a capture from last quarter is still there
unless you marked it protected.

Fetching the image bytes is login-gated at
`GET /realsense/{camera_id}/captures/{capture_id}/color.jpg` and
`/depth.png`. The metadata and the listings are open.

## Refusals you should expect

| Code | Meaning | What to do |
|---|---|---|
| 400 | the capture alias needs a `camera` | read `cameras` from the body and name one |
| 401 | login required | present `X-Api-Key`, or sign in |
| 404 | no camera configured, or an unknown camera id | re-read `GET /realsense/cameras` |
| 409 | a motion is in flight, or the camera is stopped | poll `/status`, retry when `activity` is `idle` |
| 412 | a safety gate refused: sash not parked, vision rejected | fix the physical precondition; do not retry blindly |
| 422 | the target is not on the whitelist from here | re-read `allowed_actions` |
| 423 | claim required or held by someone else | take the claim, or wait for the holder |
| 503 | camera service or hardware unavailable | check `details.realsense` and camera service health; retry only after recovery |

## Where to go next

- [API reference](agent-docs/api-reference) — every route, body and refusal.
- [OpenAPI](openapi.json) — request and response schemas.
- `GET /status` — the live contract. Read it before each decision, not once at
  the start of a long run.
