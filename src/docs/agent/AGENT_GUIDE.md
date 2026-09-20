# xArm translocation — agent guide

This service drives a UFactory xArm5 on a linear rail with a BioGripper Gen2,
plus an Intel RealSense D435i mounted **eye-in-hand** on that gripper. It
speaks STATUS_SPEC v1.1: read `GET /status` before acting, and treat
`allowed_actions` as the contract for what will be honoured right now.

Base URL on the lab tailnet: `http://sdl2-pc-03-cytation.tail6a1dd7.ts.net:8000`.

## The three gates

Every route sits in exactly one tier. Knowing which one saves you a round of
guessing at a 401 or a 423.

| Tier | What it covers | How you pass it |
|---|---|---|
| Open | `GET /status`, depth numbers, capture metadata, this document | nothing |
| Login | anything that ships a frame or turns the camera on | session cookie, or `X-Api-Key` |
| Claim | anything that moves the arm or writes a record | `X-Claim-Token` from `POST /control/claim` |

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
  `graph.mode`, `graph.record`. The camera adds `realsense.capture`.
- `details.motion_graph.current_node` — where the arm is on the graph, or
  `null` when it is off-grid after a stop or a raw move.
- `details.realsense` — camera health, and `details.realsense.captures` for
  the store's occupancy.

The camera never changes `equipment_status`. An unplugged camera shows up on
`components.realsense_camera` and leaves the arm's state alone, because arm
motion does not depend on it.

## Moving the arm

Motion is a graph, not free space. Nodes are named positions; edges are
whitelisted transitions. In `strict` mode only the edges out of the current
node are legal, and `allowed_actions` enumerates them as `move.<node_id>`.

A fume-hood sash interlock withholds hood and Opentrons targets while the sash
is not parked. Those targets vanish from `allowed_actions` and the endpoint
answers **412**. This is a safety floor, not an error.

`POST /control/stop` is always available while the device is reachable.

## The depth camera

The camera is local USB hardware owned by this process, unlike the lab PTZ
cameras which are network devices driven through the dashboard. It idles by
default and starts on the first request, then stops itself after an idle
timeout.

Two kinds of read, and the difference matters:

- **Transient** — `GET /realsense/snapshot.jpg`, `depth.png`, `stream.mjpg`,
  `depth?x=&y=`, `intrinsics`. Nothing survives the response. Use these to
  look.
- **Durable** — `POST /control/realsense/capture`. Writes one aligned frameset
  to disk as `color.jpg` + `depth.png` + `meta.json` and returns a
  `capture_id`. Use this when the frame is evidence.

`GET /realsense/depth?x=&y=` returns metres and a camera-frame 3-D point for
one pixel, median-filtered over a `window` patch. It is an open read and it
will **not** start the pipeline: a stopped camera answers **409** rather than
letting an anonymous read switch hardware on. If you want numbers from a cold
camera, take the claim and capture, or `POST /realsense/start` with a login.

## Captures

A capture is the unit later vision work consumes, which is why `meta.json`
carries the arm state the frames were taken from. A depth map without its pose
is a picture, not a measurement.

`meta.json` holds the capture id and UTC timestamp, the camera's serial,
firmware and stream profiles, the intrinsics and depth scale, the arm block
(`node_id`, joints, TCP position, rail position, gripper state, connected),
who requested it, your label and tags, and a SHA-256 for each file.

Body fields, all optional: `label`, `node_id` (defaults to the arm's current
node), `tags`, and `protected`. Set `protected: true` only for records that
must outlive retention, such as a reference frame for a node — it exempts the
capture from both retention bounds.

Retention is enforced after every write: 30 days and 20 GB, oldest first, age
before size. Do not build anything that assumes a capture from last quarter is
still there unless you marked it protected.

Fetching the image bytes is login-gated at
`GET /realsense/captures/{id}/color.jpg` and `/depth.png`. The metadata and the
listing are open.

## Refusals you should expect

| Code | Meaning | What to do |
|---|---|---|
| 401 | login required | present `X-Api-Key`, or sign in |
| 409 | a motion is in flight, or the camera is stopped | poll `/status`, retry when `activity` is `idle` |
| 412 | a safety gate refused: sash not parked, vision rejected | fix the physical precondition; do not retry blindly |
| 422 | the target is not on the whitelist from here | re-read `allowed_actions` |
| 423 | claim required or held by someone else | take the claim, or wait for the holder |
| 503 | camera extra or hardware missing | check `details.realsense`; this is not retryable |

## Where to go next

- [API reference](agent-docs/api-reference) — every route, body and refusal.
- [OpenAPI](openapi.json) — request and response schemas.
- `GET /status` — the live contract. Read it before each decision, not once at
  the start of a long run.
