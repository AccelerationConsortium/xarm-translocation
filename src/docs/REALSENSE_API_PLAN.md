# RealSense camera API — plan

**Status:** proposal, 2026-09-18. Phase 0 shipped (commit `079e085`);
everything below is open. Decisions marked **D-n** need a human answer
before their phase starts. **Settled 2026-09-18:** the camera is mounted on
the BioGripper Gen2 → eye-in-hand (D-2); calibration uses **AprilTags**;
plain RGB photo/video use is a first-class requirement.

## Where we are (Phase 0, shipped)

`/realsense/*` gives the arm process a working depth camera: enumeration,
start/stop, colour + colourised-depth JPEG, raw 16-bit depth PNG, MJPEG
preview, `depth?x=&y=` → metres + camera-frame XYZ, intrinsics, and health on
`/status` (`components.realsense_camera`, `details.realsense`). Verified on
the bench: D435i on USB 3, 640×480 @ 30 depth+colour at the time; since
2026-09-19 both streams run at **1280×720 @ 30** (negotiated first try on
USB 3.2, ~70 % depth fill on the bench scene).

What it is **not** yet: a control surface a workflow or agent can act on, a
source of records, or anything that knows where the camera is relative to
the arm. Those are the phases below.

## Design rules (apply to every phase)

1. **The camera stays a subsystem of `xarm_translocation`**, not its own
   `equipment.yaml` entry. Its data is only meaningful next to the arm's
   pose (which node, which joints, where the rail is), the STATUS_SPEC
   `camera` kind is shaped for Tapo PTZ gateways, and a second device would
   need a second claim for one physical action. See **D-1**.
2. **Actions that produce a record or move the arm go under `/control/*`
   and are claim-gated** (STATUS_SPEC §5), advertised in `allowed_actions`
   under their catalog names, withheld exactly when they would be refused
   (§6.2). Pure reads stay open, as today.
3. **The camera never decides `equipment_status`.** When a phase makes a
   move *depend* on a camera reading (Phase 4), that feature owns its gate:
   HTTP 412 with a structured body + `allowed_actions` mirroring, never a
   `degraded` arm because a lens is dirty.
4. **Everything testable without hardware.** The fake `pyrealsense2` in
   `test/test_realsense_camera.py` grows with each phase; endpoint tests use
   a scripted camera. Bench verification is recorded in the CHANGELOG, not
   assumed.
5. **Records leave the device the way everything else does**: files on the
   device PC with a retention policy, a row to `/api/ingest/events` per
   capture, never a payload in `/status`.

---

## Phase 1 — Capture records (`realsense.capture`) + plain RGB use

**Goal:** one call that freezes *what the camera saw and where the arm was*,
and keeps it. This is the unit every later phase consumes (references,
verification evidence, training data for plate detection).

| Method | Path | Gate | Behaviour |
|---|---|---|---|
| POST | `/control/realsense/capture` | claim | Grab the latest aligned frameset; write `color.jpg`, `depth.png` (16-bit), `meta.json`; return `{capture_id, urls, meta}`. Body: `{label?, node_id?, tags?[]}`. 409 `realsense_not_streaming` when the pipeline is off (or start on demand — **D-3**). |
| GET | `/realsense/captures?limit=&node_id=&since=` | open | List, newest first. |
| GET | `/realsense/captures/{id}` | open | `meta.json`. |
| GET | `/realsense/captures/{id}/color.jpg` · `/depth.png` · `/depth_color.jpg` | login | The files. |
| DELETE | `/control/realsense/captures/{id}` | claim | Remove one capture. |

`meta.json` carries: capture id, UTC timestamp, device serial/firmware,
stream profiles, intrinsics + depth scale, **arm state at capture** (current
motion-graph node or `null`, joints, TCP pose, rail position, gripper state,
`activity`), the requester (claim owner / login identity), label/tags, and
the file checksums. Layout: `C:\SDL_Data\xarm\realsense\<YYYY-MM-DD>\<id>\`,
retention by count and days in `realsense.yaml` (`captures: {root, keep_days,
keep_max}`), pruned on write.

**Plain camera use.** Colour snapshots (`/realsense/snapshot.jpg`) and the
live MJPEG preview already exist; Phase 1 adds **video recording**:
`POST /control/realsense/record/start {max_seconds, fps, stream}` and
`POST /control/realsense/record/stop` (claim) writing an MP4 (H.264 via
Pillow-free `imageio-ffmpeg`, or MJPEG-in-AVI if we want zero new deps) into
the same capture store, listed and served through the same `captures`
routes; `details.realsense.recording = {active, since, seconds}`;
`realsense.record` in `allowed_actions`. A recording is also stopped by
`max_seconds` and by camera loss. Stream resolution is a config change in
`realsense.yaml` — both streams are at 1280×720 @ 30 as of 2026-09-19, and
1920×1080 colour remains available; the two are independent profiles, so
depth could be dropped back to its native 848×480 without touching colour.

Also: emit a `realsense_capture` event to `/api/ingest/events` via the
existing `events_exporter` (id, node, label, sizes); advertise
`realsense.capture` in `allowed_actions` when the camera is streaming (or
startable on demand) and the arm is not mid-motion; add
`details.realsense.captures = {count, last_id, last_at}`.

**Tests:** capture store (layout, meta, retention, checksum), the endpoint
(claim gate, 409 paths, arm-state snapshot with and without a controller),
`allowed_actions` mirroring.

**Effort:** ~1 day. No new dependencies.

## Phase 2 — Measurement primitives

**Goal:** the numbers a workflow or planner asks for, without shipping images.

| Method | Path | Gate | Returns |
|---|---|---|---|
| GET | `/realsense/depth/roi?x0=&y0=&x1=&y1=` | open | `{median_m, mean_m, min_m, max_m, valid_fraction, n}` over a rectangle; the robot-grade version of `depth?x=&y=`. |
| GET | `/realsense/depth/points?x=..&y=..` (repeatable) | open | Batch of `depth_at` results in one round-trip. |
| POST | `/realsense/measure/plane` | login | Fit a plane to an ROI (RANSAC over deprojected points): `{normal, distance_m, inlier_fraction, rms_m}`. Answers "is the deck level / how far is the bench". |
| GET | `/realsense/pointcloud.ply?stride=4&roi=` | login | Decimated PLY of the current frame (camera frame), for offline inspection. Capped size. |
| GET | `/realsense/imu` | open | Latest accel/gyro from the D435i Motion Module (enable the stream when `imu.enabled: true`); gives the gravity vector → mount tilt sanity check for Phase 3. |

All computed from the latest `FrameBundle`; numpy only (plane fit is a
40-line RANSAC, no OpenCV). Add `snapshot.jpg?annotate=roi` later if the
panel wants to draw the box.

**Tests:** synthetic depth planes/steps through the fake; ROI edge cases;
plane fit tolerance.

**Effort:** ~1 day.

## Phase 3 — Where the camera is (extrinsics) — **needs D-2**

**Goal:** answer `depth_at` in the **arm base frame** (and per node, in the
deck frame), so a measurement can become a motion target or a check.

The camera rides on the BioGripper Gen2, so this is **hand-eye
calibration** (`cv2.calibrateHandEye`): drive the arm to N (≥ 10) poses that
all see one **AprilTag** fixed to the deck, record the tag pose in the camera
for each and the arm's tool pose from the SDK, solve for the constant
`T_tool_camera`. Every measurement then goes
`camera → tool (fixed, from calibration) → base (live joints, xArm forward
kinematics)`.

**AprilTags** (family 36h11, printed at a known size — 60–100 mm works at
the arm's working distances) via OpenCV's ArUco module
(`cv2.aruco.DICT_APRILTAG_36h11` + `estimatePoseSingleMarkers` using the
colour intrinsics `/realsense/intrinsics` already reports). No separate
AprilTag library. The same detector serves beyond calibration: a tag on a
plate, a deck slot, or the hood frame gives its full 6-DoF pose in the camera
(and, after calibration, in the arm base), which is what a plate locator or
an arrival check wants.

| Method | Path | Gate | Behaviour |
|---|---|---|---|
| GET | `/realsense/tags?family=36h11&size_m=0.08` | open | Detect AprilTags in the current colour frame: `[{id, corners_px, pose_camera: {t_m, rvec}, reproj_err}]`; `frame=base` after calibration. Useful on its own (plates, slots). |
| POST | `/control/realsense/calibration/sample` | claim | Detect the calibration tag in the current frame, record `(tag pose in camera, arm tool pose)` as one sample. |
| POST | `/control/realsense/calibration/solve` | claim | Solve from ≥ N samples; persist `T_*` + residuals to `src/settings/realsense_calibration.yaml`; return `{rms_mm, samples, extrinsics}`. |
| GET | `/realsense/calibration` | open | Current extrinsics, method, rms, sample count, `valid` flag. |
| GET | `/realsense/depth?x=&y=&frame=base\|tool\|camera` | open | Extends the existing read. |

Adds an optional **`vision` extra** (`opencv-python-headless`, with
`cv2.aruco`) for tag detection and the hand-eye solve only; the camera layer
stays OpenCV-free. Check the wheel exists for the service venv's Python 3.14
before relying on it (it did for 3.13). `details.realsense.
calibration = {valid, rms_mm, solved_at}`. Calibration age > N days → a
`warning`, never a state change.

**Tests:** synthetic fiducial poses with a known transform → solver
recovers it; frame conversion round-trips; endpoints without OpenCV return
503 `vision_extra_missing`.

**Effort:** 2–3 days including a bench session for the actual calibration.

## Phase 4 — Vision verification of arrival (`realsense.verify_node`)

**Goal:** the C3 metric in `docs/xarm_and_measurement_plan.md` Step 7 — an
independent check that the arm physically reached a motion-graph node
before a recovery resumes.

- **Reference per node:** `POST /control/graph/node/{id}/reference` (claim)
  → a Phase 1 capture flagged `reference`, stored against the node in
  `motion_graph.yaml`'s companion file (`node_references.yaml`: capture id,
  ROI, thresholds). One reference per node; re-recording replaces it.
- **Verify:** `POST /control/realsense/verify_node {node_id?, dry_run?}`
  (claim) → capture, compare against the node's reference within its ROI
  (depth: median absolute difference + valid-fraction; colour: normalised
  cross-correlation), return `{verdict: confirmed|rejected|no_reference,
  confidence, depth_delta_m, evidence_capture_id}`; emit a
  `vision_verification` event with the plan's `extra` keys
  (`recovery_id`, `telemetry_state`, `vision_verdict`, `vision_confidence`,
  `resumed`).
- **Gate:** `recover_to` without `force` calls verify first when
  `verification.required: true`; a `rejected` verdict refuses with **412**
  `{error: "vision_rejected", detail, evidence_capture_id, hint}` and
  `graph.recover_to` is withheld from `allowed_actions` while the last
  verdict for the pinned node is `rejected` (§6.2). `force=true` bypasses,
  audited. `details.realsense.last_verification` mirrors it.

Start with the fixed-mount or eye-in-hand geometry from Phase 3 only if the
reference ROI needs re-projection; a **same-pose image comparison needs no
extrinsics**, which is why this can ship before Phase 3 if the camera is
eye-in-hand (the view is identical at the same node by construction).

**Tests:** identical / shifted / occluded synthetic frames → verdicts;
412 body shape; `allowed_actions` mirroring; the `force` bypass audit.

**Effort:** 2 days + bench tuning of thresholds per node.

## Phase 5 — Lab-wide surface

Things outside this repo that make the camera usable by workflows, the
dashboard, and the assistant. Each is small once Phases 1–2 exist.

- **Skill catalog** (`ac-organic-lab/skills/.../robot_arm.py`): add
  `realsense.capture`, `realsense.verify_node` SkillDefs (endpoint, args,
  `requires_states`), so `lab.skills()` / `execute_plan` / the MCP
  `execute_plan` tool see them. Names must equal what `allowed_actions`
  advertises (the Phase 1/4 names above).
- **Dashboard `RobotArmTile`**: a thumbnail from `/realsense/snapshot.jpg`
  via the `/device/*` proxy (login-gated at the device, so it inherits the
  edge session), and the `components.realsense_camera` row.
- **Assistant**: extend `capture_camera_snapshot` in `mcp_server.py` (or
  add `capture_arm_camera`) to fetch `/realsense/snapshot.jpg` and the
  latest `depth/roi` numbers, so "what is in front of the arm right now" is
  answerable.
- **Events**: `realsense_capture` and `vision_verification` types
  registered in `OBSERVABILITY.md`.

## Cross-cutting

- **Auth.** Today: reads open, camera-on/video login-gated. Phases 1/3/4
  add claim-gated `/control/*` verbs — the same three tiers the arm already
  has. Nothing new to design; keep `/realsense/depth*` open (numbers only).
- **Bandwidth.** MJPEG at 10 fps ≈ 1–2 Mbps per viewer through the edge; the
  Tapo streams already taught us the campus radio is the bottleneck. Keep
  the preview opt-in in the panel (it is) and never embed it in the
  dashboard grid — snapshots there.
- **Storage.** `C:\SDL_Data\xarm\realsense\`, outside the repo tree, pruned by
  retention. A 1280×720 capture measured ~250 KB on the bench (JPEG + 16-bit
  PNG; ~55 KB at the previous 640×480), so `keep_max_gb: 20` is ~80k captures.
- **Firmware.** 5.11.1.100 works; update to ≥ 5.13 with the RealSense Viewer
  during a bench session, before Phase 3 (calibration numbers should be
  taken on the firmware we intend to run).
- **Hardware notes.** This PC has five USB 3 sockets, all otherwise taken;
  the camera lives in the one freed on 2026-09-18. In a USB 2 socket it
  silently loses its RGB sensor — `usb_type` on `/realsense/status` is the
  tell.

## Decisions needed

- **D-1 — Subsystem of the arm, or its own equipment entry?**
  Recommendation: subsystem (rule 1). Revisit only if a second consumer that
  is not the arm appears.
- ~~**D-2 — Mount**~~ — **settled: eye-in-hand** (on the BioGripper Gen2).
  Phase 4's same-pose comparison therefore needs no extrinsics and can ship
  before Phase 3.
- **D-3 — Should a `/control/realsense/capture` start the pipeline on
  demand?** Recommendation: yes (it is claim-gated, so already an authorised
  actor), and let the idle timeout stop it.
- **D-4 — Ordering.** Default is 1 → 2 → 4 (eye-in-hand) → 3 → 5, because
  captures + verification deliver the C3 metric soonest. If plate *location*
  (a measurement that becomes a target) is the priority, do 3 before 4.

## Suggested first slice

Phase 1 in full plus `depth/roi` from Phase 2: one claim-gated verb that
produces a durable, arm-annotated record, one read that returns
robot-grade numbers. Both are hardware-verifiable in an afternoon, both are
needed by everything after, and neither depends on a decision beyond D-3.
