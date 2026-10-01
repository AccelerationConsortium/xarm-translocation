# Intel RealSense depth camera

The xArm service can own an Intel RealSense depth camera (D435i and any
other librealsense-supported model) plugged into the device PC over USB 3.
It is a **second, unrelated camera** from the "Lab Camera" in the panel:

| | Lab Camera (`/camera/*`) | Depth Camera (`/realsense/*`) |
|---|---|---|
| Hardware | Tapo PTZ network camera | RealSense USB camera on *this* PC |
| Reached through | the ac-organic-lab dashboard passthrough | librealsense (`pyrealsense2`) directly |
| What it gives the arm | a pannable overview that follows the motion graph | colour + **metric depth** per pixel, in the camera frame |
| Config | `src/settings/camera_tracking.yaml` | `src/settings/realsense.yaml` |
| Code | `src/core/camera_tracker.py` | `src/core/realsense_camera.py` |

The depth camera is the primitive later vision work builds on — a "did the
arm really arrive at this node" check, a plate locator, an obstacle check
before a hood move. This release ships the foundation: reliable capture,
health on `/status`, snapshots + live preview, and pixel → metres.

## Setup

1. **Plug it in** with a USB 3 *data* cable (many USB-C cables are
   charge-only) directly into a blue USB 3 port. Windows should list
   *Intel(R) RealSense(TM) Depth Camera 435i* in Device Manager with no
   driver install — if it does not, the problem is the cable or port.
2. **Install the extra once per venv:**

   ```powershell
   C:\SDL_Tools\uv.exe sync --extra realsense
   ```

   This adds `pyrealsense2`, `numpy` and `Pillow`. The service boots without
   them; `/realsense/<id>/status` then reports `installed: false` with the fix.
   If the `xarm` NSSM service is running, sync with `--no-install-project`
   or stop it first — the running `pyxarm.exe` shim otherwise blocks the
   sync (see DEVICE_PC_SETUP.md, "own-service .exe lock").
3. **Enable it** in `src/settings/realsense.yaml` (`enabled: true`, then one
   entry under `cameras:` per camera, each with a device-local `id` matching
   `^[a-z0-9][a-z0-9_-]{0,31}$` — the first is `rs435i`). A `serial` is
   required as soon as two cameras are configured, or enumeration order
   decides which id points at which lens. Restart the service.
4. Open the panel: a **Depth Camera** card appears per camera. It
   starts the pipeline on the first request (`start_on_demand: true`) and
   stops it after `idle_timeout_seconds` without a viewer.

## HTTP surface

Reads that cannot switch the camera on are open like `GET /status`;
anything that starts the pipeline or ships video is **login-gated** (same
policy as the PTZ preview's authenticated viewing sessions). Nothing is
claim-gated — looking is not arm actuation.

Every camera is addressed by its id (`{id}` below); `GET /realsense/cameras`
lists them with a ready-made URL per route.

| Method | Path | Gate | Returns |
|---|---|---|---|
| GET | `/realsense/cameras` | open | `{cameras:[{id, label, state, streaming, start_on_demand, device, urls}], default, reason}`; never 404s |
| GET | `/realsense/{id}/status` | open | enumeration + pipeline state (`camera_id`, `configured`, `installed`, `state`, `devices[]`, `device`, `streams`, `fps_measured`, `warnings[]`, `reason`) |
| POST | `/realsense/{id}/start` | login | opens the pipeline; **503** `realsense_unavailable` (disabled / driver missing / no camera), **502** `realsense_error` (librealsense refused the profiles — usually a USB 2 link) |
| POST | `/realsense/{id}/stop` | login | releases the device |
| GET | `/realsense/{id}/snapshot.jpg?stream=color\|depth` | login | one JPEG (colour, or colourised depth); starts on demand |
| GET | `/realsense/{id}/depth.png` | login | raw 16-bit depth map, lossless; `X-Depth-Scale-M` header converts to metres |
| GET | `/realsense/{id}/stream.mjpg?stream=color\|depth&fps=10` | login | `multipart/x-mixed-replace` MJPEG for an `<img>`; paced server-side |
| GET | `/realsense/{id}/depth?x=&y=&window=5` | open | `{distance_m, point_m:[X,Y,Z], valid_samples, …}`; **409** when stopped — never starts the camera |
| GET | `/realsense/{id}/intrinsics` | open | pinhole intrinsics per stream + `depth_scale_m` + `aligned_to` |

`point_m` is a pinhole deprojection in the frame of the stream the depth
map is expressed in (colour when `align_depth_to_color: true`): +X right,
+Y down, +Z out of the lens, metres. `window` takes the median of the
non-zero depths in an odd `window × window` patch — use 5 for anything a
robot acts on; RealSense depth is noisy per-pixel and has zero-valued holes.

Every failure body is `{"detail": {"error": <code>, "reason": <text>}}`
with codes `realsense_not_configured` (404, no camera configured at all),
`camera_not_found` (404, plus the `cameras` that do exist),
`realsense_unavailable` (503), `realsense_not_streaming` (409),
`realsense_error` (502), `bad_request` (400).

## On `/status`

When `enabled: true`, the envelope carries:

- `components.realsense_camera` — `connected` (a camera is on the bus),
  `state` ∈ `streaming | idle | disconnected | driver_missing | error`,
  and a one-line `message` (label · model · serial · fps · reason).
- `details.realsense` — the machine-readable twin: `state`, `installed`,
  `device`, `devices`, `streams`, `fps_measured`, `frames_captured`,
  `last_frame_age_s`, `warnings`, `reason`.

Both are **absent** when the feature is not configured, so unmigrated
deployments see an unchanged envelope — and both are **present before
`/connect`** (the `requires_init` envelope carries them), because the
camera does not wait for the arm.

**The camera never changes `equipment_status`.** Arm motion does not depend
on it, so an unplugged camera is a component fact, not a `degraded` arm —
the same reasoning STATUS_SPEC §2.2 applies to the sash interlock being
blind. If a future feature makes a move *depend* on a camera reading, that
feature owns the gate (412 + `allowed_actions` mirroring), not this layer.

## Design notes

- **Optional at every layer.** Missing extra, `enabled: false`, or no
  camera each produce a camera object whose `describe()` explains itself;
  nothing raises into the arm's control path and the service always boots.
- **A daemon capture thread owns the pipeline.** `wait_for_frames` blocks,
  so request handlers only read the latest `FrameBundle` under a lock. Five
  consecutive frame failures (`max_consecutive_frame_failures`) mark the
  camera lost and release it; `POST /realsense/{id}/start` brings it back.
- **A process-wide registry, not per-connection.** The cameras outlive
  `/connect` / `/disconnect`: operators want to see the bench before the arm
  is up. `realsense_camera.configure_cameras()` builds `{id: camera}` from the
  YAML at import; `status_builder` reads it through `realsense_camera.cameras()`
  so `/status` and `/realsense/{id}/status` cannot disagree. A malformed,
  duplicate or serial-less entry is logged and skipped, never raised — a typo
  in a camera id must not stop the arm service from booting.
- **Injectable backend.** `RealSenseCamera(config, camera_id=…, rs_module=…, np_module=…)`
  lets the tests drive the entire lifecycle against a fake `pyrealsense2`
  (`test/test_realsense_camera.py`); the HTTP layer is tested against a
  scripted camera (`test/test_realsense_api.py`).
- **No OpenCV.** Pillow encodes JPEG/PNG; librealsense's own `colorizer`
  renders the depth map. Keeps the extra small.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `installed: false` | extra not synced into this venv | `uv sync --extra realsense` (stop the service or add `--no-install-project`) |
| `no RealSense device connected` | not enumerated by Windows | USB 3 *data* cable, direct blue port; confirm in Device Manager |
| `warnings: ["USB 2.x link …"]` or 502 on start | camera negotiated USB 2 | different port/cable, or drop `fps` / resolution in the YAML |
| stream freezes, then `state: error` | cable yanked / power dip | re-seat, `POST /realsense/{id}/start` |
| preview blank through the `:6001` legacy proxy | that proxy buffers whole bodies; MJPEG never ends | open the panel on the API port (`:8000/web/`) |
| `distance_m: null` | depth hole (black / reflective / too close, < 0.2 m) | larger `window`, move the target, or check the IR projector is not blocked |

## Bench check (optional, no service needed)

```powershell
cd C:\Users\sdl2\Projects\realsense-d435i
.venv\Scripts\python check_camera.py     # identity, firmware, USB link
.venv\Scripts\python preview.py          # live colour + depth window
```

That standalone folder predates this integration and is handy for isolating
"camera vs service" when something does not show up.
