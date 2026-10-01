# Bounded native/aligned RealSense diagnostic export

`POST /control/realsense/{camera_id}/diagnostic` requires the existing control
claim and returns an `application/zip` attachment with `X-Capture-ID`.
By default the camera must already be streaming with depth-to-color alignment
enabled. `?start_if_idle=true` explicitly allows starting the service-owned
camera using `preserve_sensor_settings=True`; no sensor options are written
and hardware reset is forbidden. The endpoint never calls robot control. Only one export per camera is allowed at a time.

After at least 30 incoming framesets and two seconds of warm-up, the service
copies 20 successive SDK framesets in its existing capture thread. This is a
minimum warm-up interval, not a claim of verified exposure convergence or
hardware synchronization. Inspect each stream's frame numbers and timestamps
for repeated frames, gaps, or timing differences. Sensor options are read during
warm-up; no options are written. Collection times out after 30 seconds, fails
on incomplete frames/camera errors, and limits copied arrays to 256 MiB.
Encoding and ZIP assembly run outside the camera thread. No partial archive
is returned on failure. The archive is returned directly, not persisted to the
normal capture store; the caller must save the response body.

One shared capture ID contains `manifest.json` and 20 numbered directories:

- `native_depth.npy`: original Z16 values before alignment.
- `native_color.npy`: lossless original color array; channel order is specified
  by the original stream format in metadata (normally BGR8).
- `aligned_depth.npy` and `aligned_depth.png`: identical service-aligned depth
  values from that same input frameset; PNG is lossless 16-bit.
- `metadata.json`: separate native depth/color and aligned-depth frame numbers,
  timestamp domains, timestamps in milliseconds, supported SDK metadata in its
  SDK-defined units (including exposure and sensor timestamps where available),
  stream profiles, intrinsics, distortion models and coefficients. Unsupported
  metadata is null; SDK read errors are explicit objects.

The manifest links every artifact with size and SHA-256, records actual serial,
firmware, SDK version, active profiles, depth scale and current sensor options.
Depth-to-color extrinsics come from the original depth/color frame profiles:
rotation is the SDK column-major nine-element array; translation is in metres.
Git commit, dirty status (including untracked files), and Python source hashes
are recorded at module import and again at export. Unknown provenance is marked
explicitly with null fields and an error, never reported as a clean deployment.

Deployment must be coordinated with the camera-PC operator. A normal service
restart may run existing startup configuration writes; this diagnostic endpoint
does not bypass or change startup behavior. Do not restart the running pipeline
as part of a request that forbids sensor-setting changes. This source addition
alone cannot install itself into an already-running Python process.

For an already idle camera, an authorized camera-PC operator can start the
**service-owned camera object** with `start(preserve_sensor_settings=True)`.
This optional startup mode skips exposure-priority writes and fails instead of
performing a hardware reset if frames do not arrive. It uses the configured
stream profiles and does not create an independent capture process. The HTTP
diagnostic endpoint requires a valid control claim, including for the optional
`start_if_idle=true` startup. SSH access does not supply that claim automatically.

Run `tools/collect_realsense_diagnostic.py` on the camera PC after claiming the
device. It prompts for the existing claim token without echoing it, renews the
claim during download, requests the setting-preserving startup, and verifies
all artifact hashes and the 20-frame count before finalizing the ZIP. It leaves
your claim held and does not issue robot commands.
