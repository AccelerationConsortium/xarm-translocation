# Standalone camera service

Camera ownership can be delegated to the public
[SDL Camera Server](https://github.com/AccelerationConsortium/sdl-camera-server).
Set `XARM_CAMERA_SERVICE_CONFIG` to an absolute path to a private JSON file:

```json
{"url":"http://127.0.0.1:8070","token":"REPLACE_WITH_CAMERA_SCOPED_TOKEN","cameras":["rs435i","rs405"]}
```

Stop the previous camera owner before starting the standalone service. Configure
its aliases with the existing serials and stream profiles, and use the existing
capture root to preserve archive IDs. With this environment variable enabled,
xArm retains `/realsense/rs435i/*` and `/realsense/rs405/*` URLs, authentication,
and robot claim requirements. Snapshots, streams, diagnostics and archive data
come from the camera service. Status reads use cached telemetry so a camera
outage does not block robot status. Robot context attached to a capture is sampled
separately; it is not synchronized to exposure time.

The local `/usb` router is disabled in this mode to avoid a second camera owner.
The OT2 gateway owns its public `/cameras/overhead/*` facade, with a service token
scoped only to the overhead camera. This does not change robot motion endpoints.

## Validated deployment, 2026-09-25

Cytation runs `sdl-camera-server` from `C:\SDL_Deploy\sdl-camera-server`, listening
on `127.0.0.1:8070`. Private configuration, logs, acceptance reports and rollback
backup are under `C:\SDL_Data\camera-service`. D435i and D405 snapshots, depth,
streams, captures, concurrent frame progression and the HTE overhead C920 were
tested. Both robot status APIs stayed available during a camera-service restart;
all camera snapshots recovered. The HTE gateway stayed ready/idle. xArm requires
initialization after its deployment restart; validation did not initialize or move it.

Gibbie has a separate installation and virtual environment under
`C:\Users\sdl2\Projects\sdl-camera-server`, with service `sdl-camera-server` bound
only to `127.0.0.1:8071`. Its workflow folder was not touched. D435i alias `d435i`
supports `/v1/cameras/d435i/{status,snapshot.jpg,stream.mjpg,depth.png,intrinsics}`.
Private credentials, reports and captures stay in ignored `local/`.
Run `.venv\Scripts\python.exe local\camera_python_example.py` from that directory
for an HTTP Python client example. Color/depth at 1280x720, frame freshness,
streaming, Python access and service restart recovery passed. Remote network
access is pending approved caller scope; no firewall rule was opened.
