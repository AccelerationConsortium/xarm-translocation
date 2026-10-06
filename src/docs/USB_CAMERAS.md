# USB color cameras

Local implementation only; not deployed or hardware-verified on the xArm PC.

The USB camera manager and FastAPI router are independent of the arm controller.
The router accepts the host application's login dependency, so another lab PC's
service can reuse it without importing xArm control code. This is not yet a
separately published package or standalone camera service.

## Installation

On the computer physically connected to the camera:

```sh
uv sync --extra realsense --extra usb-camera
```

Keep `--extra realsense` on computers using RealSense; `uv sync` removes extras
not selected. A USB-only host needs just `--extra usb-camera`. OpenCV and the
OS enumeration library are optional; missing libraries do not prevent service
startup. No USB cameras are opened at startup.

## Discovery and endpoints

| Method | Path | Purpose |
|---|---|---|
| GET | `/cameras` | Combined configured RealSense and discovered USB camera catalog |
| GET | `/usb/cameras` | Refresh USB enumeration; returns device IDs and URLs |
| GET | `/usb/{id}/status` | Device identity and capture state; does not start capture |
| POST | `/usb/{id}/start` | Start capture and wait for the first frame |
| POST | `/usb/{id}/stop` | Stop capture for all viewers of this camera |
| GET | `/usb/{id}/snapshot.jpg` | JPEG snapshot, starting capture if necessary |
| GET | `/usb/{id}/stream.mjpg?fps=10` | MJPEG preview; output rate 0.5–30 fps |

Follow the `urls` returned by discovery; do not guess numeric capture indices.
Discovery reports the device name, OS path, current OpenCV index/backend, USB
vendor/product IDs when available, and whether a persistent identity was found.
`default` is non-null only when exactly one USB camera is discovered. An empty
list with `available: true` means no devices; `available: false` includes a reason
such as a missing driver or enumeration failure.

Enumeration uses DirectShow on Windows, V4L2 on Linux and AVFoundation on macOS.
These OS backends are selected automatically using
[cv2-enumerate-cameras](https://github.com/lukehugh/cv2_enumerate_cameras).
No COM port or manually configured camera index is needed. Linux users require
permission to access video devices; Windows/macOS camera privacy settings apply.
Integrated webcams can appear too. Interfaces named RealSense are excluded from
USB discovery so the existing RealSense driver retains ownership.

IDs derive from OS device paths, preferring `/dev/v4l/by-id` on Linux. They remain
stable across index changes when persistent paths are available. Moving a camera
to another USB port or replacing a camera without a serial can change its ID;
`persistent_identity: false` warns that only a transient path/index was available.
Refresh discovery after plugging in or moving hardware. Reconnection is on demand,
not an automatic reopening of a failed stream. Discovery does not start streaming
(Linux enumeration may open a device descriptor to query its capabilities).

Start, stop and video routes use the same login dependency as RealSense video;
they do not require a robot claim and work before `/connect`. Unknown cameras
return 404; camera open/read failures return 503. Frame waits time out after five
seconds, stale frames are not returned, and video responses disable caching.

Each camera has one capture thread shared by all viewers. It releases the camera
after 30 seconds without a frame consumer, on explicit stop, or during service
shutdown. A stream disconnect leaves other viewers running. Some native drivers
can hang inside capture; stop waits at most two seconds and prevents a second
capture owner until the old thread exits. This thread-based implementation does
not provide process isolation from native driver crashes.

USB cameras advertise color, snapshot and MJPEG capabilities only. No stereo
depth, calibrated intrinsics, or durable capture archive is fabricated. Existing
RealSense routes, capture formats, and all robot/freehand routes are unchanged.

## Validation

`test/test_usb_camera.py` covers discovery, changing indices, one-owner capture,
missing libraries, unplug/replug, stale-frame rejection, idle release and blocked
drivers. `test/test_usb_camera_api.py` covers discovery/media contracts,
authentication, error responses, streaming and application integration using
fake cameras. Physical camera enumeration and images must still be checked on
the target computer after a separately authorized deployment.
