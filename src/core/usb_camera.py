"""Local color-camera discovery and capture; independent of the arm and HTTP.

Optional OpenCV dependencies are loaded only on discovery. Enumeration never
starts video capture. Each active camera has one thread shared by all viewers.
"""
from __future__ import annotations

import hashlib
import platform
import threading
import time
from pathlib import Path


class CameraUnavailable(RuntimeError):
    pass


class OpenCVBackend:
    def __init__(self):
        try:
            import cv2
            from cv2_enumerate_cameras import enumerate_cameras
        except (ImportError, OSError) as exc:
            raise CameraUnavailable("USB camera support requires uv sync --extra usb-camera") from exc
        self.cv2 = cv2
        self.enumerate = enumerate_cameras
        self.api = {"Windows": cv2.CAP_DSHOW, "Linux": cv2.CAP_V4L2,
                    "Darwin": cv2.CAP_AVFOUNDATION}.get(platform.system())
        if self.api is None:
            raise CameraUnavailable("Unsupported camera platform")

    def devices(self):
        devices = []
        for info in self.enumerate(self.api):
            # RealSense RGB interfaces must not compete with librealsense.
            if "realsense" in (info.name or "").lower():
                continue
            path = str(info.path or "")
            identity = path or f"{info.backend}:{info.index}:{info.name}"
            stable = bool(path)
            if platform.system() == "Linux":
                stable = False
                for link in sorted(Path('/dev/v4l/by-id').glob('*')):
                    if path and link.resolve() == Path(path).resolve():
                        identity, stable = str(link), True
                        break
            devices.append({
                "id": "usb-" + hashlib.sha256(identity.encode()).hexdigest()[:16],
                "name": info.name, "path": path, "index": info.index,
                "backend": info.backend, "vid": info.vid, "pid": info.pid,
                "identity": identity, "persistent_identity": stable,
            })
        return devices

    def open(self, device):
        return self.cv2.VideoCapture(device['index'], device['backend'])

    def encode(self, frame):
        ok, data = self.cv2.imencode('.jpg', frame, [self.cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            raise CameraUnavailable('JPEG encoding failed')
        return data.tobytes()


class USBCamera:
    def __init__(self, device, backend, idle_timeout=30.0):
        self.device = dict(device)
        self.backend = backend
        self.present = True
        self._condition = threading.Condition()
        self._thread = None
        self._stop = threading.Event()
        self._jpeg = None
        self._frame_number = 0
        self._frame_at = 0.0
        self._reason = None
        self.idle_timeout = idle_timeout
        self._last_consumer_at = time.monotonic()

    def describe(self):
        with self._condition:
            running = bool(self._thread and self._thread.is_alive() and not self._stop.is_set())
            return {"id": self.device['id'], "kind": "usb", "label": self.device['name'],
                    "device": dict(self.device), "present": self.present,
                    "streaming": running and self._jpeg is not None,
                    "state": ('disconnected' if not self.present else
                              'error' if self._reason else
                              'stopping' if self._thread and self._thread.is_alive() and self._stop.is_set()
                              else 'streaming' if running and self._jpeg
                              else 'starting' if running else 'off'),
                    "reason": self._reason, "capabilities": ["color", "snapshot", "mjpeg"],
                    "frame_number": self._frame_number,
                    "frame_age_s": time.monotonic() - self._frame_at if self._jpeg else None}

    def start(self):
        with self._condition:
            self._last_consumer_at = time.monotonic()
            if not self.present:
                raise CameraUnavailable('Camera disconnected; refresh /usb/cameras')
            if self._thread and self._thread.is_alive():
                if self._stop.is_set():
                    raise CameraUnavailable('Previous capture is still stopping')
                return
            self._stop = threading.Event()
            self._jpeg = None
            self._reason = None
            self._thread = threading.Thread(target=self._capture, daemon=True,
                                            name=f"camera-{self.device['id']}")
            self._thread.start()

    def _capture(self):
        cap = None
        try:
            cap = self.backend.open(dict(self.device))
            if not cap.isOpened():
                raise CameraUnavailable('Cannot open camera (disconnected or in use)')
            while not self._stop.is_set():
                if self.idle_timeout and time.monotonic() - self._last_consumer_at > self.idle_timeout:
                    break
                ok, frame = cap.read()
                if not ok:
                    raise CameraUnavailable('Camera stopped delivering frames')
                jpeg = self.backend.encode(frame)
                with self._condition:
                    self._jpeg = jpeg
                    self._frame_number += 1
                    self._frame_at = time.monotonic()
                    self._condition.notify_all()
        except Exception as exc:
            with self._condition:
                self._reason = str(exc)
        finally:
            try:
                if cap is not None:
                    cap.release()
            finally:
                with self._condition:
                    self._stop.set()
                    self._jpeg = None
                    self._condition.notify_all()

    def jpeg(self, timeout=5.0, after=0):
        deadline = time.monotonic() + timeout
        with self._condition:
            self._last_consumer_at = time.monotonic()
            while True:
                if self._stop.is_set() or not self._thread:
                    raise CameraUnavailable(self._reason or 'Camera stopped')
                if (self._jpeg and self._frame_number > after
                        and time.monotonic() - self._frame_at < 2.0):
                    return self._jpeg, self._frame_number
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise CameraUnavailable('Timed out waiting for a fresh camera frame')
                self._condition.wait(remaining)

    def stop(self):
        with self._condition:
            self._stop.set()
            self._jpeg = None
            thread = self._thread
            self._condition.notify_all()
        if thread:
            # A faulty native driver can block read(); never block the API
            # indefinitely or start a second owner while that thread is alive.
            thread.join(timeout=2.0)


class CameraManager:
    def __init__(self, backend=None):
        self._backend = backend
        self._cameras = {}
        self._lock = threading.RLock()

    def discover(self):
        with self._lock:
            if self._backend is None:
                self._backend = OpenCVBackend()
            try:
                devices = self._backend.devices()
            except Exception as exc:
                raise CameraUnavailable(f'Camera enumeration failed: {exc}') from exc
            found = {device['id'] for device in devices}
            for key, camera in self._cameras.items():
                if key not in found and camera.present:
                    camera.present = False
                    camera.stop()
            for device in devices:
                key = device['id']
                if key not in self._cameras:
                    self._cameras[key] = USBCamera(device, self._backend)
                else:
                    camera = self._cameras[key]
                    if camera.device['index'] != device['index']:
                        camera.stop()
                    camera.device = dict(device)
                    camera.present = True
            return [self._cameras[d['id']].describe() for d in devices]

    def get(self, camera_id):
        # Re-enumerate so an index change cannot silently select another camera.
        self.discover()
        with self._lock:
            camera = self._cameras.get(camera_id)
            if camera is None or not camera.present:
                raise KeyError(camera_id)
            return camera

    def stop_all(self):
        with self._lock:
            for camera in self._cameras.values():
                camera.stop()
