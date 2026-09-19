"""Tests for the /realsense/* endpoints (core/xarm_api_server.py).

The camera is swapped for a scripted fake via ``realsense_camera.set_shared``
so these exercise only the HTTP contract: status codes per failure class,
media types, headers, the start-on-demand path, and that the open reads
never switch the camera on. The camera class itself is covered by
``test_realsense_camera.py``; the two suites meet at the method names the
fake implements.
"""

import os
import sys
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core import realsense_camera as rc  # noqa: E402
from src.core.realsense_camera import (  # noqa: E402
    RealSenseError,
    RealSenseNotStreaming,
    RealSenseUnavailable,
)
from src.core.xarm_api_server import app  # noqa: E402


JPEG = b"\xff\xd8\xff\xe0fakejpeg\xff\xd9"
PNG = b"\x89PNG\r\n\x1a\nfakepng"


class FakeCamera:
    """Scripted stand-in implementing the surface the endpoints call."""

    def __init__(self, configured=True, start_on_demand=True):
        self.configured = configured
        self.autostart = False
        self.start_on_demand = start_on_demand
        self.streaming = False
        self.start_error = None      # exception instance to raise from start()
        self.calls = []
        self.frame = SimpleNamespace(frame_number=42, timestamp_ms=1234.5, depth_scale=0.001)
        self.mjpeg_parts = [b"--xarm-realsense-frame\r\npart1\r\n", b"--xarm-realsense-frame\r\npart2\r\n"]

    def describe(self):
        self.calls.append("describe")
        return {"configured": self.configured, "installed": True, "streaming": self.streaming,
                "state": "streaming" if self.streaming else "off", "devices": [{"serial": "S1"}],
                "device": None, "reason": None if self.streaming else "pipeline stopped",
                "warnings": [], "fps_measured": 29.9 if self.streaming else None}

    def start(self):
        self.calls.append("start")
        if self.start_error is not None:
            raise self.start_error
        self.streaming = True
        return self.describe()

    def stop(self):
        self.calls.append("stop")
        self.streaming = False

    def ensure_started(self):
        self.calls.append("ensure_started")
        if self.streaming:
            return
        if not self.start_on_demand:
            raise RealSenseNotStreaming("RealSense pipeline is stopped; POST /realsense/start (start_on_demand is off)")
        self.start()

    def _need_stream(self):
        if not self.streaming:
            raise RealSenseNotStreaming("RealSense pipeline is stopped")

    def jpeg(self, kind="color"):
        self.calls.append(("jpeg", kind))
        self._need_stream()
        if kind == "depth" and getattr(self, "depth_disabled", False):
            raise RealSenseError("depth stream is disabled in realsense.yaml")
        return JPEG, self.frame

    def depth_png(self):
        self.calls.append("depth_png")
        self._need_stream()
        return PNG, self.frame

    def mjpeg_frames(self, kind="color", max_fps=10.0):
        self.calls.append(("mjpeg", kind, max_fps))
        self._need_stream()
        yield from self.mjpeg_parts

    @staticmethod
    def mjpeg_content_type():
        return "multipart/x-mixed-replace; boundary=xarm-realsense-frame"

    def depth_at(self, x, y, *, window=1):
        self.calls.append(("depth_at", x, y, window))
        self._need_stream()
        if x >= 640:
            raise ValueError(f"pixel ({x}, {y}) outside the 640x480 depth map")
        return {"pixel": [x, y], "window": window, "distance_m": 0.5, "point_m": [0.0, 0.0, 0.5],
                "valid_samples": window * window, "frame_number": 42, "frame": "color"}

    def intrinsics(self):
        self.calls.append("intrinsics")
        return {"streaming": self.streaming, "depth_scale_m": 0.001, "aligned_to": "color",
                "streams": {"color": {"fx": 600.0}}}


@pytest.fixture
def fake_cam():
    cam = FakeCamera()
    previous = rc.shared_camera()
    rc.set_shared(cam)
    try:
        yield cam
    finally:
        rc.set_shared(previous)


@pytest.fixture
def client(monkeypatch, fake_cam):
    # No arm connected: every /realsense/* route must work regardless.
    monkeypatch.setattr('src.core.xarm_api_server.controller', None)
    with TestClient(app) as c:
        yield c


# ---------------------------------------------------------------------------
# Status / configuration
# ---------------------------------------------------------------------------

def test_status_answers_before_connect(client, fake_cam):
    r = client.get("/realsense/status")
    assert r.status_code == 200
    body = r.json()
    assert body["configured"] is True and body["streaming"] is False
    assert body["reason"] == "pipeline stopped"
    assert "start" not in fake_cam.calls          # a read never switches the camera on


def test_status_when_unconfigured_is_200_not_404(client, fake_cam):
    fake_cam.configured = False
    r = client.get("/realsense/status")
    assert r.status_code == 200 and r.json()["configured"] is False


def test_status_without_shared_camera(monkeypatch, client):
    rc.set_shared(None)
    r = client.get("/realsense/status")
    assert r.status_code == 200
    assert r.json() == {"configured": False, "installed": False,
                        "reason": "realsense module not initialised"}


@pytest.mark.parametrize("method,path", [
    ("post", "/realsense/start"), ("post", "/realsense/stop"),
    ("get", "/realsense/snapshot.jpg"), ("get", "/realsense/depth.png"),
    ("get", "/realsense/stream.mjpg"), ("get", "/realsense/depth?x=1&y=1"),
    ("get", "/realsense/intrinsics"),
])
def test_everything_else_404s_when_unconfigured(client, fake_cam, method, path):
    fake_cam.configured = False
    r = getattr(client, method)(path)
    assert r.status_code == 404
    assert r.json()["detail"]["error"] == "realsense_not_configured"
    assert fake_cam.calls == []


# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

def test_start_and_stop(client, fake_cam):
    r = client.post("/realsense/start")
    assert r.status_code == 200 and r.json()["streaming"] is True
    assert fake_cam.streaming
    r = client.post("/realsense/stop")
    assert r.status_code == 200 and r.json()["streaming"] is False
    assert not fake_cam.streaming


def test_start_unavailable_is_503_with_reason(client, fake_cam):
    fake_cam.start_error = RealSenseUnavailable("no RealSense device connected")
    r = client.post("/realsense/start")
    assert r.status_code == 503
    assert r.json()["detail"] == {"error": "realsense_unavailable",
                                  "reason": "no RealSense device connected"}


def test_start_librealsense_failure_is_502(client, fake_cam):
    fake_cam.start_error = RealSenseError("RealSense pipeline failed to start: Couldn't resolve requests")
    r = client.post("/realsense/start")
    assert r.status_code == 502
    assert r.json()["detail"]["error"] == "realsense_error"
    assert "Couldn't resolve requests" in r.json()["detail"]["reason"]


# ---------------------------------------------------------------------------
# Images
# ---------------------------------------------------------------------------

def test_snapshot_starts_on_demand_and_returns_jpeg(client, fake_cam):
    r = client.get("/realsense/snapshot.jpg")
    assert r.status_code == 200
    assert r.headers["content-type"] == "image/jpeg"
    assert r.headers["cache-control"] == "no-store"
    assert r.headers["x-frame-number"] == "42"
    assert r.headers["x-frame-timestamp-ms"] == "1234.5"
    assert r.content == JPEG
    assert fake_cam.calls[:2] == ["ensure_started", "start"]
    assert ("jpeg", "color") in fake_cam.calls


def test_snapshot_depth_kind(client, fake_cam):
    r = client.get("/realsense/snapshot.jpg?stream=depth")
    assert r.status_code == 200 and ("jpeg", "depth") in fake_cam.calls


def test_snapshot_bad_kind_is_400(client, fake_cam):
    r = client.get("/realsense/snapshot.jpg?stream=infrared")
    assert r.status_code == 400
    assert r.json()["detail"]["error"] == "bad_request"
    assert "start" not in fake_cam.calls


def test_snapshot_respects_start_on_demand_off(client, fake_cam):
    fake_cam.start_on_demand = False
    r = client.get("/realsense/snapshot.jpg")
    assert r.status_code == 409
    assert r.json()["detail"]["error"] == "realsense_not_streaming"
    assert "start" not in fake_cam.calls


def test_snapshot_unavailable_is_503(client, fake_cam):
    fake_cam.start_error = RealSenseUnavailable("pyrealsense2 not installed")
    r = client.get("/realsense/snapshot.jpg")
    assert r.status_code == 503


def test_snapshot_disabled_stream_is_502(client, fake_cam):
    fake_cam.depth_disabled = True
    r = client.get("/realsense/snapshot.jpg?stream=depth")
    assert r.status_code == 502
    assert "depth stream is disabled" in r.json()["detail"]["reason"]


def test_depth_png(client, fake_cam):
    r = client.get("/realsense/depth.png")
    assert r.status_code == 200
    assert r.headers["content-type"] == "image/png"
    assert r.headers["x-depth-scale-m"] == "0.001"
    assert r.content == PNG


def test_mjpeg_stream(client, fake_cam):
    with client.stream("GET", "/realsense/stream.mjpg?stream=depth&fps=5") as r:
        assert r.status_code == 200
        assert r.headers["content-type"] == "multipart/x-mixed-replace; boundary=xarm-realsense-frame"
        assert r.headers["cache-control"] == "no-store"
        body = b"".join(r.iter_bytes())
    assert body == b"".join(fake_cam.mjpeg_parts)
    assert ("mjpeg", "depth", 5.0) in fake_cam.calls
    assert "start" in fake_cam.calls              # started on demand


def test_mjpeg_fps_is_clamped(client, fake_cam):
    with client.stream("GET", "/realsense/stream.mjpg?fps=500") as r:
        b"".join(r.iter_bytes())
    assert ("mjpeg", "color", 30.0) in fake_cam.calls


def test_mjpeg_unavailable_is_503_before_streaming(client, fake_cam):
    fake_cam.start_error = RealSenseUnavailable("no RealSense device connected")
    r = client.get("/realsense/stream.mjpg")
    assert r.status_code == 503


# ---------------------------------------------------------------------------
# Numeric reads
# ---------------------------------------------------------------------------

def test_depth_at_does_not_start_camera(client, fake_cam):
    r = client.get("/realsense/depth?x=10&y=20")
    assert r.status_code == 409
    assert r.json()["detail"]["error"] == "realsense_not_streaming"
    assert "start" not in fake_cam.calls and "ensure_started" not in fake_cam.calls


def test_depth_at_when_streaming(client, fake_cam):
    fake_cam.streaming = True
    r = client.get("/realsense/depth?x=10&y=20")
    assert r.status_code == 200
    assert r.json()["distance_m"] == 0.5 and r.json()["window"] == 5   # default window
    assert ("depth_at", 10, 20, 5) in fake_cam.calls
    r = client.get("/realsense/depth?x=10&y=20&window=1")
    assert r.json()["window"] == 1


def test_depth_at_out_of_bounds_is_400(client, fake_cam):
    fake_cam.streaming = True
    r = client.get("/realsense/depth?x=640&y=0")
    assert r.status_code == 400
    assert "outside the 640x480" in r.json()["detail"]["reason"]


def test_depth_at_requires_coordinates(client, fake_cam):
    assert client.get("/realsense/depth").status_code == 422


def test_intrinsics(client, fake_cam):
    r = client.get("/realsense/intrinsics")
    assert r.status_code == 200
    assert r.json()["aligned_to"] == "color"
    assert "start" not in fake_cam.calls


# ---------------------------------------------------------------------------
# Gating
# ---------------------------------------------------------------------------

def _route_dependency_names(path, method):
    for route in app.routes:
        if getattr(route, "path", None) == path and method in getattr(route, "methods", set()):
            return {d.call.__name__ for d in route.dependant.dependencies}
    raise AssertionError(f"route {method} {path} not found")


@pytest.mark.parametrize("path,method", [
    ("/realsense/start", "POST"), ("/realsense/stop", "POST"),
    ("/realsense/snapshot.jpg", "GET"), ("/realsense/depth.png", "GET"),
    ("/realsense/stream.mjpg", "GET"),
])
def test_camera_on_and_video_routes_are_login_gated(path, method):
    deps = _route_dependency_names(path, method)
    assert "require_login" in deps
    assert "require_claim" not in deps        # looking is not arm actuation


@pytest.mark.parametrize("path", ["/realsense/status", "/realsense/depth", "/realsense/intrinsics"])
def test_numeric_reads_are_open(path):
    deps = _route_dependency_names(path, "GET")
    assert "require_login" not in deps and "require_claim" not in deps


def test_lifespan_autostart_and_shutdown(monkeypatch, fake_cam):
    fake_cam.autostart = True
    monkeypatch.setattr('src.core.xarm_api_server.controller', None)
    with TestClient(app):
        assert fake_cam.streaming and "start" in fake_cam.calls
    assert not fake_cam.streaming and fake_cam.calls[-1] == "stop"


def test_lifespan_autostart_failure_is_not_fatal(monkeypatch, fake_cam):
    fake_cam.autostart = True
    fake_cam.start_error = RealSenseUnavailable("no RealSense device connected")
    monkeypatch.setattr('src.core.xarm_api_server.controller', None)
    with TestClient(app) as c:
        assert c.get("/health").status_code == 200
    assert not fake_cam.streaming
