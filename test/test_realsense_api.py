"""Tests for the /realsense/* endpoints (core/xarm_api_server.py).

The camera registry is swapped for scripted fakes via
``realsense_camera.set_cameras`` so these exercise only the HTTP contract:
the camera listing, addressing a camera by its device-local id, status codes
per failure class, media types, headers, the start-on-demand path, and that
the open reads never switch a camera on. The camera class itself is covered
by ``test_realsense_camera.py``; the two suites meet at the method names the
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

    def __init__(self, configured=True, start_on_demand=True, camera_id="rs435i",
                 label="xArm depth camera"):
        self.camera_id = camera_id
        self.label = label
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
        return {"camera_id": self.camera_id, "label": self.label,
                "configured": self.configured, "installed": True, "streaming": self.streaming,
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
            raise RealSenseNotStreaming(
                f"RealSense pipeline is stopped; POST /realsense/{self.camera_id}/start "
                "(start_on_demand is off)"
            )
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
def registry():
    """Install a registry for the duration of one test and put it back."""
    previous = rc.cameras()
    installed = {}

    def install(*cameras):
        installed.clear()
        installed.update({cam.camera_id: cam for cam in cameras})
        rc.set_cameras(installed)
        return installed

    yield install
    rc.set_cameras(previous)


@pytest.fixture
def fake_cam(registry):
    cam = FakeCamera()
    registry(cam)
    return cam


CAM = "rs435i"


@pytest.fixture
def client(monkeypatch, fake_cam):
    # No arm connected: every /realsense/* route must work regardless.
    monkeypatch.setattr('src.core.xarm_api_server.controller', None)
    with TestClient(app) as c:
        yield c


# ---------------------------------------------------------------------------
# Camera listing (the discovery endpoint)
# ---------------------------------------------------------------------------

def test_cameras_lists_the_single_camera_with_its_urls(client, fake_cam):
    body = client.get("/realsense/cameras").json()
    assert body["default"] == CAM and body["reason"] is None
    assert [c["id"] for c in body["cameras"]] == [CAM]
    entry = body["cameras"][0]
    assert entry["label"] == "xArm depth camera"
    assert entry["state"] == "off" and entry["streaming"] is False
    assert entry["start_on_demand"] is True
    assert entry["urls"] == {
        "status": f"/realsense/{CAM}/status",
        "snapshot": f"/realsense/{CAM}/snapshot.jpg",
        "depth_png": f"/realsense/{CAM}/depth.png",
        "stream": f"/realsense/{CAM}/stream.mjpg",
        "depth": f"/realsense/{CAM}/depth",
        "intrinsics": f"/realsense/{CAM}/intrinsics",
        "captures": f"/realsense/{CAM}/captures",
        "capture": f"/control/realsense/{CAM}/capture",
    }
    assert "start" not in fake_cam.calls          # listing never switches a camera on


def test_cameras_lists_two_cameras_and_has_no_default(client, registry):
    registry(FakeCamera(camera_id="rs435i"), FakeCamera(camera_id="overhead"))
    body = client.get("/realsense/cameras").json()
    assert [c["id"] for c in body["cameras"]] == ["overhead", "rs435i"]   # sorted by id
    assert body["default"] is None                # two cameras: nothing to assume
    assert body["reason"] is None


def test_cameras_is_empty_with_a_reason_when_none_are_configured(client, registry):
    registry()
    body = client.get("/realsense/cameras").json()
    assert body == {"cameras": [], "default": None,
                    "reason": "no RealSense cameras configured"}


def test_cameras_listing_is_an_open_read(client):
    deps = _route_dependency_names("/realsense/cameras", "GET")
    assert "require_login" not in deps and "require_claim" not in deps


# ---------------------------------------------------------------------------
# Status / configuration
# ---------------------------------------------------------------------------

def test_status_answers_before_connect(client, fake_cam):
    r = client.get(f"/realsense/{CAM}/status")
    assert r.status_code == 200
    body = r.json()
    assert body["configured"] is True and body["streaming"] is False
    assert body["reason"] == "pipeline stopped"
    assert "start" not in fake_cam.calls          # a read never switches the camera on


@pytest.mark.parametrize("method,path", [
    ("get", f"/realsense/{CAM}/status"),
    ("post", f"/realsense/{CAM}/start"), ("post", f"/realsense/{CAM}/stop"),
    ("get", f"/realsense/{CAM}/snapshot.jpg"), ("get", f"/realsense/{CAM}/depth.png"),
    ("get", f"/realsense/{CAM}/stream.mjpg"), ("get", f"/realsense/{CAM}/depth?x=1&y=1"),
    ("get", f"/realsense/{CAM}/intrinsics"),
])
def test_everything_404s_when_no_camera_is_configured(client, registry, method, path):
    """No cameras at all is a different refusal from an unknown id: the first
    says the feature is off, the second says you named the wrong lens."""
    registry()
    r = getattr(client, method)(path)
    assert r.status_code == 404
    assert r.json()["detail"]["error"] == "realsense_not_configured"
    assert r.json()["detail"]["hint"]


@pytest.mark.parametrize("method,path", [
    ("get", "/realsense/nope/status"),
    ("post", "/realsense/nope/start"), ("post", "/realsense/nope/stop"),
    ("get", "/realsense/nope/snapshot.jpg"), ("get", "/realsense/nope/depth.png"),
    ("get", "/realsense/nope/stream.mjpg"), ("get", "/realsense/nope/depth?x=1&y=1"),
    ("get", "/realsense/nope/intrinsics"), ("get", "/realsense/nope/captures"),
])
def test_unknown_camera_is_404_listing_the_ids_that_exist(client, registry, method, path):
    registry(FakeCamera(camera_id="rs435i"), FakeCamera(camera_id="overhead"))
    r = getattr(client, method)(path)
    assert r.status_code == 404
    detail = r.json()["detail"]
    assert detail["error"] == "camera_not_found"
    assert detail["camera_id"] == "nope"
    assert detail["cameras"] == ["overhead", "rs435i"]


def test_a_camera_is_addressed_by_its_own_id(client, registry):
    """Two cameras, two pipelines: starting one must not start the other."""
    first, second = FakeCamera(camera_id="rs435i"), FakeCamera(camera_id="overhead")
    registry(first, second)
    assert client.post("/realsense/overhead/start").status_code == 200
    assert second.streaming and not first.streaming
    assert client.get("/realsense/rs435i/status").json()["streaming"] is False


# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

def test_start_and_stop(client, fake_cam):
    r = client.post(f"/realsense/{CAM}/start")
    assert r.status_code == 200 and r.json()["streaming"] is True
    assert fake_cam.streaming
    r = client.post(f"/realsense/{CAM}/stop")
    assert r.status_code == 200 and r.json()["streaming"] is False
    assert not fake_cam.streaming


def test_start_unavailable_is_503_with_reason(client, fake_cam):
    fake_cam.start_error = RealSenseUnavailable("no RealSense device connected")
    r = client.post(f"/realsense/{CAM}/start")
    assert r.status_code == 503
    assert r.json()["detail"] == {"error": "realsense_unavailable",
                                  "reason": "no RealSense device connected"}


def test_start_librealsense_failure_is_502(client, fake_cam):
    fake_cam.start_error = RealSenseError("RealSense pipeline failed to start: Couldn't resolve requests")
    r = client.post(f"/realsense/{CAM}/start")
    assert r.status_code == 502
    assert r.json()["detail"]["error"] == "realsense_error"
    assert "Couldn't resolve requests" in r.json()["detail"]["reason"]


# ---------------------------------------------------------------------------
# Images
# ---------------------------------------------------------------------------

def test_snapshot_starts_on_demand_and_returns_jpeg(client, fake_cam):
    r = client.get(f"/realsense/{CAM}/snapshot.jpg")
    assert r.status_code == 200
    assert r.headers["content-type"] == "image/jpeg"
    assert r.headers["cache-control"] == "no-store"
    assert r.headers["x-frame-number"] == "42"
    assert r.headers["x-frame-timestamp-ms"] == "1234.5"
    assert r.content == JPEG
    assert fake_cam.calls[:2] == ["ensure_started", "start"]
    assert ("jpeg", "color") in fake_cam.calls


def test_snapshot_depth_kind(client, fake_cam):
    r = client.get(f"/realsense/{CAM}/snapshot.jpg?stream=depth")
    assert r.status_code == 200 and ("jpeg", "depth") in fake_cam.calls


def test_snapshot_bad_kind_is_400(client, fake_cam):
    r = client.get(f"/realsense/{CAM}/snapshot.jpg?stream=infrared")
    assert r.status_code == 400
    assert r.json()["detail"]["error"] == "bad_request"
    assert "start" not in fake_cam.calls


def test_snapshot_respects_start_on_demand_off(client, fake_cam):
    fake_cam.start_on_demand = False
    r = client.get(f"/realsense/{CAM}/snapshot.jpg")
    assert r.status_code == 409
    assert r.json()["detail"]["error"] == "realsense_not_streaming"
    assert "start" not in fake_cam.calls


def test_snapshot_unavailable_is_503(client, fake_cam):
    fake_cam.start_error = RealSenseUnavailable("pyrealsense2 not installed")
    r = client.get(f"/realsense/{CAM}/snapshot.jpg")
    assert r.status_code == 503


def test_snapshot_disabled_stream_is_502(client, fake_cam):
    fake_cam.depth_disabled = True
    r = client.get(f"/realsense/{CAM}/snapshot.jpg?stream=depth")
    assert r.status_code == 502
    assert "depth stream is disabled" in r.json()["detail"]["reason"]


def test_depth_png(client, fake_cam):
    r = client.get(f"/realsense/{CAM}/depth.png")
    assert r.status_code == 200
    assert r.headers["content-type"] == "image/png"
    assert r.headers["x-depth-scale-m"] == "0.001"
    assert r.content == PNG


def test_mjpeg_stream(client, fake_cam):
    with client.stream("GET", f"/realsense/{CAM}/stream.mjpg?stream=depth&fps=5") as r:
        assert r.status_code == 200
        assert r.headers["content-type"] == "multipart/x-mixed-replace; boundary=xarm-realsense-frame"
        assert r.headers["cache-control"] == "no-store"
        body = b"".join(r.iter_bytes())
    assert body == b"".join(fake_cam.mjpeg_parts)
    assert ("mjpeg", "depth", 5.0) in fake_cam.calls
    assert "start" in fake_cam.calls              # started on demand


def test_mjpeg_fps_is_clamped(client, fake_cam):
    with client.stream("GET", f"/realsense/{CAM}/stream.mjpg?fps=500") as r:
        b"".join(r.iter_bytes())
    assert ("mjpeg", "color", 30.0) in fake_cam.calls


def test_mjpeg_unavailable_is_503_before_streaming(client, fake_cam):
    fake_cam.start_error = RealSenseUnavailable("no RealSense device connected")
    r = client.get(f"/realsense/{CAM}/stream.mjpg")
    assert r.status_code == 503


# ---------------------------------------------------------------------------
# Numeric reads
# ---------------------------------------------------------------------------

def test_depth_at_does_not_start_camera(client, fake_cam):
    r = client.get(f"/realsense/{CAM}/depth?x=10&y=20")
    assert r.status_code == 409
    assert r.json()["detail"]["error"] == "realsense_not_streaming"
    assert "start" not in fake_cam.calls and "ensure_started" not in fake_cam.calls


def test_depth_at_when_streaming(client, fake_cam):
    fake_cam.streaming = True
    r = client.get(f"/realsense/{CAM}/depth?x=10&y=20")
    assert r.status_code == 200
    assert r.json()["distance_m"] == 0.5 and r.json()["window"] == 5   # default window
    assert ("depth_at", 10, 20, 5) in fake_cam.calls
    r = client.get(f"/realsense/{CAM}/depth?x=10&y=20&window=1")
    assert r.json()["window"] == 1


def test_depth_at_out_of_bounds_is_400(client, fake_cam):
    fake_cam.streaming = True
    r = client.get(f"/realsense/{CAM}/depth?x=640&y=0")
    assert r.status_code == 400
    assert "outside the 640x480" in r.json()["detail"]["reason"]


def test_depth_at_requires_coordinates(client, fake_cam):
    assert client.get(f"/realsense/{CAM}/depth").status_code == 422


def test_intrinsics(client, fake_cam):
    r = client.get(f"/realsense/{CAM}/intrinsics")
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


# The routes are registered with their path template, so gating is asserted
# against "/realsense/{camera_id}/..." rather than one camera's concrete path.
@pytest.mark.parametrize("path,method", [
    ("/realsense/{camera_id}/start", "POST"), ("/realsense/{camera_id}/stop", "POST"),
    ("/realsense/{camera_id}/snapshot.jpg", "GET"), ("/realsense/{camera_id}/depth.png", "GET"),
    ("/realsense/{camera_id}/stream.mjpg", "GET"),
])
def test_camera_on_and_video_routes_are_login_gated(path, method):
    deps = _route_dependency_names(path, method)
    assert "require_login" in deps
    assert "require_claim" not in deps        # looking is not arm actuation


@pytest.mark.parametrize("path", ["/realsense/cameras", "/realsense/{camera_id}/status",
                                  "/realsense/{camera_id}/depth",
                                  "/realsense/{camera_id}/intrinsics"])
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
