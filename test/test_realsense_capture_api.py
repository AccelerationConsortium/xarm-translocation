"""Tests for the capture + agent-documentation endpoints.

Companion to ``test_realsense_api.py``: the camera and the store are both
swapped for fakes so these exercise only the HTTP contract -- gating, status
codes, media types, and the fact that one capture pairs colour and depth from
a *single* frameset rather than two independent ``latest()`` reads.

The store itself is covered by ``test_realsense_captures.py``; the two suites
meet at the method names the fake implements.
"""

import os
import sys
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core import realsense_camera as rc  # noqa: E402
from src.core import realsense_captures as rcap  # noqa: E402
from src.core.realsense_camera import RealSenseNotStreaming  # noqa: E402
from src.core.xarm_api_server import app  # noqa: E402


JPEG = b"\xff\xd8\xff\xe0fakejpeg\xff\xd9"
PNG = b"\x89PNG\r\n\x1a\nfakepng"


class FakeCamera:
    """Only the surface the capture endpoint touches."""

    def __init__(self, start_on_demand=True):
        self.configured = True
        self.autostart = False  # the app lifespan reads this at startup
        self.start_on_demand = start_on_demand
        self.streaming = False
        self.label = "xArm depth camera"
        self.align_depth_to_color = True
        self.bundles = 0
        self.calls = []

    def describe(self):
        return {"configured": True, "installed": True, "streaming": self.streaming,
                "device": {"serial": "S1", "firmware": "5.11.1.100"},
                "library_version": "2.58.4",
                "streams": {"color": {"width": 640, "height": 480, "fps": 30}}}

    def ensure_started(self):
        self.calls.append("ensure_started")
        if self.streaming:
            return
        if not self.start_on_demand:
            raise RealSenseNotStreaming("pipeline stopped")
        self.streaming = True

    def latest(self, *, mark_consumer=True):
        self.calls.append("latest")
        if not self.streaming:
            raise RealSenseNotStreaming("pipeline stopped")
        self.bundles += 1
        # A distinct frame number per call: the capture must use one bundle.
        return SimpleNamespace(
            color=object(), depth=object(), depth_color=object(),
            depth_scale=0.001, intrinsics={"color": {"fx": 600.0}},
            frame_number=100 + self.bundles, timestamp_ms=1234.5, captured_at=0.0,
        )

    def encode_jpeg(self, bundle, kind="color"):
        self.calls.append(("encode_jpeg", bundle.frame_number, kind))
        return JPEG

    def encode_depth_png(self, bundle):
        self.calls.append(("encode_depth_png", bundle.frame_number))
        return PNG


class FakeClaimManager:
    def verify_token(self, token):  # cooperative: no claim held -> allowed
        return None

    def claimed_by(self):
        return {"session_id": "s1", "owner": "agent@lab"}


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
def store(tmp_path):
    previous = rcap.shared_store()
    configured = rcap.configure_shared(
        {"enabled": True, "root": str(tmp_path / "captures"), "keep_days": 30, "keep_max_gb": 20}
    )
    try:
        yield configured
    finally:
        rcap.set_shared(previous)


@pytest.fixture
def client(monkeypatch, fake_cam, store):
    """A connected-enough controller: the capture verb is claim-gated, and
    require_claim needs a controller object to reach its claim manager."""
    controller = SimpleNamespace(
        claim_manager=FakeClaimManager(),
        disconnect=lambda: None,  # the app lifespan calls this on shutdown
        is_connected=True,
        current_node="deck_1",
        current_gripper_state="open",
        last_joints=[0.0, 1.0, 2.0, 3.0, 4.0],
        last_position=[100.0, 0.0, 200.0, 180.0, 0.0, 0.0],
        last_track_position=42.0,
    )
    monkeypatch.setattr('src.core.xarm_api_server.controller', controller)
    with TestClient(app) as c:
        yield c


# ---------------------------------------------------------------------------
# Capture
# ---------------------------------------------------------------------------

class TestCapture:
    def test_capture_writes_and_returns_urls(self, client):
        response = client.post("/control/realsense/capture", json={"label": "arrival"})
        assert response.status_code == 200
        body = response.json()
        cid = body["capture_id"]
        assert body["urls"]["color"] == f"/realsense/captures/{cid}/color.jpg"
        assert body["urls"]["depth"] == f"/realsense/captures/{cid}/depth.png"
        assert body["meta"]["label"] == "arrival"

    def test_capture_uses_one_frameset_for_both_images(self, client, fake_cam):
        """Colour and depth must come from the same bundle, or the record
        pairs two different moments and stops being a measurement."""
        client.post("/control/realsense/capture", json={})
        encodes = [c for c in fake_cam.calls if isinstance(c, tuple)]
        frame_numbers = {c[1] for c in encodes}
        assert len(frame_numbers) == 1
        assert fake_cam.calls.count("latest") == 1

    def test_capture_records_arm_state(self, client):
        meta = client.post("/control/realsense/capture", json={}).json()["meta"]
        assert meta["arm"]["node_id"] == "deck_1"
        assert meta["arm"]["joints"] == [0.0, 1.0, 2.0, 3.0, 4.0]
        assert meta["arm"]["track_position"] == 42.0
        assert meta["arm"]["gripper_state"] == "open"
        assert meta["arm"]["connected"] is True

    def test_explicit_node_id_overrides_the_live_one(self, client):
        meta = client.post("/control/realsense/capture",
                           json={"node_id": "hood_2"}).json()["meta"]
        assert meta["arm"]["node_id"] == "hood_2"

    def test_capture_records_camera_and_frame_metadata(self, client):
        meta = client.post("/control/realsense/capture", json={}).json()["meta"]
        assert meta["camera"]["device"]["serial"] == "S1"
        assert meta["camera"]["library_version"] == "2.58.4"
        assert meta["frame"]["depth_scale_m"] == 0.001
        assert meta["frame"]["aligned_depth_to_color"] is True
        assert meta["intrinsics"] == {"color": {"fx": 600.0}}

    def test_capture_starts_the_pipeline_on_demand(self, client, fake_cam):
        assert fake_cam.streaming is False
        assert client.post("/control/realsense/capture", json={}).status_code == 200
        assert "ensure_started" in fake_cam.calls

    def test_capture_409_when_start_on_demand_is_off(self, client, fake_cam):
        fake_cam.start_on_demand = False
        response = client.post("/control/realsense/capture", json={})
        assert response.status_code == 409
        assert response.json()["detail"]["error"] == "realsense_not_streaming"

    def test_capture_404_when_store_disabled(self, client, monkeypatch):
        rcap.set_shared(rcap.CaptureStore({"enabled": False}))
        response = client.post("/control/realsense/capture", json={})
        assert response.status_code == 404
        assert response.json()["detail"]["error"] == "captures_not_configured"

    def test_capture_accepts_an_empty_body(self, client):
        assert client.post("/control/realsense/capture").status_code == 200

    def test_protected_flag_is_recorded(self, client):
        meta = client.post("/control/realsense/capture",
                           json={"protected": True}).json()["meta"]
        assert meta["protected"] is True


# ---------------------------------------------------------------------------
# Reading captures
# ---------------------------------------------------------------------------

class TestCaptureReads:
    def test_list_is_open_and_reports_retention(self, client):
        client.post("/control/realsense/capture", json={"label": "one"})
        body = client.get("/realsense/captures").json()
        assert body["count"] == 1
        assert body["retention"]["keep_days"] == 30
        assert body["retention"]["keep_max_gb"] == 20

    def test_list_filters_by_node(self, client):
        client.post("/control/realsense/capture", json={"node_id": "a"})
        client.post("/control/realsense/capture", json={"node_id": "b"})
        body = client.get("/realsense/captures", params={"node_id": "a"}).json()
        assert body["count"] == 1

    def test_meta_endpoint_returns_the_record(self, client):
        cid = client.post("/control/realsense/capture", json={}).json()["capture_id"]
        body = client.get(f"/realsense/captures/{cid}").json()
        assert body["capture_id"] == cid
        assert body["meta"]["capture_id"] == cid

    def test_meta_404_for_unknown_id(self, client):
        cid = rcap.new_capture_id()
        assert client.get(f"/realsense/captures/{cid}").status_code == 404

    def test_files_are_served_with_the_right_media_types(self, client):
        cid = client.post("/control/realsense/capture", json={}).json()["capture_id"]
        color = client.get(f"/realsense/captures/{cid}/color.jpg")
        depth = client.get(f"/realsense/captures/{cid}/depth.png")
        assert color.status_code == 200
        assert color.headers["content-type"] == "image/jpeg"
        assert color.content == JPEG
        assert depth.headers["content-type"] == "image/png"
        assert depth.content == PNG

    def test_unknown_filename_is_404_not_a_traversal(self, client):
        cid = client.post("/control/realsense/capture", json={}).json()["capture_id"]
        assert client.get(f"/realsense/captures/{cid}/meta.json.bak").status_code == 404

    def test_delete_removes_the_capture(self, client):
        cid = client.post("/control/realsense/capture", json={}).json()["capture_id"]
        assert client.delete(f"/control/realsense/captures/{cid}").status_code == 200
        assert client.get(f"/realsense/captures/{cid}").status_code == 404

    def test_delete_404_for_unknown_id(self, client):
        assert client.delete(
            f"/control/realsense/captures/{rcap.new_capture_id()}"
        ).status_code == 404


# ---------------------------------------------------------------------------
# Agent documentation
# ---------------------------------------------------------------------------

class TestAgentDocs:
    def test_agent_guide_is_markdown(self, client):
        response = client.get("/agent-docs")
        assert response.status_code == 200
        assert response.headers["content-type"].startswith("text/markdown")
        assert "xArm translocation" in response.text

    def test_api_reference_is_markdown(self, client):
        response = client.get("/agent-docs/api-reference")
        assert response.status_code == 200
        assert response.headers["content-type"].startswith("text/markdown")
        assert "/control/realsense/capture" in response.text

    def test_llms_txt_indexes_the_documents(self, client):
        response = client.get("/llms.txt")
        assert response.status_code == 200
        assert response.headers["content-type"].startswith("text/plain")
        for fragment in ("agent-docs", "agent-docs/api-reference", "openapi.json"):
            assert fragment in response.text

    def test_documents_are_open_reads(self, client):
        """Documentation is not actuation: no claim, no login."""
        for path in ("/agent-docs", "/agent-docs/api-reference", "/llms.txt"):
            assert client.get(path).status_code == 200

    def test_routes_are_advertised_in_openapi(self, client):
        paths = client.get("/openapi.json").json()["paths"]
        for path in ("/agent-docs", "/agent-docs/api-reference", "/llms.txt",
                     "/control/realsense/capture", "/realsense/captures"):
            assert path in paths
