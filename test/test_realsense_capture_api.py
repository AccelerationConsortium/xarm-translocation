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

    def __init__(self, start_on_demand=True, camera_id="rs435i"):
        self.camera_id = camera_id
        self.configured = True
        self.autostart = False  # the app lifespan reads this at startup
        self.start_on_demand = start_on_demand
        self.streaming = False
        self.label = "xArm depth camera"
        self.align_depth_to_color = True
        self.bundles = 0
        self.calls = []

    def describe(self):
        return {"camera_id": self.camera_id, "configured": True, "installed": True,
                "streaming": self.streaming,
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


CAM = "rs435i"


@pytest.fixture
def registry():
    previous = rc.cameras()

    def install(*cameras):
        rc.set_cameras({cam.camera_id: cam for cam in cameras})

    yield install
    rc.set_cameras(previous)


@pytest.fixture
def fake_cam(registry):
    cam = FakeCamera()
    registry(cam)
    return cam


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
        # Connection is read from states["connection"], exactly as /status
        # reads it -- there is no is_connected attribute on the real
        # controller, and assuming one stamped every capture disconnected.
        states={"connection": SimpleNamespace(value="enabled")},
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
        response = client.post(f"/control/realsense/{CAM}/capture", json={"label": "arrival"})
        assert response.status_code == 200
        body = response.json()
        cid = body["capture_id"]
        assert body["camera_id"] == CAM
        assert body["meta"]["camera_id"] == CAM
        assert body["urls"]["color"] == f"/realsense/{CAM}/captures/{cid}/color.jpg"
        assert body["urls"]["depth"] == f"/realsense/{CAM}/captures/{cid}/depth.png"
        assert body["meta"]["label"] == "arrival"

    def test_capture_uses_one_frameset_for_both_images(self, client, fake_cam):
        """Colour and depth must come from the same bundle, or the record
        pairs two different moments and stops being a measurement."""
        client.post(f"/control/realsense/{CAM}/capture", json={})
        encodes = [c for c in fake_cam.calls if isinstance(c, tuple)]
        frame_numbers = {c[1] for c in encodes}
        assert len(frame_numbers) == 1
        assert fake_cam.calls.count("latest") == 1

    def test_connected_is_read_from_the_states_map(self, client, monkeypatch):
        """Regression: the first version read a non-existent is_connected
        attribute, so every capture claimed the arm was disconnected even
        while recording its live joints and TCP pose."""
        meta = client.post(f"/control/realsense/{CAM}/capture", json={}).json()["meta"]
        assert meta["arm"]["connected"] is True

        import src.core.xarm_api_server as srv
        srv.controller.states = {"connection": SimpleNamespace(value="disabled")}
        meta = client.post(f"/control/realsense/{CAM}/capture", json={}).json()["meta"]
        assert meta["arm"]["connected"] is False

    def test_missing_states_map_does_not_break_a_capture(self, client):
        """Metadata is never worth failing a frame over."""
        import src.core.xarm_api_server as srv
        srv.controller.states = {}
        response = client.post(f"/control/realsense/{CAM}/capture", json={})
        assert response.status_code == 200
        assert response.json()["meta"]["arm"]["connected"] is False

    def test_capture_records_arm_state(self, client):
        meta = client.post(f"/control/realsense/{CAM}/capture", json={}).json()["meta"]
        assert meta["arm"]["node_id"] == "deck_1"
        assert meta["arm"]["joints"] == [0.0, 1.0, 2.0, 3.0, 4.0]
        assert meta["arm"]["track_position"] == 42.0
        assert meta["arm"]["gripper_state"] == "open"
        assert meta["arm"]["connected"] is True

    def test_explicit_node_id_overrides_the_live_one(self, client):
        meta = client.post(f"/control/realsense/{CAM}/capture",
                           json={"node_id": "hood_2"}).json()["meta"]
        assert meta["arm"]["node_id"] == "hood_2"

    def test_capture_records_camera_and_frame_metadata(self, client):
        meta = client.post(f"/control/realsense/{CAM}/capture", json={}).json()["meta"]
        assert meta["camera"]["device"]["serial"] == "S1"
        assert meta["camera"]["library_version"] == "2.58.4"
        assert meta["frame"]["depth_scale_m"] == 0.001
        assert meta["frame"]["aligned_depth_to_color"] is True
        assert meta["intrinsics"] == {"color": {"fx": 600.0}}

    def test_capture_starts_the_pipeline_on_demand(self, client, fake_cam):
        assert fake_cam.streaming is False
        assert client.post(f"/control/realsense/{CAM}/capture", json={}).status_code == 200
        assert "ensure_started" in fake_cam.calls

    def test_capture_409_when_start_on_demand_is_off(self, client, fake_cam):
        fake_cam.start_on_demand = False
        response = client.post(f"/control/realsense/{CAM}/capture", json={})
        assert response.status_code == 409
        assert response.json()["detail"]["error"] == "realsense_not_streaming"

    def test_capture_404_when_store_disabled(self, client, monkeypatch):
        rcap.set_shared(rcap.CaptureStore({"enabled": False}))
        response = client.post(f"/control/realsense/{CAM}/capture", json={})
        assert response.status_code == 404
        assert response.json()["detail"]["error"] == "captures_not_configured"

    def test_capture_accepts_an_empty_body(self, client):
        assert client.post(f"/control/realsense/{CAM}/capture").status_code == 200

    def test_protected_flag_is_recorded(self, client):
        meta = client.post(f"/control/realsense/{CAM}/capture",
                           json={"protected": True}).json()["meta"]
        assert meta["protected"] is True


# ---------------------------------------------------------------------------
# The fixed-path alias
#
# POST /control/realsense/capture exists because a SkillDef carries one fixed
# endpoint string that the skill executor and the dashboard passthrough send
# verbatim -- an agent plan cannot template a camera id into a path. These
# pin down how it picks the camera, because getting that wrong files evidence
# under the wrong lens.
# ---------------------------------------------------------------------------

class TestCaptureAlias:
    def test_alias_resolves_the_sole_camera(self, client, fake_cam):
        body = client.post("/control/realsense/capture", json={"label": "one"}).json()
        assert body["camera_id"] == CAM
        assert body["urls"]["meta"].startswith(f"/realsense/{CAM}/captures/")

    def test_alias_accepts_an_empty_body(self, client):
        assert client.post("/control/realsense/capture").status_code == 200

    def test_alias_with_two_cameras_and_no_camera_field_is_400(self, client, registry):
        registry(FakeCamera(camera_id="rs435i"), FakeCamera(camera_id="overhead"))
        response = client.post("/control/realsense/capture", json={})
        assert response.status_code == 400
        detail = response.json()["detail"]
        assert detail["error"] == "camera_required"
        assert detail["cameras"] == ["overhead", "rs435i"]

    def test_alias_with_an_explicit_camera(self, client, registry):
        first, second = FakeCamera(camera_id="rs435i"), FakeCamera(camera_id="overhead")
        registry(first, second)
        body = client.post("/control/realsense/capture",
                           json={"camera": "overhead", "label": "two"}).json()
        assert body["camera_id"] == "overhead"
        assert body["meta"]["camera_id"] == "overhead"
        assert second.calls and not first.calls

    def test_alias_with_an_unknown_camera_is_404(self, client, registry):
        registry(FakeCamera(camera_id="rs435i"), FakeCamera(camera_id="overhead"))
        response = client.post("/control/realsense/capture", json={"camera": "nope"})
        assert response.status_code == 404
        detail = response.json()["detail"]
        assert detail["error"] == "camera_not_found"
        assert detail["cameras"] == ["overhead", "rs435i"]

    def test_camera_field_is_ignored_on_the_nested_route(self, client, registry):
        """The path is the authority there; a stray body field cannot redirect
        a capture to another camera."""
        first, second = FakeCamera(camera_id="rs435i"), FakeCamera(camera_id="overhead")
        registry(first, second)
        body = client.post(f"/control/realsense/{CAM}/capture",
                           json={"camera": "overhead"}).json()
        assert body["camera_id"] == CAM


# ---------------------------------------------------------------------------
# Reading captures
# ---------------------------------------------------------------------------

class TestCaptureReads:
    def test_list_is_open_and_reports_retention(self, client):
        client.post(f"/control/realsense/{CAM}/capture", json={"label": "one"})
        body = client.get(f"/realsense/{CAM}/captures").json()
        assert body["count"] == 1
        assert body["retention"]["keep_days"] == 30
        assert body["retention"]["keep_max_gb"] == 20

    def test_unscoped_list_spans_every_camera(self, client, registry):
        first, second = FakeCamera(camera_id="rs435i"), FakeCamera(camera_id="overhead")
        registry(first, second)
        client.post("/control/realsense/rs435i/capture", json={"label": "a"})
        client.post("/control/realsense/overhead/capture", json={"label": "b"})
        everything = client.get("/realsense/captures").json()
        assert everything["count"] == 2
        assert {c["camera_id"] for c in everything["captures"]} == {"rs435i", "overhead"}
        scoped = client.get("/realsense/overhead/captures").json()
        assert scoped["camera_id"] == "overhead"
        assert [c["label"] for c in scoped["captures"]] == ["b"]

    def test_list_filters_by_node(self, client):
        client.post(f"/control/realsense/{CAM}/capture", json={"node_id": "a"})
        client.post(f"/control/realsense/{CAM}/capture", json={"node_id": "b"})
        body = client.get(f"/realsense/{CAM}/captures", params={"node_id": "a"}).json()
        assert body["count"] == 1

    def test_meta_endpoint_returns_the_record(self, client):
        cid = client.post(f"/control/realsense/{CAM}/capture", json={}).json()["capture_id"]
        body = client.get(f"/realsense/{CAM}/captures/{cid}").json()
        assert body["capture_id"] == cid
        assert body["meta"]["capture_id"] == cid

    def test_meta_404_for_unknown_id(self, client):
        cid = rcap.new_capture_id()
        assert client.get(f"/realsense/{CAM}/captures/{cid}").status_code == 404

    def test_files_are_served_with_the_right_media_types(self, client):
        cid = client.post(f"/control/realsense/{CAM}/capture", json={}).json()["capture_id"]
        color = client.get(f"/realsense/{CAM}/captures/{cid}/color.jpg")
        depth = client.get(f"/realsense/{CAM}/captures/{cid}/depth.png")
        assert color.status_code == 200
        assert color.headers["content-type"] == "image/jpeg"
        assert color.content == JPEG
        assert depth.headers["content-type"] == "image/png"
        assert depth.content == PNG

    def test_unknown_filename_is_404_not_a_traversal(self, client):
        cid = client.post(f"/control/realsense/{CAM}/capture", json={}).json()["capture_id"]
        assert client.get(f"/realsense/{CAM}/captures/{cid}/meta.json.bak").status_code == 404

    def test_delete_removes_the_capture(self, client):
        cid = client.post(f"/control/realsense/{CAM}/capture", json={}).json()["capture_id"]
        assert client.delete(f"/control/realsense/{CAM}/captures/{cid}").status_code == 200
        assert client.get(f"/realsense/{CAM}/captures/{cid}").status_code == 404

    def test_delete_404_for_unknown_id(self, client):
        assert client.delete(
            f"/control/realsense/{CAM}/captures/{rcap.new_capture_id()}"
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
                     "/realsense/cameras", "/realsense/captures",
                     "/control/realsense/capture",
                     "/control/realsense/{camera_id}/capture",
                     "/realsense/{camera_id}/status",
                     "/realsense/{camera_id}/captures/{capture_id}"):
            assert path in paths


def test_diagnostic_returns_zip_without_starting_camera(client, fake_cam):
    fake_cam.diagnostic_export = lambda: ('diagnostic-test', b'PK-test')
    response = client.post(f'/control/realsense/{CAM}/diagnostic')
    assert response.status_code == 200
    assert response.content == b'PK-test'
    assert response.headers['content-type'] == 'application/zip'
    assert response.headers['x-capture-id'] == 'diagnostic-test'
    assert fake_cam.calls == []


def test_diagnostic_stopped_camera_is_conflict(client, fake_cam):
    def stopped():
        raise RealSenseNotStreaming('already streaming required')
    fake_cam.diagnostic_export = stopped
    response = client.post(f'/control/realsense/{CAM}/diagnostic')
    assert response.status_code == 409
    assert fake_cam.calls == []


def test_diagnostic_optional_start_preserves_sensor_settings(client, fake_cam):
    calls = []
    fake_cam.start = lambda **kwargs: calls.append(kwargs)
    fake_cam.diagnostic_export = lambda: ('diagnostic-test', b'PK-test')
    response = client.post(f'/control/realsense/{CAM}/diagnostic?start_if_idle=true')
    assert response.status_code == 200
    assert calls == [{'preserve_sensor_settings': True}]
