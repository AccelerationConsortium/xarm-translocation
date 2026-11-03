"""Offline tests for the lab PTZ camera and RealSense routes.

The dashboard and the SDL camera service are httpx MockTransports; nothing
here opens a socket or loads a camera SDK.
"""

import json
import time

import httpx
import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from core.remote_realsense import RemoteCamera, RemoteService
from robot_motion.app import create_app
from robot_motion.config import Settings, load_settings

SECRET = "edge-fixture"
OPERATOR = {"X-Auth-User": "op@example.invalid", "X-Edge-Auth": SECRET}
LAB = {"dashboard_base_url": "http://dash.invalid", "camera_id": "cam_ligand_tapo_d246", "lens": "tele"}
CAMERA_SNAPSHOT = {
    "id": "cam_ligand_tapo_d246",
    "status": {
        "equipment_status": "ready",
        "details": {
            "privacy_mode": False,
            "streaming_enabled": True,
            "go2rtc_reachable": True,
            "presets": [{"id": "1", "name": "handover"}, {"id": "2", "name": "CNC"}],
            "lenses": [
                {"id": "wide", "label": "Wide", "mse_url": "/streams/api/ws?src=cam_ligand_tapo_d246_wide"},
                {"id": "tele", "label": "Tele", "mse_url": "/streams/api/ws?src=cam_ligand_tapo_d246_tele"},
            ],
        },
    },
    "camera": {"lenses": [{"id": "wide", "ptz_capable": False}, {"id": "tele", "ptz_capable": True}]},
}


class Dashboard:
    """Records control POSTs; answers /api/equipment with one camera."""

    def __init__(self, control_status=200):
        self.control_status = control_status
        self.posts = []

    def __call__(self, request):
        if request.method == "GET" and request.url.path == "/api/equipment":
            return httpx.Response(200, json={"equipment": [CAMERA_SNAPSHOT]})
        if request.method == "POST":
            self.posts.append(request)
            if self.control_status == 403:
                return httpx.Response(403, json={"detail": "op@example.invalid is not authorized to control cam"})
            if self.control_status >= 500:
                return httpx.Response(self.control_status, text="gateway down")
            return httpx.Response(self.control_status, json={"ok": True})
        return httpx.Response(404)


def lab_client(dashboard, edge_secret=SECRET):
    settings = Settings(lab_camera=LAB)
    return TestClient(
        create_app(settings, edge_secret=edge_secret, camera_transport=httpx.MockTransport(dashboard))
    )


def test_camera_settings_are_validated_and_service_file_resolves_beside_config(tmp_path):
    with pytest.raises(ValidationError):
        Settings(lab_camera={**LAB, "dashboard_base_url": "http://dash.invalid/api"})
    with pytest.raises(ValidationError):
        Settings(lab_camera={**LAB, "camera_id": "Bad Id"})
    duplicate = {"id": "d435i", "label": "x"}
    with pytest.raises(ValidationError):
        Settings(camera_service={"service_file": "s.json", "cameras": [duplicate, duplicate]})
    path = tmp_path / "robot-motion.local.json"
    path.write_text(json.dumps({
        "camera_service": {"service_file": "camera-service.local.json", "cameras": [duplicate]},
    }))
    settings = load_settings(path)
    assert settings.camera_service.service_file == str((tmp_path / "camera-service.local.json").resolve())


def test_unconfigured_cameras_answer_absent_without_errors():
    with TestClient(create_app(edge_secret=SECRET)) as client:
        config = client.get("/camera/config").json()
        assert config["configured"] is False and config["available"] is False
        listing = client.get("/realsense/cameras").json()
        assert listing["cameras"] == [] and listing["default"] is None
        assert "camera_service" in listing["reason"]
        assert client.get("/realsense/d435i/status").json()["detail"]["error"] == "realsense_not_configured"
        assert client.get("/cameras").json()["sources"]["lab"] == {"configured": False}
        assert client.post("/camera/ptz", json={"pan": 0}, headers=OPERATOR).status_code == 404
        status = client.get("/status").json()
        assert "realsense" not in status["details"]
        assert not any(name.startswith("realsense_") for name in status["components"])


def test_lab_camera_config_reports_lenses_presets_and_no_follow():
    with lab_client(Dashboard()) as client:
        config = client.get("/camera/config").json()
    assert config["configured"] is True and config["available"] is True
    assert config["camera_id"] == "cam_ligand_tapo_d246"
    # The configured lens is the default stream; the panel keys on src=.
    assert config["stream_url"].endswith("src=cam_ligand_tapo_d246_tele")
    assert [(lens["id"], lens["ptz_capable"]) for lens in config["lenses"]] == [("wide", False), ("tele", True)]
    assert config["presets"] == [{"id": "1", "name": "handover"}, {"id": "2", "name": "CNC"}]
    assert config["follow_supported"] is False
    assert config["following"] is False and config["connected"] is False


def test_lab_camera_unreachable_dashboard_is_unavailable_not_an_error():
    def down(request):
        raise httpx.ConnectError("refused", request=request)

    with lab_client(down) as client:
        response = client.get("/camera/config")
    assert response.status_code == 200
    assert response.json()["available"] is False
    assert "dashboard unreachable" in response.json()["reason"]


def test_follow_is_refused():
    with lab_client(Dashboard()) as client:
        response = client.post("/camera/follow", json={"enabled": True}, headers=OPERATOR)
    assert response.status_code == 409
    assert response.json()["detail"]["error"] == "follow_not_supported"


def test_ptz_requires_the_edge_identity_and_the_operators_own_credential():
    dashboard = Dashboard()
    with lab_client(dashboard, edge_secret="") as client:
        assert client.post("/camera/ptz", json={"pan": 0}).status_code == 503
    with lab_client(dashboard) as client:
        assert client.post("/camera/ptz", json={"pan": 0}).status_code == 401
        forged = {**OPERATOR, "X-Edge-Auth": "wrong", "Cookie": "ac_auth_session=s"}
        assert client.post("/camera/ptz", json={"pan": 0}, headers=forged).status_code == 401
        # Vouched for by the edge but carrying no dashboard credential.
        refused = client.post("/camera/ptz", json={"pan": 0}, headers=OPERATOR)
        assert refused.status_code == 401
    assert dashboard.posts == []


def test_ptz_forwards_only_the_session_cookie_and_the_verbatim_body():
    dashboard = Dashboard()
    body = {"direction": "up", "speed": 0.5, "duration_ms": 1500}
    headers = {**OPERATOR, "X-Claim-Token": "claim", "Cookie": "ac_auth_session=sess-1; other=zzz"}
    with lab_client(dashboard) as client:
        response = client.post("/camera/ptz", json=body, headers=headers)
        assert response.status_code == 200 and response.json() == {"ok": True}
        assert client.post("/camera/preset", json={"preset_id": "2"}, headers=headers).status_code == 200
        assert client.post("/camera/preset", json={}, headers=headers).status_code == 422
    ptz, preset = dashboard.posts
    assert str(ptz.url) == "http://dash.invalid/api/equipment/cam_ligand_tapo_d246/control/ptz"
    assert json.loads(ptz.content) == body
    assert ptz.headers["cookie"] == "ac_auth_session=sess-1"
    for leaked in ("x-claim-token", "x-edge-auth", "x-auth-user", "x-api-key"):
        assert leaked not in ptz.headers
    assert str(preset.url).endswith("/control/preset/goto")
    assert json.loads(preset.content) == {"preset_id": "2"}


def test_ptz_prefers_a_machine_principals_api_key():
    dashboard = Dashboard()
    with lab_client(dashboard) as client:
        headers = {**OPERATOR, "X-Api-Key": "agent-key", "Cookie": "ac_auth_session=sess-1"}
        assert client.post("/camera/ptz", json={"pan": 0}, headers=headers).status_code == 200
    (post,) = dashboard.posts
    assert post.headers["x-api-key"] == "agent-key"
    assert "cookie" not in post.headers


@pytest.mark.parametrize("upstream,expected", [(403, 403), (401, 401), (500, 502)])
def test_ptz_surfaces_dashboard_refusals(upstream, expected):
    with lab_client(Dashboard(control_status=upstream)) as client:
        response = client.post(
            "/camera/ptz", json={"pan": 0}, headers={**OPERATOR, "Cookie": "ac_auth_session=s"}
        )
    assert response.status_code == expected
    assert response.json()["detail"]["reason"]


class CameraService:
    """The SDL camera service's /v1 surface for one RealSense alias."""

    def __init__(self):
        self.requests = []
        self.stream_status = 200

    def __call__(self, request):
        self.requests.append(request)
        path = request.url.path
        if path == "/v1/cameras":
            return httpx.Response(200, json={"cameras": [{
                "id": "d435i", "state": "streaming", "streaming": True, "present": True,
                "installed": True, "start_on_demand": True,
                "device": {"name": "Intel RealSense D435I", "usb_type": "3.2"},
            }]})
        if path == "/v1/store":
            return httpx.Response(200, json={"enabled": True, "count": 0, "bytes": 0})
        if path == "/v1/cameras/d435i/snapshot.jpg":
            return httpx.Response(200, content=b"\xff\xd8jpeg", headers={"X-Frame-Number": "7", "X-Frame-Timestamp-Ms": "12.5"})
        if path == "/v1/cameras/d435i/stream.mjpg":
            if self.stream_status != 200:
                return httpx.Response(self.stream_status, text="not streaming")
            return httpx.Response(200, content=b"--camera-frame\r\npart\r\n")
        if path == "/v1/cameras/d435i/depth":
            return httpx.Response(200, json={"distance_m": 0.5, "x": int(request.url.params["x"])})
        if path == "/v1/cameras/d435i/start":
            return httpx.Response(200, json={"streaming": True})
        if path == "/v1/cameras/d435i/calibration":
            return httpx.Response(200, json={"stereo_baseline": {"baseline_m": 0.05},
                                             "refresh": request.url.params.get("refresh")})
        return httpx.Response(404)


@pytest.fixture
def stereo():
    upstream = CameraService()
    # The transport reaches the per-stream clients as well as the shared one.
    service = RemoteService({"url": "http://camera.invalid", "token": "service-token", "cameras": ["d435i"]},
                            transport=httpx.MockTransport(upstream))
    camera = RemoteCamera(
        "d435i", {"label": "UR5e depth camera", "short_label": "D435i", "mount": {"location": "bench"}}, service
    )
    return upstream, (service, {"d435i": camera})


def sampled(client, service):
    deadline = time.monotonic() + 5
    while not service.sampled and time.monotonic() < deadline:
        time.sleep(0.02)
    assert service.sampled, "camera service poll never ran"


def test_realsense_routes_follow_the_xarm_contract(stereo):
    upstream, pair = stereo
    app = create_app(Settings(), edge_secret=SECRET, stereo_cameras=pair)
    with TestClient(app) as client:
        sampled(client, pair[0])
        listing = client.get("/realsense/cameras").json()
        (entry,) = listing["cameras"]
        assert listing["default"] == "d435i" and listing["reason"] is None
        assert entry["short_label"] == "D435i" and entry["streaming"] is True
        assert entry["urls"]["stream"] == "/realsense/d435i/stream.mjpg"
        assert entry["urls"]["start"] == "/realsense/d435i/start"
        assert client.get("/cameras").json()["cameras"][0]["kind"] == "realsense"
        assert client.get("/realsense/d435i/status").json()["state"] == "streaming"
        assert client.get("/realsense/nope/status").json()["detail"]["cameras"] == ["d435i"]

        # Pixels and lifecycle need the edge identity; depth readings do not.
        for method, path in [
            ("GET", "/realsense/d435i/snapshot.jpg"),
            ("GET", "/realsense/d435i/stream.mjpg"),
            ("GET", "/realsense/d435i/depth.png"),
            ("POST", "/realsense/d435i/start"),
            ("POST", "/realsense/d435i/stop"),
        ]:
            assert client.request(method, path).status_code == 401, path
        assert client.get("/realsense/d435i/depth?x=3&y=4").json()["distance_m"] == 0.5
        # Stored calibration is an open read relayed with its refresh flag.
        assert entry["urls"]["calibration"] == "/realsense/d435i/calibration"
        assert "calibration" in client.get("/cameras").json()["cameras"][0]["capabilities"]
        assert client.get("/realsense/d435i/calibration").json() == {
            "stereo_baseline": {"baseline_m": 0.05}, "refresh": "false"}
        assert client.get("/realsense/d435i/calibration?refresh=true").json()["refresh"] == "true"

        snapshot = client.get("/realsense/d435i/snapshot.jpg", headers=OPERATOR)
        assert snapshot.content == b"\xff\xd8jpeg"
        assert snapshot.headers["x-frame-number"] == "7"
        assert client.get("/realsense/d435i/snapshot.jpg?stream=ir", headers=OPERATOR).status_code == 400
        stream = client.get("/realsense/d435i/stream.mjpg?fps=99", headers=OPERATOR)
        assert stream.headers["content-type"].startswith("multipart/x-mixed-replace")
        assert stream.content == b"--camera-frame\r\npart\r\n"
        assert client.post("/realsense/d435i/start", headers=OPERATOR).json() == {"streaming": True}

        # An upstream refusal is a status code, never an empty 200 stream.
        upstream.stream_status = 409
        refused = client.get("/realsense/d435i/stream.mjpg", headers=OPERATOR)
        assert refused.status_code == 503
        assert refused.json()["detail"]["error"] == "realsense_unavailable"

        status = client.get("/status").json()
        assert status["components"]["realsense_d435i"]["connected"] is True
        assert status["details"]["realsense"]["cameras"]["d435i"]["state"] == "streaming"
        assert status["equipment_status"] == "unknown"  # cameras never set it
    sent = [r for r in upstream.requests if r.url.path == "/v1/cameras/d435i/stream.mjpg"]
    assert sent[0].url.params["fps"] == "30.0"
    assert all(r.headers["authorization"] == "Bearer service-token" for r in upstream.requests)


def test_camera_service_misconfiguration_keeps_the_robot_service_up(tmp_path):
    settings = Settings(camera_service={
        "service_file": str(tmp_path / "missing.json"),
        "cameras": [{"id": "d435i", "label": "UR5e depth camera"}],
    })
    with TestClient(create_app(settings)) as client:
        assert client.get("/health").status_code == 200
        listing = client.get("/realsense/cameras").json()
        assert listing["cameras"] == []
        assert "rejected" in listing["reason"]


# ── capture records (the xArm's /control/realsense/capture) ─────────
class CaptureService(CameraService):
    """The camera service's store: POST writes a record, the reads return it."""

    def __init__(self):
        super().__init__()
        self.records = {}

    def __call__(self, request):
        path = request.url.path
        base = "/v1/cameras/d435i/captures"
        if path == base and request.method == "POST":
            self.requests.append(request)
            body = json.loads(request.content)
            capture_id = f"cap{len(self.records) + 1}"
            meta = {**body["context"], "label": body["label"], "tags": body["tags"], "camera_id": "d435i",
                    "captured_at": f"2026-11-03T00:00:0{len(self.records)}Z",
                    "files": {"color.jpg": {}, "depth.png": {}}}
            self.records[capture_id] = meta
            return httpx.Response(200, json={"capture_id": capture_id, "camera_id": "d435i", "meta": meta,
                                             "urls": {"meta": f"{base}/{capture_id}"}})
        if path == base:
            self.requests.append(request)
            items = [dict(m, capture_id=k) for k, m in self.records.items()]
            return httpx.Response(200, json={"camera_id": "d435i", "captures": items, "count": len(items)})
        if path.startswith(base + "/"):
            self.requests.append(request)
            capture_id, *rest = path[len(base) + 1:].split("/")
            if capture_id not in self.records:
                return httpx.Response(404, json={"detail": "capture_not_found"})
            if rest:
                return httpx.Response(200, content=b"\xff\xd8capture")
            return httpx.Response(200, json={"capture_id": capture_id, "meta": self.records[capture_id]})
        return super().__call__(request)


@pytest.fixture
def stored():
    from core.remote_realsense import RemoteStore

    upstream = CaptureService()
    service = RemoteService({"url": "http://camera.invalid", "token": "service-token", "cameras": ["d435i"]},
                            transport=httpx.MockTransport(upstream))
    camera = RemoteCamera("d435i", {"label": "UR3e depth camera"}, service)
    return upstream, (service, {"d435i": camera}, RemoteStore(service))


def posted(upstream):
    return [json.loads(r.content) for r in upstream.requests
            if r.method == "POST" and r.url.path == "/v1/cameras/d435i/captures"]


def test_captures_without_control_need_the_edge_identity_and_relay_to_the_store(stored):
    upstream, triple = stored
    with TestClient(create_app(Settings(), edge_secret=SECRET, stereo_cameras=triple)) as client:
        sampled(client, triple[0])
        assert "captures" in client.get("/cameras").json()["cameras"][0]["capabilities"]
        assert client.post("/control/realsense/capture", json={}).status_code == 401
        taken = client.post("/control/realsense/capture", json={"label": "pan-check", "tags": ["vial"]},
                            headers=OPERATOR)
        assert taken.status_code == 200, taken.text
        body = taken.json()
        assert body["capture_id"] == "cap1"
        assert body["urls"] == {"meta": "/realsense/d435i/captures/cap1",
                                "color": "/realsense/d435i/captures/cap1/color.jpg",
                                "depth": "/realsense/d435i/captures/cap1/depth.png"}
        (sent,) = posted(upstream)
        assert sent["label"] == "pan-check" and sent["tags"] == ["vial"]
        assert sent["context"]["requested_by"] == "op@example.invalid"
        assert sent["context"]["arm"]["connected"] is False

        # Metadata is an open read; the image bytes are a frame like any other.
        assert client.get("/realsense/captures").json()["count"] == 1
        assert client.get("/realsense/d435i/captures").json()["captures"][0]["capture_id"] == "cap1"
        assert client.get("/realsense/d435i/captures/cap1").json()["meta"]["label"] == "pan-check"
        assert client.get("/realsense/d435i/captures/nope").json()["detail"]["error"] == "capture_not_found"
        assert client.get("/realsense/d435i/captures/cap1/color.jpg").status_code == 401
        image = client.get("/realsense/d435i/captures/cap1/color.jpg", headers=OPERATOR)
        assert image.content == b"\xff\xd8capture" and image.headers["content-type"] == "image/jpeg"
        assert client.get("/realsense/d435i/captures/cap1/meta.json", headers=OPERATOR).status_code == 404
        assert client.post("/control/realsense/nope/capture", headers=OPERATOR).status_code == 404
        # No control, nothing to advertise: the catalog skill claims first.
        assert "realsense.capture" not in client.get("/status").json()["allowed_actions"]


def test_no_camera_service_means_no_capture_routes():
    with TestClient(create_app(Settings(), edge_secret=SECRET)) as client:
        paths = client.get("/openapi.json").json()["paths"]
        assert not any("captures" in p or p.startswith("/control") for p in paths)


def test_with_control_a_capture_is_claim_gated_stamped_with_the_pose_and_withheld_while_moving(stored):
    from tests_robot_motion.test_motion import HEADERS
    from tests_robot_motion.test_motion import SECRET as ARM_SECRET
    from tests_robot_motion.test_motion import Rig, connected, dispatched, in_background, wait_for

    upstream, triple = stored
    rig = Rig()
    rig.world.ticks_per_move = 5000  # about 20 s at the fake's packet rate
    app = create_app(rig.config, observer=rig.observer, control_session_factory=rig.session,
                     edge_secret=ARM_SECRET, stereo_cameras=triple)
    with TestClient(app) as client:
        held = connected(client)
        assert "realsense.capture" in client.get("/status").json()["allowed_actions"]
        refused = client.post("/control/realsense/capture", json={}, headers=HEADERS)
        assert refused.status_code == 423 and refused.json()["detail"]["error"] == "claim_required"
        taken = client.post("/control/realsense/capture", json={"node_id": "safe_bal"}, headers=held)
        assert taken.status_code == 200, taken.text
        (sent,) = posted(upstream)
        arm = sent["context"]["arm"]
        assert arm["connected"] is True and arm["model"] == rig.config.model
        assert arm["node_id"] == "safe_bal" and len(arm["joints"]) == 6 and len(arm["position"]) == 6
        assert sent["context"]["requested_by"] == HEADERS["X-Auth-User"]

        # A slow joint move is in flight: no capture, and none advertised.
        thread, _result = in_background(client, "/control/freehand/joints", {"angles": [20, 0, 0, 0, 0, 0]}, held)
        wait_for(lambda: dispatched(client), what="move dispatch")
        assert "realsense.capture" not in client.get("/status").json()["allowed_actions"]
        moving = client.post("/control/realsense/capture", json={}, headers=held)
        assert moving.status_code == 409 and moving.json()["detail"]["error"] == "arm_moving"
        assert len(posted(upstream)) == 1
        assert client.post("/move/stop", headers=HEADERS).status_code == 200
        thread.join(5)
