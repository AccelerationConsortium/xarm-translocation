"""Offline tests: no real sockets or robot-control constructors."""

import io
import math
import sys
import time
from contextlib import contextmanager
from importlib.resources import files

import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from robot_motion.app import create_app
from robot_motion.config import Settings
from robot_motion.drivers.ur import URObserver, interpret
from robot_motion.graph import Graph


def graph(model="ur5e", count=6):
    return {
        "robot_model": model,
        "nodes": [
            {"id": "a", "joints_deg": [0] * count},
            {"id": "b", "joints_deg": [1] * count},
            {"id": "c", "joints_deg": [2] * count},
        ],
        "edges": [{"source": "a", "target": "b"}, {"source": "b", "target": "c"}],
    }


def test_import_does_not_load_vendor_modules():
    assert "rtde_control" not in sys.modules
    assert "rtde_receive" not in sys.modules
    assert "xarm" not in sys.modules


@pytest.mark.parametrize(
    "config",
    [
        {"control_enabled": True},
        {"driver": "ur", "model": "mg400"},
        {"driver": "ur", "model": "ur5e", "observe": True},
        {"driver": "none", "observe": True},
        {
            "driver": "mg400",
            "model": "mg400",
            "observe": True,
            "robot_host": "example.invalid",
        },
        {"timeout_s": float("inf")},
    ],
)
def test_invalid_or_control_configuration_is_rejected(config):
    with pytest.raises(ValidationError):
        Settings(**config)


@pytest.mark.parametrize(
    "model,count",
    [("xarm5", 5), ("ur3e", 6), ("ur5e", 6), ("ur5_cb3", 6), ("mg400", 4)],
)
def test_model_specific_graph_and_existing_path_planner(model, count):
    g = Graph.model_validate(graph(model, count))
    assert g.path("a", "c") == ["b", "c"]
    assert g.path("a", "a") == []
    bad = graph(model, count + 1)
    with pytest.raises(ValidationError):
        Graph.model_validate(bad)


def test_graph_rejects_invalid_structure_and_nonfinite_coordinates():
    cases = []
    bad = graph()
    bad["nodes"][0]["joints_deg"][0] = math.nan
    cases.append(bad)
    bad = graph()
    bad["edges"][0]["target"] = "missing"
    cases.append(bad)
    bad = graph()
    bad["nodes"][1]["id"] = "a"
    cases.append(bad)
    bad = graph()
    bad["edges"].append(bad["edges"][0])
    cases.append(bad)
    bad = graph()
    bad["edges"][0]["mode"] = "linear"
    cases.append(bad)
    bad = graph()
    bad["nodes"][0]["id"] = "<script>"
    cases.append(bad)
    for bad in cases:
        with pytest.raises(ValidationError):
            Graph.model_validate(bad)


def test_all_default_ui_and_documentation_assets_are_packaged():
    with TestClient(create_app()) as client:
        for path in [
            "/",
            "/health",
            "/status",
            "/drivers",
            "/graph",
            "/docs",
            "/openapi.json",
            "/agent-docs",
            "/agent-docs/api-reference",
            "/llms.txt",
            "/web",
            "/web/",
            "/web/index.html",
            "/web/graph.html",
            "/web/main.js?v=20260930-clean-labels",
            "/web/graph.js",
            "/web/workspace.js",
            "/web/camera-player.js",
            "/web/realsense-card.js",
            "/web/cytoscape.min.js",
            "/web/style.css",
            "/web/graph.css",
            "/web/workspace.css",
        ]:
            response = client.get(path)
            assert response.status_code == 200, path
        status = client.get("/status").json()
        assert status["equipment_status"] == "unknown"
        assert status["allowed_actions"] == []
        assert status["details"]["control_enabled"] is False
        for path in [
            "/control/claim",
            "/control/graph/move_to",
            "/control/startup",
            "/connect",
        ]:
            assert client.post(path, json={}).status_code == 404


def test_shared_ui_is_exact_and_never_exposes_python_or_other_files():
    with TestClient(create_app()) as client:
        for asset in ["index.html", "main.js", "workspace.js", "style.css", "cytoscape.min.js"]:
            response = client.get("/web/" + asset)
            assert response.content == files("web").joinpath(asset).read_bytes()
        assert client.get("/web/").content == files("web").joinpath("index.html").read_bytes()
        for path in [
            "/web/server.py",
            "/web/__init__.py",
            "/web/pyxarm/style.css",
            "/web/%2e%2e/server.py",
            "/web/%2e%2e%2fserver.py",
            "/web/%2e%2e%2frobot_motion%2fapp.py",
        ]:
            assert client.get(path).status_code == 404, path


def test_shared_ui_read_polls_answer_absent_features_and_status_maps_telemetry():
    class Observer:
        def read(self):
            return {
                "equipment_status": "ready",
                "activity": "idle",
                "message": "fixture",
                "components": {},
                "details": {
                    "telemetry": {
                        "valid": True,
                        "source": "rtde_receive",
                        "joints_deg": [0, 1, 2, 3, 4, 5],
                        "tcp_mm_rpy_deg": [100, 200, 300, 10, 20, 30],
                    }
                },
            }

    settings = Settings(
        driver="ur", model="ur5e", observe=True, robot_host="example.invalid",
        ur_transport="rtde",
    )
    with TestClient(create_app(settings, observer=Observer())) as client:
        deadline = time.monotonic() + 5
        while True:
            status = client.get("/status").json()
            if status["details"].get("current_joints") or time.monotonic() > deadline:
                break
            time.sleep(0.02)
        details = status["details"]
        assert details["current_joints"] == [0, 1, 2, 3, 4, 5]
        assert details["current_position"] == [100, 200, 300, 10, 20, 30]
        assert details["num_joints"] == 6
        assert details["connection_details"] == {
            "host": "example.invalid", "port": 30004, "profile_name": "ur5e",
        }
        assert status["allowed_actions"] == []
        assert details["control_enabled"] is False
        assert client.get("/graph/layout").json() == {
            "positions": {}, "expanded": {}, "pan": None, "zoom": None,
        }
        for path in ["/locations", "/track/locations"]:
            assert client.get(path).json() == {"locations": [], "positions": {}}
        assert client.get("/interlocks/sash").json()["configured"] is False
        assert client.get("/auth/config").json() == {"enabled": False}
        assert client.get("/auth/me").json() == {"authenticated": False, "identity": None}
        assert client.get("/camera/config").json()["configured"] is False
        assert client.get("/assistant/status").json()["enabled"] is False
        assert client.get("/api/configurations").json() == []
        assert client.post("/graph/layout", json={"positions": {}}).status_code == 405
        with client.websocket_connect("/ws") as websocket:
            message = websocket.receive_json()
        assert message["type"] == "status_update"
        assert message["data"]["details"]["current_joints"] == [0, 1, 2, 3, 4, 5]
        assert message["data"]["allowed_actions"] == []
    # Without a valid sample the xArm-named keys are null, never zero.
    with TestClient(create_app()) as client:
        details = client.get("/status").json()["details"]
        assert details["current_joints"] is None
        assert details["current_position"] is None
        assert details["num_joints"] is None
        assert details["connection_details"] is None


def test_offline_edits_never_replace_configured_graph_or_status(tmp_path):
    import json

    path = tmp_path / "graph.local.json"
    original = graph()
    path.write_text(json.dumps(original))
    app = create_app(Settings(driver="ur", model="ur5e", graph_file=str(path)))
    with TestClient(app) as client:
        edited = graph()
        edited["nodes"][0]["joints_deg"] = [25] * 6
        assert client.post("/graph/validate", json=edited).status_code == 200
        response = client.post("/graph/preview", json={"graph": edited, "source": "a", "target": "c"})
        assert response.json()["executed"] is False
        assert client.get("/graph").json()["graph"]["nodes"][0]["joints_deg"] == [0] * 6
        status = client.get("/status").json()
        assert status["allowed_actions"] == []
        assert status["details"]["control_enabled"] is False
        assert status["details"]["monitoring_only"] is True
        assert status["equipment_status"] == "unknown"
        assert json.loads(path.read_text()) == original


def test_linear_target_requires_explicit_tcp_and_missing_joints_are_not_zero():
    with TestClient(create_app()) as client:
        candidate = graph()
        candidate["edges"][0]["mode"] = "linear"
        assert client.post("/graph/validate", json=candidate).status_code == 422
        candidate["nodes"][1]["tcp_mm_rpy_deg"] = [100, 200, 300, 0, 0, 0]
        assert client.post("/graph/validate", json=candidate).status_code == 200
        del candidate["nodes"][0]["joints_deg"]
        assert client.post("/graph/validate", json=candidate).status_code == 422


def test_preview_is_offline_and_never_authorizes_hardware():
    with TestClient(create_app()) as client:
        result = client.post(
            "/graph/preview", json={"graph": graph(), "source": "a", "target": "c"}
        )
        assert result.status_code == 200
        assert result.json()["path"] == ["a", "b", "c"]
        assert result.json()["executed"] is False
        assert result.json()["physical_validation"] is False
        assert (
            client.post(
                "/graph/preview", json={"graph": graph(), "source": "c", "target": "a"}
            ).status_code
            == 422
        )
        assert (
            client.post(
                "/graph/preview",
                json={"graph": graph(), "source": "missing", "target": "a"},
            ).status_code
            == 422
        )


class FakeSocket:
    def __init__(self, replies):
        self.data = io.BytesIO(replies)
        self.sent = []

    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass

    def makefile(self, _):
        return self.data

    def sendall(self, data):
        self.sent.append(data)


@pytest.mark.parametrize(
    "model,safety",
    [
        ("ur5e", b"Safetystatus: NORMAL\n"),
        ("ur5_cb3", b"Command not found\nSafetymode: NORMAL\n"),
    ],
)
def test_readonly_queries_and_cb3_fallback(model, safety):
    sock = FakeSocket(
        b"Connected: Universal Robots Dashboard Server\nRobotmode: RUNNING\n"
        + safety
        + b"PLAYING private-program.urp\n"
    )
    settings = Settings(driver="ur", model=model, robot_host="example.invalid")
    observer = URObserver(settings, connector=lambda *_a, **_k: sock)
    observation = observer.read()
    assert observation["equipment_status"] == "busy"
    assert observation["activity"] == "running"
    assert observation["details"]["program_state"] == "PLAYING"
    assert b"private-program" not in str(observation).encode()
    assert set(sock.sent) <= {
        b"robotmode\n",
        b"safetystatus\n",
        b"safetymode\n",
        b"programState\n",
    }
    assert sock.data.closed


@pytest.mark.parametrize(
    "mode,safety,program,state,activity",
    [
        ("RUNNING", "NORMAL", "PLAYING", "busy", "running"),
        ("RUNNING", "NORMAL", "STOPPED", "ready", "idle"),
        ("RUNNING", "REDUCED", "PLAYING", "degraded", "running"),
        ("RUNNING", "NORMAL", "PAUSED", "degraded", "idle"),
        ("RUNNING", "ROBOT_EMERGENCY_STOP", "STOPPED", "e_stop", "idle"),
        ("RUNNING", "PROTECTIVE_STOP", "STOPPED", "error", "idle"),
        ("RUNNING", "PROTECTIVE_STOP", "PLAYING", "error", "running"),
        ("POWER_OFF", "NORMAL", "STOPPED", "requires_init", "idle"),
        ("RUNNING", "UNRECOGNIZED", "STOPPED", "unknown", "unknown"),
    ],
)
def test_status_contract(mode, safety, program, state, activity):
    value = interpret(mode, safety, program)
    assert value["equipment_status"] == state
    assert value["activity"] == activity
