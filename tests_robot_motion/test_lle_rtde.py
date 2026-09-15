"""Offline RTDE regression suite: SDK factories are fakes, no robot sockets."""

import math
import sys
import time
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from robot_motion.app import create_app
from robot_motion.config import Settings
from robot_motion.drivers.lle_rtde import URArm, tcp_to_mm_deg, vector6
from robot_motion.drivers.ur import interpret
from robot_motion.drivers.ur_rtde import URRTDEObserver


class Receiver:
    def __init__(self):
        self.timestamp = 10.0
        self.advancing = True
        self.connected = True
        self.disconnections = 0
        self.joints = [0, math.pi / 2, -math.pi / 2, math.pi, 0.1, -0.2]
        self.pose = [0.1, -0.2, 0.3, 0, 0, math.pi / 2]

    def isConnected(self):
        return self.connected

    def getTimestamp(self):
        if self.advancing:
            self.timestamp += 0.02
        return self.timestamp

    def getActualQ(self):
        return self.joints

    def getActualTCPPose(self):
        return self.pose

    def disconnect(self):
        self.disconnections += 1
        self.connected = False


@pytest.fixture
def arm():
    pytest.importorskip("scipy")
    receiver, calls = Receiver(), []

    def factory(host, **kwargs):
        calls.append((host, kwargs))
        return receiver

    return (
        URArm("robot.invalid", timeout=0.1, receiver_factory=factory),
        receiver,
        calls,
    )


def test_constructing_and_discovery_do_not_import_or_connect():
    import subprocess

    script = """
import builtins
original = builtins.__import__
def guarded(name, *args, **kwargs):
    assert name.split('.')[0] not in {'rtde_control', 'rtde_receive', 'rtde_io', 'components'}, name
    return original(name, *args, **kwargs)
builtins.__import__ = guarded
from robot_motion.app import create_app
from robot_motion.config import Settings
from robot_motion.drivers import inventory
from robot_motion.drivers.lle_rtde import URArm
URArm('robot.invalid')
create_app(Settings(driver='ur', model='ur5e', observe=True, robot_host='robot.invalid', ur_transport='rtde'))
assert inventory()['drivers']['ur']['control'] == 'not yet implemented'
"""
    completed = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, timeout=15
    )
    assert completed.returncode == 0, completed.stderr


def test_only_receive_interface_is_constructed(monkeypatch):
    calls = []
    receiver = Receiver()
    monkeypatch.setitem(
        sys.modules,
        "rtde_receive",
        SimpleNamespace(
            RTDEReceiveInterface=lambda *args, **kwargs: (
                calls.append((args, kwargs)) or receiver
            ),
        ),
    )
    arm = URArm("robot.invalid")
    assert not calls
    arm.connect()
    arm.connect()
    assert calls == [
        (
            ("robot.invalid",),
            {
                "frequency": 50.0,
                "variables": ["timestamp", "actual_q", "actual_TCP_pose"],
            },
        )
    ]
    assert not hasattr(arm, "rtde_c") and not hasattr(arm, "gripper")
    assert not hasattr(arm, "movej") and not hasattr(arm, "movel")
    arm.disconnect()
    arm.disconnect()
    assert receiver.disconnections == 1


def test_lle_unit_conventions_and_persistent_receive_connection(arm):
    wrapper, receiver, calls = arm
    with pytest.raises(ConnectionError):
        wrapper.get_joints()
    first, second = wrapper.read(), wrapper.read()
    assert len(calls) == 1
    assert second["controller_timestamp_s"] > first["controller_timestamp_s"]
    assert first["joints_deg"][:4] == pytest.approx([0, 90, -90, 180])
    assert first["tcp_mm_rpy_deg"] == pytest.approx([100, -200, 300, 0, 0, 90])
    assert first["tcp_m_rotvec_rad"] == receiver.pose
    assert wrapper.joint_positions == first["joints_deg"]
    assert wrapper.get_tcp_pose() == first["tcp_mm_rpy_deg"]


def test_nontrivial_rotation_is_not_componentwise_degrees():
    rotation = pytest.importorskip("scipy.spatial.transform").Rotation
    raw = [0.1, 0.2, 0.3, 0.6, -0.7, 0.8]
    converted = tcp_to_mm_deg(raw)
    assert converted[3:] != pytest.approx([math.degrees(v) for v in raw[3:]])
    expected = rotation.from_rotvec(raw[3:]).as_matrix()
    actual = rotation.from_euler("xyz", converted[3:], degrees=True).as_matrix()
    assert actual == pytest.approx(expected)


@pytest.mark.parametrize(
    "value",
    [
        [],
        [0] * 5,
        [0] * 7,
        [float("nan")] * 6,
        [float("inf")] * 6,
        [True] * 6,
        ["0"] * 6,
    ],
)
def test_invalid_coordinates_never_become_zero(value):
    with pytest.raises(ValueError):
        vector6(value)


@pytest.mark.parametrize(
    "failure", ["disconnected", "stalled", "restarted", "joints", "pose"]
)
def test_receive_failure_invalidates_session(arm, failure):
    wrapper, receiver, _ = arm
    wrapper.read()
    if failure == "disconnected":
        receiver.connected = False
    elif failure == "stalled":
        receiver.advancing = False
    elif failure == "restarted":
        receiver.timestamp = 1
    elif failure == "joints":
        receiver.joints = [math.nan] * 6
    else:
        receiver.pose = [0] * 5
    with pytest.raises((ConnectionError, TimeoutError, ValueError)):
        wrapper.read()
    assert wrapper.rtde_r is None
    assert receiver.disconnections == 1


def test_initial_stalled_stream_is_not_a_valid_zero_pose(arm):
    wrapper, receiver, _ = arm
    receiver.timestamp = 0
    receiver.joints = receiver.pose = [0] * 6
    receiver.advancing = False
    with pytest.raises(TimeoutError):
        wrapper.read()
    assert wrapper.rtde_r is None


def test_packet_newer_than_last_poll_is_not_necessarily_fresh(arm):
    wrapper, receiver, _ = arm
    wrapper.read()
    # Stream advances between polls, then freezes. Do not timestamp this cached
    # packet as a fresh measurement merely because it is newer than the last.
    receiver.timestamp += 1
    receiver.advancing = False
    with pytest.raises(TimeoutError):
        wrapper.read()
    assert receiver.disconnections == 1


def test_failed_receive_reconnects_to_new_stream(arm):
    wrapper, receiver, _ = arm
    receiver.connected = False
    with pytest.raises(ConnectionError):
        wrapper.read()
    replacement = Receiver()
    wrapper._factory = lambda *_args, **_kwargs: replacement
    assert wrapper.read()["valid"] is True
    assert receiver.disconnections == 1


@pytest.mark.parametrize("transport", ["rtde_control", "unknown"])
def test_control_transport_not_configurable(transport):
    with pytest.raises(ValidationError):
        Settings(driver="ur", model="ur5e", ur_transport=transport)


def test_transport_is_ur_only_and_does_not_enable_control():
    with pytest.raises(ValidationError):
        Settings(ur_transport="rtde")
    with pytest.raises(ValidationError):
        Settings(driver="ur", model="ur5e", ur_transport="rtde", control_enabled=True)
    assert Settings(driver="ur", model="ur5e").ur_transport == "dashboard"


def settings():
    return Settings(
        driver="ur",
        model="ur5e",
        observe=True,
        robot_host="robot.invalid",
        ur_transport="rtde",
        poll_interval_s=5,
    )


def dashboard(safety="NORMAL"):
    return SimpleNamespace(read=lambda: interpret("RUNNING", safety, "STOPPED"))


def test_dashboard_faults_are_not_hidden_by_valid_rtde(arm):
    wrapper, _, _ = arm
    observer = URRTDEObserver(
        settings(), arm=wrapper, dashboard=dashboard("PROTECTIVE_STOP")
    )
    result = observer.read()
    assert result["equipment_status"] == "error"
    assert result["details"]["telemetry"]["valid"] is True
    observer.close()


def test_telemetry_failure_does_not_invent_hardware_fault_or_retain_pose(arm):
    wrapper, receiver, _ = arm
    observer = URRTDEObserver(settings(), arm=wrapper, dashboard=dashboard())
    assert observer.read()["details"]["telemetry"]["valid"] is True
    receiver.connected = False
    result = observer.read()
    assert result["equipment_status"] == "ready"
    assert result["components"]["telemetry"].connected is False
    assert result["details"]["telemetry"] == {"valid": False, "source": "rtde_receive"}


def test_cached_service_status_staleness_and_shutdown(arm, monkeypatch):
    import robot_motion.app as module

    wrapper, receiver, calls = arm
    observer = URRTDEObserver(settings(), arm=wrapper, dashboard=dashboard())
    with TestClient(create_app(settings(), observer=observer)) as client:
        deadline = time.monotonic() + 3
        while True:
            value = client.get("/status").json()
            if value["details"].get("telemetry", {}).get("valid"):
                break
            assert time.monotonic() < deadline
            time.sleep(0.01)
        assert value["details"]["observed_time"]
        timestamp = receiver.timestamp
        for _ in range(3):
            value = client.get("/status").json()
            assert value["allowed_actions"] == []
            assert value["details"]["control_enabled"] is False
        assert receiver.timestamp == timestamp and len(calls) == 1
        for path in ["/connect", "/control/jog", "/control/movej", "/control/claim"]:
            assert client.post(path, json={}).status_code == 404
        clock = time.monotonic
        monkeypatch.setattr(
            module, "time", SimpleNamespace(monotonic=lambda: clock() + 40)
        )
        stale = client.get("/status").json()
        assert stale["equipment_status"] == "unknown"
        assert "telemetry" not in stale["details"]
        assert stale["details"]["observed_time"] is None
    assert receiver.disconnections == 1


def test_shutdown_drains_inflight_read_before_close():
    import threading

    started, release = threading.Event(), threading.Event()
    calls = []

    def read():
        started.set()
        assert release.wait(3)
        calls.append("read finished")
        return interpret("RUNNING", "NORMAL", "STOPPED")

    observer = SimpleNamespace(read=read, close=lambda: calls.append("closed"))
    with TestClient(create_app(settings(), observer=observer)):
        assert started.wait(3)
        timer = threading.Timer(0.05, release.set)
        timer.start()
    timer.join()
    assert calls == ["read finished", "closed"]
