"""Offline tests for the config-gated claim surface and joint-step routes.

Control and receive interfaces are fakes; no vendor constructor, socket or
robot is touched. Limits are synthetic fixture values, not UR5e bounds.
"""

import math
import sys
import time
from uuid import uuid4

import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from robot_motion.app import create_app
from robot_motion.config import Settings
from robot_motion.drivers.ur_control import (
    FEEDBACK_VARIABLES,
    ControlFeedback,
    ControlSession,
    mode_name,
    ROBOT_MODES,
    SAFETY_MODES,
)

SECRET = "offline-test-secret"
OPERATOR = "operator@example.invalid"
HEADERS = {"X-Auth-User": OPERATOR, "X-Edge-Auth": SECRET}
LIMITS = {
    "commissioning_id": "OFFLINE-FIXTURE-ONLY",
    "lower_deg": [-1.0] * 6,
    "upper_deg": [1.0] * 6,
    "stop_deceleration_deg_s2": 10.0,
}


def settings(**overrides):
    base = dict(
        driver="ur",
        model="ur5e",
        observe=True,
        robot_host="robot.invalid",
        ur_transport="rtde",
        control_enabled=True,
        control={"authorized_operators": [OPERATOR], "joint_step": LIMITS},
    )
    base.update(overrides)
    return Settings(**base)


class Observer:
    def read(self):
        return {
            "equipment_status": "ready",
            "activity": "idle",
            "message": "fixture",
            "components": {},
            "details": {},
        }


class World:
    """Shared fake robot state: joints in radians, controller enumerations."""

    def __init__(self):
        self.joints = [0.0] * 6
        self.robot_mode = 7
        self.safety_mode = 1


class Receiver:
    def __init__(self, world):
        self.world = world
        self.timestamp = 10.0
        self.connected = True
        self.disconnections = 0

    def isConnected(self):
        return self.connected

    def getTimestamp(self):
        self.timestamp += 0.008
        return self.timestamp

    def getActualQ(self):
        return list(self.world.joints)

    def getActualQd(self):
        return [0.0] * 6

    def getRobotMode(self):
        return self.world.robot_mode

    def getSafetyMode(self):
        return self.world.safety_mode

    def disconnect(self):
        self.disconnections += 1
        self.connected = False


class Control:
    def __init__(self, world):
        self.world = world
        self.connected = True
        self.accepted = True
        self.moves = []
        self.stops = []
        self.disconnections = 0

    def isConnected(self):
        return self.connected

    def moveJ(self, target, speed, acceleration, asynchronous):
        self.moves.append((list(target), speed, acceleration, asynchronous))
        if self.accepted:
            self.world.joints = list(target)
        return self.accepted

    def stopJ(self, deceleration):
        self.stops.append(deceleration)

    def disconnect(self):
        self.disconnections += 1
        self.connected = False


class Rig:
    def __init__(self, config):
        self.config = config
        self.world = World()
        self.control = Control(self.world)
        self.receiver = Receiver(self.world)
        self.control_calls = []
        self.receiver_calls = []

    def session(self):
        def control_factory(host):
            self.control_calls.append(host)
            return self.control

        def receiver_factory(host, **kwargs):
            self.receiver_calls.append((host, kwargs))
            return self.receiver

        return ControlSession(
            self.config, control_factory=control_factory, receiver_factory=receiver_factory
        )

    def client(self, secret=SECRET):
        return TestClient(
            create_app(
                self.config,
                observer=Observer(),
                control_session_factory=self.session,
                edge_secret=secret,
            )
        )


def claim(client, session_id="session-a"):
    response = client.post(
        "/control/claim",
        json={"owner": "ignored client value", "session_id": session_id, "ttl_s": 60},
        headers=HEADERS,
    )
    assert response.status_code == 200, response.text
    return {**HEADERS, "X-Claim-Token": response.json()["claim_token"]}


def step(joint=2, delta_deg=0.1):
    return {"request_id": str(uuid4()), "joint": joint, "delta_deg": delta_deg}


@pytest.mark.parametrize(
    "overrides",
    [
        {"control": None},
        {"ur_transport": "dashboard"},
        {"observe": False, "robot_host": None},
        {"control": {"authorized_operators": [], "joint_step": LIMITS}},
        {"control": {"authorized_operators": ["not an email"], "joint_step": LIMITS}},
        {"control": {"authorized_operators": [OPERATOR], "joint_step": {**LIMITS, "lower_deg": [2.0] * 6}}},
        {"control": {"authorized_operators": [OPERATOR], "joint_step": {**LIMITS, "max_step_deg": 1.0}}},
    ],
)
def test_control_configuration_must_be_explicit_and_coherent(overrides):
    with pytest.raises(ValidationError):
        settings(**overrides)


def test_control_block_without_enable_flag_is_staged_only():
    staged = settings(control_enabled=False)
    with TestClient(create_app(staged, observer=Observer())) as client:
        for path in ["/control/claim", "/connect", "/control/joint_step", "/control/stop"]:
            assert client.post(path, json={}, headers=HEADERS).status_code == 404, path
        status = client.get("/status").json()
        assert status["allowed_actions"] == []
        assert status["details"]["control_enabled"] is False
        assert status["details"]["claimed_by"] is None


def test_enabling_control_imports_no_vendor_sdk_before_connect():
    rig = Rig(settings())
    with rig.client() as client:
        assert client.get("/health").status_code == 200
    assert "rtde_control" not in sys.modules
    assert rig.control_calls == []


def test_identity_is_edge_verified_and_allowlisted():
    rig = Rig(settings())
    with rig.client() as client:
        body = {"owner": "x", "session_id": "s", "ttl_s": 30}
        assert client.post("/control/claim", json=body).status_code == 401
        bad = {"X-Auth-User": OPERATOR, "X-Edge-Auth": "wrong"}
        assert client.post("/control/claim", json=body, headers=bad).status_code == 401
        unlisted = {"X-Auth-User": "someone@example.invalid", "X-Edge-Auth": SECRET}
        assert client.post("/control/claim", json=body, headers=unlisted).status_code == 403
        assert client.post("/control/stop", headers=unlisted).status_code == 403
        assert client.post("/control/claim", json=body, headers=HEADERS).status_code == 200
    with rig.client(secret=None) as client:
        response = client.post("/control/claim", json=body, headers=HEADERS)
        assert response.status_code == 503
        assert response.json()["detail"]["error"] == "edge_identity_not_configured"
        assert client.post("/control/stop", headers=HEADERS).status_code == 503


def test_claim_lifecycle_with_hard_enforcement():
    rig = Rig(settings())
    with rig.client() as client:
        status = client.get("/status").json()
        assert status["details"]["control_enabled"] is True
        assert status["details"]["monitoring_only"] is False
        assert status["allowed_actions"] == ["control.stop", "connect"]
        assert status["details"]["claimed_by"] is None
        held = claim(client)
        claimed_by = client.get("/status").json()["details"]["claimed_by"]
        assert claimed_by["owner"] == OPERATOR and claimed_by["session_id"] == "session-a"
        conflict = client.post(
            "/control/claim",
            json={"owner": OPERATOR, "session_id": "session-b", "ttl_s": 30},
            headers=HEADERS,
        )
        assert conflict.status_code == 409
        assert conflict.json()["claimed_by"]["session_id"] == "session-a"
        assert "Retry-After" in conflict.headers
        # Motion without the held token is locked, including with no claim at all.
        locked = client.post("/control/joint_step", json=step(), headers=HEADERS)
        assert locked.status_code == 423
        assert locked.json()["detail"]["claimed_by"]["owner"] == OPERATOR
        assert client.post("/connect", headers=HEADERS).status_code == 423
        # The holder without an open control session is refused, not moved.
        no_session = client.post("/control/joint_step", json=step(), headers=held)
        assert no_session.status_code == 409
        assert no_session.json()["detail"]["error"] == "no_control_session"
        assert client.post("/control/heartbeat", headers=held).status_code == 204
        assert client.post("/control/heartbeat", headers={**HEADERS, "X-Claim-Token": "nope"}).status_code == 401
        assert client.post("/control/release", headers=held).status_code == 204
        assert client.get("/status").json()["details"]["claimed_by"] is None
        assert client.post("/control/joint_step", json=step(), headers=held).status_code == 423
    assert rig.control_calls == [] and rig.control.moves == []


def test_connect_step_stop_and_disconnect_round_trip():
    rig = Rig(settings())
    with rig.client() as client:
        held = claim(client)
        connected = client.post("/connect", headers=held)
        assert connected.status_code == 200, connected.text
        assert connected.json()["connected"] is True
        assert rig.control_calls == ["robot.invalid"]
        assert rig.receiver_calls == [
            ("robot.invalid", {"frequency": 125, "variables": FEEDBACK_VARIABLES})
        ]
        assert client.post("/connect", headers=held).status_code == 409
        status = client.get("/status").json()
        assert status["allowed_actions"] == ["control.stop", "control.joint_step"]
        assert status["details"]["control_session"]["open"] is True
        assert status["details"]["control_session"]["latched"] is None

        request = step(joint=2, delta_deg=0.1)
        moved = client.post("/control/joint_step", json=request, headers=held)
        assert moved.status_code == 200, moved.text
        body = moved.json()
        assert body["completed"] is True
        assert body["request_id"] == request["request_id"]
        assert body["target_deg"] == pytest.approx([0, 0.1, 0, 0, 0, 0])
        assert body["measured_deg"] == pytest.approx([0, 0.1, 0, 0, 0, 0])
        assert body["commissioning_id"] == "OFFLINE-FIXTURE-ONLY"
        assert len(rig.control.moves) == 1
        target, speed, acceleration, asynchronous = rig.control.moves[0]
        assert target == pytest.approx([0, math.radians(0.1), 0, 0, 0, 0])
        assert speed == pytest.approx(math.radians(0.5))
        assert acceleration == pytest.approx(math.radians(1.0))
        assert asynchronous is True
        assert rig.control.stops == []

        replay = client.post("/control/joint_step", json=request, headers=held)
        assert replay.status_code == 412
        assert "never replayed" in replay.json()["detail"]["reason"]
        too_big = client.post("/control/joint_step", json={**step(), "delta_deg": 0.3}, headers=held)
        assert too_big.status_code == 412

        # STOP needs identity but no claim; it latches further steps.
        stopped = client.post("/control/stop", headers=HEADERS)
        assert stopped.status_code == 200
        assert stopped.json()["stop_requested"] is True
        assert stopped.json()["stop_confirmed"] is False
        assert rig.control.stops == [pytest.approx(math.radians(10.0))]
        latched = client.post("/control/joint_step", json=step(), headers=held)
        assert latched.status_code == 412
        assert "latched" in latched.json()["detail"]["reason"]
        status = client.get("/status").json()
        assert status["allowed_actions"] == ["control.stop"]
        assert status["details"]["control_session"]["latched"]
        last_event = status["details"]["control_session"]["last_event"]
        assert last_event["kind"] == "joint_step_refused"
        assert last_event["operator"] == OPERATOR
        assert len(rig.control.moves) == 1

        assert client.post("/move/stop", headers=HEADERS).status_code == 200
        disconnected = client.post("/disconnect", headers=held)
        assert disconnected.status_code == 200
        assert disconnected.json() == {"connected": False, "close_error": None}
        assert rig.control.disconnections == 1 and rig.receiver.disconnections == 1
        assert client.get("/status").json()["allowed_actions"] == ["control.stop", "connect"]
        assert client.post("/control/joint_step", json=step(), headers=held).status_code == 409


def test_unacknowledged_move_fails_closed_and_reports_stop():
    rig = Rig(settings())
    rig.control.accepted = False
    with rig.client() as client:
        held = claim(client)
        assert client.post("/connect", headers=held).status_code == 200
        failed = client.post("/control/joint_step", json=step(), headers=held)
        assert failed.status_code == 500
        body = failed.json()
        assert body["error"] == "joint_step_failed"
        assert body["stop_attempted"] is True and body["stop_confirmed"] is False
        assert body["latched"] is True
        assert len(rig.control.stops) == 1
        assert client.post("/control/joint_step", json=step(), headers=held).status_code == 412
        # Reconciliation is an explicit, claimed reconnect.
        assert client.post("/disconnect", headers=held).status_code == 200
        assert client.post("/connect", headers=held).status_code == 200
        assert client.get("/status").json()["details"]["control_session"]["latched"] is None


def test_unsafe_controller_state_refuses_before_any_command():
    rig = Rig(settings())
    with rig.client() as client:
        held = claim(client)
        assert client.post("/connect", headers=held).status_code == 200
        rig.world.safety_mode = 3  # PROTECTIVE_STOP
        time.sleep(0.05)
        refused = client.post("/control/joint_step", json=step(), headers=held)
        assert refused.status_code == 412
        assert "safety state" in refused.json()["detail"]["reason"]
        assert rig.control.moves == [] and rig.control.stops == []


def test_stop_without_a_session_is_honest():
    rig = Rig(settings())
    with rig.client() as client:
        response = client.post("/control/stop", headers=HEADERS)
        assert response.status_code == 200
        assert response.json()["stop_requested"] is False
        assert response.json()["control_session"] is False
        assert "not a safety-rated stop" in response.json()["notice"]


def test_feedback_reader_stamps_only_new_packets_and_names_modes():
    class Static(Receiver):
        def getTimestamp(self):
            return self.timestamp

    world = World()
    world.joints = [math.pi / 2, 0, 0, 0, 0, 0]
    receiver = Static(world)
    now = [100.0]
    feedback = ControlFeedback(receiver, frequency_hz=125, clock=lambda: now[0])
    assert feedback.poll_once() is True
    first = feedback.read()
    assert first["joints_deg"] == pytest.approx((90, 0, 0, 0, 0, 0))
    assert first["robot_mode"] == "RUNNING" and first["safety_mode"] == "NORMAL"
    assert first["received_monotonic_s"] == 100.0
    now[0] = 101.0
    assert feedback.poll_once() is False
    # A repeated packet is not re-stamped, and a bounded wait for a newer one
    # times out to the same packet, which the executor then rejects as stale.
    started = time.monotonic()
    repeat = feedback.read(wait_s=0.05)
    assert time.monotonic() - started >= 0.04
    assert repeat["received_monotonic_s"] == 100.0
    receiver.timestamp += 0.008
    world.safety_mode = 7
    assert feedback.poll_once() is True
    newer = feedback.read(wait_s=0)
    assert newer["safety_mode"] == "ROBOT_EMERGENCY_STOP"
    assert newer["received_monotonic_s"] == 101.0
    assert mode_name(ROBOT_MODES, 42) == "UNKNOWN_42"
    assert mode_name(SAFETY_MODES, "x") == "UNKNOWN"
    assert mode_name(SAFETY_MODES, True) == "UNKNOWN"
    with pytest.raises(ConnectionError):
        ControlFeedback(Static(world), frequency_hz=125).read(wait_s=0)
