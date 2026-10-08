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
    """Dashboard-shaped observation; tests flip program_state or make it fail."""

    def __init__(self):
        self.program_state = "STOPPED"
        self.failing = False

    def read(self):
        if self.failing:
            raise ConnectionError("fixture dashboard unreachable")
        return {
            "equipment_status": "ready",
            "activity": "idle",
            "message": "fixture",
            "components": {},
            "details": {
                "robotmode": "RUNNING",
                "safetystatus": "NORMAL",
                "program_state": self.program_state,
            },
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
        self.watchdogs = []
        self.watchdog_accepted = True
        self.kicks = 0
        self.kick_ok = True
        self.program_running = True
        self.script_stops = 0

    def isConnected(self):
        return self.connected

    def isProgramRunning(self):
        return self.program_running

    def stopScript(self):
        self.script_stops += 1
        self.program_running = False

    def setWatchdog(self, min_frequency):
        self.watchdogs.append(min_frequency)
        return self.watchdog_accepted

    def kickWatchdog(self):
        self.kicks += 1
        return self.kick_ok

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
        self.observer = Observer()
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
                observer=self.observer,
                control_session_factory=self.session,
                edge_secret=secret,
            )
        )


def wait_observed(client):
    """The poll runs in a thread at startup; wait for its first observation."""
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if client.get("/status").json()["details"].get("robotmode"):
            return
        time.sleep(0.02)
    raise AssertionError("fixture observation never arrived")


def claim(client, session_id="session-a"):
    wait_observed(client)
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
        wait_observed(client)
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


def test_session_routes_need_no_claim_until_someone_holds_one():
    # The xArm order: Connect, then Take Control. Never against another holder.
    rig = Rig(settings())
    with rig.client() as client:
        wait_observed(client)
        connected = client.post("/connect", headers=HEADERS)
        assert connected.status_code == 200, connected.text
        # The session opens, but moving still needs the claim.
        assert client.post("/control/joint_step", json=step(), headers=HEADERS).status_code == 423
        assert client.post("/disconnect", headers=HEADERS).status_code == 200

        held = claim(client)
        refused = client.post("/connect", headers=HEADERS)
        assert refused.status_code == 423
        assert refused.json()["detail"]["claimed_by"]["owner"] == OPERATOR
        assert client.post("/connect", headers={**HEADERS, "X-Claim-Token": "stale"}).status_code == 423
        assert client.post("/connect", headers=held).status_code == 200
        assert client.post("/disconnect", headers=HEADERS).status_code == 423
        assert client.get("/status").json()["details"]["control_session"]["open"] is True
        assert client.post("/disconnect", headers=held).status_code == 200
    assert rig.control.moves == []


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
        assert status["allowed_actions"] == ["control.stop", "disconnect", "control.joint_step"]
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
        assert status["allowed_actions"] == ["control.stop", "disconnect"]
        assert status["details"]["control_session"]["latched"]
        last_event = status["details"]["control_session"]["last_event"]
        assert last_event["kind"] == "joint_step_refused"
        assert last_event["operator"] == OPERATOR
        assert len(rig.control.moves) == 1

        assert client.post("/move/stop", headers=HEADERS).status_code == 200
        disconnected = client.post("/disconnect", headers=held)
        assert disconnected.status_code == 200
        assert disconnected.json() == {"connected": False, "close_error": None, "message": "Arm session closed"}
        assert rig.control.disconnections == 1 and rig.receiver.disconnections == 1
        # The control script is ended, not left running for the watchdog.
        assert rig.control.script_stops == 1
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


def test_connect_refuses_unless_the_robot_is_observed_idle():
    rig = Rig(settings())
    rig.observer.program_state = "PLAYING"  # someone else's program owns the robot
    with rig.client() as client:
        held = claim(client)
        refused = client.post("/connect", headers=held)
        assert refused.status_code == 412
        detail = refused.json()["detail"]
        assert detail["error"] == "robot_not_idle"
        assert detail["observed"]["program_state"] == "PLAYING"
        assert rig.control_calls == [] and rig.receiver_calls == []
        status = client.get("/status").json()
        assert status["details"]["control_session"]["last_event"]["kind"] == "connect_refused"
        # STATUS_SPEC 6.2: an action that would 412 is not offered.
        assert status["allowed_actions"] == ["control.stop"]
    # PAUSED is not idle either: the paused program still owns the robot.
    paused = Rig(settings())
    paused.observer.program_state = "PAUSED"
    with paused.client() as client:
        held = claim(client)
        assert client.post("/connect", headers=held).json()["detail"]["error"] == "robot_not_idle"
        assert paused.control_calls == []
    # No fresh observation at all: unknown is not idle.
    fresh = Rig(settings())
    fresh.observer.failing = True
    with fresh.client() as client:
        token = client.post(
            "/control/claim",
            json={"owner": "x", "session_id": "s", "ttl_s": 60},
            headers=HEADERS,
        ).json()["claim_token"]
        refused = client.post("/connect", headers={**HEADERS, "X-Claim-Token": token})
        assert refused.status_code == 412
        assert refused.json()["detail"]["error"] == "observation_unavailable"
        assert fresh.control_calls == []


def test_watchdog_is_armed_kicked_and_fails_closed():
    rig = Rig(settings(control={
        "authorized_operators": [OPERATOR], "joint_step": LIMITS, "watchdog_hz": 20,
    }))
    with rig.client() as client:
        held = claim(client)
        assert client.post("/connect", headers=held).status_code == 200
        assert rig.control.watchdogs == [20]
        deadline = time.monotonic() + 2
        while rig.control.kicks < 3 and time.monotonic() < deadline:
            time.sleep(0.02)
        assert rig.control.kicks >= 3
        session = client.get("/status").json()["details"]["control_session"]
        assert session["watchdog_ok"] is True and session["watchdog_error"] is None
        # The controller stops acknowledging: no more steps, status says why.
        rig.control.kick_ok = False
        deadline = time.monotonic() + 2
        while rig.control.kicks and client.get("/status").json()["details"]["control_session"]["watchdog_ok"]:
            assert time.monotonic() < deadline, "watchdog failure never surfaced"
            time.sleep(0.02)
        status = client.get("/status").json()
        assert "control.joint_step" not in status["allowed_actions"]
        assert "watchdog" in status["details"]["control_session"]["watchdog_error"].lower() or status["details"]["control_session"]["watchdog_error"]
        kicks_after_fault = rig.control.kicks
        refused = client.post("/control/joint_step", json=step(), headers=held)
        assert refused.status_code == 412
        assert "disconnected" in refused.json()["detail"]["reason"]
        time.sleep(0.1)
        assert rig.control.kicks == kicks_after_fault  # kicking stopped
        assert rig.control.moves == []
        assert client.post("/disconnect", headers=held).status_code == 200
    # A controller that refuses the watchdog never gets a session.
    rig = Rig(settings())
    rig.control.watchdog_accepted = False
    with rig.client() as client:
        held = claim(client)
        failed = client.post("/connect", headers=held)
        assert failed.status_code == 502
        assert "watchdog" in failed.json()["detail"]["message"].lower()
        assert rig.control.disconnections == 1 and rig.receiver.disconnections == 1
        assert rig.control.script_stops == 1  # uploaded, so ended
        assert client.get("/status").json()["allowed_actions"] == ["control.stop", "connect"]


def test_audit_file_records_every_control_event(tmp_path):
    import json

    audit = tmp_path / "control-audit.jsonl"
    rig = Rig(settings(control={
        "authorized_operators": [OPERATOR], "joint_step": LIMITS, "audit_file": str(audit),
    }))
    with rig.client() as client:
        held = claim(client)
        assert client.post("/connect", headers=held).status_code == 200
        request = step()
        assert client.post("/control/joint_step", json=request, headers=held).status_code == 200
        assert client.post("/control/stop", headers=HEADERS).status_code == 200
        assert client.post("/disconnect", headers=held).status_code == 200
        assert client.post("/control/release", headers=held).status_code == 204
        # A second release (token already gone) is still 204 but not audited.
        assert client.post("/control/release", headers=held).status_code == 204
    events = [json.loads(line) for line in audit.read_text(encoding="utf-8").splitlines()]
    assert [e["kind"] for e in events] == ["claim", "connect", "joint_step", "stop", "disconnect", "release"]
    assert all(e["operator"] == OPERATOR for e in events)
    assert events[5]["session_id"] == "session-a"
    assert events[2]["request_id"] == request["request_id"]
    assert events[3]["requested"] is True


def test_relative_audit_file_resolves_beside_the_config(tmp_path):
    import json

    from robot_motion.config import load_settings

    config = tmp_path / "robot.local.json"
    config.write_text(json.dumps({
        "driver": "ur", "model": "ur5e", "observe": True, "robot_host": "robot.invalid",
        "ur_transport": "rtde", "control_enabled": True,
        "control": {"authorized_operators": [OPERATOR], "joint_step": LIMITS,
                    "audit_file": "logs/control.jsonl"},
    }))
    loaded = load_settings(config)
    assert loaded.control.audit_file == str((tmp_path / "logs" / "control.jsonl").resolve())


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
