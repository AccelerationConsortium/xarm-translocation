"""Offline tests for the Robotiq gripper driver and its claimed routes.

The URCap is a fake socket that simulates the registers and finger travel;
no network socket or gripper is touched. Values are fixtures, not
commissioned limits.
"""

import threading
import time

import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from robot_motion.app import create_app
from robot_motion.config import Settings
from robot_motion.drivers.robotiq import (
    GripperBusy,
    GripperSettings,
    RobotiqClient,
    RobotiqGripper,
)

SECRET = "offline-test-secret"
OPERATOR = "operator@example.invalid"
HEADERS = {"X-Auth-User": OPERATOR, "X-Edge-Auth": SECRET}
GRIPPER = {"model": "robotiq_2f140", "poll_interval_s": 0.2, "move_timeout_s": 1.0}


class URCap:
    """Simulated Robotiq URCap socket server state."""

    def __init__(self):
        self.reg = {"ACT": 1, "GTO": 1, "STA": 3, "OBJ": 3, "FLT": 0, "POS": 3, "PRE": 0, "SPE": 255, "FOR": 10}
        self.step = 60            # counts per GET POS while moving
        self.object_at = None     # raw position where closing fingers meet a part
        self.stuck = False
        self.activation_gets = 0
        self.sets = []
        self.lines = []
        self.reachable = True
        self.connections = 0
        self.flt_format = "{:02d}"
        self.ack = "ack\n"

    def handle(self, line):
        self.lines.append(line)
        words = line.split()
        if words[0] == "GET":
            var = words[1]
            if var == "POS":
                self.advance()
            if var == "STA" and self.reg["ACT"] == 1 and self.reg["STA"] != 3:
                self.activation_gets += 1
                if self.activation_gets >= 3:
                    self.reg.update(STA=3, POS=3, PRE=0, OBJ=3)
            value = self.flt_format.format(self.reg[var]) if var == "FLT" else str(self.reg[var])
            return f"{var} {value}"
        if words[0] == "SET":
            pairs = list(zip(words[1::2], (int(v) for v in words[2::2])))
            self.sets.append(pairs)
            for var, value in pairs:
                if var == "POS":
                    self.reg["PRE"] = value
                    self.reg["OBJ"] = 0 if value != self.reg["POS"] else 3
                elif var == "ACT":
                    self.reg["ACT"] = value
                    if value == 0:
                        self.reg.update(STA=0, GTO=0)
                    else:
                        self.reg["STA"] = 1
                        self.activation_gets = 0
                elif var == "GTO" and value == 0:
                    self.reg["GTO"] = 0
                    if self.reg["OBJ"] == 0:
                        self.reg["OBJ"] = 3
                elif var in self.reg:
                    self.reg[var] = value
            return "ack"
        return "?"

    def advance(self):
        r = self.reg
        if r["GTO"] != 1 or r["STA"] != 3 or r["OBJ"] != 0 or self.stuck:
            return
        target = r["PRE"]
        if self.object_at is not None and target > r["POS"] and r["POS"] + self.step >= self.object_at:
            r.update(POS=self.object_at, OBJ=2)
            return
        if abs(target - r["POS"]) <= self.step:
            r.update(POS=target, OBJ=3)
        else:
            r["POS"] += self.step if target > r["POS"] else -self.step

    def connector(self, address, timeout):
        if not self.reachable:
            raise ConnectionRefusedError("fixture gripper unreachable")
        self.connections += 1
        return FakeSocket(self)


class FakeSocket:
    def __init__(self, cap):
        self.cap = cap
        self.pending = b""

    def sendall(self, data):
        if not self.cap.reachable:
            raise BrokenPipeError("fixture gripper went away")
        for line in data.decode().splitlines():
            reply = self.cap.handle(line)
            self.pending += (self.cap.ack if reply == "ack" else reply + "\n").encode()

    def recv(self, n):
        chunk, self.pending = self.pending[:n], self.pending[n:]
        return chunk

    def close(self):
        pass


class Observer:
    def __init__(self):
        self.program_state = "STOPPED"
        self.safety = "NORMAL"

    def read(self):
        return {
            "equipment_status": "ready",
            "activity": "idle",
            "message": "fixture",
            "components": {},
            "details": {
                "robotmode": "RUNNING",
                "safetystatus": self.safety,
                "program_state": self.program_state,
            },
        }


def settings(control=True, gripper=None, **overrides):
    base = dict(
        driver="ur",
        model="ur5e",
        observe=True,
        robot_host="robot.invalid",
        ur_transport="rtde",
        control_enabled=control,
        control={"authorized_operators": [OPERATOR]} if control else None,
        gripper=gripper or GRIPPER,
    )
    base.update(overrides)
    return Settings(**base)


class Rig:
    def __init__(self, config=None):
        self.config = config or settings()
        self.cap = URCap()
        self.observer = Observer()
        self.gripper = RobotiqGripper(
            self.config.gripper,
            "robot.invalid",
            client=RobotiqClient("robot.invalid", 63352, 1.0, connector=self.cap.connector),
        )

    def client(self):
        return TestClient(
            create_app(self.config, observer=self.observer, edge_secret=SECRET, gripper=self.gripper)
        )


def ready(client, state="enabled"):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        status = client.get("/status").json()
        if status["details"].get("robotmode") and status["components"]["gripper"]["state"] == state:
            return status
        time.sleep(0.02)
    raise AssertionError(f"gripper never reported {state}")


def claim(client):
    response = client.post(
        "/control/claim", json={"owner": "x", "session_id": "s", "ttl_s": 60}, headers=HEADERS
    )
    assert response.status_code == 200, response.text
    return {**HEADERS, "X-Claim-Token": response.json()["claim_token"]}


# ── configuration ───────────────────────────────────────────────────
@pytest.mark.parametrize(
    "overrides",
    [
        {"gripper": {**GRIPPER, "default_force_pct": 60, "max_force_pct": 50}},
        {"gripper": {**GRIPPER, "open_raw": 100, "closed_raw": 120}},
        {"gripper": {**GRIPPER, "model": "robotiq_3f"}},
        {"gripper": {**GRIPPER, "unknown": 1}},
    ],
)
def test_gripper_configuration_is_validated(overrides):
    with pytest.raises(ValidationError):
        settings(**overrides)


def test_gripper_needs_an_observed_ur_and_control_needs_something_to_control():
    with pytest.raises(ValidationError):
        Settings(driver="ur", model="ur5e", robot_host="robot.invalid", gripper=GRIPPER)
    with pytest.raises(ValidationError):
        Settings(
            driver="ur", model="ur5e", observe=True, robot_host="robot.invalid", ur_transport="rtde",
            control_enabled=True, control={"authorized_operators": [OPERATOR]},
        )
    gripper_only = settings()
    assert gripper_only.control.joint_step is None


# ── driver ──────────────────────────────────────────────────────────
def test_client_refuses_unlisted_registers_and_auto_release():
    cap = URCap()
    client = RobotiqClient("robot.invalid", 63352, 1.0, connector=cap.connector)
    with pytest.raises(ValueError):
        client.get("SNU")
    with pytest.raises(ValueError):
        client.set([("ATR", 1)])
    with pytest.raises(ValueError):
        client.set([("POS", 256)])
    assert cap.lines == []


def test_client_reads_hex_fault_codes_and_reconnects_after_a_drop():
    cap = URCap()
    cap.reg["FLT"] = 0x0E
    cap.flt_format = "{:02X}"
    client = RobotiqClient("robot.invalid", 63352, 1.0, connector=cap.connector)
    assert client.get("FLT") == 0x0E
    cap.reachable = False
    with pytest.raises(OSError):
        client.get("POS")
    cap.reachable = True
    assert client.get("POS") == 3
    assert cap.connections == 2


@pytest.mark.parametrize("ack", ["ack\n", "ack"])
def test_client_accepts_set_ack_with_or_without_a_newline(ack):
    cap = URCap()
    cap.ack = ack
    client = RobotiqClient("robot.invalid", 63352, 1.0, connector=cap.connector)
    client.set([("SPE", 255)])
    assert client.get("POS") == 3
    client.set([("FOR", 10)])
    assert client.get("FLT") == 0


def test_driver_maps_stroke_to_position_and_rejects_concurrent_commands():
    cap = URCap()
    g = RobotiqGripper(
        GripperSettings(**GRIPPER), "robot.invalid",
        client=RobotiqClient("robot.invalid", 63352, 1.0, connector=cap.connector),
    )
    assert g.mm_to_raw(140) == 0 and g.mm_to_raw(0) == 255 and g.mm_to_raw(70) == 128
    assert g.raw_to_mm(0) == 140.0 and g.raw_to_mm(255) == 0.0
    g._begin()
    try:
        with pytest.raises(GripperBusy):
            g.move(0, 128, 51)
    finally:
        g._end()


# ── monitoring without control ──────────────────────────────────────
def test_monitoring_only_reads_and_publishes_panel_fields():
    rig = Rig(settings(control=False))
    with rig.client() as client:
        status = ready(client)
        assert status["components"]["gripper"] == {
            "connected": True, "state": "enabled", "message": None, "last_event_at": None,
        }
        details = status["details"]
        assert details["gripper"]["opening_mm"] == pytest.approx(138.4, abs=0.1)
        assert details["connection_details"]["gripper_type"] == "robotiq_2f140"
        config = details["connection_details"]["gripper_config"]
        assert config["stroke_range"] == {"min": 0, "max": 140.0}
        assert config["force_range"] == {"min": 0, "max": 50}
        assert status["allowed_actions"] == []
        for path in ["/gripper/open", "/gripper/close", "/component/enable", "/control/claim"]:
            assert client.post(path, json={}, headers=HEADERS).status_code == 404
    assert rig.cap.sets == []
    assert all(line.startswith("GET ") for line in rig.cap.lines)


def test_unreachable_gripper_is_unknown_and_refused():
    rig = Rig()
    rig.cap.reachable = False
    with rig.client() as client:
        deadline = time.monotonic() + 5
        while not client.get("/status").json()["details"].get("robotmode"):
            assert time.monotonic() < deadline
            time.sleep(0.02)
        time.sleep(0.3)
        status = client.get("/status").json()
        assert status["components"]["gripper"]["connected"] is False
        assert status["components"]["gripper"]["state"] == "unknown"
        assert not any(a.startswith("gripper.") for a in status["allowed_actions"])
        headers = claim(client)
        response = client.post("/gripper/open", json={}, headers=headers)
        assert response.status_code == 412
        assert response.json()["detail"]["error"] == "gripper_unavailable"


# ── claimed commands ────────────────────────────────────────────────
def test_gripper_only_control_has_no_arm_routes_and_gates_every_command():
    rig = Rig()
    with rig.client() as client:
        ready(client)
        assert client.post("/connect", headers=HEADERS).status_code == 404
        assert client.post("/control/joint_step", json={}, headers=HEADERS).status_code == 404
        assert client.post("/gripper/open", json={}).status_code in (401, 423)
        assert client.post("/gripper/open", json={}, headers=HEADERS).status_code == 423
        unlisted = {"X-Auth-User": "someone@example.invalid", "X-Edge-Auth": SECRET}
        assert client.post("/control/claim", json={"owner": "x", "session_id": "s", "ttl_s": 30}, headers=unlisted).status_code == 403
        status = client.get("/status").json()
        assert set(status["allowed_actions"]) == {"control.stop", "gripper.open", "gripper.close", "gripper.move"}
        assert status["details"]["control_session"]["arm_control"] is False
    assert rig.cap.sets == []


def test_open_close_and_stroke_move_the_fingers_with_session_force():
    rig = Rig()
    with rig.client() as client:
        ready(client)
        headers = claim(client)
        closed = client.post("/gripper/close", json={"force": None}, headers=headers)
        assert closed.status_code == 200, closed.text
        assert closed.json()["position_raw"] == 255
        assert closed.json()["object"] == "at_requested_position"
        assert rig.cap.sets[-1] == [("POS", 255), ("SPE", 128), ("FOR", 51), ("GTO", 1)]

        assert client.post("/control/freehand/gripper/force", json={"force": 30}, headers=headers).status_code == 200
        half = client.post("/control/freehand/gripper/stroke", json={"stroke": 70, "force": None}, headers=headers)
        assert half.status_code == 200, half.text
        assert half.json()["position_raw"] == 128
        assert half.json()["opening_mm"] == pytest.approx(70, abs=0.6)
        assert rig.cap.sets[-1] == [("POS", 128), ("SPE", 128), ("FOR", 76), ("GTO", 1)]
        assert client.get("/status").json()["details"]["connection_details"]["gripper_config"]["force"] == 30

        opened = client.post("/gripper/open", json={"speed": 20}, headers=headers)
        assert opened.status_code == 200
        assert opened.json()["position_raw"] == 0
        assert rig.cap.sets[-1][1] == ("SPE", 51)


def test_closing_on_a_part_reports_contact():
    rig = Rig()
    rig.cap.object_at = 190
    with rig.client() as client:
        ready(client)
        headers = claim(client)
        response = client.post("/gripper/close", json={}, headers=headers)
        assert response.status_code == 200
        assert response.json()["object"] == "contact_closing"
        assert response.json()["object_detected"] is True
        assert response.json()["position_raw"] == 190


def test_limits_are_refused_not_clamped():
    rig = Rig()
    with rig.client() as client:
        ready(client)
        headers = claim(client)
        assert client.post("/gripper/close", json={"force": 80}, headers=headers).status_code == 422
        assert client.post("/control/freehand/gripper/force", json={"force": 51}, headers=headers).status_code == 422
        assert client.post("/gripper/move/stroke", json={"stroke": 141}, headers=headers).status_code == 422
        assert client.post("/gripper/open", json={"wait": False}, headers=headers).status_code == 422
    assert rig.cap.sets == []


@pytest.mark.parametrize(
    "condition, error",
    [
        ("program_playing", "robot_not_idle"),
        ("protective_stop", "robot_not_idle"),
        ("fault", "gripper_fault"),
        ("not_activated", "gripper_not_activated"),
    ],
)
def test_preconditions_refuse_412_and_mirror_allowed_actions(condition, error):
    rig = Rig()
    if condition == "program_playing":
        rig.observer.program_state = "PLAYING"
    elif condition == "protective_stop":
        rig.observer.safety = "PROTECTIVE_STOP"
    elif condition == "fault":
        rig.cap.reg["FLT"] = 0x0E
    else:
        rig.cap.reg.update(ACT=0, STA=0)
    expected_state = {"fault": "fault", "not_activated": "disabled"}.get(condition, "enabled")
    with rig.client() as client:
        ready(client, state=expected_state)
        headers = claim(client)
        for action, path in [("gripper.open", "/gripper/open"), ("gripper.close", "/gripper/close")]:
            listed = action in client.get("/status").json()["allowed_actions"]
            response = client.post(path, json={}, headers=headers)
            assert not listed
            assert response.status_code == 412
            assert response.json()["detail"]["error"] == error
        assert rig.cap.sets == []
        if condition == "not_activated":
            assert "gripper.activate" in client.get("/status").json()["allowed_actions"]


def test_activation_resets_then_activates_and_is_a_noop_when_active():
    rig = Rig()
    rig.cap.reg.update(ACT=0, STA=0, GTO=0)
    with rig.client() as client:
        ready(client, state="disabled")
        headers = claim(client)
        assert client.post("/component/enable", json={"component": "track"}, headers=headers).status_code == 404
        response = client.post("/component/enable", json={"component": "gripper"}, headers=headers)
        assert response.status_code == 200, response.text
        assert response.json()["already_active"] is False
        assert rig.cap.sets[:2] == [[("ACT", 0), ("ATR", 0)], [("ACT", 1)]]
        again = client.post("/component/enable", json={"component": "gripper"}, headers=headers)
        assert again.json()["already_active"] is True
        assert len(rig.cap.sets) == 2


def test_a_move_that_never_finishes_is_stopped_and_reported():
    rig = Rig()
    with rig.client() as client:
        ready(client)
        headers = claim(client)
        rig.cap.stuck = True
        response = client.post("/gripper/close", json={}, headers=headers)
        assert response.status_code == 500
        body = response.json()
        assert body["error"] == "gripper_failed" and body["stop_attempted"] is True
        assert rig.cap.sets[-1] == [("GTO", 0)]


def test_stop_cancels_an_inflight_move_but_never_touches_an_idle_gripper():
    rig = Rig()
    with rig.client() as client:
        ready(client)
        headers = claim(client)
        idle = client.post("/control/stop", headers=HEADERS)
        assert idle.status_code == 200 and idle.json()["gripper_stop_requested"] is False
        assert rig.cap.sets == []

        rig.cap.step = 1
        result = {}
        mover = threading.Thread(
            target=lambda: result.update(r=client.post("/gripper/close", json={}, headers=headers))
        )
        mover.start()
        deadline = time.monotonic() + 2
        while not rig.gripper.busy:
            assert time.monotonic() < deadline
            time.sleep(0.005)
        busy = client.post("/gripper/open", json={}, headers=headers)
        assert busy.status_code == 409
        stopped = client.post("/control/stop", headers=HEADERS)
        assert stopped.json()["gripper_stop_requested"] is True
        mover.join(timeout=5)
        assert result["r"].status_code == 500
        assert "cancelled" in result["r"].json()["reason"]
        assert [("GTO", 0)] in rig.cap.sets
