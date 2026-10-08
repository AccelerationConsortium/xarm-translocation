"""Offline tests for arm motion (control.motion): joint and Cartesian moves and
jogs, their limits, STOP, latches and reset, and the fast force telemetry.

The robot is a fake that steps toward each commanded target one controller
packet at a time, with a toy linear kinematics (TCP position follows J1-J3,
orientation is J4-J6 as a rotation vector). Limits are synthetic fixture
values, not UR5e bounds. No vendor SDK, socket or robot is touched.
"""

import math
import threading
import time
from uuid import uuid4

import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from robot_motion.app import create_app
from robot_motion.config import Settings
from robot_motion.drivers.ur_control import MOTION_FEEDBACK_VARIABLES, ControlSession
from robot_motion.drivers.ur_motion import MOVE_ACTIONS, MotionLimits, profile_time

pytest.importorskip("scipy")

SECRET = "offline-test-secret"
OPERATOR = "operator@example.invalid"
HEADERS = {"X-Auth-User": OPERATOR, "X-Edge-Auth": SECRET}
MOTION = {
    "commissioning_id": "OFFLINE-FIXTURE-ONLY",
    "joint_lower_deg": [-90.0] * 6,
    "joint_upper_deg": [90.0] * 6,
    "workspace": {"x_mm": [150, 450], "y_mm": [-150, 150], "z_mm": [150, 450]},
    "max_joint_speed_deg_s": 20,
    "joint_accel_deg_s2": 40,
    "max_linear_speed_mm_s": 100,
    "linear_accel_mm_s2": 500,
    "stop_joint_decel_deg_s2": 90,
    "stop_linear_decel_mm_s2": 1000,
}
MOVE_PATHS = {
    "arm.move_joints": ("/control/freehand/joints", {"angles": [2, 0, 0, 0, 0, 0]}),
    "arm.jog_joint": ("/control/freehand/joint_jog", {"joint": 1, "delta": 2}),
    "arm.move_linear": (
        "/control/freehand/linear",
        {"x": 310, "y": 0, "z": 300, "roll": 0, "pitch": 0, "yaw": 0},
    ),
    "arm.jog_linear": ("/control/freehand/relative", {"dx": 5}),
}


K = 0.3  # toy kinematics: metres of TCP travel per radian of J1-J3


def fk(q):
    return [0.3 + K * q[0], K * q[1], 0.3 + K * q[2], q[3], q[4], q[5]]


def ik(pose):
    return [(pose[0] - 0.3) / K, pose[1] / K, (pose[2] - 0.3) / K, pose[3], pose[4], pose[5]]


def settings(motion=None, **overrides):
    base = dict(
        driver="ur",
        model="ur5e",
        observe=True,
        robot_host="robot.invalid",
        ur_transport="rtde",
        control_enabled=True,
        control={"authorized_operators": [OPERATOR], "motion": {**MOTION, **(motion or {})}},
    )
    base.update(overrides)
    return Settings(**base)


class World:
    """Joints in radians; a commanded move advances one step per packet."""

    def __init__(self):
        self.q = [0.0] * 6
        self.target = None
        self.step = [0.0] * 6
        self.left = 0
        self.ticks_per_move = 10
        self.stuck = False
        self.robot_mode = 7
        self.safety_mode = 1
        self.wrench = [1.0, 2.0, 3.0, 0.0, 0.0, 0.0]
        self.speed_fraction = 1.0
        self.overspeed = 1.0
        self.velocity = [0.0] * 6
        self.on_tick = None
        self.lock = threading.Lock()

    def command(self, target, speed):
        """speed: the leading joint's rad/s, reported while moving."""
        with self.lock:
            self.target = list(target)
            self.left = self.ticks_per_move
            self.step = [(t - q) / self.left for q, t in zip(self.q, target)]
            lead = max(abs(v) for v in self.step) or 1.0
            self.velocity = [v / lead * speed for v in self.step]

    def halt(self):
        with self.lock:
            self.left, self.target = 0, None

    def tick(self):
        with self.lock:
            if self.left and not self.stuck:
                self.q = [q + s for q, s in zip(self.q, self.step)]
                self.left -= 1
                if self.left == 0:
                    self.q = list(self.target)
        if self.on_tick:
            self.on_tick(self)

    def velocities(self):
        with self.lock:
            return [v * self.overspeed for v in self.velocity] if self.left else [0.0] * 6


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
        self.world.tick()
        return self.timestamp

    def getActualQ(self):
        return list(self.world.q)

    def getActualQd(self):
        return self.world.velocities()

    def getRobotMode(self):
        return self.world.robot_mode

    def getSafetyMode(self):
        return self.world.safety_mode

    def getActualTCPPose(self):
        return fk(self.world.q)

    def getActualTCPForce(self):
        return list(self.world.wrench)

    def getTargetSpeedFraction(self):
        return self.world.speed_fraction

    def disconnect(self):
        self.disconnections += 1
        self.connected = False


class Control:
    def __init__(self, world):
        self.world = world
        self.connected = True
        self.moves = []
        self.stops = []
        self.fk = fk
        self.ik = lambda pose, qnear: ik(pose)
        self.fk_calls = 0
        self.ik_calls = 0
        self.joints_ok = True
        self.pose_ok = True
        self.has_solution = True
        self.on_plan = None  # hook called during planning
        self.program_running = True
        self.zeroed = 0
        self.disconnections = 0
        self.script_stops = 0
        self.teaching = False
        self.teach_calls = []
        self.teach_ok = True
        self.end_teach_ok = True

    def isConnected(self):
        return self.connected

    def teachMode(self):
        self.teach_calls.append("on")
        self.teaching = True
        return self.teach_ok

    def endTeachMode(self):
        self.teach_calls.append("off")
        if self.end_teach_ok:
            self.teaching = False
        return self.end_teach_ok

    def stopScript(self):
        self.script_stops += 1
        self.program_running = False
        self.teaching = False  # teach mode ends with the script

    def setWatchdog(self, _hz):
        return True

    def kickWatchdog(self):
        return True

    def moveJ(self, q, speed, acceleration, asynchronous):
        self.moves.append(("moveJ", list(q), speed, acceleration, asynchronous))
        self.world.command(q, speed)
        return True

    def moveL(self, pose, speed, acceleration, asynchronous):
        self.moves.append(("moveL", list(pose), speed, acceleration, asynchronous))
        self.world.command(ik(pose), speed / K)
        return True

    def stopJ(self, deceleration):
        self.stops.append(("stopJ", deceleration))
        self.world.halt()

    def stopL(self, deceleration):
        self.stops.append(("stopL", deceleration))
        self.world.halt()

    def getForwardKinematics(self, q):
        self.fk_calls += 1
        return self.fk(q)

    def getInverseKinematics(self, pose, qnear):
        self.ik_calls += 1
        return self.ik(pose, qnear)

    def getInverseKinematicsHasSolution(self, pose, qnear):
        return self.has_solution

    def isPoseWithinSafetyLimits(self, pose):
        return self.pose_ok

    def isJointsWithinSafetyLimits(self, q):
        if self.on_plan:
            self.on_plan()
        return self.joints_ok

    def isProgramRunning(self):
        return self.program_running

    def getTCPOffset(self):
        return [0.0, 0.0, 0.2, 0.0, 0.0, 0.0]

    def zeroFtSensor(self):
        self.zeroed += 1
        return True

    def disconnect(self):
        self.disconnections += 1
        self.connected = False


class Observer:
    def __init__(self):
        self.remote_control = True
        self.program_state = "STOPPED"

    def read(self):
        return {
            "equipment_status": "ready",
            "activity": "idle",
            "message": "fixture",
            "components": {},
            "details": {
                "robotmode": "RUNNING",
                "safetystatus": "NORMAL",
                "program_state": self.program_state,
                "remote_control": self.remote_control,
                "operational_mode": "AUTOMATIC",
            },
        }


class Rig:
    def __init__(self, config=None):
        self.config = config or settings()
        self.world = World()
        self.observer = Observer()
        self.control = Control(self.world)
        self.receiver = Receiver(self.world)
        self.receiver_calls = []

    def session(self):
        def receiver_factory(host, **kwargs):
            self.receiver_calls.append(kwargs)
            return self.receiver

        return ControlSession(
            self.config, control_factory=lambda host: self.control, receiver_factory=receiver_factory
        )

    def client(self):
        return TestClient(
            create_app(
                self.config,
                observer=self.observer,
                control_session_factory=self.session,
                edge_secret=SECRET,
            )
        )


def wait_for(predicate, timeout=5, what="condition"):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, f"{what} never happened"
        time.sleep(0.01)


def connected(client):
    wait_for(lambda: client.get("/status").json()["details"].get("robotmode"), what="first observation")
    token = client.post(
        "/control/claim", json={"owner": "x", "session_id": "panel", "ttl_s": 60}, headers=HEADERS
    ).json()["claim_token"]
    held = {**HEADERS, "X-Claim-Token": token}
    response = client.post("/connect", json={"profile_name": "ur5e"}, headers=held)
    assert response.status_code == 200, response.text
    return held


def allowed(client):
    return client.get("/status").json()["allowed_actions"]


def dispatched(client):
    """True once a move has been sent to the controller (planning is over)."""
    return client.get("/status").json()["details"]["control_session"]["active_move"] is not None


def in_background(client, path, body, headers):
    result = {}
    thread = threading.Thread(target=lambda: result.update(r=client.post(path, json=body, headers=headers)))
    thread.start()
    return thread, result


# ── configuration ────────────────────────────────────────────────────
@pytest.mark.parametrize(
    "change",
    [
        {"stop_joint_decel_deg_s2": 10},  # softer than the move acceleration
        {"stop_linear_decel_mm_s2": 100},
        {"default_joint_speed_deg_s": 30},  # above the cap
        {"max_joint_speed_deg_s": 90},  # above the code ceiling
        {"max_linear_speed_mm_s": 400},
        {"joint_lower_deg": [10.0] * 6, "joint_upper_deg": [5.0] * 6},
        {"workspace": {"x_mm": [450, 150], "y_mm": [-150, 150], "z_mm": [150, 450]}},
    ],
)
def test_motion_limits_must_be_coherent(change):
    with pytest.raises(ValidationError):
        MotionLimits(**{**MOTION, **change})


def test_joint_step_and_motion_are_exclusive():
    joint_step = {
        "commissioning_id": "X", "lower_deg": [-1.0] * 6, "upper_deg": [1.0] * 6,
        "stop_deceleration_deg_s2": 10.0,
    }
    with pytest.raises(ValidationError):
        Settings(
            driver="ur", model="ur5e", observe=True, robot_host="robot.invalid", ur_transport="rtde",
            control_enabled=True,
            control={"authorized_operators": [OPERATOR], "joint_step": joint_step, "motion": MOTION},
        )


def test_trapezoid_duration():
    assert profile_time(0, 10, 10) == 0
    assert profile_time(100, 10, 10) == pytest.approx(11)  # cruise + one ramp
    assert profile_time(1, 10, 10) == pytest.approx(2 * math.sqrt(0.1))  # never reaches speed


# ── session and status ───────────────────────────────────────────────
def test_connect_opens_a_motion_session_and_offers_the_moves():
    rig = Rig()
    with rig.client() as client:
        wait_for(lambda: client.get("/status").json()["details"].get("robotmode"))
        assert allowed(client) == ["control.stop", "connect"]
        for path, body in MOVE_PATHS.values():
            assert client.post(path, json=body, headers=HEADERS).status_code == 423
        held = connected(client)
        assert rig.receiver_calls == [{"frequency": 125, "variables": MOTION_FEEDBACK_VARIABLES}]
        status = client.get("/status").json()
        assert status["allowed_actions"] == [
            "control.stop", "disconnect", *MOVE_ACTIONS, "arm.manual_mode", "arm.zero_force_sensor",
        ]
        session = status["details"]["control_session"]
        assert session["mode"] == "motion" and session["open"] is True
        assert session["tcp_offset_mm_rpy_deg"] == [0.0, 0.0, 200.0, 0.0, 0.0, 0.0]
        assert session["limits"]["workspace_mm"]["x"] == [150, 450]
        assert session["limits"]["default_joint_speed_deg_s"] == 5  # a quarter of the cap
        assert status["details"]["control_implementation"] == "motion"
        assert status["details"]["manual_mode"] is False
        assert client.post("/disconnect", headers=held).json()["message"] == "Arm session closed"
        assert allowed(client) == ["control.stop", "connect"]
        # Disconnect ends the control script rather than leaving it to the watchdog.
        assert rig.control.script_stops == 1 and rig.control.disconnections == 1


def test_connect_refuses_in_local_control_and_does_not_offer_it():
    rig = Rig()
    rig.observer.remote_control = False
    with rig.client() as client:
        wait_for(lambda: client.get("/status").json()["details"].get("robotmode"))
        token = client.post(
            "/control/claim", json={"owner": "x", "session_id": "s", "ttl_s": 60}, headers=HEADERS
        ).json()["claim_token"]
        refused = client.post("/connect", headers={**HEADERS, "X-Claim-Token": token})
        assert refused.status_code == 412
        assert refused.json()["detail"]["error"] == "remote_control_off"
        assert "connect" not in allowed(client)
        assert rig.receiver_calls == []


# ── joint moves ──────────────────────────────────────────────────────
def test_move_joints_checks_the_path_then_moves_and_verifies():
    rig = Rig()
    with rig.client() as client:
        held = connected(client)
        moved = client.post("/control/freehand/joints", json={"angles": [10, -5, 20, 0, 0, 0]}, headers=held)
        assert moved.status_code == 200, moved.text
        body = moved.json()
        assert body["ok"] is True and body["moved"] is True and body["kind"] == "joint"
        assert body["measured_joints_deg"] == pytest.approx([10, -5, 20, 0, 0, 0])
        assert body["speed"] == 5
        kind, target, speed, acceleration, asynchronous = rig.control.moves[0]
        assert kind == "moveJ" and asynchronous is True
        assert target == pytest.approx([math.radians(v) for v in (10, -5, 20, 0, 0, 0)])
        assert speed == pytest.approx(math.radians(5)) and acceleration == pytest.approx(math.radians(40))
        assert rig.control.fk_calls == body["path_samples_checked"] == 11  # every 2 degrees of 20
        assert rig.control.stops == []
        # The same target again is a no-op, not a second move.
        again = client.post("/control/freehand/joints", json={"angles": [10, -5, 20, 0, 0, 0]}, headers=held)
        assert again.json()["moved"] is False and len(rig.control.moves) == 1


def test_joint_jog_is_relative_and_capped():
    rig = Rig()
    with rig.client() as client:
        held = connected(client)
        jog = client.post("/control/freehand/joint_jog", json={"joint": 3, "delta": -4, "speed": 8}, headers=held)
        assert jog.status_code == 200, jog.text
        assert jog.json()["measured_joints_deg"] == pytest.approx([0, 0, -4, 0, 0, 0])
        too_far = client.post("/control/freehand/joint_jog", json={"joint": 1, "delta": 11}, headers=held)
        assert too_far.status_code == 422 and too_far.json()["detail"]["error"] == "jog_too_large"
        assert len(rig.control.moves) == 1


@pytest.mark.parametrize(
    "body,error",
    [
        ({"angles": [0, 0, 0, 0, 0, 0], "speed": 25}, "above_commissioned_limit"),
        ({"angles": [100, 0, 0, 0, 0, 0]}, "target_outside_envelope"),
        ({"angles": [0, 0, 90, 0, 0, 0]}, "path_leaves_workspace"),  # z would reach 457 mm
    ],
)
def test_joint_targets_outside_the_limits_are_refused_before_sending(body, error):
    rig = Rig()
    with rig.client() as client:
        held = connected(client)
        refused = client.post("/control/freehand/joints", json=body, headers=held)
        assert refused.status_code == 422, refused.text
        assert refused.json()["detail"]["error"] == error
        assert rig.control.moves == []
        assert "arm.move_joints" in allowed(client)  # request-specific, not a state


def test_an_intermediate_path_violation_is_caught():
    rig = Rig()
    # Inside the box at both ends, outside part-way (a bulging arc).
    rig.control.fk = lambda q: [0.6, 0, 0.3, 0, 0, 0] if 0.05 < q[0] < 0.1 else fk(q)
    with rig.client() as client:
        held = connected(client)
        refused = client.post("/control/freehand/joints", json={"angles": [10, 0, 0, 0, 0, 0]}, headers=held)
        assert refused.status_code == 422
        assert refused.json()["detail"]["error"] == "path_leaves_workspace"
        assert 0 < refused.json()["detail"]["fraction"] < 1
        assert rig.control.moves == []


def test_controller_safety_limits_are_consulted():
    rig = Rig()
    rig.control.joints_ok = False
    with rig.client() as client:
        held = connected(client)
        refused = client.post("/control/freehand/joints", json={"angles": [5, 0, 0, 0, 0, 0]}, headers=held)
        assert refused.json()["detail"]["error"] == "outside_controller_limits"
        assert rig.control.moves == []


# ── linear moves ─────────────────────────────────────────────────────
def test_cartesian_jog_keeps_orientation_and_uses_si_units():
    rig = Rig()
    with rig.client() as client:
        held = connected(client)
        jog = client.post("/control/freehand/relative", json={"dx": 20, "dy": -10, "dz": 0, "speed": 40}, headers=held)
        assert jog.status_code == 200, jog.text
        body = jog.json()
        assert body["kind"] == "linear"
        assert body["measured_tcp_mm_rpy_deg"][:3] == pytest.approx([320, -10, 300], abs=1e-6)
        kind, pose, speed, acceleration, _ = rig.control.moves[0]
        assert kind == "moveL"
        assert pose == pytest.approx([0.32, -0.01, 0.3, 0, 0, 0])
        assert speed == pytest.approx(0.04) and acceleration == pytest.approx(0.5)
        too_far = client.post("/control/freehand/relative", json={"dz": 60}, headers=held)
        assert too_far.json()["detail"]["error"] == "jog_too_large"


def test_absolute_linear_move_converts_rpy_and_checks_the_box():
    rig = Rig()
    with rig.client() as client:
        held = connected(client)
        move = {"x": 330, "y": 20, "z": 280, "roll": 0, "pitch": 0, "yaw": 5}
        moved = client.post("/control/freehand/linear", json=move, headers=held)
        assert moved.status_code == 200, moved.text
        assert moved.json()["measured_tcp_mm_rpy_deg"] == pytest.approx([330, 20, 280, 0, 0, 5], abs=1e-6)
        pose = rig.control.moves[0][1]
        assert pose[5] == pytest.approx(math.radians(5))
        outside = client.post("/control/freehand/linear", json={**move, "z": 500}, headers=held)
        assert outside.status_code == 422
        assert outside.json()["detail"]["error"] == "target_outside_workspace"
        assert len(rig.control.moves) == 1


def test_a_wrist_flip_along_the_line_is_refused():
    rig = Rig()

    def flipping(pose, qnear):
        q = ik(pose)
        if pose[0] > 0.31:
            q[3] += 1.0  # the solver jumps to another configuration
        return q

    rig.control.ik = flipping
    with rig.client() as client:
        held = connected(client)
        refused = client.post("/control/freehand/relative", json={"dx": 40}, headers=held)
        assert refused.status_code == 422
        assert refused.json()["detail"]["error"] == "path_configuration_change"
        assert rig.control.moves == []
        rig.control.has_solution = False
        unreachable = client.post("/control/freehand/relative", json={"dx": 5}, headers=held)
        assert unreachable.json()["detail"]["error"] == "unreachable"


def test_a_failed_kinematics_query_refuses_without_moving():
    rig = Rig()
    rig.control.ik = lambda pose, qnear: []  # what a solver failure can look like
    with rig.client() as client:
        held = connected(client)
        refused = client.post("/control/freehand/relative", json={"dx": 5}, headers=held)
        assert refused.status_code == 502
        assert refused.json()["detail"]["error"] == "planning_failed"
        assert rig.control.moves == [] and rig.control.stops == []
        assert "arm.jog_linear" in allowed(client)  # not latched: nothing was sent


def test_a_line_that_would_spin_a_joint_past_its_cap_is_refused():
    rig = Rig()

    def wrist_winds(pose, qnear):
        q = ik(pose)
        q[3] = 10 * (pose[0] - 0.3)  # 5.7 deg of wrist per 10 mm: no single jump, but fast
        return q

    rig.control.ik = wrist_winds
    with rig.client() as client:
        held = connected(client)
        refused = client.post("/control/freehand/relative", json={"dx": 40, "speed": 100}, headers=held)
        assert refused.status_code == 422, refused.text
        detail = refused.json()["detail"]
        assert detail["error"] == "joint_speed_exceeded"
        assert detail["peak_joint_speed_deg_s"] == pytest.approx(57.3, abs=0.5)
        assert detail["max_speed_mm_s"] == pytest.approx(34.9, abs=0.5)
        slower = client.post("/control/freehand/relative", json={"dx": 40, "speed": 30}, headers=held)
        assert slower.status_code == 200, slower.text


def test_a_joint_turning_faster_than_the_cap_stops_the_move():
    rig = Rig()
    rig.world.ticks_per_move = 200
    rig.world.overspeed = 10  # measured joint speed ten times the commanded one
    with rig.client() as client:
        held = connected(client)
        failed = client.post("/control/freehand/joints", json={"angles": [10, 0, 0, 0, 0, 0]}, headers=held)
        assert failed.status_code == 500 and "deg/s" in failed.json()["reason"]
        assert rig.control.stops[0][0] == "stopJ"


def test_a_move_inside_the_tolerance_is_reported_not_sent():
    rig = Rig()
    with rig.client() as client:
        held = connected(client)
        tiny = client.post("/control/freehand/joint_jog", json={"joint": 1, "delta": 0.03}, headers=held)
        assert tiny.status_code == 200 and tiny.json()["moved"] is False
        nudge = client.post("/control/freehand/relative", json={"dx": 0.2}, headers=held)
        assert nudge.status_code == 200 and nudge.json()["moved"] is False
        assert rig.control.moves == []


def test_a_stop_during_planning_means_nothing_is_sent():
    rig = Rig()
    with rig.client() as client:
        held = connected(client)
        stops = []

        def stop_from_the_panel():
            # The STOP arrives from another request while planning runs (and
            # holds the SDK lock): its cancel lands at once, its stopJ after.
            rig.control.on_plan = None
            stopper = threading.Thread(target=lambda: stops.append(client.post("/control/stop", headers=HEADERS)))
            stopper.start()
            wait_for(lambda: client.get("/status").json()["details"]["control_session"]["latched"], what="cancel")

        rig.control.on_plan = stop_from_the_panel
        refused = client.post("/control/freehand/joints", json={"angles": [5, 0, 0, 0, 0, 0]}, headers=held)
        assert refused.status_code == 412
        assert refused.json()["detail"]["error"] == "motion_latched"
        assert rig.control.moves == []
        wait_for(lambda: stops, what="stop response")
        assert stops[0].status_code == 200


def test_the_pendant_speed_slider_gates_and_stretches_moves():
    rig = Rig()
    rig.world.speed_fraction = 0.05
    with rig.client() as client:
        held = connected(client)
        refused = client.post("/control/freehand/joint_jog", json={"joint": 1, "delta": 2}, headers=held)
        assert refused.status_code == 412 and refused.json()["detail"]["error"] == "speed_slider_low"
        rig.world.speed_fraction = 0.5
        time.sleep(0.05)
        assert client.post("/control/freehand/joint_jog", json={"joint": 1, "delta": 2}, headers=held).status_code == 200


# ── stop, faults, latch and reset ────────────────────────────────────
@pytest.mark.parametrize(
    "path,body,stop",
    [
        ("/control/freehand/joints", {"angles": [20, 0, 0, 0, 0, 0]}, ("stopJ", math.radians(90))),
        ("/control/freehand/relative", {"dx": 40}, ("stopL", 1.0)),
    ],
)
def test_stop_halts_the_move_latches_and_clear_errors_reenables(path, body, stop):
    rig = Rig()
    rig.world.ticks_per_move = 5000  # about 20 s at the fake's packet rate
    with rig.client() as client:
        held = connected(client)
        thread, result = in_background(client, path, body, held)
        wait_for(lambda: dispatched(client), what="move dispatch")
        status = client.get("/status").json()
        assert status["allowed_actions"] == ["control.stop"]  # nothing else while moving
        busy = client.post("/control/freehand/joint_jog", json={"joint": 1, "delta": 1}, headers=held)
        assert busy.status_code == 409 and busy.json()["detail"]["error"] == "motion_busy"
        assert client.post("/disconnect", headers=held).status_code == 409
        stopped = client.post("/move/stop", headers=HEADERS)  # the panel's STOP; no claim needed
        assert stopped.status_code == 200 and stopped.json()["stop_requested"] is True
        thread.join(5)
        failed = result["r"]
        assert failed.status_code == 500
        assert failed.json()["latched"] is True and failed.json()["stop_attempted"] is True
        kind, deceleration = rig.control.stops[0]  # the STOP route's own stop comes first
        assert kind == stop[0] and deceleration == pytest.approx(stop[1])
        status = client.get("/status").json()
        assert "control.reset" in status["allowed_actions"]
        assert not set(MOVE_ACTIONS) & set(status["allowed_actions"])
        latched = client.post("/control/freehand/joint_jog", json={"joint": 1, "delta": 1}, headers=held)
        assert latched.status_code == 412 and latched.json()["detail"]["error"] == "motion_latched"
        cleared = client.post("/clear/errors", headers=held)  # the panel's Clear errors
        assert cleared.status_code == 200 and cleared.json()["reset"] is True
        assert client.post("/control/stop", headers=HEADERS).status_code == 200  # an idle STOP latches too
        assert "control.reset" in allowed(client)
        assert client.post("/clear/errors", headers=held).json()["reset"] is True
        assert set(MOVE_ACTIONS) <= set(allowed(client))
        assert client.post("/control/reset", headers=held).json()["reset"] is False


def test_force_guard_stops_a_move_when_contact_builds_up():
    rig = Rig(settings({"force_guard_n": 10}))
    rig.world.ticks_per_move = 200

    def contact(world):
        if world.left and world.left < 150:
            world.wrench = [1.0, 2.0, 18.0, 0.0, 0.0, 0.0]  # +15 N from the start

    rig.world.on_tick = contact
    with rig.client() as client:
        held = connected(client)
        failed = client.post("/control/freehand/relative", json={"dz": -30}, headers=held)
        assert failed.status_code == 500
        assert "Force guard" in failed.json()["reason"]
        assert rig.control.stops and rig.control.stops[0][0] == "stopL"


def test_a_protective_stop_mid_move_fails_and_reset_waits_for_the_script():
    rig = Rig()
    rig.world.ticks_per_move = 200

    def protective_stop(world):
        if world.left and world.left < 100:
            world.safety_mode = 3

    rig.world.on_tick = protective_stop
    with rig.client() as client:
        held = connected(client)
        failed = client.post("/control/freehand/joints", json={"angles": [10, 0, 0, 0, 0, 0]}, headers=held)
        assert failed.status_code == 500
        assert "PROTECTIVE_STOP" in failed.json()["reason"]
        rig.world.on_tick = None
        rig.control.program_running = False  # the controller ended the script
        blocked = client.post("/control/reset", headers=held)
        assert blocked.status_code == 412 and blocked.json()["detail"]["error"] == "control_script_stopped"
        assert "control.reset" not in allowed(client)
        rig.world.safety_mode = 1
        rig.control.program_running = True
        wait_for(lambda: "control.reset" in allowed(client), what="reset offered")
        assert client.post("/control/reset", headers=held).json()["reset"] is True


def test_a_stuck_move_times_out_and_stops():
    rig = Rig()
    rig.world.stuck = True
    with rig.client() as client:
        held = connected(client)
        failed = client.post("/control/freehand/joint_jog", json={"joint": 2, "delta": 1}, headers=held)
        assert failed.status_code == 500 and "deadline" in failed.json()["reason"]
        assert rig.control.stops[0][0] == "stopJ"


def test_losing_the_claim_mid_move_stops_the_arm():
    rig = Rig()
    rig.world.ticks_per_move = 5000
    with rig.client() as client:
        held = connected(client)
        thread, result = in_background(client, "/control/freehand/joints", {"angles": [20, 0, 0, 0, 0, 0]}, held)
        wait_for(lambda: dispatched(client), what="move dispatch")
        assert client.post("/control/release", headers=held).status_code == 204
        thread.join(5)
        assert result["r"].status_code == 500
        assert "claim" in result["r"].json()["reason"].lower()
        assert rig.control.stops


# ── STATUS_SPEC 6.2: allowed_actions mirrors the 412s ────────────────
@pytest.mark.parametrize(
    "state",
    ["normal", "outside_workspace", "protective_stop", "latched", "link_down", "script_stopped", "slider_low"],
)
def test_allowed_actions_mirror_move_refusals(state):
    rig = Rig()
    with rig.client() as client:
        held = connected(client)
        if state == "outside_workspace":
            rig.world.q = [0, math.radians(88.8), 0, 0, 0, 0]  # y = 155 mm
        elif state == "protective_stop":
            rig.world.safety_mode = 3
        elif state == "latched":
            client.post("/control/stop", headers=HEADERS)
        elif state == "link_down":
            rig.control.connected = False
        elif state == "script_stopped":
            rig.control.program_running = False
        elif state == "slider_low":
            rig.world.speed_fraction = 0.05
        time.sleep(0.05)
        offered = set(allowed(client))
        for action, (path, body) in MOVE_PATHS.items():
            code = client.post(path, json=body, headers=held).status_code
            assert (action in offered) == (code != 412), (state, action, code)
            if state == "normal":
                assert code == 200


def test_zero_force_sensor_needs_the_session_and_a_still_arm():
    rig = Rig()
    with rig.client() as client:
        held = connected(client)
        zeroed = client.post("/control/force/zero", headers=held)
        assert zeroed.status_code == 200 and rig.control.zeroed == 1
        client.post("/disconnect", headers=held)
        assert client.post("/control/force/zero", headers=held).status_code == 409


def test_every_motion_event_is_audited(tmp_path):
    import json

    audit = tmp_path / "audit.jsonl"
    config = settings()
    config = config.model_copy(update={"control": config.control.model_copy(update={"audit_file": str(audit)})})
    rig = Rig(config)
    with rig.client() as client:
        held = connected(client)
        request_id = str(uuid4())
        assert client.post(
            "/control/freehand/joint_jog", json={"joint": 1, "delta": 2, "request_id": request_id}, headers=held
        ).status_code == 200
        replay = client.post(
            "/control/freehand/joint_jog", json={"joint": 1, "delta": 2, "request_id": request_id}, headers=held
        )
        assert replay.status_code == 422 and replay.json()["detail"]["error"] == "request_replayed"
        # A reset keeps the memory of request ids that already ran.
        client.post("/control/stop", headers=HEADERS)
        assert client.post("/control/reset", headers=held).json()["reset"] is True
        again = client.post(
            "/control/freehand/joint_jog", json={"joint": 1, "delta": 2, "request_id": request_id}, headers=held
        )
        assert again.json()["detail"]["error"] == "request_replayed"
        client.post("/control/freehand/joints", json={"angles": [0, 0, 0, 0, 0, 0], "speed": 99}, headers=held)
    kinds = [json.loads(line)["kind"] for line in audit.read_text().splitlines()]
    assert kinds == [
        "claim", "connect", "arm.jog_joint", "arm.jog_joint_done",
        "arm.jog_joint", "arm_refused", "stop", "reset", "arm.jog_joint", "arm_refused", "arm_refused",
    ]


# ── fast telemetry with the force sensor ─────────────────────────────
def test_status_publishes_live_force_between_slow_dashboard_polls():
    reads = {"telemetry": 0, "dashboard": 0}

    class Split:
        failing = False

        def read_dashboard(self):
            reads["dashboard"] += 1
            return Observer().read()

        def read_telemetry(self):
            if self.failing:
                raise ConnectionError("stream down")
            reads["telemetry"] += 1
            n = reads["telemetry"]
            return {
                "valid": True,
                "source": "rtde_receive",
                "controller_timestamp_s": float(n),
                "joints_deg": [float(n), 0, 0, 0, 0, 0],
                "tcp_mm_rpy_deg": [300, 0, 300, 0, 0, 0],
                "tcp_m_rotvec_rad": [0.3, 0, 0.3, 0, 0, 0],
                "tcp_force": {"force_n": [0, 0, 5.0], "torque_nm": [0, 0, 0], "force_magnitude_n": 5.0, "frame": "base"},
            }

        def read(self):  # never used on the split path
            raise AssertionError("combined read used")

    observer = Split()
    config = Settings(
        driver="ur", model="ur5e", observe=True, robot_host="robot.invalid", ur_transport="rtde",
        poll_interval_s=60, telemetry_interval_s=0.1,
    )
    with TestClient(create_app(config, observer=observer)) as client:
        wait_for(lambda: client.get("/status").json()["details"].get("telemetry", {}).get("valid"))
        first = client.get("/status").json()["details"]["current_joints"][0]
        wait_for(lambda: client.get("/status").json()["details"]["current_joints"][0] > first, what="live update")
        status = client.get("/status").json()
        assert status["details"]["telemetry"]["tcp_force"]["force_magnitude_n"] == 5.0
        assert status["components"]["telemetry"]["state"] == "receiving"
        assert reads["dashboard"] == 1  # the slow poll did not speed up
        observer.failing = True
        wait_for(lambda: client.get("/status").json()["details"]["telemetry"]["valid"] is False)
        status = client.get("/status").json()
        assert status["details"]["current_joints"] is None  # never the last position
        assert status["components"]["telemetry"]["connected"] is False


# ── manual (teach) mode ──────────────────────────────────────────────
def manual(client, enable, headers):
    return client.post("/robot/manual", json={"enable": enable}, headers=headers)


def session_state(client):
    return client.get("/status").json()["details"]["control_session"]


def test_manual_mode_frees_the_arm_refuses_moves_and_turns_off():
    rig = Rig()
    with rig.client() as client:
        assert manual(client, True, HEADERS).status_code == 423  # the claim is needed
        held = connected(client)
        on = manual(client, True, held)
        assert on.status_code == 200, on.text
        assert on.json()["manual_mode"] is True and on.json()["changed"] is True
        assert rig.control.teach_calls == ["on"] and rig.control.teaching
        status = client.get("/status").json()
        assert status["details"]["manual_mode"] is True
        assert status["details"]["control_session"]["manual_mode"] is True
        # Only turning it off, disconnecting and STOP are offered.
        assert status["allowed_actions"] == ["control.stop", "disconnect", "arm.manual_mode"]
        for path, body in MOVE_PATHS.values():
            refused = client.post(path, json=body, headers=held)
            assert refused.status_code == 412 and refused.json()["detail"]["error"] == "manual_mode"
        zero = client.post("/control/force/zero", headers=held)
        assert zero.status_code == 412 and zero.json()["detail"]["error"] == "manual_mode"
        assert rig.control.moves == [] and rig.control.zeroed == 0
        assert manual(client, True, held).json()["changed"] is False  # already on
        off = client.post("/control/manual", json={"enable": False}, headers=held)  # the same route
        assert off.status_code == 200 and off.json() == {
            "ok": True, "manual_mode": False, "changed": True, "message": "Manual mode off",
        }
        assert rig.control.teach_calls == ["on", "off"] and not rig.control.teaching
        assert session_state(client)["latched"] is None  # turning it off is not a fault
        assert set(MOVE_ACTIONS) <= set(allowed(client))
        assert manual(client, False, held).json()["changed"] is False
        assert client.post("/control/freehand/joint_jog", json={"joint": 1, "delta": 1}, headers=held).status_code == 200


def test_manual_mode_is_how_an_arm_outside_the_envelope_comes_back():
    rig = Rig()
    with rig.client() as client:
        held = connected(client)
        rig.world.q = [math.radians(95), 0, 0, 0, 0, 0]  # J1 past the 90 deg fixture limit
        time.sleep(0.05)
        offered = allowed(client)
        assert "arm.manual_mode" in offered and not set(MOVE_ACTIONS) & set(offered)
        assert manual(client, True, held).status_code == 200
        rig.world.q = [0.0] * 6  # guided back in by hand
        assert manual(client, False, held).status_code == 200
        wait_for(lambda: set(MOVE_ACTIONS) <= set(allowed(client)), what="moves offered again")


@pytest.mark.parametrize("state", ["latched", "protective_stop", "script_stopped", "moving"])
def test_manual_mode_needs_a_ready_still_arm(state):
    rig = Rig()
    with rig.client() as client:
        held = connected(client)
        if state == "latched":
            client.post("/control/stop", headers=HEADERS)
        elif state == "protective_stop":
            rig.world.safety_mode = 3
        elif state == "script_stopped":
            rig.control.program_running = False
        elif state == "moving":
            rig.world.ticks_per_move = 5000
            rig.world.command([0.5] + [0.0] * 5, 0.1)
        time.sleep(0.05)
        assert "arm.manual_mode" not in allowed(client)
        refused = manual(client, True, held)
        assert refused.status_code == 412, refused.text
        assert rig.control.teach_calls == []


def test_stop_ends_manual_mode_and_latches():
    rig = Rig()
    with rig.client() as client:
        held = connected(client)
        assert manual(client, True, held).status_code == 200
        stopped = client.post("/move/stop", headers=HEADERS)  # no claim needed
        assert stopped.status_code == 200 and stopped.json()["stop_requested"] is True
        assert rig.control.teach_calls == ["on", "off"] and not rig.control.teaching
        state = session_state(client)
        assert state["manual_mode"] is False
        assert state["latched"] == "Manual mode ended: Stop requested"
        assert client.post("/clear/errors", headers=held).json()["reset"] is True
        assert set(MOVE_ACTIONS) <= set(allowed(client))


@pytest.mark.parametrize("cause", ["claim_released", "protective_stop", "script_stopped", "feedback_lost"])
def test_manual_mode_ends_itself_when_its_conditions_go(cause):
    rig = Rig()
    with rig.client() as client:
        held = connected(client)
        assert manual(client, True, held).status_code == 200
        if cause == "claim_released":
            assert client.post("/control/release", headers=held).status_code == 204
        elif cause == "protective_stop":
            rig.world.safety_mode = 3
        elif cause == "script_stopped":
            rig.control.program_running = False
        elif cause == "feedback_lost":
            rig.receiver.connected = False
        wait_for(lambda: not session_state(client)["manual_mode"], what="teach mode ending")
        latched = session_state(client)["latched"]
        assert latched.startswith("Manual mode ended:"), latched
        expected = {
            "claim_released": "claim",
            "protective_stop": "PROTECTIVE_STOP",
            "script_stopped": "script stopped",
            "feedback_lost": "feedback link dropped",
        }[cause]
        assert expected in latched
        if cause == "script_stopped":
            # Teach mode ended with the script; there is nothing to send.
            assert rig.control.teach_calls == ["on"]
        else:
            assert rig.control.teach_calls == ["on", "off"]


def test_disconnect_ends_manual_mode_and_the_control_script():
    rig = Rig()
    with rig.client() as client:
        held = connected(client)
        assert manual(client, True, held).status_code == 200
        assert client.post("/disconnect", headers=held).status_code == 200
        assert rig.control.teach_calls == ["on", "off"]
        assert rig.control.script_stops == 1 and not rig.control.teaching
        assert client.get("/status").json()["details"]["manual_mode"] is False


def test_a_teach_mode_the_controller_refuses_is_ended_and_latched():
    rig = Rig()
    rig.control.teach_ok = False
    with rig.client() as client:
        held = connected(client)
        failed = manual(client, True, held)
        assert failed.status_code == 500
        body = failed.json()
        assert body["error"] == "manual_mode_failed" and body["latched"] is True
        assert body["manual_mode"] is False
        assert rig.control.teach_calls == ["on", "off"]  # ended in case it took effect
        assert "did not accept teach mode" in session_state(client)["latched"]


def test_a_teach_mode_that_will_not_end_stays_reported_on():
    rig = Rig()
    with rig.client() as client:
        held = connected(client)
        assert manual(client, True, held).status_code == 200
        rig.control.end_teach_ok = False
        failed = manual(client, False, held)
        assert failed.status_code == 500 and failed.json()["manual_mode"] is True
        state = session_state(client)
        assert state["manual_mode"] is True and "did not end" in state["latched"]
        for path, body in MOVE_PATHS.values():
            assert client.post(path, json=body, headers=held).status_code == 412
        # Clear errors cannot swap the executor out from under teach mode.
        reset = client.post("/clear/errors", headers=held)
        assert reset.status_code == 412 and reset.json()["detail"]["error"] == "manual_mode"
        # Disconnect still ends it, with the control script.
        assert client.post("/disconnect", headers=held).status_code == 200
        assert rig.control.script_stops == 1 and not rig.control.teaching
