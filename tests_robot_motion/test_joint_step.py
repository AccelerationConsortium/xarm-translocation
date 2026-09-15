"""All joint-step execution uses fake feedback/control; no vendor constructors."""

import math
import threading
from uuid import uuid4

import pytest
from pydantic import ValidationError

from core.claims import ClaimManager
from robot_motion.drivers.joint_step import (
    JointFeedback,
    JointStep,
    JointStepExecutor,
    JointStepFailed,
    JointStepLimits,
    JointStepRefused,
)


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class Control:
    def __init__(self):
        self.connected = True
        self.moves = []
        self.stops = []
        self.target = None
        self.accepted = True
        self.move_error = None
        self.stop_error = None

    def isConnected(self):
        return self.connected

    def moveJ(self, target, speed, acceleration, asynchronous):
        self.moves.append((target, speed, acceleration, asynchronous))
        if self.move_error:
            raise self.move_error
        self.target = tuple(math.degrees(q) for q in target)
        return self.accepted

    def stopJ(self, deceleration):
        self.stops.append(deceleration)
        if self.stop_error:
            raise self.stop_error


@pytest.fixture
def rig():
    clock, control = Clock(), Control()
    claims = ClaimManager(enforce=True, clock=clock)
    token = claims.acquire("offline fixture", "test-session").token
    state = {
        "timestamp": 1.0,
        "position": (0.0,) * 6,
        "mutate": lambda value: value,
        "authorized": True,
        "reads": 0,
        "authorization_calls": [],
    }

    def feedback():
        state["timestamp"] += 0.02
        state["reads"] += 1
        value = dict(
            joints_deg=(
                control.target if control.target is not None else state["position"]
            ),
            velocities_deg_s=(0.0,) * 6,
            controller_timestamp_s=state["timestamp"],
            received_monotonic_s=clock(),
            controller_connected=True,
            robot_mode="RUNNING",
            safety_mode="NORMAL",
        )
        return state["mutate"](value)

    def authorize(request, target, commissioning_id):
        state["authorization_calls"].append(
            (request.request_id, target, commissioning_id)
        )
        return state["authorized"]

    # These are synthetic test coordinates, NOT limits for a physical robot.
    limits = JointStepLimits(
        commissioning_id="OFFLINE-FIXTURE-ONLY",
        lower_deg=(-1.0,) * 6,
        upper_deg=(1.0,) * 6,
        stop_deceleration_deg_s2=10.0,
    )
    executor = JointStepExecutor(
        control=control,
        read_feedback=feedback,
        claims=claims,
        authorize=authorize,
        limits=limits,
        clock=clock,
        sleep=clock.sleep,
    )
    return executor, control, claims, token, state, clock


def step(**kwargs):
    return JointStep(request_id=uuid4(), joint=2, delta_deg=0.1, **kwargs)


def execute(rig, request=None):
    return rig[0].execute(request or step(), claim_token=rig[3])


def test_single_joint_degrees_to_radians_and_measured_completion(rig):
    executor, control, _, _, state, clock = rig
    result = execute(rig)
    assert len(control.moves) == 1
    target, speed, acceleration, asynchronous = control.moves[0]
    assert target == pytest.approx([0, math.radians(0.1), 0, 0, 0, 0])
    assert speed == math.radians(0.5)
    assert acceleration == math.radians(1)
    assert asynchronous is True
    assert result["completed"] is True
    assert result["measured_deg"] == pytest.approx(result["target_deg"])
    assert clock() >= 100.12
    assert len(state["authorization_calls"]) >= 4
    assert state["authorization_calls"][-1][1] == result["target_deg"]
    assert not control.stops


def test_negative_joint_step_does_not_wrap_angles(rig):
    result = execute(rig, JointStep(request_id=uuid4(), joint=6, delta_deg=-0.1))
    assert result["target_deg"] == pytest.approx([0, 0, 0, 0, 0, -0.1])


@pytest.mark.parametrize(
    "change",
    [
        {"max_step_deg": 0.51},
        {"speed_deg_s": 1.1},
        {"acceleration_deg_s2": 2.1},
        {"session_travel_deg": 1.1},
        {"feedback_max_age_s": 0.21},
        {"position_tolerance_deg": 0.01},
        {"commissioning_id": ""},
        {"lower_deg": (2.0,) * 6},
        {"upper_deg": (0.0,) * 5},
        {"stop_deceleration_deg_s2": float("nan")},
        {"speed_deg_s": True},
    ],
)
def test_commissioning_parameters_cannot_exceed_software_caps(rig, change):
    with pytest.raises(ValidationError):
        JointStepLimits.model_validate({**rig[0].limits.model_dump(), **change})


@pytest.mark.parametrize(
    "change",
    [
        {"joint": 0},
        {"joint": 7},
        {"joint": True},
        {"joint": "1"},
        {"delta_deg": 0},
        {"delta_deg": 0.001},
        {"delta_deg": 1.0},
        {"delta_deg": float("nan")},
        {"delta_deg": float("inf")},
        {"delta_deg": True},
        {"linear": True},
    ],
)
def test_invalid_request_rejected(change):
    with pytest.raises(ValidationError):
        JointStep.model_validate(
            {"request_id": uuid4(), "joint": 1, "delta_deg": 0.1, **change}
        )


@pytest.mark.parametrize(
    "failure",
    [
        "no_claim",
        "expired_claim",
        "advisory_claim",
        "unauthorized",
        "disconnected",
        "moving",
        "unsafe",
        "stale",
        "future",
        "malformed",
        "bounds",
        "step_limit",
        "position_changed",
        "clock_restart",
    ],
)
def test_preflight_failures_never_move_or_stop(rig, failure):
    executor, control, claims, token, state, clock = rig
    request = step()
    if failure == "no_claim":
        claims.release(token)
    elif failure == "expired_claim":
        clock.sleep(31)
    elif failure == "advisory_claim":
        claims.disable_enforcement()
    elif failure == "unauthorized":
        state["authorized"] = False
    elif failure == "disconnected":
        control.connected = False
    elif failure == "step_limit":
        request = JointStep(request_id=uuid4(), joint=1, delta_deg=0.2)
    else:

        def mutate(value):
            if failure == "moving":
                value["velocities_deg_s"] = (0.1,) * 6
            if failure == "unsafe":
                value["safety_mode"] = "PROTECTIVE_STOP"
            if failure == "stale":
                value["received_monotonic_s"] -= 1
            if failure == "future":
                value["received_monotonic_s"] += 1
            if failure == "malformed":
                value["joints_deg"] = (0,) * 5
            if failure == "bounds":
                value["joints_deg"] = (2,) * 6
            if failure == "position_changed" and state["reads"] > 1:
                value["joints_deg"] = (0.1,) * 6
            if failure == "clock_restart":
                value["controller_timestamp_s"] = 1.0
            return value

        state["mutate"] = mutate
    with pytest.raises((JointStepRefused, ValidationError)):
        execute(rig, request)
    assert control.moves == control.stops == []


def test_target_envelope_is_checked_without_clamping(rig):
    rig[4]["position"] = (0, 0.95, 0, 0, 0, 0)
    with pytest.raises(JointStepRefused, match="Target exceeds"):
        execute(rig)
    assert rig[1].moves == []


def test_duplicate_and_cumulative_session_limit(rig):
    request = step()
    execute(rig, request)
    with pytest.raises(JointStepRefused, match="Duplicate"):
        execute(rig, request)
    for _ in range(4):
        execute(rig)
    with pytest.raises(JointStepRefused, match="travel budget"):
        execute(rig)
    assert len(rig[1].moves) == 5


@pytest.mark.parametrize(
    "failure",
    [
        "lost_claim",
        "lost_auth",
        "disconnected",
        "frozen",
        "unsafe",
        "overshoot",
        "other_joint",
        "timeout",
        "cancel",
    ],
)
def test_fault_after_dispatch_stops_and_latches_without_retry(rig, failure):
    executor, control, claims, token, state, clock = rig

    def mutate(value):
        if control.moves:
            if failure == "lost_claim":
                claims.release(token)
            if failure == "lost_auth":
                state["authorized"] = False
            if failure == "disconnected":
                control.connected = False
            if failure == "frozen":
                value["controller_timestamp_s"] = 1.04
            if failure == "unsafe":
                value["safety_mode"] = "PROTECTIVE_STOP"
            if failure == "overshoot":
                value["joints_deg"] = (0, 0.2, 0, 0, 0, 0)
            if failure == "other_joint":
                value["joints_deg"] = (0.1, 0.1, 0, 0, 0, 0)
            if failure == "timeout":
                value["joints_deg"] = (0,) * 6
            if failure == "cancel":
                executor.request_stop()
        return value

    state["mutate"] = mutate
    with pytest.raises(JointStepFailed) as raised:
        execute(rig)
    assert raised.value.stop_attempted is True
    assert raised.value.stop_confirmed is False
    assert control.stops == [math.radians(10)]
    with pytest.raises(JointStepRefused):
        execute(rig)
    assert len(control.moves) == 1


@pytest.mark.parametrize("response", ["false", "exception"])
def test_dispatch_failure_is_ambiguous_and_never_replayed(rig, response):
    control = rig[1]
    if response == "false":
        control.accepted = False
    else:
        control.move_error = ConnectionError("reply lost after send")
    control.stop_error = ConnectionError("stop reply lost")
    with pytest.raises(JointStepFailed) as raised:
        execute(rig)
    assert raised.value.stop_error == "stop reply lost"
    assert raised.value.stop_confirmed is False
    assert len(control.moves) == len(control.stops) == 1
    with pytest.raises(JointStepRefused):
        execute(rig)


def test_authorization_delay_cannot_use_stale_preflight(rig):
    executor, control, _, _, _, clock = rig

    def authorize(_request, target, _commissioning):
        if target is not None:
            clock.sleep(1)
        return True

    executor.authorize = authorize
    with pytest.raises(JointStepRefused, match="stale before dispatch"):
        execute(rig)
    assert not control.moves and not control.stops


def test_stop_before_dispatch_does_not_stop_another_robot_operation(rig):
    rig[0].request_stop()
    with pytest.raises(JointStepRefused):
        execute(rig)
    assert not rig[1].moves and not rig[1].stops


@pytest.mark.parametrize("failure", ["claim", "stop", "advisory"])
def test_authorization_cannot_race_claim_loss_or_stop_before_dispatch(rig, failure):
    executor, control, claims, token, _, _ = rig

    def authorize(_request, target, _commissioning):
        if target is not None:
            if failure == "claim":
                claims.release(token)
            elif failure == "stop":
                executor.request_stop()
            else:
                claims.disable_enforcement()
        return True

    executor.authorize = authorize
    with pytest.raises(JointStepRefused):
        execute(rig)
    assert not control.moves and not control.stops


def test_concurrent_request_is_refused_not_queued(rig):
    executor, control, _, _, _, _ = rig
    entered, release = threading.Event(), threading.Event()
    errors = []
    original = executor.read_feedback

    def blocked():
        entered.set()
        assert release.wait(2)
        return original()

    executor.read_feedback = blocked

    def worker():
        try:
            execute(rig)
        except Exception as exc:
            errors.append(exc)

    thread = threading.Thread(target=worker)
    thread.start()
    try:
        assert entered.wait(2)
        with pytest.raises(JointStepRefused, match="already active"):
            execute(rig)
    finally:
        release.set()
        thread.join(3)
    assert not thread.is_alive() and not errors
    assert len(control.moves) == 1


def test_no_live_control_routes_or_vendor_imports_are_added():
    import subprocess
    import sys

    script = """
import sys
from robot_motion.drivers.joint_step import JointStepExecutor
from robot_motion.app import create_app
assert 'rtde_control' not in sys.modules
assert 'rtde_receive' not in sys.modules
assert not any(p.startswith('/control') for p in create_app().openapi()['paths'])
"""
    result = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, timeout=15
    )
    assert result.returncode == 0, result.stderr
