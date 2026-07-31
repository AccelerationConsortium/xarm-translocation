"""Tests for the STATUS_SPEC v1.2 endpoints (``GET /``, ``/health``, ``/status``).

These tests validate that the FastAPI service speaks the AC Organic
``EquipmentStatus`` envelope and that ``GET /status`` is side-effect-free
across the controller states the dashboard cares about.

The plan that introduced these tests lives at
``.cursor/plans/xarm-status-spec-migration_*.plan.md`` in the workspace root.
"""

from __future__ import annotations

import os
import sys
import threading
from copy import deepcopy
from unittest.mock import MagicMock

import pytest
from fastapi.testclient import TestClient

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core.models import (  # noqa: E402
    PROTOCOL_VERSION,
    EquipmentStatus,
    HealthResponse,
    ProbeResponse,
)
from src.core.status_builder import (  # noqa: E402
    EQUIPMENT_ID,
    EQUIPMENT_NAME,
    build_status,
)
from src.core.xarm_api_server import app  # noqa: E402
from src.core.xarm_controller import ComponentState  # noqa: E402


# ---------------------------------------------------------------------------
# Shared fixtures
# ---------------------------------------------------------------------------


def _fake_controller(**overrides):
    """Return a MagicMock that mimics the surface ``status_builder`` reads.

    Defaults represent a fully-initialized hardware-mode controller (arm
    ENABLED, no errors, no motion in progress). Override fields per test.
    """
    mc = MagicMock()
    mc.alive = True
    # Real hardware unless a test opts in — a bare MagicMock attribute is
    # truthy, which would flip every envelope to dry_run.
    mc.is_simulated = False
    mc._motion_in_progress = False
    mc.last_error_code = 0
    mc.last_error = None
    mc.states = {
        'connection': ComponentState.ENABLED,
        'arm': ComponentState.ENABLED,
        'gripper': ComponentState.ENABLED,
        'track': ComponentState.ENABLED,
        'force_torque': ComponentState.DISABLED,
    }
    mc.has_gripper.return_value = True
    mc.has_track.return_value = True
    mc.has_force_torque_sensor.return_value = False
    mc.last_position = [300, 0, 300, 180, 0, 0]
    mc.last_joints = [0, 0, 0, 0, 0]
    mc.last_track_position = 0.0
    mc.last_force_torque = [0.0] * 6
    mc.force_torque_calibrated = False
    mc.gripper_type = 'bio'
    mc.model = 5
    mc.model_name = 'xArm5'
    mc.num_joints = 5
    mc.tcp_speed = 100
    mc.angle_speed = 20
    mc.host = '127.0.0.1'
    mc.profile_name = 'robot'
    mc.xarm_config = {'port': 18333}
    mc.current_gripper_config = {}

    for key, value in overrides.items():
        setattr(mc, key, value)
    return mc


def _bare_motion_controller():
    """A real XArmController with only its motion bookkeeping initialized.

    ``__new__`` skips ``__init__`` (which would talk to hardware); the
    motion depth / lock / latch are the only state enter_motion() and
    exit_motion() touch.
    """
    from src.core.xarm_controller import XArmController

    controller = XArmController.__new__(XArmController)
    controller._motion_depth = 0
    controller._motion_lock = threading.Lock()
    controller._activity_since = None
    return controller


@pytest.fixture
def client(monkeypatch):
    """TestClient with no controller installed (the default no-op state)."""
    monkeypatch.setattr('src.core.xarm_api_server.controller', None)
    with TestClient(app) as test_client:
        yield test_client


@pytest.fixture
def client_with_controller(monkeypatch):
    """Factory: install a fake controller and return the TestClient."""
    def _make(controller):
        monkeypatch.setattr('src.core.xarm_api_server.controller', controller)
        return TestClient(app)
    return _make


# ---------------------------------------------------------------------------
# GET /  --  Probe
# ---------------------------------------------------------------------------


def test_probe_returns_protocol_version(client):
    response = client.get('/')
    assert response.status_code == 200
    body = response.json()
    parsed = ProbeResponse(**body)
    assert parsed.equipment_id == EQUIPMENT_ID
    assert parsed.equipment_name == EQUIPMENT_NAME
    assert parsed.protocol_version == PROTOCOL_VERSION


# ---------------------------------------------------------------------------
# GET /health
# ---------------------------------------------------------------------------


def test_health_returns_healthy(client):
    response = client.get('/health')
    assert response.status_code == 200
    parsed = HealthResponse(**response.json())
    assert parsed.status == 'healthy'


# ---------------------------------------------------------------------------
# GET /status  --  state mapping
# ---------------------------------------------------------------------------


def test_status_returns_requires_init_when_no_controller(client):
    response = client.get('/status')
    assert response.status_code == 200
    parsed = EquipmentStatus(**response.json())
    assert parsed.equipment_status == 'requires_init'
    assert parsed.required_actions == ['connect']
    assert parsed.equipment_id == EQUIPMENT_ID
    assert parsed.equipment_kind == 'robot_arm'
    assert parsed.protocol_version == PROTOCOL_VERSION
    # Populated from package metadata (uv install) or cli.__version__
    # (from-source runs like this test); null was the pre-fix regression.
    assert parsed.equipment_version is not None


def test_status_returns_error_when_controller_has_error_code(client_with_controller):
    controller = _fake_controller(last_error_code=31)
    test_client = client_with_controller(controller)

    response = test_client.get('/status')
    assert response.status_code == 200
    parsed = EquipmentStatus(**response.json())
    assert parsed.equipment_status == 'error'
    assert parsed.last_error is not None
    assert parsed.last_error.code == '31'
    assert parsed.required_actions == ['clear_errors']


def test_status_returns_busy_when_motion_in_progress(client_with_controller):
    controller = _fake_controller(_motion_in_progress=True)
    test_client = client_with_controller(controller)

    response = test_client.get('/status')
    assert response.status_code == 200
    parsed = EquipmentStatus(**response.json())
    assert parsed.equipment_status == 'busy'
    # v1.2 §2.3 invariant: busy is definitionally healthy + running.
    assert parsed.activity == 'running'


def test_status_returns_ready_when_alive_and_arm_enabled(client_with_controller):
    controller = _fake_controller()
    test_client = client_with_controller(controller)

    response = test_client.get('/status')
    assert response.status_code == 200
    parsed = EquipmentStatus(**response.json())
    assert parsed.equipment_status == 'ready'
    # v1.2 §2.3 invariant: ready ⇒ idle.
    assert parsed.activity == 'idle'
    assert parsed.components['arm'].connected is True
    assert parsed.components['arm'].state == 'enabled'
    # Track metric with unit
    assert parsed.metrics['track_position'].unit == 'mm'


def test_status_returns_requires_init_when_controller_present_but_disconnected(
    client_with_controller,
):
    controller = _fake_controller()
    controller.states['connection'] = ComponentState.DISABLED
    controller.states['arm'] = ComponentState.DISABLED
    test_client = client_with_controller(controller)

    response = test_client.get('/status')
    assert response.status_code == 200
    parsed = EquipmentStatus(**response.json())
    assert parsed.equipment_status == 'requires_init'
    assert parsed.required_actions == ['connect']


# ---------------------------------------------------------------------------
# GET /status  --  v1.2 activity axis (§2.3)
# ---------------------------------------------------------------------------


def test_activity_is_running_while_degraded(client_with_controller):
    """The motivating v1.2 case: health and activity are independent.

    A controller that is connected but not fully alive is ``degraded``
    (§2.2), and a move in flight is still a run. Neither fact suppresses
    the other, and the pair ``busy`` + ``degraded`` — which §2.3 forbids —
    is never emitted.
    """
    controller = _fake_controller(alive=False, _motion_in_progress=True)
    test_client = client_with_controller(controller)

    parsed = EquipmentStatus(**test_client.get('/status').json())
    assert parsed.equipment_status == 'degraded'
    assert parsed.activity == 'running'


def test_activity_is_idle_when_degraded_between_moves(client_with_controller):
    controller = _fake_controller(alive=False, _motion_in_progress=False)
    test_client = client_with_controller(controller)

    parsed = EquipmentStatus(**test_client.get('/status').json())
    assert parsed.equipment_status == 'degraded'
    assert parsed.activity == 'idle'


def test_activity_is_idle_when_disconnected(client_with_controller):
    """§2.3 invariant: requires_init ⇒ idle, even with a latched flag.

    A connection dropping mid-move can leave ``_motion_in_progress`` set.
    A move that can no longer be executing must not read as a run.
    """
    controller = _fake_controller(_motion_in_progress=True)
    controller.states['connection'] = ComponentState.DISABLED
    controller.states['arm'] = ComponentState.DISABLED
    test_client = client_with_controller(controller)

    parsed = EquipmentStatus(**test_client.get('/status').json())
    assert parsed.equipment_status == 'requires_init'
    assert parsed.activity == 'idle'
    assert parsed.activity_since is None


def test_no_controller_envelope_is_idle_not_unknown():
    """requires_init ⇒ idle for the no-controller envelope too."""
    envelope = build_status(None)
    assert envelope.equipment_status == 'requires_init'
    assert envelope.activity == 'idle'
    assert envelope.activity_since is None


def test_activity_is_not_derived_from_equipment_status(client_with_controller):
    """§2.3: activity must come from observed hardware state.

    Deriving it from ``equipment_status`` would make it a pure function of
    the state word. Two controllers sharing a state (``degraded``) but
    differing in observed motion must report different activity — that is
    exactly the information a derived field could not carry.
    """
    moving = _fake_controller(alive=False, _motion_in_progress=True)
    parked = _fake_controller(alive=False, _motion_in_progress=False)

    moving_envelope = build_status(moving)
    parked_envelope = build_status(parked)

    assert moving_envelope.equipment_status == parked_envelope.equipment_status
    assert moving_envelope.activity != parked_envelope.activity


def test_move_targets_withheld_while_a_motion_is_in_flight(client_with_controller):
    """§2.3: no action that would start a second concurrent run.

    ``stop`` stays listed so an abort is always reachable.
    """
    controller = _fake_controller(_motion_in_progress=True)
    controller.reachable_node_ids.return_value = ['deck_home']
    controller.graph_mode = MagicMock(value='strict')
    test_client = client_with_controller(controller)

    parsed = EquipmentStatus(**test_client.get('/status').json())
    assert parsed.equipment_status == 'busy'
    assert parsed.allowed_actions == ['stop']


def test_move_targets_listed_while_idle(client_with_controller):
    """The same controller, parked, does advertise its move targets —
    otherwise the test above would pass for the wrong reason."""
    controller = _fake_controller(_motion_in_progress=False)
    controller.reachable_node_ids.return_value = ['deck_home']
    controller.graph_mode = MagicMock(value='strict')
    test_client = client_with_controller(controller)

    parsed = EquipmentStatus(**test_client.get('/status').json())
    assert parsed.equipment_status == 'ready'
    assert 'move.deck_home' in parsed.allowed_actions


def test_no_controller_envelope_advertises_connect():
    """Both requires_init paths agree: /connect is honored, so list it."""
    envelope = build_status(None)
    assert envelope.allowed_actions == ['connect']
    assert envelope.required_actions == ['connect']


def test_activity_since_is_the_transition_instant_not_the_poll_time():
    """``activity_since`` must not advance when nothing changed.

    The pre-fix bug: it was ``datetime.now()`` on every call, so a reader
    could never recover an in-progress move's true elapsed duration.
    """
    controller = _bare_motion_controller()

    controller.enter_motion()
    started_at = controller._activity_since
    assert started_at is not None

    # A nested primitive inside the same run is not a new transition.
    controller.enter_motion()
    assert controller._activity_since == started_at

    # Two polls of an unchanged in-flight move report the same instant.
    first = build_status(_fake_controller(
        _motion_in_progress=True, _activity_since=started_at,
    ))
    second = build_status(_fake_controller(
        _motion_in_progress=True, _activity_since=started_at,
    ))
    assert first.activity_since == second.activity_since == started_at

    # The inner primitive finishing does not end the run...
    controller.exit_motion()
    assert controller._motion_in_progress is True
    assert controller._activity_since == started_at

    # ...the outer release does, and re-latches.
    controller.exit_motion()
    assert controller._motion_in_progress is False
    assert controller._activity_since > started_at


def test_exit_motion_is_clamped_at_zero():
    """An unbalanced release must not wedge the device.

    A negative depth would read as "not moving" but leave the next
    enter/exit pair unbalanced, so the arm could end up permanently
    refusing moves. Clamping keeps the guard fail-open.
    """
    controller = _bare_motion_controller()

    controller.exit_motion()
    controller.exit_motion()
    assert controller._motion_depth == 0
    assert controller._motion_in_progress is False

    controller.enter_motion()
    assert controller._motion_in_progress is True
    controller.exit_motion()
    assert controller._motion_in_progress is False


# ---------------------------------------------------------------------------
# Simulation self-identification (docker profile -> dry_run)
# ---------------------------------------------------------------------------


def test_sim_healthy_idle_reports_dry_run():
    """A simulator session must never read as the real arm being ready."""
    controller = _fake_controller(is_simulated=True)
    envelope = build_status(controller)
    assert envelope.equipment_status == 'dry_run'
    assert envelope.activity == 'idle'
    assert envelope.message.startswith('[SIMULATION]')
    assert envelope.details['simulated'] is True
    # The sim honors the same actions the real arm would.
    assert 'stop' in envelope.allowed_actions


def test_sim_moving_reports_dry_run_running():
    """dry_run + running is legal (§2.3 allows any activity for dry_run),
    and the concurrent-move withholding applies in sim exactly as on
    hardware — sim sessions are for exercising the real rules."""
    controller = _fake_controller(is_simulated=True, _motion_in_progress=True)
    controller.reachable_node_ids.return_value = ['deck_home']
    controller.graph_mode = MagicMock(value='strict')
    envelope = build_status(controller)
    assert envelope.equipment_status == 'dry_run'
    assert envelope.activity == 'running'
    assert envelope.allowed_actions == ['stop']


def test_sim_idle_advertises_move_targets():
    controller = _fake_controller(is_simulated=True)
    controller.reachable_node_ids.return_value = ['deck_home']
    controller.graph_mode = MagicMock(value='strict')
    envelope = build_status(controller)
    assert 'move.deck_home' in envelope.allowed_actions


def test_sim_fault_stays_error():
    """Fault states keep their honest value in sim — a simulator that
    cannot express failure is useless for testing recovery paths."""
    controller = _fake_controller(is_simulated=True, last_error_code=31)
    envelope = build_status(controller)
    assert envelope.equipment_status == 'error'
    assert envelope.message.startswith('[SIMULATION]')
    assert envelope.details['simulated'] is True


def test_real_hardware_envelope_carries_no_simulated_flag():
    envelope = build_status(_fake_controller())
    assert 'simulated' not in envelope.details
    assert not envelope.message.startswith('[SIMULATION]')


def test_sim_telemetry_reports_alive():
    """The panel's controls key on is_alive; a sim session must keep them
    live or the panel would be unusable against the simulator."""
    from src.core.status_builder import build_telemetry

    telemetry = build_telemetry(_fake_controller(is_simulated=True))
    assert telemetry['equipment_status'] == 'dry_run'
    assert telemetry['is_alive'] is True


def test_events_exporter_suppressed_when_simulated():
    """Sim telemetry must never land in the lab history DB as the real
    device, even with XARM_INGEST_URL configured."""
    from src.core.xarm_controller import XArmController

    controller = XArmController.__new__(XArmController)
    controller.arm = None
    controller.last_error_code = 0
    controller.last_warn_code = 0
    controller._current_graph_node = lambda: None
    controller.events_exporter = MagicMock(enabled=True)

    controller.profile_name = 'docker'
    controller._emit_event('startup')
    controller.events_exporter.emit.assert_not_called()

    controller.profile_name = 'robot'
    controller._emit_event('startup')
    controller.events_exporter.emit.assert_called_once()


# ---------------------------------------------------------------------------
# Side-effect freedom
# ---------------------------------------------------------------------------


def test_status_does_not_mutate_controller_state(client_with_controller):
    controller = _fake_controller()
    states_before = deepcopy(controller.states)
    last_position_before = list(controller.last_position)
    last_joints_before = list(controller.last_joints)
    test_client = client_with_controller(controller)

    test_client.get('/status')
    test_client.get('/status')
    test_client.get('/status')

    assert controller.states == states_before
    assert controller.last_position == last_position_before
    assert controller.last_joints == last_joints_before
    # build_status must not touch SDK methods that round-trip to hardware.
    controller.connect.assert_not_called()
    controller.motion_enable.assert_not_called()
    controller.get_current_position.assert_not_called()
    controller.get_current_joints.assert_not_called()


# ---------------------------------------------------------------------------
# build_status() unit-level coverage (no FastAPI involved)
# ---------------------------------------------------------------------------


def test_build_status_with_none_returns_disconnected_envelope():
    envelope = build_status(None)
    assert envelope.equipment_status == 'requires_init'
    assert envelope.required_actions == ['connect']
    assert envelope.components['arm'].connected is False
    assert envelope.components['arm'].state == 'disabled'


def test_status_surfaces_gripper_slip_register(client_with_controller):
    """A BIO gripper 'object slipped' fault is surfaced on /status:
    in details.gripper AND reflected on the gripper component message."""
    controller = _fake_controller(
        gripper_type="bio_gen2",
        last_gripper_motion_state="fault",
        last_gripper_object_detected=False,
        last_gripper_position_actual=82.0,
        last_gripper_error_code=12,
        last_gripper_error_text="object slipped",
    )
    test_client = client_with_controller(controller)

    response = test_client.get("/status")
    assert response.status_code == 200
    parsed = EquipmentStatus(**response.json())

    grip = parsed.details["gripper"]
    assert grip["motion_state"] == "fault"
    assert grip["object_detected"] is False
    assert grip["position_mm"] == 82.0
    assert grip["error_code"] == 12
    assert grip["error_text"] == "object slipped"

    # Fault is visible on the component message without opening details.
    assert "object slipped" in parsed.components["gripper"].message
    assert "12" in parsed.components["gripper"].message


def test_status_surfaces_gripper_object_detected_no_fault(client_with_controller):
    """A clean pickup: object_detected true, no error code, no fault text."""
    controller = _fake_controller(
        gripper_type="bio_gen2",
        last_gripper_motion_state="object_detected",
        last_gripper_object_detected=True,
        last_gripper_position_actual=82.0,
        last_gripper_error_code=0,
        last_gripper_error_text=None,
    )
    test_client = client_with_controller(controller)

    parsed = EquipmentStatus(**test_client.get("/status").json())
    grip = parsed.details["gripper"]
    assert grip["object_detected"] is True
    assert grip["error_code"] == 0
    assert "error_text" not in grip
    # No fault → message is just the gripper type, no FAULT suffix.
    assert "FAULT" not in (parsed.components["gripper"].message or "")


def test_status_omits_gripper_block_when_uncached():
    """Before any gripper move (no cached register values), details.gripper
    is absent rather than emitting placeholder nulls."""

    class _Bare:
        """Minimal controller exposing only what build_status reads, with
        the gripper register cache left at its 'never moved' defaults."""

        alive = True
        _motion_in_progress = False
        last_error_code = 0
        last_error = None
        states = {
            "connection": ComponentState.ENABLED,
            "arm": ComponentState.ENABLED,
            "gripper": ComponentState.ENABLED,
            "track": ComponentState.DISABLED,
            "force_torque": ComponentState.DISABLED,
        }
        gripper_type = "bio_gen2"
        # 'never moved' cache state
        last_gripper_motion_state = None
        last_gripper_object_detected = None
        last_gripper_position_actual = None
        last_gripper_error_code = 0
        last_gripper_error_text = None

        def has_gripper(self):
            return True

        def has_track(self):
            return False

        def has_force_torque_sensor(self):
            return False

    envelope = build_status(_Bare())
    # error_code 0 is concrete, so a minimal block with error_code is allowed,
    # but no motion_state / object_detected / position keys should appear.
    grip = envelope.details.get("gripper")
    assert grip is None or grip == {"error_code": 0}
    assert envelope.components["gripper"].message == "bio_gen2"


def test_build_status_emits_force_torque_metric_when_calibrated():
    controller = _fake_controller()
    controller.has_force_torque_sensor.return_value = True
    controller.states['force_torque'] = ComponentState.ENABLED
    controller.last_force_torque = [3.0, 4.0, 0.0, 0.0, 0.0, 0.0]
    controller.force_torque_calibrated = True

    envelope = build_status(controller)
    metric = envelope.metrics.get('force_magnitude')
    assert metric is not None
    assert metric.unit == 'N'
    assert metric.value == pytest.approx(5.0)
