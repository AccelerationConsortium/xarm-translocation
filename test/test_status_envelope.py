"""Tests for the STATUS_SPEC v1.0 endpoints (``GET /``, ``/health``, ``/status``).

These tests validate that the FastAPI service speaks the AC Organic
``EquipmentStatus`` envelope and that ``GET /status`` is side-effect-free
across the controller states the dashboard cares about.

The plan that introduced these tests lives at
``.cursor/plans/xarm-status-spec-migration_*.plan.md`` in the workspace root.
"""

from __future__ import annotations

import os
import sys
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
    mc.profile_name = 'real_hw'
    mc.xarm_config = {'port': 18333}
    mc.current_gripper_config = {}

    for key, value in overrides.items():
        setattr(mc, key, value)
    return mc


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


def test_status_returns_ready_when_alive_and_arm_enabled(client_with_controller):
    controller = _fake_controller()
    test_client = client_with_controller(controller)

    response = test_client.get('/status')
    assert response.status_code == 200
    parsed = EquipmentStatus(**response.json())
    assert parsed.equipment_status == 'ready'
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
