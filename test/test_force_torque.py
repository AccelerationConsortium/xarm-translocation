"""Tests for the 6-axis force torque sensor functionality.

Hardware paths are exercised against the mocked ``XArmAPI`` from
``conftest.py``.
"""

import os
import sys

import pytest

# Add src directory to Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core.xarm_controller import XArmController


@pytest.fixture
def ft_controller(mock_config_files, mock_xarm_api, monkeypatch):
    """An initialized controller with FT sensor wired through the mocked SDK."""
    monkeypatch.setattr('src.core.xarm_controller.XArmAPI', lambda *a, **k: mock_xarm_api)
    controller = XArmController(
        profile_name='test_profile',
        gripper_type='bio',
        enable_track=True,
        auto_enable=False,
    )
    controller.initialize()
    yield controller
    controller.disconnect()


class TestForceTorqueSensor:
    """Force torque sensor wiring against the mocked SDK."""

    def test_force_torque_sensor_availability(self, ft_controller):
        assert ft_controller.has_force_torque_sensor() is True

    def test_force_torque_sensor_enable_disable(self, ft_controller):
        assert ft_controller.enable_force_torque_sensor() is True
        assert ft_controller.is_component_enabled('force_torque') is True

        assert ft_controller.disable_force_torque_sensor() is True
        assert ft_controller.is_component_enabled('force_torque') is False

    def test_force_torque_sensor_calibration(self, ft_controller):
        ft_controller.enable_force_torque_sensor()
        assert ft_controller.calibrate_force_torque_sensor(samples=4, delay=0.0) is True
        assert ft_controller.force_torque_calibrated is True

    def test_force_torque_data_retrieval(self, ft_controller):
        ft_controller.enable_force_torque_sensor()
        ft_controller.calibrate_force_torque_sensor(samples=4, delay=0.0)

        data = ft_controller.get_force_torque_data()
        assert data is not None
        assert len(data) == 6
        assert all(isinstance(x, (int, float)) for x in data)

    def test_force_torque_magnitude_calculation(self, ft_controller):
        ft_controller.enable_force_torque_sensor()

        magnitude = ft_controller.get_force_torque_magnitude()
        assert magnitude is not None
        assert 'force_magnitude' in magnitude
        assert 'torque_magnitude' in magnitude
        assert 'total_magnitude' in magnitude
        assert all(isinstance(v, (int, float)) for v in magnitude.values())

    def test_force_torque_direction_detection(self, ft_controller):
        ft_controller.enable_force_torque_sensor()

        direction = ft_controller.get_force_torque_direction()
        assert direction is not None
        assert 'force_magnitude' in direction
        assert 'torque_magnitude' in direction

        if direction['force_direction'] is not None:
            assert len(direction['force_direction']) == 3
        if direction['torque_direction'] is not None:
            assert len(direction['torque_direction']) == 3

    def test_force_torque_safety_check(self, ft_controller):
        ft_controller.enable_force_torque_sensor()

        violation = ft_controller.check_force_torque_safety()
        assert isinstance(violation, bool)

    def test_force_torque_status(self, ft_controller):
        ft_controller.enable_force_torque_sensor()

        status = ft_controller.get_force_torque_status()
        assert status is not None
        assert 'enabled' in status
        assert 'calibrated' in status
        assert 'last_reading' in status
        assert 'magnitude' in status
        assert 'direction' in status


def test_force_torque_config_loading(mock_config_files, mock_xarm_api, monkeypatch):
    """Force torque configuration loads alongside the rest of the configs."""
    monkeypatch.setattr('src.core.xarm_controller.XArmAPI', lambda *a, **k: mock_xarm_api)
    controller = XArmController(profile_name='test_profile', auto_enable=False)

    assert hasattr(controller, 'force_torque_config')
    assert isinstance(controller.force_torque_config, dict)
