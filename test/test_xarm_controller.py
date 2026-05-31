"""Tests for XArmController class.

Hardware paths are exercised against a mocked ``XArmAPI`` via the
``initialized_controller`` / ``mock_xarm_api`` fixtures in ``conftest.py``.
"""

import sys
import os
from unittest.mock import MagicMock

import pytest

# Ensure src is in the python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

from src.core.xarm_controller import XArmController, ComponentState
from src.core.xarm_utils import SafetyLevel


class TestXArmControllerInitialization:
    """Test XArmController initialization and configuration."""

    def test_hardware_mode_creation(self, mock_config_files, mock_xarm_api, monkeypatch):
        """Test creating controller against a mocked XArmAPI."""
        monkeypatch.setattr('src.core.xarm_controller.XArmAPI', lambda *a, **k: mock_xarm_api)
        controller = XArmController(
            profile_name='test_profile',
            auto_enable=False,
            gripper_type='bio',
            enable_track=True,
        )
        assert controller.arm is not None
        assert controller.gripper_type == 'bio'
        assert controller.enable_track is True

    def test_config_loading(self, mock_config_files, mock_xarm_api, monkeypatch):
        """Test configuration loading."""
        monkeypatch.setattr('src.core.xarm_controller.XArmAPI', lambda *a, **k: mock_xarm_api)
        controller = XArmController(profile_name='test_profile', auto_enable=False)
        assert 'host' in controller.xarm_config
        assert 'GRIPPER_SPEED' in controller.gripper_config
        assert 'Speed' in controller.track_config
        assert 'positions' in controller.position_config

    def test_safety_level_configuration(self, mock_config_files, mock_xarm_api, monkeypatch):
        """Test safety level configuration."""
        monkeypatch.setattr('src.core.xarm_controller.XArmAPI', lambda *a, **k: mock_xarm_api)
        controller = XArmController(
            profile_name='test_profile',
            auto_enable=False,
            safety_level=SafetyLevel.HIGH,
        )
        assert controller.safety_level == SafetyLevel.HIGH

    def test_model_detection(self, mock_config_files, mock_xarm_api, monkeypatch):
        """Test model detection and joint count."""
        monkeypatch.setattr('src.core.xarm_controller.XArmAPI', lambda *a, **k: mock_xarm_api)
        controller = XArmController(profile_name='test_profile', auto_enable=False)
        assert controller.model in [5, 6, 7]
        assert controller.num_joints > 0


class TestHardwareMode:
    """Test hardware mode functionality with mocks."""

    def test_hardware_initialization_success(self, initialized_controller):
        """Test successful hardware initialization."""
        assert initialized_controller.states['connection'] == ComponentState.ENABLED
        assert initialized_controller.is_alive is True

    def test_hardware_initialization_failure(self, mock_config_files, monkeypatch):
        """If the SDK's connect() fails, initialize() returns False."""
        mock_api = MagicMock()
        mock_api.connect.return_value = 1  # SDK failure code
        monkeypatch.setattr('src.core.xarm_controller.XArmAPI', lambda *a, **k: mock_api)

        controller = XArmController(profile_name='test_profile', auto_enable=False)
        assert controller.initialize() is False

    def test_hardware_disconnect(self, initialized_controller):
        """Test hardware disconnection."""
        initialized_controller.disconnect()
        assert initialized_controller.states['connection'] == ComponentState.DISABLED


class TestComponentManagement:
    """Test component enable/disable functionality."""

    def test_enable_gripper_component(self, initialized_controller):
        assert initialized_controller.enable_gripper_component() is True
        assert initialized_controller.states['gripper'] == ComponentState.ENABLED

    def test_enable_track_component(self, initialized_controller):
        assert initialized_controller.enable_track_component() is True
        assert initialized_controller.states['track'] == ComponentState.ENABLED

    def test_disable_gripper_component(self, initialized_controller):
        initialized_controller.enable_gripper_component()
        assert initialized_controller.disable_gripper_component() is True
        assert initialized_controller.states['gripper'] == ComponentState.DISABLED

    def test_disable_track_component(self, initialized_controller):
        initialized_controller.enable_track_component()
        assert initialized_controller.disable_track_component() is True
        assert initialized_controller.states['track'] == ComponentState.DISABLED

    def test_component_state_checking(self, initialized_controller):
        states = initialized_controller.get_component_states()
        assert isinstance(states, dict)
        assert 'connection' in states and 'arm' in states
        assert 'gripper' in states and 'track' in states


class TestMovementMethods:
    """Test movement methods with safety features."""

    def test_move_to_position_with_collision_detection(self, initialized_controller):
        assert initialized_controller.move_to_position(
            x=300, y=0, z=300,
            roll=180, pitch=0, yaw=0,
            check_collision=True,
        ) is True

    def test_move_to_named_location(self, initialized_controller):
        assert initialized_controller.move_to_named_location('home') is True
        assert initialized_controller.move_to_named_location('pickup') is True

    def test_move_relative(self, initialized_controller):
        assert initialized_controller.move_relative(dx=10) is True

    def test_move_single_joint(self, initialized_controller):
        assert initialized_controller.move_single_joint(1, 10) is True

    def test_go_home(self, initialized_controller):
        # go_home routes through the named 'robot_home' preset, never factory home.
        assert initialized_controller.go_home() is True
        initialized_controller.arm.move_gohome.assert_not_called()

    def test_go_home_without_robot_home_raises(self, initialized_controller):
        # No 'robot_home' defined → refuse rather than fall back to factory home.
        initialized_controller.position_config['positions'].pop('robot_home', None)
        with pytest.raises(ValueError, match="robot_home"):
            initialized_controller.go_home()
        initialized_controller.arm.move_gohome.assert_not_called()

    def test_velocity_control(self, initialized_controller):
        assert initialized_controller.set_cartesian_velocity(10, 0, 0, 0, 0, 0) is True
        assert initialized_controller.set_joint_velocity([10] * initialized_controller.num_joints) is True

    def test_stop_motion(self, initialized_controller):
        assert initialized_controller.stop_motion() is True

    def test_set_manual_mode_enable(self, initialized_controller, mock_xarm_api):
        """Enabling manual mode = motion_enable + set_mode(2) + set_state(0)."""
        mock_xarm_api.motion_enable.reset_mock()
        mock_xarm_api.set_mode.reset_mock()
        mock_xarm_api.set_state.reset_mock()

        assert initialized_controller.set_manual_mode(True) is True
        mock_xarm_api.motion_enable.assert_called_with(enable=True)
        mock_xarm_api.set_mode.assert_called_with(2)
        mock_xarm_api.set_state.assert_called_with(0)

    def test_set_manual_mode_disable(self, initialized_controller, mock_xarm_api):
        """Disabling manual mode returns to position control: set_mode(0)."""
        mock_xarm_api.set_mode.reset_mock()
        mock_xarm_api.set_state.reset_mock()

        assert initialized_controller.set_manual_mode(False) is True
        mock_xarm_api.set_mode.assert_called_with(0)
        mock_xarm_api.set_state.assert_called_with(0)


class TestUniversalGripperControl:
    """Test universal gripper control methods."""

    def test_bio_gripper_control(self, initialized_controller):
        initialized_controller.enable_gripper_component()
        assert initialized_controller.open_gripper() is True
        assert initialized_controller.close_gripper() is True

    def test_biogripper_gen2_stroke_and_force_control(self, mock_config_files, mock_xarm_api, monkeypatch):
        """BioGripper Gen2 should expose stroke distance and force control."""
        monkeypatch.setattr('src.core.xarm_controller.XArmAPI', lambda *args, **kwargs: mock_xarm_api)
        controller = XArmController(
            profile_name='test_profile',
            auto_enable=False,
            gripper_type='bio_gen2',
        )
        assert controller.initialize() is True
        assert controller.enable_gripper_component() is True

        # 110 is within the official Gen2 range 71-150
        assert controller.move_gripper_to_stroke(110, speed=1500, force=80) is True
        mock_xarm_api.set_bio_gripper_g2_position.assert_called_with(
            110, speed=1500, force=80, wait=True, timeout=5
        )

        # close_position = 71 (fully closed); force float is coerced to int; speed from config default
        assert controller.close_gripper(force=75.0) is True
        mock_xarm_api.set_bio_gripper_g2_position.assert_called_with(
            71, speed=1000, force=75, wait=True, timeout=5
        )

        assert controller.set_gripper_force(60.0) is True
        mock_xarm_api.set_bio_gripper_force.assert_called_with(60)

    def test_no_gripper_configured(self, mock_config_files, mock_xarm_api, monkeypatch):
        monkeypatch.setattr('src.core.xarm_controller.XArmAPI', lambda *a, **k: mock_xarm_api)
        controller = XArmController(profile_name='test_profile', gripper_type='none', auto_enable=False)
        controller.initialize()
        assert controller.has_gripper() is False
        assert controller.open_gripper() is False


class TestLinearTrackControl:
    """Test linear track control methods."""

    def test_track_movement_with_validation(self, initialized_controller):
        initialized_controller.enable_track_component()
        assert initialized_controller.move_track_to_position(100) is True

    def test_track_speed_setting(self, initialized_controller):
        initialized_controller.enable_track_component()
        assert initialized_controller.set_track_speed(100) is True

    def test_track_reset(self, initialized_controller):
        initialized_controller.enable_track_component()
        assert initialized_controller.reset_track() is True

    def test_track_position_retrieval(self, initialized_controller):
        initialized_controller.enable_track_component()
        pos = initialized_controller.get_track_position()
        assert isinstance(pos, (int, float))

    def test_track_not_enabled(self, mock_config_files, mock_xarm_api, monkeypatch):
        monkeypatch.setattr('src.core.xarm_controller.XArmAPI', lambda *a, **k: mock_xarm_api)
        controller = XArmController(profile_name='test_profile', enable_track=False, auto_enable=False)
        controller.initialize()
        assert controller.move_track_to_position(100) is False

    def test_track_disabled(self, initialized_controller):
        initialized_controller.disable_track_component()
        assert initialized_controller.move_track_to_position(100) is False


class TestStateManagement:
    """Test state and error management."""

    def test_get_system_status(self, initialized_controller):
        status = initialized_controller.get_system_status()
        assert isinstance(status, dict)
        assert 'connection' in status
        assert 'arm' in status

    def test_error_tracking(self, initialized_controller):
        initialized_controller.arm.error_code = 1
        initialized_controller._error_warn_callback({'error_code': 1})
        assert initialized_controller.last_error_code == 1

    def test_warning_tracking(self, initialized_controller):
        initialized_controller.arm.warn_code = 1
        initialized_controller._error_warn_callback({'warn_code': 1})
        assert initialized_controller.last_warn_code == 1

    def test_is_alive_property(self, initialized_controller):
        assert initialized_controller.is_alive is True
        initialized_controller._state_changed_callback({'state': 4})
        assert initialized_controller.is_alive is False

    def test_get_error_history(self, initialized_controller):
        initialized_controller._error_warn_callback({'error_code': 10})
        history = initialized_controller.get_error_history()
        assert len(history) > 0
        assert history[0]['error_code'] == 10

    def test_clear_errors(self, initialized_controller):
        initialized_controller.arm.error_code = 1
        initialized_controller.clear_errors()
        assert initialized_controller.last_error_code == 0

    def test_clear_errors_after_stop_reenables_arm(
        self, mock_config_files, mock_xarm_api, monkeypatch
    ):
        """After an emergency_stop, clear_errors must re-assert motion_enable /
        set_mode(0) / set_state(0); otherwise the SDK stays in state 4 and
        refuses subsequent move commands."""
        monkeypatch.setattr(
            'src.core.xarm_controller.XArmAPI', lambda *a, **kw: mock_xarm_api
        )
        controller = XArmController(
            profile_name='test_profile',
            gripper_type='bio',
            enable_track=True,
            auto_enable=True,
        )
        mock_xarm_api.get_servo_angle.return_value = (0, [0] * controller.num_joints)
        assert controller.initialize() is True

        mock_xarm_api.motion_enable.reset_mock()
        mock_xarm_api.set_mode.reset_mock()
        mock_xarm_api.set_state.reset_mock()

        controller._state_changed_callback({'state': 4})
        assert controller.states['arm'] == ComponentState.ERROR

        assert controller.clear_errors() is True
        assert controller.states['arm'] == ComponentState.ENABLED
        mock_xarm_api.motion_enable.assert_called_with(enable=True)
        mock_xarm_api.set_mode.assert_called_with(0)
        mock_xarm_api.set_state.assert_called_with(0)


class TestUtilityMethods:
    """Test utility methods."""

    def test_get_current_position(self, initialized_controller):
        pos = initialized_controller.get_current_position()
        assert isinstance(pos, list) and len(pos) == 6

    def test_get_current_joints(self, initialized_controller):
        joints = initialized_controller.get_current_joints()
        assert isinstance(joints, list) and len(joints) == initialized_controller.num_joints

    def test_get_named_locations(self, initialized_controller):
        locations = initialized_controller.get_named_locations()
        assert 'home' in locations
        assert 'pickup' in locations

    def test_get_system_info(self, initialized_controller):
        info = initialized_controller.get_system_info()
        assert info['model'] == 6
        assert info['has_gripper'] is True
        assert info['has_track'] is True

    def test_check_code_success(self, initialized_controller):
        assert initialized_controller.check_code(0, 'test_op') is True

    def test_check_code_failure(self, initialized_controller):
        assert initialized_controller.check_code(1, 'test_op') is False


class TestSafetyAndValidation:
    """Test safety and validation systems."""

    def test_safety_level_enforcement(self, mock_config_files, mock_xarm_api, monkeypatch):
        """High safety should cap speeds."""
        monkeypatch.setattr('src.core.xarm_controller.XArmAPI', lambda *a, **k: mock_xarm_api)
        controller = XArmController(
            profile_name='test_profile',
            auto_enable=False,
            safety_level=SafetyLevel.HIGH,
        )
        assert controller.tcp_speed < controller.safety_config.get('max_tcp_speed', 1000)

    def test_joint_limit_validation(self, initialized_controller):
        assert initialized_controller.move_joints([500] * 6) is False

    def test_workspace_boundary_validation(self, initialized_controller):
        assert initialized_controller.move_to_position(x=9000, y=0, z=0) is False


class TestErrorHandling:
    """Test error handling scenarios."""

    def test_missing_config_files(self, mock_xarm_api, monkeypatch):
        """Controller falls back to defaults when config files are missing."""
        monkeypatch.setattr('src.core.xarm_controller.load_config', MagicMock(side_effect=FileNotFoundError))
        monkeypatch.setattr('src.core.xarm_controller.XArmAPI', lambda *a, **k: mock_xarm_api)
        controller = XArmController(profile_name='non_existent_profile', auto_enable=False)
        assert controller.initialize() is True

    def test_invalid_gripper_type(self, mock_config_files):
        with pytest.raises(ValueError, match="Invalid gripper type"):
            XArmController(profile_name='test_profile', gripper_type='invalid_gripper')

    def test_arm_none_operations(self, mock_config_files, mock_xarm_api, monkeypatch):
        """Operations fail gracefully if arm is None."""
        monkeypatch.setattr('src.core.xarm_controller.XArmAPI', lambda *a, **k: mock_xarm_api)
        controller = XArmController(profile_name='test_profile', auto_enable=False)
        controller.initialize()
        controller.arm = None
        assert controller.move_to_position(x=300, y=0, z=300) is False

    def test_position_updates_with_none_arm(self, mock_config_files, mock_xarm_api, monkeypatch):
        """Position updates don't crash if arm is None."""
        monkeypatch.setattr('src.core.xarm_controller.XArmAPI', lambda *a, **k: mock_xarm_api)
        controller = XArmController(profile_name='test_profile', auto_enable=False)
        controller.arm = None
        controller._update_positions()  # Should not raise


class TestConfigurationManagement:
    """Test advanced configuration management."""

    def test_host_priority_resolution(self, mock_config_files, mock_xarm_api, monkeypatch):
        """Direct host param takes priority over the profile."""
        monkeypatch.setattr('src.core.xarm_controller.XArmAPI', lambda *a, **k: mock_xarm_api)
        controller = XArmController(profile_name='test_profile', host='192.168.1.100', auto_enable=False)
        assert controller.host == '192.168.1.100'

    def test_model_configuration(self, mock_config_files, mock_xarm_api, monkeypatch):
        """Model is read from the profile."""
        monkeypatch.setattr('src.core.xarm_controller.XArmAPI', lambda *a, **k: mock_xarm_api)
        controller = XArmController(profile_name='test_profile', auto_enable=False)
        assert controller.model == 6

    def test_safety_config_loading(self, initialized_controller):
        assert 'workspace_limits' in initialized_controller.safety_config
