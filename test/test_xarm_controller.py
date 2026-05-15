"""
Tests for XArmController class.

This test suite covers the comprehensive features of the enhanced XArmController
including simulation mode, safety systems, performance monitoring, error recovery,
and the new streamlined API.
"""

import pytest
import time
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Ensure src is in the python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

from src.core.xarm_controller import XArmController, ComponentState
from src.core.xarm_utils import SafetyLevel


@pytest.fixture
def simulation_controller(mock_config_files):
    """Create a simulation mode controller for testing without hardware."""
    return XArmController(
        profile_name='test_profile',
        simulation_mode=True,
        auto_enable=False,
        gripper_type='bio',
        enable_track=True
    )


@pytest.fixture
def hardware_controller(mock_config_files):
    """Create a hardware mode controller for testing with mocks."""
    return XArmController(
        profile_name='test_profile',
        simulation_mode=False,
        auto_enable=False,
        gripper_type='bio',
        enable_track=True
    )


class TestXArmControllerInitialization:
    """Test XArmController initialization and configuration."""
    
    def test_simulation_mode_creation(self, simulation_controller):
        """Test creating controller in simulation mode."""
        controller = simulation_controller
        assert controller.simulation_mode is True
        assert controller.arm is not None
        assert controller.gripper_type == 'bio'
        assert controller.enable_track is True

    def test_hardware_mode_creation(self, hardware_controller):
        """Test creating controller in hardware mode."""
        controller = hardware_controller
        assert controller.simulation_mode is False
        assert controller.arm is not None
        assert controller.gripper_type == 'bio'
        assert controller.enable_track is True

    def test_auto_enable_functionality(self, mock_config_files):
        """Test auto_enable functionality."""
        with patch('src.core.xarm_controller.XArmController.initialize') as mock_init:
            XArmController(
                profile_name='test_profile',
                simulation_mode=True,
                auto_enable=True
        )
            mock_init.assert_called_once()
    
    def test_config_loading(self, simulation_controller):
        """Test configuration loading."""
        controller = simulation_controller
        assert 'host' in controller.xarm_config
        assert 'GRIPPER_SPEED' in controller.gripper_config
        assert 'Speed' in controller.track_config
        assert 'positions' in controller.position_config

    def test_safety_level_configuration(self, mock_config_files):
        """Test safety level configuration."""
        controller = XArmController(
            profile_name='test_profile',
            simulation_mode=True,
            safety_level=SafetyLevel.HIGH
        )
        assert controller.safety_level == SafetyLevel.HIGH

    def test_model_detection(self, simulation_controller):
        """Test model detection and joint count."""
        controller = simulation_controller
        assert controller.model in [5, 6, 7]
        assert controller.num_joints > 0


class TestSimulationMode:
    """Test simulation mode specific functionality."""

    def test_simulation_initialization(self, simulation_controller):
        """Test simulation mode initialization."""
        controller = simulation_controller
        success = controller.initialize()
        assert success is True
        assert controller.states['connection'] == ComponentState.ENABLED

    def test_simulation_arm_methods(self, simulation_controller):
        """Test simulation arm methods are available."""
        controller = simulation_controller
        controller.initialize()
        
        assert hasattr(controller.arm, 'connect')
        assert hasattr(controller.arm, 'get_position')
        assert hasattr(controller.arm, 'get_servo_angle')
        assert hasattr(controller.arm, 'set_servo_angle')

    def test_simulation_movement(self, simulation_controller):
        """Test movement in simulation mode."""
        controller = simulation_controller
        controller.initialize()
        
        success = controller.move_joints([0, 0, 0, 0, 0, 0], speed=50)
        assert success is True

    def test_simulation_gripper_control(self, simulation_controller):
        """Test gripper control in simulation mode."""
        controller = simulation_controller
        controller.initialize()
        controller.enable_gripper_component()
        
        assert controller.open_gripper() is True
        assert controller.close_gripper() is True

    def test_biogripper_gen2_stroke_and_force_control(self, mock_config_files, mock_xarm_api, monkeypatch):
        """BioGripper Gen2 should expose stroke distance and force control."""
        monkeypatch.setattr('src.core.xarm_controller.XArmAPI', lambda *args, **kwargs: mock_xarm_api)
        controller = XArmController(
            profile_name='test_profile',
            simulation_mode=False,
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

    def test_simulation_track_control(self, simulation_controller):
        """Test track control in simulation mode."""
        controller = simulation_controller
        controller.initialize()
        controller.enable_track_component()
        
        assert controller.move_track_to_position(100) is True


class TestHardwareMode:
    """Test hardware mode functionality with mocks."""

    def test_hardware_initialization_success(self, initialized_controller):
        """Test successful hardware initialization."""
        assert initialized_controller.states['connection'] == ComponentState.ENABLED
        assert initialized_controller.is_alive is True

    def test_hardware_initialization_failure(self, monkeypatch):
        """Test hardware initialization failure handling."""
        # This test ensures that if the SDK's connect() method fails,
        # our controller's initialize() correctly returns False.
        mock_api = MagicMock()
        mock_api.connect.return_value = 1  # SDK failure code
        monkeypatch.setattr('src.core.xarm_controller.XArmAPI', lambda *args, **kwargs: mock_api)
        
        # We need to instantiate the controller *after* the patch is applied.
        controller = XArmController(profile_name='test_profile', simulation_mode=False, auto_enable=False)
        assert controller.initialize() is False, "initialize() should return False on connection failure."

    def test_hardware_disconnect(self, initialized_controller):
        """Test hardware disconnection."""
        initialized_controller.disconnect()
        assert initialized_controller.states['connection'] == ComponentState.DISABLED


class TestComponentManagement:
    """Test component enable/disable functionality."""
    
    def test_enable_gripper_component(self, initialized_controller):
        """Test enabling gripper component."""
        assert initialized_controller.enable_gripper_component() is True
        assert initialized_controller.states['gripper'] == ComponentState.ENABLED
    
    def test_enable_track_component(self, initialized_controller):
        """Test enabling track component."""
        assert initialized_controller.enable_track_component() is True
        assert initialized_controller.states['track'] == ComponentState.ENABLED
    
    def test_disable_gripper_component(self, initialized_controller):
        """Test disabling gripper component."""
        initialized_controller.enable_gripper_component()
        assert initialized_controller.disable_gripper_component() is True
        assert initialized_controller.states['gripper'] == ComponentState.DISABLED
    
    def test_disable_track_component(self, initialized_controller):
        """Test disabling track component."""
        initialized_controller.enable_track_component()
        assert initialized_controller.disable_track_component() is True
        assert initialized_controller.states['track'] == ComponentState.DISABLED
    
    def test_component_state_checking(self, initialized_controller):
        """Test component state checking methods."""
        states = initialized_controller.get_component_states()
        assert isinstance(states, dict)
        assert 'connection' in states and 'arm' in states
        assert 'gripper' in states and 'track' in states


class TestEnhancedMovementMethods:
    """Test enhanced movement methods with safety features."""
    
    def test_move_to_position_with_collision_detection(self, initialized_controller):
        """Test move_to_position with collision detection."""
        success = initialized_controller.move_to_position(
            x=300, y=0, z=300, 
            roll=180, pitch=0, yaw=0,
            check_collision=True
        )
        assert success is True
    
    def test_move_joints_with_safety_validation(self, simulation_controller):
        """Test move_joints with safety validation."""
        simulation_controller.initialize()
        assert simulation_controller.move_joints([0] * 6, check_collision=True) is True
    
    def test_move_to_named_location(self, simulation_controller):
        """Test move_to_named_location method."""
        simulation_controller.initialize()
        assert simulation_controller.move_to_named_location('home') is True
        assert simulation_controller.move_to_named_location('pickup') is True
    
    def test_move_relative(self, initialized_controller):
        """Test relative movement."""
        assert initialized_controller.move_relative(dx=10) is True
    
    def test_move_single_joint(self, initialized_controller):
        """Test moving a single joint."""
        assert initialized_controller.move_single_joint(1, 10) is True
    
    def test_go_home_enhanced(self, initialized_controller):
        """Test go_home method."""
        assert initialized_controller.go_home() is True
    
    def test_velocity_control(self, initialized_controller):
        """Test velocity control methods."""
        assert initialized_controller.set_cartesian_velocity([10, 0, 0, 0, 0, 0]) is True
        assert initialized_controller.set_joint_velocity([10] * initialized_controller.num_joints) is True

    def test_stop_motion(self, initialized_controller):
        """Test emergency stop."""
        assert initialized_controller.stop_motion() is True


class TestUniversalGripperControl:
    """Test universal gripper control methods."""
    
    def test_bio_gripper_control(self, initialized_controller):
        """Test BIO gripper control."""
        initialized_controller.enable_gripper_component()
        assert initialized_controller.open_gripper() is True
        assert initialized_controller.close_gripper() is True

    def test_standard_gripper_control(self, mock_config_files):
        """Test standard gripper control."""
        controller = XArmController(profile_name='test_profile', gripper_type='standard', simulation_mode=True)
        controller.initialize()
        controller.enable_gripper_component()
        assert controller.open_gripper() is True
        assert controller.close_gripper() is True
    
    def test_robotiq_gripper_control(self, mock_config_files):
        """Test Robotiq gripper control."""
        controller = XArmController(profile_name='test_profile', gripper_type='robotiq', simulation_mode=True)
        controller.initialize()
        controller.enable_gripper_component()
        assert controller.open_gripper() is True
        assert controller.close_gripper() is True
    
    def test_no_gripper_configured(self, mock_config_files):
        """Test behavior with no gripper."""
        controller = XArmController(profile_name='test_profile', gripper_type='none', simulation_mode=True)
        controller.initialize()
        assert controller.has_gripper() is False
        assert controller.open_gripper() is False


class TestLinearTrackControl:
    """Test linear track control methods."""
    
    def test_track_movement_with_validation(self, initialized_controller):
        """Test track movement with validation."""
        initialized_controller.enable_track_component()
        assert initialized_controller.move_track_to_position(100) is True
    
    def test_track_speed_setting(self, initialized_controller):
        """Test setting track speed."""
        initialized_controller.enable_track_component()
        assert initialized_controller.set_track_speed(100) is True

    def test_track_reset(self, initialized_controller):
        """Test resetting the track position."""
        initialized_controller.enable_track_component()
        assert initialized_controller.reset_track() is True

    def test_track_position_retrieval(self, initialized_controller):
        """Test retrieving track position."""
        initialized_controller.enable_track_component()
        pos = initialized_controller.get_track_position()
        assert isinstance(pos, (int, float))
    
    def test_track_not_enabled(self, mock_config_files):
        """Test that track methods fail if track is not enabled."""
        controller = XArmController(profile_name='test_profile', enable_track=False, simulation_mode=True)
        controller.initialize()
        assert controller.move_track_to_position(100) is False

    def test_track_disabled(self, initialized_controller):
        """Test that track methods fail if component is disabled."""
        initialized_controller.disable_track_component()
        assert initialized_controller.move_track_to_position(100) is False


class TestStateManagementAndMonitoring:
    """Test state and error management."""
    
    def test_get_system_status(self, initialized_controller):
        """Test getting system status."""
        status = initialized_controller.get_system_status()
        assert isinstance(status, dict)
        assert 'connection' in status
        assert 'arm' in status
    
    def test_error_tracking(self, initialized_controller):
        """Test error code tracking."""
        initialized_controller.arm.error_code = 1
        initialized_controller._error_warn_callback({'error_code': 1})
        assert initialized_controller.last_error_code == 1
    
    def test_warning_tracking(self, initialized_controller):
        """Test warning code tracking."""
        initialized_controller.arm.warn_code = 1
        initialized_controller._error_warn_callback({'warn_code': 1})
        assert initialized_controller.last_warn_code == 1
    
    def test_is_alive_property(self, initialized_controller):
        """Test is_alive property."""
        assert initialized_controller.is_alive is True
        initialized_controller._state_changed_callback({'state': 4})
        assert initialized_controller.is_alive is False
    
    def test_get_error_history(self, initialized_controller):
        """Test retrieving error history."""
        initialized_controller._error_warn_callback({'error_code': 10})
        history = initialized_controller.get_error_history()
        assert len(history) > 0
        assert history[0]['error_code'] == 10

    def test_clear_errors(self, initialized_controller):
        """Test clearing errors."""
        initialized_controller.arm.error_code = 1
        initialized_controller.clear_errors()
        assert initialized_controller.last_error_code == 0


class TestUtilityMethods:
    """Test utility methods."""
    
    def test_get_current_position(self, initialized_controller):
        """Test getting current position."""
        pos = initialized_controller.get_current_position()
        assert isinstance(pos, list) and len(pos) == 6
    
    def test_get_current_joints(self, initialized_controller):
        """Test getting current joint angles."""
        joints = initialized_controller.get_current_joints()
        assert isinstance(joints, list) and len(joints) == initialized_controller.num_joints
    
    def test_get_named_locations(self, initialized_controller):
        """Test getting named locations."""
        locations = initialized_controller.get_named_locations()
        assert 'home' in locations
        assert 'pickup' in locations

    def test_get_system_info(self, initialized_controller):
        """Test getting system info."""
        info = initialized_controller.get_system_info()
        assert info['model'] == 6
        assert info['has_gripper'] is True
        assert info['has_track'] is True
    
    def test_check_code_success(self, initialized_controller):
        """Test check_code for success."""
        assert initialized_controller.check_code(0, 'test_op') is True
    
    def test_check_code_failure(self, initialized_controller):
        """Test check_code for failure."""
        assert initialized_controller.check_code(1, 'test_op') is False


class TestSafetyAndValidation:
    """Test safety and validation systems."""
    
    def test_safety_level_enforcement(self, mock_config_files):
        """Test safety level speed enforcement."""
        controller = XArmController(profile_name='test_profile', safety_level=SafetyLevel.HIGH, simulation_mode=True)
        # High safety should cap speeds
        assert controller.tcp_speed < controller.safety_config.get('max_tcp_speed', 1000)
    
    def test_joint_limit_validation(self, simulation_controller):
        """Test joint limit validation."""
        simulation_controller.initialize()
        # This should fail validation
        assert simulation_controller.move_joints([500] * 6) is False

    def test_workspace_boundary_validation(self, initialized_controller):
        """Test workspace boundary validation."""
        # This should fail validation
        assert initialized_controller.move_to_position(x=9000, y=0, z=0) is False


class TestErrorHandling:
    """Test error handling scenarios."""
    
    def test_missing_config_files(self, monkeypatch):
        """Test handling of missing configuration files."""
        # Patch load_config to simulate FileNotFoundError
        monkeypatch.setattr('src.core.xarm_controller.load_config', MagicMock(side_effect=FileNotFoundError))
        controller = XArmController(profile_name='non_existent_profile', simulation_mode=True)
        # Should initialize with default values
        assert controller.initialize() is True
    
    def test_invalid_gripper_type(self, mock_config_files):
        """Test handling of an invalid gripper type."""
        with pytest.raises(ValueError, match="Invalid gripper type"):
            XArmController(profile_name='test_profile', gripper_type='invalid_gripper')
    
    def test_arm_none_operations(self, mock_config_files):
        """Test that operations fail gracefully if arm is None."""
        controller = XArmController(profile_name='test_profile', simulation_mode=True)
        controller.initialize() # Initialize to set up states
        controller.arm = None  # Manually set arm to None after initialization
        assert controller.move_to_position(x=300, y=0, z=300) is False, "Movement should fail if arm is None."
    
    def test_position_updates_with_none_arm(self, mock_config_files):
        """Test that position updates don't crash if arm is None."""
        controller = XArmController(profile_name='test_profile', simulation_mode=True)
        controller.arm = None
        controller._update_positions()  # Should not raise an exception


class TestConfigurationManagement:
    """Test advanced configuration management."""

    def test_host_priority_resolution(self, mock_config_files):
        """Test that host passed directly to constructor takes priority."""
        controller = XArmController(profile_name='test_profile', host='192.168.1.100')
        assert controller.host == '192.168.1.100'

    def test_model_configuration(self, mock_config_files):
        """Test model configuration from profile."""
        controller = XArmController(profile_name='test_profile')
        assert controller.model == 6

    def test_safety_config_loading(self, initialized_controller):
        """Test that safety configuration is loaded."""
        assert 'workspace_limits' in initialized_controller.safety_config

class TestBackwardCompatibility:
    """Test backward compatibility of methods if any."""
    pass # No legacy methods to test for now 