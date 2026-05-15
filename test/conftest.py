"""
Pytest configuration and fixtures for xarm-translocation tests.
"""

import pytest
import sys
import os
from unittest.mock import Mock, MagicMock, patch

# Add src to path to allow imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core.xarm_controller import XArmController, ComponentState


@pytest.fixture
def mock_xarm_api():
    """Create a mock XArmAPI for testing."""
    mock_arm = Mock()
    
    # Mock basic properties
    mock_arm.connected = True
    mock_arm.state = 0
    mock_arm.mode = 0
    mock_arm.error_code = 0
    mock_arm.warn_code = 0
    
    # Mock position and joint methods
    mock_arm.get_position.return_value = (0, [300, 0, 300, 180, 0, 0])
    mock_arm.get_servo_angle.return_value = (0, [0, 0, 0, 0, 0, 0])
    
    # Mock movement methods
    mock_arm.set_servo_angle.return_value = 0
    mock_arm.set_position.return_value = 0
    mock_arm.move_gohome.return_value = 0
    
    # Mock connection and control methods
    mock_arm.connect.return_value = 0
    mock_arm.disconnect.return_value = 0
    mock_arm.clean_warn.return_value = 0
    mock_arm.clean_error.return_value = 0
    mock_arm.motion_enable.return_value = 0
    mock_arm.set_mode.return_value = 0
    mock_arm.set_state.return_value = 0
    mock_arm.emergency_stop.return_value = 0
    mock_arm.register_error_warn_changed_callback.return_value = 0
    mock_arm.register_state_changed_callback.return_value = 0
    
    # Mock gripper methods
    mock_arm.set_bio_gripper_enable.return_value = 0
    mock_arm.open_bio_gripper.return_value = 0
    mock_arm.close_bio_gripper.return_value = 0
    mock_arm.set_bio_gripper_g2_position.return_value = 0
    mock_arm.set_bio_gripper_force.return_value = 0
    mock_arm.set_bio_gripper_speed.return_value = 0
    mock_arm.get_bio_gripper_g2_position.return_value = (0, 0)
    mock_arm.set_gripper_enable.return_value = 0
    mock_arm.set_gripper_position.return_value = 0
    mock_arm.robotiq_reset.return_value = 0
    mock_arm.robotiq_set_activate.return_value = 0
    mock_arm.robotiq_set_position.return_value = 0
    mock_arm.robotiq_open.return_value = 0
    mock_arm.robotiq_close.return_value = 0
    
    # Mock linear track methods
    mock_arm.set_linear_track_enable.return_value = 0
    mock_arm.set_linear_track_speed.return_value = 0
    mock_arm.set_linear_track_pos.return_value = 0
    mock_arm.get_linear_track_pos.return_value = (0, 0)
    
    # Mock velocity control methods
    mock_arm.vc_set_cartesian_velocity.return_value = 0
    mock_arm.vc_set_joint_velocity.return_value = 0
    mock_arm.set_only_check_type.return_value = 0

    # Mock attributes for predictive maintenance
    mock_arm.temperatures = [30.0] * 8
    mock_arm.joints_torque = [0.0] * 8
    mock_arm.currents = [0.0] * 8
    
    return mock_arm


@pytest.fixture
def mock_config_files(monkeypatch):
    """Mocks the configuration loading to prevent file I/O."""
    
    mock_configs = {
        'xarm_config': {
            'default_profile': 'test_profile',
            'profiles': {
                'test_profile': {
        'host': '127.0.0.1',
                    'model': 6
                }
            }
        },
        'gripper_config': {
            'GRIPPER_SPEED': 300,
            'bio': {
                'name': 'BioGripper Gen1',
                'type': 'bio',
                'has_stroke_control': False,
                'has_force_control': False,
                'speed': 300,
            },
            'bio_gen2': {
                'name': 'BioGripper Gen2',
                'type': 'bio_gen2',
                'has_stroke_control': True,
                'has_force_control': True,
                'speed': 1000,
                'force': 50,
                'timeout': 5,
                'stroke_range': {'min': 71, 'max': 150},
                'speed_range': {'min': 0, 'max': 4000},
                'force_range': {'min': 1, 'max': 100},
                'open_position': 150,
                'close_position': 71,
            },
        },
        'position_config': {
            'positions': {
        'home': {'x': 300, 'y': 0, 'z': 300, 'roll': 180, 'pitch': 0, 'yaw': 0},
        'pickup': {'x': 400, 'y': 100, 'z': 200, 'roll': 180, 'pitch': 0, 'yaw': 0}
    }
        },
        'linear_track_config': {'Speed': 200, 'Acc': 1000},
        'safety_config': {}
    }

    def mock_load_config(file_path):
        filename = os.path.basename(file_path)
        if 'xarm_config' in filename:
            return mock_configs['xarm_config']
        elif 'gripper_config' in filename:
            return mock_configs['gripper_config']
        elif 'position_config' in filename:
            return mock_configs['position_config']
        elif 'linear_track_config' in filename:
            return mock_configs['linear_track_config']
        elif 'safety' in filename:
            return mock_configs['safety_config']
        return {}

    monkeypatch.setattr('src.core.xarm_controller.load_config', mock_load_config)


@pytest.fixture
def controller_no_auto_enable(mock_config_files):
    """Create XArmController with auto_enable=False for testing."""
    return XArmController(
        profile_name='test_profile',
        gripper_type='bio',
        enable_track=True,
        auto_enable=False
    )


@pytest.fixture
def controller_auto_enable(mock_config_files):
    """Create XArmController with auto_enable=True for testing."""
    return XArmController(
        profile_name='test_profile',
        gripper_type='bio',
        enable_track=True,
        auto_enable=True
    )


@pytest.fixture
def initialized_controller(mock_config_files, mock_xarm_api, monkeypatch):
    """Create an initialized controller with a mocked arm instance."""
    monkeypatch.setattr('src.core.xarm_controller.XArmAPI', lambda *args, **kwargs: mock_xarm_api)
    
    controller = XArmController(
        profile_name='test_profile',
        gripper_type='bio',
        enable_track=True,
        auto_enable=False
    )

    # Dynamically set mock properties based on controller's model
    mock_xarm_api.get_servo_angle.return_value = (0, [0] * controller.num_joints)
    mock_xarm_api.temperatures = [30.0] * controller.num_joints
    mock_xarm_api.joints_torque = [0.0] * controller.num_joints
    mock_xarm_api.currents = [0.0] * controller.num_joints

    # Safely initialize the controller
    success = controller.initialize()
    
    assert success, "Mocked controller initialization failed. This is a fixture error."
    
    return controller 