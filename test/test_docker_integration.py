"""
Docker integration tests for XArmController.
These tests require a running Docker simulator.
"""

import pytest
import time
import sys
import os

# Add project root to the Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core.xarm_controller import XArmController, ComponentState


@pytest.mark.integration
class TestDockerIntegration:
    """Integration tests that run against the Docker simulator."""
    
    @pytest.fixture
    def docker_controller(self):
        """Create a controller configured for Docker simulator."""
        return XArmController(
            profile_name='docker',
            gripper_type='bio',
            enable_track=True,
            auto_enable=False
        )
    
    def test_docker_connection(self, docker_controller):
        """Test connection to Docker simulator."""
        if not is_docker_available():
            pytest.skip("Docker simulator not available at 127.0.0.1")
        
        success = docker_controller.initialize()
        assert success is True
        assert docker_controller.arm.connected is True
        assert docker_controller.get_component_states()['connection'] == 'enabled'
        
        docker_controller.disconnect()
    
    def test_docker_full_workflow(self, docker_controller):
        """Test complete workflow with Docker simulator."""
        if not is_docker_available():
            pytest.skip("Docker simulator not available")
            
        success = docker_controller.initialize()
        assert success, "Failed to initialize controller for Docker workflow test"
        
        try:
            status = docker_controller.get_system_status()
            assert status['connection']['connected'] is True
            
            # Note: Docker simulator has limitations, so we don't assert success for components
            docker_controller.enable_gripper_component()
            docker_controller.enable_track_component()
            
            position = docker_controller.get_current_position()
            assert position is not None
            
            joints = docker_controller.get_current_joints()
            assert joints is not None
            
        finally:
            docker_controller.disconnect()
    
    def test_docker_error_handling(self, docker_controller):
        """Test error handling with Docker simulator."""
        if not is_docker_available():
            pytest.skip("Docker simulator not available")
        
        success = docker_controller.initialize()
        assert success, "Failed to initialize controller for error handling test"
        
        try:
            assert docker_controller.move_to_named_location('invalid_location') is False
            assert docker_controller.is_alive
        finally:
            docker_controller.disconnect()


@pytest.mark.integration 
@pytest.mark.slow
class TestDockerStressTest:
    """Stress tests for Docker integration."""
    
    def test_multiple_connections(self):
        """Test multiple connect/disconnect cycles."""
        if not is_docker_available():
                pytest.skip("Docker simulator not available")
            
        for _ in range(3):
            controller = XArmController(profile_name='docker', auto_enable=False)
            assert controller.initialize() is True
            assert controller.is_alive
            controller.disconnect()
            time.sleep(0.1)


@pytest.mark.integration
class TestDockerComponentIsolation:
    """Test individual component functionality with Docker."""
    
    @pytest.fixture
    def initialized_docker_controller(self):
        """Fixture for initialized Docker controller."""
        if not is_docker_available():
            pytest.skip("Docker simulator not available")
        
        controller = XArmController(profile_name='docker', auto_enable=False)
        assert controller.initialize(), "Setup: failed to initialize controller"
        yield controller
        controller.disconnect()
    
    def test_docker_arm_only(self, initialized_docker_controller):
        """Test arm functionality without other components."""
        controller = initialized_docker_controller
        assert controller.move_joints([10, 0, 0, 0, 0, 0]) is True
        time.sleep(0.5)
        assert controller.move_joints([0, 0, 0, 0, 0, 0]) is True
    
    def test_docker_gripper_isolation(self, initialized_docker_controller):
        """Test gripper functionality in isolation."""
        controller = initialized_docker_controller
        if not controller.enable_gripper_component():
            pytest.skip("Gripper not available in simulator")
        
        assert controller.open_gripper() is True
        time.sleep(0.5)
        assert controller.close_gripper() is True
    
    def test_docker_track_isolation(self, initialized_docker_controller):
        """Test linear track functionality in isolation."""
        controller = initialized_docker_controller
        if not controller.enable_track_component():
            pytest.skip("Linear track not available in simulator")
        
        assert controller.move_track_to_position(25) is True
        time.sleep(0.5)
        assert controller.reset_track() is True


def is_docker_available():
    """Singleton check for Docker availability to speed up tests."""
    if not hasattr(is_docker_available, "available"):
        arm = None
        try:
            from xarm.wrapper import XArmAPI
            # Use docker profile settings for the check
            arm = XArmAPI('127.0.0.1', check_joint_limit=False, do_not_open=True)
            arm.connect()
            is_docker_available.available = arm.connected
        except Exception:
            is_docker_available.available = False
        finally:
            if arm and arm.connected:
                arm.disconnect()
    return is_docker_available.available

@pytest.mark.integration
def test_docker_availability_marker():
    """A simple test that marks the suite based on Docker's availability."""
    if not is_docker_available():
        pytest.skip("Docker simulator is not running or not reachable at 127.0.0.1. Please start it to run integration tests.") 