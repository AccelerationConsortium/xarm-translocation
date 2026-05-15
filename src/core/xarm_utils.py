"""
xArm Utility Functions

This module contains utility functions for the xArm controller that are
independent of the main controller class and can be used across multiple
modules.
"""

import yaml
import time
import traceback
import math
import os
from typing import Dict, List, Optional, Tuple, Any
from enum import Enum


class SafetyLevel(Enum):
    """Safety level definitions"""
    EMERGENCY = 0
    HIGH = 1
    MEDIUM = 2
    LOW = 3

# =============================================================================
# GENERAL UTILITIES
# =============================================================================

def pprint(*args, **kwargs):
    """
    Pretty print with timestamp and caller info for debugging.
    
    Args:
        *args: Arguments to print
        **kwargs: Keyword arguments for print
    """
    try:
        stack_tuple = traceback.extract_stack(limit=2)[0]
        print('[{}][{}] {}'.format(time.strftime(
            '%Y-%m-%d %H:%M:%S', time.localtime(time.time())),
            stack_tuple[1], ' '.join(map(str, args)))
        )
    except:
        print(*args, **kwargs)


def check_return_code(code: int, operation_name: str, arm_state: Optional[int] = None, error_code: Optional[int] = None) -> bool:
    """
    Check if an operation was successful based on return code.
    
    Args:
        code: Return code from operation
        operation_name: Name of the operation for error reporting
        arm_state: Current arm state (optional)
        error_code: Current error code (optional)
        
    Returns:
        True if operation was successful, False otherwise
    """
    if code != 0:
        print(f'{operation_name} failed: code={code}, state={arm_state}, error={error_code}')
        return False
    return True


def clamp_value(value: float, min_val: float, max_val: float) -> float:
    """
    Clamp a value between minimum and maximum bounds.
    
    Args:
        value: Value to clamp
        min_val: Minimum bound
        max_val: Maximum bound
        
    Returns:
        Clamped value
    """
    return max(min_val, min(max_val, value))


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [-180, 180] range.
    
    Args:
        angle: Angle in degrees
        
    Returns:
        Normalized angle
    """
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def calculate_distance(pos1: List[float], pos2: List[float]) -> float:
    """
    Calculate Euclidean distance between two positions.
    
    Args:
        pos1: First position [x, y, z, ...]
        pos2: Second position [x, y, z, ...]
        
    Returns:
        Distance between positions
    """
    if len(pos1) < 3 or len(pos2) < 3:
        return 0.0
    
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(pos1[:3], pos2[:3])))


def is_position_close(pos1: List[float], pos2: List[float], tolerance: float = 1.0) -> bool:
    """
    Check if two positions are close within tolerance.
    
    Args:
        pos1: First position
        pos2: Second position
        tolerance: Distance tolerance
        
    Returns:
        True if positions are within tolerance
    """
    return calculate_distance(pos1, pos2) <= tolerance

# =============================================================================
# CONFIGURATION UTILITIES
# =============================================================================

def load_config(file_path: str) -> Dict[str, Any]:
    """
    Load YAML configuration file.
    
    Args:
        file_path: Path to the YAML configuration file
        
    Returns:
        Dictionary containing configuration data, empty dict if file not found
    """
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Warning: Config file {file_path} not found, using defaults")
        return {}
    except Exception as e:
        print(f"Error loading config {file_path}: {e}")
        return {}


def get_default_config(config_type: str) -> Dict[str, Any]:
    """
    Provide default configurations if files are missing.
    
    Args:
        config_type: Type of configuration ('xarm_config', 'gripper_config', etc.)
        
    Returns:
        Dictionary containing default configuration values
    """
    defaults = {
        'xarm_config': {
            'host': '127.0.0.1',
            'model': 7,
            'tcp_speed': 100,
            'tcp_acc': 1000,
            'angle_speed': 20,
            'angle_acc': 500,
            'tcp_speed_limit': 1000,
            'tcp_acc_limit': 50000,
            'angle_speed_limit': 180,
            'angle_acc_limit': 1145,
            'do_not_open': False,
            'is_radian': False,
            'ignore_exit_state': False,
            'check_tcp_limit': True,
            'check_joint_limit': True,
            'check_cmdnum_limit': True,
            'check_is_ready': True,
            'check_is_pause': True,
            'max_callback_thread_count': 10,
            'max_cmdnum': 256,
            'init_axis': 6,
            'report_type': 'normal',
            'baud_checkset': True,
            'min_tcp_speed': 0.1,
            'min_tcp_acc': 1.0,
            'max_tcp_speed': 1000,
            'max_tcp_acc': 50000,
            'min_joint_speed': 0.1,
            'min_joint_acc': 1.0,
            'max_joint_speed': 180,
            'max_joint_acc': 1145,
        },
        'gripper_config': {
            'GRIPPER_SPEED': 5000,
            'GRIPPER_FORCE': 100,
            'bio_gripper': {
                'enable': True,
                'speed': 5000,
                'force': 100,
                'close_pos': 800,
                'open_pos': 0,
                'timeout': 5
            },
            'standard_gripper': {
                'enable': True,
                'speed': 5000,
                'force': 100,
                'close_pos': 800,
                'open_pos': 0,
                'timeout': 5
            },
            'robotiq_gripper': {
                'enable': True,
                'speed': 255,
                'force': 255,
                'close_pos': 255,
                'open_pos': 0,
                'timeout': 5
            }
        },
        'track_config': {
            'enable': True,
            'speed': 200,
            'pos_limit': [-700, 700],
            'speed_limit': [1, 1000],
            'acc_limit': [1, 20000],
            'timeout': 10
        },
        'position_config': {
            'positions': {
                'ROBOT_HOME': [0.0, 0.0, 0.0, 0.0, 0.0],
            }
        },
        'safety_config': {
            'workspace_limits': DEFAULT_SAFETY_BOUNDARIES,
            'joint_limits': DEFAULT_JOINT_LIMITS,
            'collision_zones': [
                {'name': 'table', 'bounds': {'x': (-400, 400), 'y': (-400, 400), 'z': (-50, 0)}},
                {'name': 'base', 'bounds': {'x': (-150, 150), 'y': (-150, 150), 'z': (0, 100)}}
            ],
            'max_tcp_speed': 1000,
            'max_joint_speed': 180,
            'collision_sensitivity': DEFAULT_COLLISION_SENSITIVITY,
            'temperature_limits': DEFAULT_TEMPERATURE_THRESHOLDS
        }
    }
    
    return defaults.get(config_type, {})


# =============================================================================
# VALIDATION UTILITIES
# =============================================================================

def validate_target_position(position: List[float], safety_boundaries: Dict[str, Tuple[float, float]]) -> Tuple[bool, Optional[str]]:
    """
    Validate target position against safety boundaries.
    
    Args:
        position: Target position [x, y, z, roll, pitch, yaw]
        safety_boundaries: Dictionary of safety boundaries for each axis
        
    Returns:
        Tuple of (is_valid, error_message)
    """
    if len(position) < 6:
        return False, "Position must have at least 6 values"
    
    x, y, z, roll, pitch, yaw = position[:6]
    
    # Check workspace limits
    checks = [
        ('x', x, safety_boundaries.get('x', (-700, 700))),
        ('y', y, safety_boundaries.get('y', (-700, 700))),
        ('z', z, safety_boundaries.get('z', (-100, 700))),
        ('roll', roll, safety_boundaries.get('roll', (-180, 180))),
        ('pitch', pitch, safety_boundaries.get('pitch', (-180, 180))),
        ('yaw', yaw, safety_boundaries.get('yaw', (-180, 180)))
    ]
    
    for axis, value, (min_val, max_val) in checks:
        if not (min_val <= value <= max_val):
            return False, f"{axis} value {value} outside safety limits [{min_val}, {max_val}]"
    
    return True, None


def validate_joint_angles(angles: List[float], model_joint_limits: List[Tuple[Any, Any]]) -> Tuple[bool, Optional[str]]:
    """
    Validate joint angles against model-specific limits.
    
    Args:
        angles: Joint angles to validate
        model_joint_limits: List of (min, max) tuples for the specific model's joints
        
    Returns:
        Tuple of (is_valid, error_message)
    """
    num_joints = len(model_joint_limits)
    
    if len(angles) < num_joints:
        return False, f"Need at least {num_joints} joint angles, got {len(angles)}"
    
    for i, (angle, (min_limit, max_limit)) in enumerate(zip(angles[:num_joints], model_joint_limits)):
        if angle < min_limit or angle > max_limit:
            return False, f"Joint {i+1} angle {angle} outside limits [{min_limit}, {max_limit}]"
    
    return True, None


def validate_track_position(position: float, limits: Tuple[float, float], danger_zones: List[Dict[str, Any]]) -> Tuple[bool, Optional[str]]:
    """
    Validate linear track position.
    
    Args:
        position: Track position to validate
        limits: (min_position, max_position)
        danger_zones: List of danger zones to check against
        
    Returns:
        Tuple of (is_valid, error_message)
    """
    min_pos, max_pos = limits
    if not (min_pos <= position <= max_pos):
        return False, f"Track position {position} outside limits [{min_pos}, {max_pos}]"
        
    for zone in danger_zones:
        if zone.get('start', 0) <= position <= zone.get('end', 700):
            if zone.get('block_movement', False):
                return False, f"Track position {position}mm is in a blocked danger zone: {zone.get('name', 'Unknown')}"
            else:
                print(f"Warning: Track position {position}mm is in danger zone: {zone.get('name', 'Unknown')}")
                
    return True, None


def validate_track_speed(speed: float, limits: Tuple[float, float]) -> Tuple[bool, Optional[str]]:
    """
    Validate linear track speed.
    
    Args:
        speed: Track speed to validate
        limits: (min_speed, max_speed)
        
    Returns:
        Tuple of (is_valid, error_message)
    """
    min_speed, max_speed = limits
    if not (min_speed <= speed <= max_speed):
        return False, f"Track speed {speed} outside limits [{min_speed}, {max_speed}]"
    return True, None


# =============================================================================
# COLLISION DETECTION UTILITIES
# =============================================================================

def check_joint_collision_simulation(angles: List[float], model_joint_limits: List[Tuple[Any, Any]]) -> bool:
    """
    Check for joint collisions in simulation mode.
    
    Args:
        angles: Joint angles to check
        model_joint_limits: List of (min, max) tuples for the specific model's joints
        
    Returns:
        True if collision detected, False otherwise
    """
    # Simple collision check - uses joint limits validation
    is_valid, _ = validate_joint_angles(angles, model_joint_limits)
    return not is_valid


def check_workspace_collision_simulation(pose: List[float], collision_zones: List[Dict[str, Any]]) -> Tuple[bool, Optional[str]]:
    """
    Check for workspace collisions in simulation mode.
    
    Args:
        pose: Target pose [x, y, z, roll, pitch, yaw]
        collision_zones: List of collision zone definitions
        
    Returns:
        Tuple of (collision_detected, zone_name)
    """
    if len(pose) < 6:
        return False, None
        
    x, y, z, roll, pitch, yaw = pose[:6]
    
    for zone in collision_zones:
        bounds = zone['bounds']
        if (bounds['x'][0] <= x <= bounds['x'][1] and
            bounds['y'][0] <= y <= bounds['y'][1] and
            bounds['z'][0] <= z <= bounds['z'][1]):
            return True, zone['name']
    
    return False, None


# =============================================================================
# SPEED AND ACCELERATION UTILITIES
# =============================================================================

def validate_speed_limits(speed: float, min_speed: float, max_speed: float, speed_type: str = "speed") -> Tuple[bool, Optional[str]]:
    """
    Validate speed against limits.
    
    Args:
        speed: Speed value to validate
        min_speed: Minimum allowed speed
        max_speed: Maximum allowed speed
        speed_type: Type of speed for error messages
        
    Returns:
        Tuple of (is_valid, error_message)
    """
    if speed < min_speed or speed > max_speed:
        return False, f"{speed_type} {speed} outside limits [{min_speed}, {max_speed}]"
    return True, None


def calculate_safe_speed(distance: float, max_speed: float, acceleration: float) -> float:
    """
    Calculate safe speed based on distance and acceleration.
    
    Args:
        distance: Distance to travel
        max_speed: Maximum allowed speed
        acceleration: Acceleration value
        
    Returns:
        Safe speed value
    """
    # Simple speed calculation: v = sqrt(2*a*d), but clamped to max_speed
    safe_speed = math.sqrt(2 * acceleration * abs(distance))
    return min(safe_speed, max_speed) 

# =============================================================================
# HARDWARE CAPABILITY LIMITS
# =============================================================================

# These constants define the absolute physical limits of the hardware.
# User-defined safety settings should not exceed these values.
HARDWARE_LIMITS = {
    'workspace_limits': {
        'x': (-800, 800),
        'y': (-800, 800),
        'z': (-400, 850),
        'roll': (-360, 360),
        'pitch': (-180, 180),
        'yaw': (-360, 360)
    },
    'max_tcp_speed': 1500,  # mm/s
    'max_joint_speed': 200, # deg/s
    'temperature_limits': {
        'warning': 80, # deg C
        'critical': 95  # deg C
    },
    'collision_sensitivity': (0, 5) # min/max range
}

# =============================================================================
# SAFETY CONSTRAINTS AND DEFAULT CONFIGURATIONS
# =============================================================================

# Joint limits for different xArm models (degrees)
DEFAULT_JOINT_LIMITS = {
    5: [(-360, 360), (-118, 120), (-225, 11), (-97, 180), (-360, 360)],
    6: [(-360, 360), (-118, 120), (-225, 11), (-360, 360), (-97, 180), (-360, 360)],
    7: [(-360, 360), (-118, 120), (-360, 360), (-11, 225),(-360, 360), (-97, 180), (-360, 360)],
}

# Default performance thresholds
DEFAULT_PERFORMANCE_THRESHOLDS = {
    'max_cycle_time': 30.0,  # seconds - increased for realistic robot movements
    'max_accuracy_error': 1.0,  # mm
    'max_utilization': 85.0  # percentage
}

# Default temperature thresholds for predictive maintenance
DEFAULT_TEMPERATURE_THRESHOLDS = {
    'warning': 60,  # deg C
    'critical': 75  # deg C
}

# Safety multipliers for different safety levels
SAFETY_LEVEL_MULTIPLIERS = {
    SafetyLevel.EMERGENCY: 0.1,
    SafetyLevel.HIGH: 0.5,
    SafetyLevel.MEDIUM: 0.8,
    SafetyLevel.LOW: 1.0
}

# Default safety boundaries
DEFAULT_SAFETY_BOUNDARIES = {
    'x': (-700, 700),
    'y': (-700, 700),
    'z': (-200, 700),
    'roll': (-180, 180),
    'pitch': (-180, 180),
    'yaw': (-180, 180)
}

# Default collision sensitivity
DEFAULT_COLLISION_SENSITIVITY = 3


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def validate_and_apply_safety_config(user_config: Dict[str, Any]) -> Dict[str, Any]:
    """
    Validates user-provided safety config against hardware limits.
    Clamps values to hardware capabilities and prints warnings if changes are made.

    Args:
        user_config: The safety configuration loaded from safety.yaml.

    Returns:
        A validated safety configuration dictionary.
    """
    validated_config = user_config.copy()
    
    # Validate workspace_limits
    user_limits = validated_config.get('workspace_limits', {})
    hw_limits = HARDWARE_LIMITS['workspace_limits']
    validated_limits = {}
    for axis, (hw_min, hw_max) in hw_limits.items():
        user_min, user_max = user_limits.get(axis, (hw_min, hw_max))
        safe_min = max(user_min, hw_min)
        safe_max = min(user_max, hw_max)
        if (safe_min, safe_max) != (user_min, user_max):
            print(f"Warning: Workspace limit for axis '{axis}' was clamped from ({user_min}, {user_max}) to ({safe_min}, {safe_max}) to stay within hardware capabilities.")
        validated_limits[axis] = (safe_min, safe_max)
    validated_config['workspace_limits'] = validated_limits

    # Validate max speeds
    user_tcp_speed = validated_config.get('max_tcp_speed', HARDWARE_LIMITS['max_tcp_speed'])
    hw_tcp_speed = HARDWARE_LIMITS['max_tcp_speed']
    safe_tcp_speed = min(user_tcp_speed, hw_tcp_speed)
    if safe_tcp_speed != user_tcp_speed:
        print(f"Warning: 'max_tcp_speed' clamped from {user_tcp_speed} to {safe_tcp_speed} (hardware limit).")
    validated_config['max_tcp_speed'] = safe_tcp_speed

    user_joint_speed = validated_config.get('max_joint_speed', HARDWARE_LIMITS['max_joint_speed'])
    hw_joint_speed = HARDWARE_LIMITS['max_joint_speed']
    safe_joint_speed = min(user_joint_speed, hw_joint_speed)
    if safe_joint_speed != user_joint_speed:
        print(f"Warning: 'max_joint_speed' clamped from {user_joint_speed} to {safe_joint_speed} (hardware limit).")
    validated_config['max_joint_speed'] = safe_joint_speed

    # Validate temperature limits
    user_temps = validated_config.get('temperature_limits', {})
    hw_temps = HARDWARE_LIMITS['temperature_limits']
    safe_temps = {}
    for level, hw_max in hw_temps.items():
        user_val = user_temps.get(level, hw_max)
        safe_val = min(user_val, hw_max)
        if safe_val != user_val:
            print(f"Warning: Temperature limit '{level}' clamped from {user_val} deg C to {safe_val} deg C (hardware limit).")
        safe_temps[level] = safe_val
    validated_config['temperature_limits'] = safe_temps

    # Validate collision sensitivity
    user_sensitivity = validated_config.get('collision_sensitivity', DEFAULT_COLLISION_SENSITIVITY)
    hw_min_sens, hw_max_sens = HARDWARE_LIMITS['collision_sensitivity']
    safe_sensitivity = clamp_value(user_sensitivity, hw_min_sens, hw_max_sens)
    if safe_sensitivity != user_sensitivity:
        print(f"Warning: 'collision_sensitivity' clamped from {user_sensitivity} to {safe_sensitivity} to be within hardware range [{hw_min_sens}, {hw_max_sens}].")
    validated_config['collision_sensitivity'] = safe_sensitivity
    
    return validated_config

def get_safety_speed_limits(safety_level: SafetyLevel, max_tcp_speed: int = 1000, max_joint_speed: int = 180) -> Tuple[int, int]:
    """Calculate speed limits based on safety level."""
    SAFETY_LEVEL_MULTIPLIERS = {
        'EMERGENCY': 0.1,
        'HIGH': 0.25,
        'MEDIUM': 0.5,
        'LOW': 1.0
    }
    
    multiplier = SAFETY_LEVEL_MULTIPLIERS.get(safety_level.name, 0.5)

    safe_tcp_speed = int(max_tcp_speed * multiplier)
    safe_joint_speed = int(max_joint_speed * multiplier)
    
    return safe_tcp_speed, safe_joint_speed


def apply_movement_parameter_limits(tcp_speed: float, tcp_acc: float, angle_speed: float, angle_acc: float, 
                                   max_tcp_speed: int, max_joint_speed: int) -> Tuple[float, float, float, float]:
    """
    Apply safety limits to movement parameters.
    
    Args:
        tcp_speed: Raw TCP speed
        tcp_acc: Raw TCP acceleration
        angle_speed: Raw joint speed
        angle_acc: Raw joint acceleration
        max_tcp_speed: Maximum allowed TCP speed
        max_joint_speed: Maximum allowed joint speed
        
    Returns:
        Tuple of validated (tcp_speed, tcp_acc, angle_speed, angle_acc)
    """
    validated_tcp_speed = max(1, min(max_tcp_speed, tcp_speed))
    validated_tcp_acc = max(1, min(50000, tcp_acc))
    validated_angle_speed = max(1, min(max_joint_speed, angle_speed))
    validated_angle_acc = max(1, min(1145, angle_acc))
    
    return validated_tcp_speed, validated_tcp_acc, validated_angle_speed, validated_angle_acc


def create_default_performance_metrics() -> Dict[str, Any]:
    """
    Create default performance metrics structure.
    
    Returns:
        Dictionary with performance metrics structure
    """
    from collections import deque
    
    return {
        'cycle_times': deque(maxlen=100),
        'accuracy_errors': deque(maxlen=100),
        'tcp_utilization': deque(maxlen=100),
        'joint_utilization': deque(maxlen=100),
        'command_success_rate': deque(maxlen=100)
    }


def get_joint_limits_for_model(model: int) -> List[Tuple[int, int]]:
    """
    Get joint limits for a specific xArm model.
    
    Args:
        model: xArm model number (5, 6, 7)
        
    Returns:
        List of (min, max) tuples for each joint
    """
    return DEFAULT_JOINT_LIMITS.get(model, DEFAULT_JOINT_LIMITS[7])


def check_operation_result(code: int, operation_name: str, arm_state: Optional[int] = None, 
                          error_code: Optional[int] = None, is_simulation: bool = False) -> bool:
    """
    Check if an operation was successful and handle errors.
    
    Args:
        code: Return code from operation (None or 0 typically means success)
        operation_name: Name of the operation for logging
        arm_state: Current arm state (optional)
        error_code: Current error code (optional)
        is_simulation: Whether running in simulation mode
        
    Returns:
        True if operation was successful, False otherwise
    """
    if is_simulation:
        # In simulation mode, assume success if code is 0 or None
        return code == 0 or code is None
    
    # For xArm SDK, None or 0 typically indicates success
    if code is None or code == 0:
        return True
        
    return check_return_code(code, operation_name, arm_state, error_code)