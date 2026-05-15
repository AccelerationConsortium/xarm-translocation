import yaml
import time
from enum import Enum
from collections import deque
from xarm.wrapper import XArmAPI
import os
from typing import Dict, List, Optional, Tuple, Any, Callable
import threading

from core.xarm_utils import (
    SafetyLevel, load_config, get_default_config, validate_target_position,
    validate_joint_angles, validate_track_position, validate_track_speed,
    check_joint_collision_simulation, check_workspace_collision_simulation,
    DEFAULT_PERFORMANCE_THRESHOLDS, DEFAULT_TEMPERATURE_THRESHOLDS,
    DEFAULT_SAFETY_BOUNDARIES, DEFAULT_COLLISION_SENSITIVITY,
    get_safety_speed_limits, apply_movement_parameter_limits, create_default_performance_metrics,
    get_joint_limits_for_model, check_operation_result, validate_and_apply_safety_config
)

class ComponentState(Enum):
    """Enum for component states"""
    UNKNOWN = "unknown"
    DISABLED = "disabled"
    ENABLING = "enabling"
    ENABLED = "enabled"
    ERROR = "error"
    MAINTENANCE = "maintenance"  # State for maintenance mode

class XArmController:
    """
    xArm controller with intelligent error recovery, improved safety validation,
    better configuration management, state tracking, and performance monitoring.
    """
    @staticmethod
    def _normalize_gripper_type(gripper_type: Optional[str]) -> str:
        """Normalize user/config aliases to the controller's gripper type keys."""
        value = (gripper_type or 'bio_gen2').lower().replace('-', '_').replace(' ', '_')
        aliases = {
            'bio_g2': 'bio_gen2',
            'biogripper_g2': 'bio_gen2',
            'biogripper_gen2': 'bio_gen2',
            'bio_gripper_gen2': 'bio_gen2',
            'bio_gripper_g2': 'bio_gen2',
            'biogripper': 'bio',
            'bio_gripper': 'bio',
        }
        normalized = aliases.get(value, value)
        valid_grippers = ['bio', 'bio_gen2', 'standard', 'robotiq', 'none']
        if normalized not in valid_grippers:
            raise ValueError(f"Invalid gripper type '{gripper_type}'. Must be one of {valid_grippers}")
        return normalized

    def __init__(self, host: Optional[str] = None, profile_name: Optional[str] = None,
                 gripper_type: Optional[str] = None, enable_track: bool = True,
                 auto_enable: bool = True, model: Optional[int] = None,
                 simulation_mode: bool = False, safety_level: SafetyLevel = SafetyLevel.MEDIUM):
        """
        Args:
            host (str, optional): The IP address of the xArm. If provided, this
                                overrides any host in the config file. Defaults to None.
            profile_name (str, optional): The name of the connection profile from xarm_config.yaml.
            gripper_type (str): Type of gripper ('bio', 'bio_gen2', 'standard', 'robotiq', or 'none')
            enable_track (bool): Whether to enable the linear track
            auto_enable (bool): Whether to automatically enable components during initialization
            model (int): xArm model (5, 6, 7). If None, will be detected from config
            simulation_mode (bool): Enable simulation mode (no hardware required)
            safety_level (SafetyLevel): Safety level for validation strictness
        """
        # The provided simulation_mode parameter is the source of truth.
        self.simulation_mode = simulation_mode
        
        self.safety_level = safety_level
        self.gripper_type = self._normalize_gripper_type(gripper_type)
        self.enable_track = enable_track
        self.auto_enable = auto_enable
        self.profile_name = profile_name

        # Initialize configuration attributes to help linter
        self.xarm_config = {}
        self.gripper_config = {}
        self.track_config = {}
        self.position_config = {}
        self.safety_config = {}
        self.force_torque_config = {}

        # Configuration loading
        self._load_configurations()

        # Let a connection profile choose the installed gripper unless an
        # explicit constructor value was provided.
        if gripper_type is None:
            self.gripper_type = self._normalize_gripper_type(
                self.xarm_config.get('gripper_type', 'bio_gen2')
            )
        self.current_gripper_config = self._resolve_current_gripper_config()

        # Determine the connection host with clear priority
        # 1. Direct `host` parameter
        # 2. Host from the selected profile
        # 3. Default to '127.0.0.1'
        self.host = host or self.xarm_config.get('host', '127.0.0.1')

        # Determine model
        # 1. Direct `model` parameter
        # 2. Model from the selected profile
        # 3. Default to 6
        self.model = model or self.xarm_config.get('model', 6)
        self.num_joints = self.model if self.model in [5, 6, 7] else 6  # 850 has 6 joints

        # Model name for API server
        self.model_name = f"xArm{self.model}"

        # Initialize state management
        self._initialize_state_management()

        # Initialize safety systems
        self._initialize_safety_systems()

        # Initialize error recovery system
        self._initialize_error_recovery()

        # Initialize robot arm connection
        if self.simulation_mode:
            # In simulation mode, create a mock arm object
            self.arm = self._create_simulation_arm()
        else:
            # For Docker simulator connections, we MUST disable the SDK's built-in
            # joint limit checking. The simulator doesn't provide a valid serial
            # number, causing the check to crash. For real hardware, we want this check enabled.
            disable_sdk_joint_check = self.profile_name and 'docker' in self.profile_name.lower()
            if disable_sdk_joint_check:
                print("Docker profile detected, disabling SDK joint limit checks to prevent serial number bug.")

            # Use official SDK with do_not_open parameter
            self.arm = XArmAPI(
                self.host,
                do_not_open=True,
                check_joint_limit=not disable_sdk_joint_check
            )

        # Movement parameters with validation
        self._setup_movement_parameters()

        # Initialize simulation state and collision detection
        if self.simulation_mode:
            self.last_position = [300, 0, 300, 180, 0, 0]
            self.last_joints = [0] * self.num_joints
            self.last_track_position = 0

        # Initialize if auto_enable is True
        if auto_enable:
            try:
                self.initialize()
            except Exception as e:
                print(f"Auto-initialization failed: {e}")

    def _load_configurations(self):
        """Load configurations from YAML files, using a profile-based system."""
        # Load the main configuration file which contains profiles
        main_config_path = os.path.join('src', 'settings', 'xarm_config.yaml')
        try:
            full_config = load_config(main_config_path)
        except FileNotFoundError:
            print(f"Warning: Main config file {main_config_path} not found, using defaults.")
            full_config = {}

        # Determine which profile to use and load it into self.xarm_config
        profile_to_use = self.profile_name or full_config.get('default_profile')
        if profile_to_use:
            self.xarm_config = full_config.get('profiles', {}).get(profile_to_use, {})
            if not self.xarm_config:
                print(f"Warning: Profile '{profile_to_use}' not found. Using empty config for xArm.")
        else:
            print("Warning: No profile specified and no default_profile found. Using empty config for xArm.")
            self.xarm_config = {}

        # Load other component configurations as before
        component_configs = {
            'gripper_config': 'gripper_config.yaml' if self.gripper_type != 'none' else None,
            'track_config': 'linear_track_config.yaml' if self.enable_track else None,
            'position_config': 'position_config.yaml',
            'safety_config': 'safety.yaml',
            'force_torque_config': 'force_torque_config.yaml'
        }

        for config_attr, file_name in component_configs.items():
            if file_name:
                file_path = os.path.join('src', 'settings', file_name)
                try:
                    setattr(self, config_attr, load_config(file_path))
                except FileNotFoundError:
                    print(f"Warning: Config file {file_path} not found, using defaults for {config_attr}")
                    setattr(self, config_attr, get_default_config(config_attr))
            else:
                setattr(self, config_attr, {})

    def _resolve_current_gripper_config(self):
        """Return the active gripper's config while tolerating legacy shapes."""
        if self.gripper_type == 'none':
            return {}

        config = self.gripper_config or {}
        defaults = config.get('default', {}) if isinstance(config, dict) else {}
        active = {}
        if isinstance(config, dict):
            active = config.get(self.gripper_type, {})
            if not active and self.gripper_type == 'bio_gen2':
                active = config.get('bio_g2', {}) or config.get('biogripper_gen2', {})
            if not active and any(key in config for key in ('GRIPPER_SPEED', 'GRIPPER_FORCE')):
                active = config

        merged = {**defaults, **(active or {})}
        merged.setdefault('type', self.gripper_type)
        merged.setdefault('name', self.gripper_type.replace('_', ' ').title())
        merged.setdefault('speed', config.get('GRIPPER_SPEED', 300) if isinstance(config, dict) else 300)
        merged.setdefault('force', config.get('GRIPPER_FORCE', 100) if isinstance(config, dict) else 100)
        merged.setdefault('open_position', 0 if self.gripper_type == 'bio_gen2' else 850)
        merged.setdefault('close_position', 850 if self.gripper_type == 'bio_gen2' else 0)
        return merged

    def _gripper_setting(self, key, default=None):
        """Read active gripper settings with legacy uppercase fallbacks."""
        active = getattr(self, 'current_gripper_config', {}) or {}
        if key in active:
            return active[key]
        legacy_key = {
            'speed': 'GRIPPER_SPEED',
            'force': 'GRIPPER_FORCE',
            'open_position': 'OPEN_POSITION',
            'close_position': 'CLOSE_POSITION',
        }.get(key, key.upper())
        return (self.gripper_config or {}).get(legacy_key, default)

    @staticmethod
    def _coerce_int(value, name):
        """The xArm Gen2 gripper SDK packs command values as integers."""
        try:
            return int(round(float(value)))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be numeric") from exc

    def _initialize_state_management(self):
        """Initialize state management system with callbacks."""
        # Component states
        self.states = {
            'connection': ComponentState.DISABLED,
            'arm': ComponentState.DISABLED,
            'gripper': ComponentState.DISABLED,
            'track': ComponentState.DISABLED,
            'force_torque': ComponentState.DISABLED
        }

        # Error tracking with automatic cleanup
        self.error_history = deque(maxlen=1000)
        self.last_error_code = 0
        self.last_warn_code = 0

        # Position tracking with history for analysis
        self.position_history = deque(maxlen=100)
        self.last_position = [300, 0, 300, 180, 0, 0]  # Default position
        self.last_joints = [0] * self.num_joints
        self.last_track_position = 0
        self.last_gripper_position = self._gripper_setting('open_position', 0)
        self.last_gripper_force = self._gripper_setting('force', 100)
        self.last_gripper_speed = self._gripper_setting('speed', 300)

        # Force torque sensor tracking
        self.force_torque_history = deque(maxlen=1000)
        self.last_force_torque = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [fx, fy, fz, tx, ty, tz]
        self.force_torque_zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Calibrated zero point
        self.force_torque_calibrated = False
        self.force_torque_alerts_active = False
        self.last_alert_time = 0

        # Motion state tracking
        self._motion_in_progress = False

        # State tracking
        self.alive = True
        self._ignore_exit_state = False

        #Performance tracking system
        self.performance_metrics = create_default_performance_metrics()

        self.performance_thresholds = DEFAULT_PERFORMANCE_THRESHOLDS.copy()

        # Predictive maintenance monitoring
        self.temperature_history = deque(maxlen=100)
        self.torque_history = deque(maxlen=100)
        self.current_history = deque(maxlen=100)

        self.temperature_thresholds = self.safety_config.get('temperature_limits', DEFAULT_TEMPERATURE_THRESHOLDS)

        # Real-time monitoring thread
        self._monitoring_thread = None
        self._monitoring_active = False

        # Callback system for event-driven programming
        self._callbacks = {
            'state_changed': [],
            'error_occurred': [],
            'motion_complete': [],
            'safety_violation': [],
            'maintenance_alert': []
        }

        # Joint limits for different models (degrees)
        self.joint_limits = get_joint_limits_for_model(self.model)

    def _initialize_safety_systems(self):
        """Initialize safety validation systems."""
        # First, validate and clamp the loaded safety config against hardware limits
        self.safety_config = validate_and_apply_safety_config(self.safety_config)

        # Safety boundaries from the now-validated config
        self.safety_boundaries = self.safety_config.get('workspace_limits', DEFAULT_SAFETY_BOUNDARIES)

        # Speed limits based on safety level using utility function
        self.max_tcp_speed, self.max_joint_speed = get_safety_speed_limits(
            self.safety_level,
            self.safety_config.get('max_tcp_speed', 1000),
            self.safety_config.get('max_joint_speed', 180)
        )

        # Collision detection sensitivity from the now-validated config
        self.collision_sensitivity = self.safety_config.get('collision_sensitivity', DEFAULT_COLLISION_SENSITIVITY)

    def _initialize_error_recovery(self):
        """Initialize intelligent error recovery system."""
        # Error recovery strategies for common error codes
        self.error_recovery_strategies = {
            # Collision errors
            31: self._handle_collision_error,
            # Joint limit errors
            23: self._handle_joint_limit_error,
            38: self._handle_hard_joint_limit_error,
            # Speed limit errors
            24: self._handle_joint_speed_error,
            60: self._handle_tcp_speed_error,
            # Communication errors
            1: self._handle_communication_error,
            2: self._handle_communication_error,
            # State errors
            4: self._handle_state_error
        }

        # Track recovery attempts to prevent infinite loops
        self.recovery_attempts = {}
        self.max_recovery_attempts = 3

    def _setup_movement_parameters(self):
        """Setup and validate movement parameters with safety limits."""
        # Basic movement parameters with validation
        raw_tcp_speed = self.xarm_config.get('tcp_speed', 100)
        raw_tcp_acc = self.xarm_config.get('tcp_acc', 2000)
        raw_angle_speed = self.xarm_config.get('angle_speed', 20)
        raw_angle_acc = self.xarm_config.get('angle_acc', 500)

        # Apply safety limits using utility function
        self.tcp_speed, self.tcp_acc, self.angle_speed, self.angle_acc = apply_movement_parameter_limits(
            raw_tcp_speed, raw_tcp_acc, raw_angle_speed, raw_angle_acc,
            self.max_tcp_speed, self.max_joint_speed
        )

        # Log if parameters were limited for safety
        if raw_tcp_speed != self.tcp_speed:
            print(f"TCP speed limited from {raw_tcp_speed} to {self.tcp_speed} for safety")
        if raw_angle_speed != self.angle_speed:
            print(f"Joint speed limited from {raw_angle_speed} to {self.angle_speed} for safety")

    def _create_simulation_arm(self):
        """Create a mock arm object for simulation mode."""
        class SimulationArm:
            def __init__(self, controller):
                self.controller = controller
                self.connected = True
                self.error_code = 0
                self.warn_code = 0
                self.state = 0
                self.mode = 0

            # Basic connection methods
            def connect(self):
                self.connected = True
                return 0

            def disconnect(self):
                self.connected = False
                return 0

            # Error and warning management
            def clean_error(self):
                self.error_code = 0
                return 0

            def clean_warn(self):
                self.warn_code = 0
                return 0

            # Motion enable and state management
            def motion_enable(self, enable=True):
                return 0

            def set_mode(self, mode):
                self.mode = mode
                return 0

            def set_state(self, state):
                self.state = state
                return 0

            # Callback registration (no-op in simulation)
            def register_error_warn_changed_callback(self, callback):
                return 0

            def register_state_changed_callback(self, callback):
                return 0

            # Position and joint methods
            def get_position(self):
                return [0, self.controller.last_position]

            def get_servo_angle(self):
                return [0, self.controller.last_joints]

            def set_servo_angle(self, angle, speed=None, mvacc=None, wait=True):
                # Basic collision detection in simulation
                if len(angle) > self.controller.num_joints:
                    return 19  # Invalid parameter
                self.controller.last_joints = angle[:self.controller.num_joints]
                return 0

            def set_position(self, x, y, z, roll, pitch, yaw, speed=None, mvacc=None, wait=True, relative=False, motion_type=0):
                # Handle relative movement
                if relative:
                    current = self.controller.last_position
                    x += current[0]
                    y += current[1]
                    z += current[2]
                    roll += current[3]
                    pitch += current[4]
                    yaw += current[5]

                self.controller.last_position = [x, y, z, roll, pitch, yaw]
                return 0

            def set_only_check_type(self, check_type):
                """Simulation of collision checking mode."""
                return 0

            # Velocity control methods
            def vc_set_cartesian_velocity(self, velocities):
                return 0

            def vc_set_joint_velocity(self, velocities):
                return 0

            # Emergency stop
            def emergency_stop(self):
                return 0

            # Home position
            def move_gohome(self, speed=None, mvacc=None, wait=True):
                self.controller.last_joints = [0] * self.controller.num_joints
                self.controller.last_position = [300, 0, 300, 180, 0, 0]
                return 0

            # Bio gripper methods
            def set_bio_gripper_enable(self, enable):
                return 0

            def open_bio_gripper(self, speed=None, wait=True):
                return 0

            def close_bio_gripper(self, speed=None, wait=True):
                return 0

            def set_bio_gripper_g2_position(self, pos, speed=2000, force=100, wait=True, timeout=5, **kwargs):
                self.controller.last_gripper_position = pos
                return 0

            def set_bio_gripper_force(self, force):
                self.controller.last_gripper_force = force
                return 0

            def set_bio_gripper_speed(self, speed):
                self.controller.last_gripper_speed = speed
                return 0

            def get_bio_gripper_g2_position(self, **kwargs):
                return [0, getattr(self.controller, 'last_gripper_position', 0)]

            # Standard gripper methods
            def set_gripper_enable(self, enable):
                return 0

            def set_gripper_position(self, pos, speed=None, wait=True):
                return 0

            # RobotIQ gripper methods
            def robotiq_reset(self):
                return 0

            def robotiq_set_activate(self, activate):
                return 0

            def robotiq_set_position(self, position, speed=None, force=None, wait=True):
                return 0

            def robotiq_open(self, wait=True):
                return 0

            def robotiq_close(self, wait=True):
                return 0

            # Linear track methods
            def set_linear_track_enable(self, enable):
                return 0

            def set_linear_track_speed(self, speed):
                return 0

            def set_linear_track_pos(self, pos, speed=None, wait=True):
                self.controller.last_track_position = pos
                return 0

            def get_linear_track_pos(self):
                return [0, self.controller.last_track_position]

            # Force torque sensor methods
            def ft_sensor_enable(self, enable):
                return 0

            def get_ft_sensor_data(self):
                # Return simulated force torque data
                # In simulation, return small random values
                import random
                simulated_data = [random.uniform(-1, 1) for _ in range(6)]
                return [0, *simulated_data]

        return SimulationArm(self)

    # =============================================================================
    # PERFORMANCE TRACKING AND MONITORING
    # =============================================================================

    def _start_monitoring_thread(self):
        """Start background monitoring thread for performance tracking."""
        if self.simulation_mode:
            return  # Skip monitoring in simulation mode

        self._monitoring_active = True
        self._monitoring_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
        self._monitoring_thread.start()
        print("Performance monitoring thread started")

    def _monitoring_loop(self):
        """Background monitoring loop for predictive maintenance and performance tracking."""
        while self._monitoring_active:
            try:
                self._check_predictive_maintenance()
                self._monitor_performance_metrics()
                self._check_performance_thresholds()
                time.sleep(0.1)  # 10Hz monitoring
            except Exception as e:
                print(f"Monitoring error: {e}")
                time.sleep(1)  # Slower retry on error

    def _check_predictive_maintenance(self):
        """Check for predictive maintenance indicators."""
        if not self.arm or self.simulation_mode:
            return

        try:
            # Check temperatures
            temps = getattr(self.arm, 'temperatures', None)
            if temps:
                self.temperature_history.append({
                    'timestamp': time.time(),
                    'temperatures': temps
                })

                # Check temperature thresholds
                for i, temp in enumerate(temps):
                    if temp > self.temperature_thresholds['critical']:
                        self._trigger_maintenance_alert('temperature_critical', {
                            'joint': i + 1,
                            'temperature': temp,
                            'threshold': self.temperature_thresholds['critical']
                        })
                    elif temp > self.temperature_thresholds['warning']:
                        self._trigger_maintenance_alert('temperature_warning', {
                            'joint': i + 1,
                            'temperature': temp,
                            'threshold': self.temperature_thresholds['warning']
                        })

            # Check joint torques
            torques = getattr(self.arm, 'joints_torque', None)
            if torques:
                self.torque_history.append({
                    'timestamp': time.time(),
                    'torques': torques
                })
                self._analyze_torque_trends()

            # Check current consumption
            currents = getattr(self.arm, 'currents', None)
            if currents:
                self.current_history.append({
                    'timestamp': time.time(),
                    'currents': currents
                })
                self._analyze_current_trends()

        except Exception as e:
            print(f"Predictive maintenance check error: {e}")

    def _monitor_performance_metrics(self):
        """Monitor and update performance metrics."""
        if not self.arm or self.simulation_mode:
            return

        try:
            # Calculate TCP utilization (simplified)
            current_pos = getattr(self.arm, 'position', self.last_position)
            if current_pos and isinstance(current_pos, (list, tuple)) and len(current_pos) >= 3:
                # Simple utilization based on movement
                tcp_util = min(100, abs(sum(current_pos[:3]) / 3000 * 100))
                self.performance_metrics['tcp_utilization'].append(tcp_util)

            # Calculate joint utilization
            current_joints = getattr(self.arm, 'angles', self.last_joints)
            if current_joints and isinstance(current_joints, (list, tuple)) and len(current_joints) > 0:
                try:
                    # Safely convert to numeric values
                    numeric_joints = [float(angle) for angle in current_joints if isinstance(angle, (int, float))]
                    if numeric_joints:
                        joint_util = min(100, sum(abs(angle) for angle in numeric_joints) / len(numeric_joints))
                        self.performance_metrics['joint_utilization'].append(joint_util)
                except (ValueError, TypeError):
                    pass  # Skip if conversion fails

        except Exception as e:
            print(f"Performance monitoring error: {e}")

    def _check_performance_thresholds(self):
        """Check if performance metrics exceed thresholds."""
        try:
            # Check cycle times
            if self.performance_metrics['cycle_times']:
                avg_cycle_time = sum(self.performance_metrics['cycle_times']) / len(self.performance_metrics['cycle_times'])
                if avg_cycle_time > self.performance_thresholds['max_cycle_time']:
                    self._trigger_maintenance_alert('performance_cycle_time', {
                        'average_cycle_time': avg_cycle_time,
                        'threshold': self.performance_thresholds['max_cycle_time']
                    })

            # Check utilization
            if self.performance_metrics['tcp_utilization']:
                avg_tcp_util = sum(self.performance_metrics['tcp_utilization']) / len(self.performance_metrics['tcp_utilization'])
                if avg_tcp_util > self.performance_thresholds['max_utilization']:
                    self._trigger_maintenance_alert('performance_tcp_utilization', {
                        'average_utilization': avg_tcp_util,
                        'threshold': self.performance_thresholds['max_utilization']
                    })

        except Exception as e:
            print(f"Performance threshold check error: {e}")

    def _analyze_torque_trends(self):
        """Analyze torque trends for predictive maintenance."""
        if len(self.torque_history) < 10:
            return

        try:
            # Simple trend analysis - could be enhanced with ML
            recent_torques = list(self.torque_history)[-10:]

            for joint_idx in range(self.num_joints):
                if joint_idx < len(recent_torques[0]['torques']):
                    torque_values = [entry['torques'][joint_idx] for entry in recent_torques]
                    avg_torque = sum(torque_values) / len(torque_values)

                    # Check for abnormal torque levels (simplified)
                    normal_torque_threshold = 50  # This should be calibrated per joint
                    if avg_torque > normal_torque_threshold:
                        self._trigger_maintenance_alert('torque_high', {
                            'joint': joint_idx + 1,
                            'average_torque': avg_torque,
                            'threshold': normal_torque_threshold
                        })

        except Exception as e:
            print(f"Torque analysis error: {e}")

    def _analyze_current_trends(self):
        """Analyze current consumption trends."""
        if len(self.current_history) < 10:
            return

        try:
            recent_currents = list(self.current_history)[-10:]

            for joint_idx in range(self.num_joints):
                if joint_idx < len(recent_currents[0]['currents']):
                    current_values = [entry['currents'][joint_idx] for entry in recent_currents]
                    avg_current = sum(current_values) / len(current_values)

                    # Check for abnormal current levels
                    normal_current_threshold = 2.0  # Amperes - should be calibrated
                    if avg_current > normal_current_threshold:
                        self._trigger_maintenance_alert('current_high', {
                            'joint': joint_idx + 1,
                            'average_current': avg_current,
                            'threshold': normal_current_threshold
                        })

        except Exception as e:
            print(f"Current analysis error: {e}")

    def _trigger_maintenance_alert(self, alert_type, data):
        """Trigger maintenance alert with rate limiting to prevent spam."""
        current_time = time.time()
        
        # Rate limiting: only show same alert type once every 60 seconds
        if not hasattr(self, '_last_alert_times'):
            self._last_alert_times = {}
        
        last_alert_time = self._last_alert_times.get(alert_type, 0)
        if current_time - last_alert_time < 60.0:  # 60 second cooldown
            return  # Skip this alert to prevent spam
        
        self._last_alert_times[alert_type] = current_time
        
        alert = {
            'timestamp': current_time,
            'type': alert_type,
            'data': data,
            'severity': 'warning' if 'warning' in alert_type else 'critical'
        }

        print(f"Maintenance Alert: {alert_type} - {data}")

        # Store in error history with maintenance type
        maintenance_error = {
            'timestamp': time.time(),
            'type': 'maintenance',
            'alert_type': alert_type,
            'data': data,
            'severity': alert['severity']
        }
        self.error_history.append(maintenance_error)

        # Trigger maintenance callbacks
        for callback in self._callbacks.get('maintenance_alert', []):
            try:
                callback(alert)
            except Exception as e:
                print(f"Maintenance callback error: {e}")

    def stop_monitoring(self):
        """Stop the monitoring thread."""
        if self._monitoring_active:
            self._monitoring_active = False
            if self._monitoring_thread and self._monitoring_thread.is_alive():
                self._monitoring_thread.join(timeout=1)
            print("Performance monitoring stopped")

    # =============================================================================
    # COLLISION DETECTION AND VALIDATION
    # =============================================================================

    def _check_joint_collision(self, angles):
        """Joint collision checking for simulation mode."""
        if self.simulation_mode:
            # Use utility function for simulation collision check
            collision_detected = check_joint_collision_simulation(angles, self.joint_limits)
            if collision_detected:
                print(f"Joint collision detected at angles: {angles}")
            return collision_detected
        else:
            # For hardware, we rely on the robot's own controller for collision detection,
            # which is active by default. Pre-flight checks here are redundant and can
            # cause issues with simulators that don't provide a serial number.
            return False

    def _check_workspace_collision(self, pose):
        """Workspace collision checking for simulation mode."""
        if not self.simulation_mode:
            return False

        # Check workspace boundaries
        if not self._validate_target_position(pose):
            return True

        # Use utility function for collision zone checking
        collision_zones = self.safety_config.get('collision_zones', [])
        collision_detected, zone_name = check_workspace_collision_simulation(pose, collision_zones)

        if collision_detected:
            print(f"Collision detected with {zone_name} at position [{pose[0]}, {pose[1]}, {pose[2]}]")

        return collision_detected

    def get_performance_metrics(self):
        """Get current performance metrics summary."""
        metrics = {}

        for metric_type, data in self.performance_metrics.items():
            if data:
                metrics[metric_type] = {
                    'current': data[-1] if data else 0,
                    'average': sum(data) / len(data),
                    'min': min(data),
                    'max': max(data),
                    'count': len(data)
                }
            else:
                metrics[metric_type] = {
                    'current': 0, 'average': 0, 'min': 0, 'max': 0, 'count': 0
                }

        return metrics

    def get_maintenance_status(self):
        """Get predictive maintenance status."""
        status = {
            'temperature': {'status': 'normal', 'alerts': []},
            'torque': {'status': 'normal', 'alerts': []},
            'current': {'status': 'normal', 'alerts': []},
            'overall_health': 'good'
        }

        # Check recent maintenance alerts
        recent_alerts = [error for error in list(self.error_history)[-50:]
                        if error.get('type') == 'maintenance']

        for alert in recent_alerts:
            alert_type = alert.get('alert_type', '')
            if 'temperature' in alert_type:
                status['temperature']['alerts'].append(alert)
                if alert.get('severity') == 'critical':
                    status['temperature']['status'] = 'critical'
                elif status['temperature']['status'] == 'normal':
                    status['temperature']['status'] = 'warning'
            elif 'torque' in alert_type:
                status['torque']['alerts'].append(alert)
                if alert.get('severity') == 'critical':
                    status['torque']['status'] = 'critical'
                elif status['torque']['status'] == 'normal':
                    status['torque']['status'] = 'warning'
            elif 'current' in alert_type:
                status['current']['alerts'].append(alert)
                if alert.get('severity') == 'critical':
                    status['current']['status'] = 'critical'
                elif status['current']['status'] == 'normal':
                    status['current']['status'] = 'warning'

        # Determine overall health
        if any(comp['status'] == 'critical' for comp in [status['temperature'], status['torque'], status['current']]):
            status['overall_health'] = 'critical'
        elif any(comp['status'] == 'warning' for comp in [status['temperature'], status['torque'], status['current']]):
            status['overall_health'] = 'warning'

        return status

    # =============================================================================
    # COLLISION DETECTION AND VALIDATION
    # =============================================================================

    def initialize(self):
        """
        Initializes the connection to the xArm, enables components, and starts monitoring.
        This method is now idempotent.
        """
        # Idempotency check: if already enabled, do nothing.
        if self.states['connection'] == ComponentState.ENABLED:
            print("Controller is already initialized.")
            return True

        self.states['connection'] = ComponentState.ENABLING

        if self.simulation_mode:
            print("Initializing Robot Arm (Simulation)...")
            self.states['connection'] = ComponentState.ENABLED
            self.states['arm'] = ComponentState.ENABLED
            print("Simulation mode: Skipping hardware connection")
            if self.auto_enable:
                self.enable_gripper_component()
                self.enable_track_component()
                print("Simulation mode: Skipping motion enable and callbacks")
            print("Simulation xArm Controller Initialized")
            return True

        print("Initializing Robot Arm (Hardware)...")
        # Add connection retry logic, especially for Docker containers
        max_retries = 3
        retry_delay = 2  # seconds
        for attempt in range(max_retries):
            try:
                # Connect to the arm
                code = self.arm.connect()
                if self.check_code(code, "connect"):
                    # Connection successful, proceed with initialization
                    # Enable motion and set mode/state
                    enable_code = self.arm.motion_enable(enable=True)
                    # Error code 3 is often "already enabled" or similar non-critical issue
                    if enable_code == 3:
                        print("Warning: motion_enable returned code 3 (likely already enabled)")
                        # Don't treat this as a fatal error - motion is already enabled
                    elif enable_code not in [None, 0]:
                        print(f"Warning: motion_enable returned code {enable_code}, continuing with initialization")
                        # For other non-zero codes, check if they're critical
                        if enable_code > 10:  # Only treat serious errors as fatal
                            if not self.check_code(enable_code, "motion_enable"):
                                continue  # Retry the connection attempt
                    
                    self.arm.set_mode(0)
                    self.arm.set_state(0)
                    time.sleep(1)

                    # Register callbacks for monitoring
                    self.arm.register_error_warn_changed_callback(self._error_warn_callback)
                    self.arm.register_state_changed_callback(self._state_changed_callback)

                    self.states['connection'] = ComponentState.ENABLED
                    self.states['arm'] = ComponentState.ENABLED

                    # Reset alive state to True after successful initialization
                    # This ensures minor errors during init don't permanently disable the controller
                    self.alive = True

                    # Auto-enable components if requested
                    if self.auto_enable:
                        if self.gripper_type != 'none':
                            self.enable_gripper_component()

                        if self.enable_track:
                            self.enable_track_component()

                    # Start monitoring thread for Phase 2 performance tracking
                    if not self.simulation_mode:
                        self._start_monitoring_thread()

                    print(f"{'Simulation' if self.simulation_mode else 'Hardware'} xArm Controller Initialized")
                    self._update_positions()
                    if self.enable_track:
                        self._update_track_position()
                    return True
                else:
                    # Connection failed, log and retry
                    print(f"Connection attempt {attempt + 1} failed. Retrying in {retry_delay}s...")
                    time.sleep(retry_delay)

            except Exception as e:
                print(f"Exception during connection attempt {attempt + 1}: {e}")
                time.sleep(retry_delay)

        # If all retries fail
        self.states['connection'] = ComponentState.ERROR
        print("Failed to connect to xArm after multiple retries.")
        return False

    def enable_gripper_component(self):
        """Enable the gripper component based on configured type."""
        if self.gripper_type == 'none':
            print("No gripper configured")
            return False

        try:
            self.states['gripper'] = ComponentState.ENABLING
            success = False

            if self.simulation_mode:
                # In simulation mode, assume gripper is always available
                success = True
                print(f"Simulation mode: {self.gripper_type.title()} gripper enabled (simulated)")
            else:
                if self.gripper_type in ('bio', 'bio_gen2'):
                    success = self._enable_bio_gripper_internal()
                elif self.gripper_type == 'standard':
                    success = self._enable_standard_gripper_internal()
                elif self.gripper_type == 'robotiq':
                    success = self._initialize_robotiq_gripper_internal()

            if success:
                self.states['gripper'] = ComponentState.ENABLED
                if not self.simulation_mode:
                    print(f"{self.gripper_type.title()} gripper enabled")
            else:
                self.states['gripper'] = ComponentState.ERROR
                print(f"Failed to enable {self.gripper_type} gripper")

            return success

        except Exception as e:
            self.states['gripper'] = ComponentState.ERROR
            print(f"Error enabling {self.gripper_type} gripper: {e}")
            return False

    def enable_track_component(self):
        """Enable the linear track component."""
        if not self.enable_track:
            print("Linear track disabled")
            return False

        try:
            self.states['track'] = ComponentState.ENABLING

            if self.simulation_mode:
                # In simulation mode, assume track is always available
                success = True
                print("Simulation mode: Linear track enabled (simulated)")
            else:
                success = self.enable_linear_track()

            if success:
                self.states['track'] = ComponentState.ENABLED
                if not self.simulation_mode:
                    print("Linear track enabled")
                    self._update_track_position()
            else:
                self.states['track'] = ComponentState.ERROR
                print("Failed to enable linear track")

            return success

        except Exception as e:
            self.states['track'] = ComponentState.ERROR
            print(f"Error enabling linear track: {e}")
            return False

    def disable_gripper_component(self):
        """Disable the gripper component."""
        if self.gripper_type == 'none':
            return True

        try:
            # Different grippers have different disable methods
            if self.gripper_type in ('bio', 'bio_gen2'):
                code = self.arm.set_bio_gripper_enable(False)
            elif self.gripper_type == 'standard':
                code = self.arm.set_gripper_enable(False)
            elif self.gripper_type == 'robotiq':
                code = self.arm.robotiq_set_activate(False)

            if code == 0:
                self.states['gripper'] = ComponentState.DISABLED
                print(f"{self.gripper_type.title()} gripper disabled")
                return True
            else:
                self.states['gripper'] = ComponentState.ERROR
                return False

        except Exception as e:
            self.states['gripper'] = ComponentState.ERROR
            print(f"Error disabling {self.gripper_type} gripper: {e}")
            return False

    def disable_track_component(self):
        """Disable the linear track component."""
        if not self.enable_track:
            return True

        try:
            result = self.arm.set_linear_track_enable(False)
            # Handle both single code and tuple return values
            code = result[0] if isinstance(result, (tuple, list)) else result
            if code == 0:
                self.states['track'] = ComponentState.DISABLED
                print("Linear track disabled")
                return True
            else:
                self.states['track'] = ComponentState.ERROR
                return False

        except Exception as e:
            self.states['track'] = ComponentState.ERROR
            print(f"Error disabling linear track: {e}")
            return False

    def _update_positions(self):
        """Update cached position information."""
        if not self.arm:
            return

        try:
            if self.simulation_mode:
                # In simulation mode, use default positions
                self.last_position = [300, 0, 300, 180, 0, 0]  # Default Cartesian position
                self.last_joints = [0] * self.num_joints  # Default joint angles
                return

            # Update Cartesian position
            ret = self.arm.get_position()
            if ret[0] == 0:
                self.last_position = ret[1:]

            # Update joint angles
            ret = self.arm.get_servo_angle()
            if ret[0] == 0:
                self.last_joints = ret[1]

        except Exception as e:
            print(f"Warning: Failed to update positions: {e}")

    def _update_track_position(self):
        """Update cached track position."""
        if not self.arm or not self.enable_track or self.states['track'] != ComponentState.ENABLED:
            return

        try:
            ret = self.arm.get_linear_track_pos()
            if ret[0] == 0:
                self.last_track_position = ret[1]
        except Exception as e:
            print(f"Warning: Failed to update track position: {e}")

    def _error_warn_callback(self, data):
        """Callback for error/warning changes with automatic recovery."""
        if data:
            if data.get('error_code', 0) != 0:
                error_code = data['error_code']
                self.last_error_code = error_code

                # Log error to history
                error_info = {
                    'timestamp': time.time(),
                    'error_code': error_code,
                    'warn_code': data.get('warn_code', 0)
                }
                self.error_history.append(error_info)

                # Trigger error callbacks
                self._trigger_callbacks('error_occurred', error_info)

                # Attempt automatic recovery
                recovery_success = self._handle_error_with_recovery(error_code)

                if not recovery_success:
                    # If recovery failed, set error state
                    self.alive = False
                    self.states['arm'] = ComponentState.ERROR
                    print(f'Error {error_code} detected, automatic recovery failed')
                else:
                    print(f'Error {error_code} detected and automatically recovered')

            if data.get('warn_code', 0) != 0:
                self.last_warn_code = data['warn_code']
                print(f'Warning: {data["warn_code"]}')

    def _state_changed_callback(self, data):
        """Callback for state changes."""
        if not self._ignore_exit_state and data and data['state'] == 4:
            self.alive = False
            self.states['arm'] = ComponentState.ERROR
            # Trigger state change callbacks
            self._trigger_callbacks('state_changed', {
                'old_state': self.states['arm'],
                'new_state': ComponentState.ERROR,
                'reason': 'Emergency state detected'
            })
            print('State 4 detected, stopping operations')

    def unregister_callback(self, event_type: str, callback: Callable):
        """Unregister a callback."""
        if event_type in self._callbacks and callback in self._callbacks[event_type]:
            self._callbacks[event_type].remove(callback)

    def _trigger_callbacks(self, event_type: str, data: Any = None):
        """Trigger all callbacks for a specific event type."""
        for callback in self._callbacks.get(event_type, []):
            try:
                callback(data)
            except Exception as e:
                print(f"Error in callback for {event_type}: {e}")

    def check_code(self, code, operation_name):
        """Check if an operation was successful."""
        # For xArm SDK, None or 0 typically indicates success
        # Some operations (like connect) return None on success
        is_success = (code is None or code == 0)
        
        if not self.is_alive or not is_success:
            self.alive = False
            state = self.arm.state if self.arm else None
            error = self.arm.error_code if self.arm else None
            return check_operation_result(code, operation_name, state, error, self.simulation_mode)
        return True

    @property
    def is_alive(self):
        """Check if the robot is in a safe operating state."""
        if self.simulation_mode:
            # In simulation mode, always return True if initialized
            return self.alive and self.arm is not None

        if self.alive and self.arm and self.arm.connected:
            # For Docker simulator, be more lenient with error codes
            is_docker = self.profile_name and 'docker' in self.profile_name.lower()
            
            if is_docker:
                # Docker simulator can have minor errors but still be functional
                # Check if we're in a critical error state (> 10 are usually serious)
                if hasattr(self.arm, 'error_code') and self.arm.error_code > 10:
                    return False
            else:
                # For real hardware, be stricter about error codes
                if hasattr(self.arm, 'error_code') and self.arm.error_code != 0:
                    return False
            
            if self._ignore_exit_state:
                return True
            if hasattr(self.arm, 'state') and self.arm.state == 5:
                cnt = 0
                while self.arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return not hasattr(self.arm, 'state') or self.arm.state < 4
        return False

    # =============================================================================
    # STATE MONITORING METHODS
    # =============================================================================

    def get_system_status(self):
        """Get comprehensive system status."""
        self._update_positions()
        if self.enable_track:
            self._update_track_position()

        return {
            'timestamp': time.time(),
            'connection': {
                'connected': self.arm.connected if self.arm else False,
                'state': self.states['connection'].value,
                'alive': self.is_alive
            },
            'arm': {
                'state': self.states['arm'].value,
                'mode': getattr(self.arm, 'mode', None) if self.arm else None,
                'robot_state': getattr(self.arm, 'state', None) if self.arm else None,
                'position': self.last_position,
                'joints': self.last_joints,
                'error_code': self.arm.error_code if self.arm else 0,
                'warn_code': getattr(self.arm, 'warn_code', 0) if self.arm else 0
            },
            'gripper': {
                'type': self.gripper_type,
                'state': self.states['gripper'].value,
                'has_gripper': self.has_gripper()
            },
            'track': {
                'state': self.states['track'].value,
                'has_track': self.has_track(),
                'position': self.last_track_position
            },
            'force_torque': {
                'state': self.states['force_torque'].value,
                'has_sensor': self.has_force_torque_sensor(),
                'calibrated': self.force_torque_calibrated,
                'last_reading': self.last_force_torque,
                'magnitude': self.get_force_torque_magnitude()
            },
            'errors': {
                'last_error': self.last_error_code,
                'last_warning': self.last_warn_code,
                'error_count': len(self.error_history)
            }
        }

    def get_component_states(self):
        """Get just the component states."""
        return {k: v.value for k, v in self.states.items()}

    def is_component_enabled(self, component):
        """Check if a specific component is enabled."""
        return self.states.get(component, ComponentState.UNKNOWN) == ComponentState.ENABLED

    def get_error_history(self, count=10):
        """Get recent error history."""
        return list(self.error_history)[-count:] if self.error_history else []

    def clear_errors(self):
        """
        Clear all robot errors and reset error states.
        This includes clearing xArm SDK errors, warnings, and controller error history.
        """
        if not self.arm:
            print("Cannot clear errors: No arm connection")
            return False

        try:
            if self.simulation_mode:
                print("[SIM] Clearing all errors and warnings")
                self.error_history.clear()
                self.last_error_code = 0
                self.last_warn_code = 0
                self.alive = True
                return True

            # Clear xArm SDK errors and warnings
            print("Clearing robot errors and warnings...")

            # Clear errors and warnings
            error_clear_code = self.arm.clean_error()
            warn_clear_code = self.arm.clean_warn()

            # Reset error tracking
            self.error_history.clear()
            self.last_error_code = 0
            self.last_warn_code = 0

            # Reset alive state if errors were cleared successfully
            if error_clear_code == 0 and warn_clear_code == 0:
                self.alive = True
                print("[OK] All errors and warnings cleared successfully")

                # Check if we need to re-enable components
                if self.auto_enable:
                    print("Re-enabling components...")
                    if self.states['arm'] == ComponentState.ERROR:
                        self.states['arm'] = ComponentState.ENABLED
                    if self.has_gripper() and self.states['gripper'] == ComponentState.ERROR:
                        self.enable_gripper_component()
                    if self.has_track() and self.states['track'] == ComponentState.ERROR:
                        self.enable_track_component()

                return True
            else:
                print(f"[WARN] Error clearing partially failed: error_clear={error_clear_code}, warn_clear={warn_clear_code}")
                return False

        except Exception as e:
            print(f"[ERROR] Failed to clear errors: {e}")
            return False

    # =============================================================================
    # LINEAR/CARTESIAN MOVEMENTS
    # =============================================================================

    def move_to_position(self, x, y, z, roll=None, pitch=None, yaw=None,
                        speed=None, check_collision=True, motion_type=0, wait=True):
        """
        Move to a Cartesian position with collision checking and intelligent planning.

        Args:
            x, y, z: Target position coordinates
            roll, pitch, yaw: Target orientation (defaults: 180, 0, 0)
            speed: Movement speed (default: tcp_speed)
            check_collision: Enable collision detection and validation
            motion_type: Motion planning type (0=default, 1=alternative)
            wait: Wait for movement completion

        Returns:
            bool: True if movement successful, False otherwise
        """
        if not self.arm:
            print("Error: Arm is not initialized. Cannot perform movement.")
            return False

        if not self.is_component_enabled('arm'):
            print("Warning: Arm is not enabled. Cannot perform movement.")
            return False

        # Set defaults
        if roll is None: roll = 180
        if pitch is None: pitch = 0
        if yaw is None: yaw = 0
        if speed is None: speed = self.tcp_speed

        target_pos = [x, y, z, roll, pitch, yaw]

        # Pre-motion validation
        if not self._validate_target_position(target_pos):
                return False

        # Collision checking
        if check_collision and not self.simulation_mode:
            # Use SDK's built-in collision checking
            self.arm.set_only_check_type(1)  # Check without moving
            check_code = self.arm.set_position(x, y, z, roll, pitch, yaw, speed=speed)
            self.arm.set_only_check_type(0)  # Reset to normal mode

            if check_code != 0:
                error_details = getattr(self.arm, 'only_check_result', None)
                print(f"Motion planning failed: code={check_code}, details={error_details}")

                # Try alternative motion planning
                if motion_type == 0:
                    print("Trying alternative motion planning (motion_type=1)")
                    return self.move_to_position(x, y, z, roll, pitch, yaw,
                                                speed, check_collision, motion_type=1, wait=wait)
                else:
                    print("Alternative motion planning also failed")
                    return False

        # Execute the movement with performance tracking
        self._motion_in_progress = True
        start_time = time.time()

        try:
            if self.simulation_mode:
                if self._check_workspace_collision(target_pos):
                    return False
                print(f"[SIM] Moved to position {target_pos}")
                self.last_position = target_pos
                success = True
            else:
                code = self.arm.set_position(x, y, z, roll, pitch, yaw,
                                           speed=speed, wait=wait, motion_type=motion_type)
                success = self.check_code(code, f'move_to_position({x}, {y}, {z})')

            # Track performance metrics
            cycle_time = time.time() - start_time
            self.performance_metrics['cycle_times'].append(cycle_time)

            # Track command success rate
            self.performance_metrics['command_success_rate'].append(1.0 if success else 0.0)

            if success:
                self._update_positions()
                # Calculate and track accuracy error
                if not self.simulation_mode:
                    actual_pos = self.get_current_position()
                    if actual_pos:
                        accuracy_error = ((actual_pos[0] - x)**2 + (actual_pos[1] - y)**2 + (actual_pos[2] - z)**2)**0.5
                        self.performance_metrics['accuracy_errors'].append(accuracy_error)

            return success

        finally:
            self._motion_in_progress = False

    def move_to_named_location(self, location_name, speed=None):
        """
        Move to a predefined location from the position config.
        Supports both joint-based and Cartesian-based location definitions.
        """
        # Check if positions are defined in config
        if 'positions' not in self.position_config:
            print(f"Error: No 'positions' section found in position config")
            return False

        if location_name not in self.position_config['positions']:
            print(f"Error: Location '{location_name}' not found in config")
            return False

        location = self.position_config['positions'][location_name]

        # Detect format: list = joint angles, dict = Cartesian coordinates
        if isinstance(location, list):
            # Joint-based location (e.g., [0.0, 0.0, 0.0, 0.0, 0.0])
            # Ensure we have enough joint angles for the model
            angles = list(location)
            while len(angles) < self.num_joints:
                angles.append(0.0)  # Pad with zeros for missing joints

            print(f"Moving to location '{location_name}' using joint angles: {angles[:self.num_joints]}")
            return self.move_joints(angles=angles[:self.num_joints], speed=speed)
        elif isinstance(location, dict):
            # Cartesian-based location (e.g., {x: 300, y: 0, z: 300, ...})
            print(f"Moving to location '{location_name}' using Cartesian coordinates")
            return self.move_to_position(
                x=location['x'], y=location['y'], z=location['z'],
                roll=location.get('roll'), pitch=location.get('pitch'), yaw=location.get('yaw'),
                speed=speed
            )
        else:
            print(f"Error: Invalid location format for '{location_name}'. Expected list (joint angles) or dict (Cartesian coordinates)")
            return False

    def move_relative(self, dx=0.0, dy=0.0, dz=0.0, droll=0.0, dpitch=0.0, dyaw=0.0, speed=None):
        """
        Move relative to current position (linear movement).
        """
        if not self.is_component_enabled('arm'):
            print("Arm is not enabled")
            return False

        if speed is None:
            speed = self.tcp_speed

        code = self.arm.set_position(x=dx, y=dy, z=dz, roll=droll, pitch=dpitch, yaw=dyaw,
                                   speed=speed, relative=True, wait=True)
        success = self.check_code(code, f'move_relative({dx}, {dy}, {dz})')
        if success:
            self._update_positions()
        return success

    # =============================================================================
    # JOINT MOVEMENTS
    # =============================================================================

    def move_joints(self, angles, speed=None, acceleration=None,
                   wait=True, check_collision=True):
        """
        Move individual joints to specified angles with comprehensive safety checking.

        Args:
            angles (list): Joint angles in degrees
            speed (float, optional): Joint movement speed (degrees/second)
            acceleration (float, optional): Joint acceleration (degrees/second^2)
            wait (bool): Wait for movement completion
            check_collision (bool): Enable collision detection and validation

        Returns:
            bool: True if movement successful, False otherwise
        """
        if not self.is_component_enabled('arm'):
            print("Arm is not enabled")
            return False

        if speed is None: speed = self.angle_speed
        if acceleration is None: acceleration = self.angle_acc

        # Validate joint angles
        if not self._validate_joint_angles(angles):
            return False

        # Collision checking for simulation mode
        if check_collision and self.simulation_mode:
            if self._check_joint_collision(angles):
                return False

        # Execute movement with performance tracking
        self._motion_in_progress = True
        start_time = time.time()

        try:
            if self.simulation_mode:
                print(f"[SIM] Joints moved to {angles}")
                self.last_joints = angles[:self.num_joints]  # Store only valid joints for model
                success = True
            else:
                # Workaround for Docker simulator serial number bug - disable range checking
                code = self.arm.set_servo_angle(angle=angles, speed=speed, mvacc=acceleration, wait=wait, check=False)
                success = self.check_code(code, f'move_joints({angles})')

            # Track performance metrics
            cycle_time = time.time() - start_time
            self.performance_metrics['cycle_times'].append(cycle_time)

            # Track command success rate
            self.performance_metrics['command_success_rate'].append(1.0 if success else 0.0)

            if success:
                self._update_positions()

            return success

        finally:
            self._motion_in_progress = False

    def move_single_joint(self, joint_id, angle, speed=None, wait=True):
        """
        Move a single joint while keeping others in place.
        """
        if not self.is_component_enabled('arm'):
            print("Arm is not enabled")
            return False

        # Get current joint angles
        ret = self.arm.get_servo_angle()
        if ret[0] != 0:
            print("Failed to get current joint angles")
            return False

        # Handle case where ret[1] might be a list or direct value
        current_angles = ret[1] if isinstance(ret[1], list) else list(ret[1:])
        if len(current_angles) <= joint_id:
            print(f"Invalid joint_id {joint_id} for {len(current_angles)} joints")
            return False

        current_angles[joint_id] = angle

        return self.move_joints(current_angles, speed=speed, wait=wait)

    # =============================================================================
    # VELOCITY CONTROL
    # =============================================================================

    def set_cartesian_velocity(self, vx=0.0, vy=0.0, vz=0.0, vroll=0.0, vpitch=0.0, vyaw=0.0):
        """
        Control the robot using Cartesian velocity commands.
        Useful for real-time control or jogging.
        """
        if not self.is_component_enabled('arm'):
            print("Arm is not enabled")
            return False

        code = self.arm.vc_set_cartesian_velocity([vx, vy, vz, vroll, vpitch, vyaw])
        return self.check_code(code, f'set_cartesian_velocity')

    def set_joint_velocity(self, velocities):
        """
        Control individual joints using velocity commands.
        """
        if not self.is_component_enabled('arm'):
            print("Arm is not enabled")
            return False

        code = self.arm.vc_set_joint_velocity(velocities)
        return self.check_code(code, f'set_joint_velocity')

    def stop_motion(self):
        """Stop all motion immediately."""
        code = self.arm.emergency_stop()
        return self.check_code(code, 'emergency_stop')

    # =============================================================================
    # GRIPPER CONTROL - Multiple Types Supported
    # =============================================================================

    def has_gripper(self):
        """Check if a gripper is configured."""
        return self.gripper_type != 'none'

    # Universal Gripper Methods
    def open_gripper(self, speed=None, force=None, wait=True):
        """Open the gripper (works with any configured gripper type)."""
        if not self.is_component_enabled('gripper'):
            print("Gripper is not enabled")
            return False

        if self.simulation_mode:
            print(f"[SIM] {self.gripper_type.title()} gripper opened")
            self.last_gripper_position = self._gripper_setting('open_position', 0)
            return True

        if self.gripper_type == 'bio':
            return self._open_bio_gripper_internal(speed=speed, wait=wait)
        elif self.gripper_type == 'bio_gen2':
            return self._set_bio_gripper_g2_position_internal(
                self._gripper_setting('open_position', 0),
                speed=speed,
                force=force,
                wait=wait,
            )
        elif self.gripper_type == 'standard':
            max_position = self._gripper_setting('open_position', 850)
            return self._set_gripper_position_internal(max_position, speed=speed, wait=wait)
        elif self.gripper_type == 'robotiq':
            if not self.simulation_mode:
                code = self.arm.robotiq_open(wait=wait)
                return self.check_code(code, 'open_robotiq_gripper')
            return True
        else:
            print("No gripper configured")
            return False

    def close_gripper(self, speed=None, force=None, wait=True):
        """Close the gripper (works with any configured gripper type)."""
        if not self.is_component_enabled('gripper'):
            print("Gripper is not enabled")
            return False

        if self.simulation_mode:
            print(f"[SIM] {self.gripper_type.title()} gripper closed")
            self.last_gripper_position = self._gripper_setting('close_position', 850)
            return True

        if self.gripper_type == 'bio':
            return self._close_bio_gripper_internal(speed=speed, wait=wait)
        elif self.gripper_type == 'bio_gen2':
            return self._set_bio_gripper_g2_position_internal(
                self._gripper_setting('close_position', 850),
                speed=speed,
                force=force,
                wait=wait,
            )
        elif self.gripper_type == 'standard':
            return self._set_gripper_position_internal(0, speed=speed, wait=wait)
        elif self.gripper_type == 'robotiq':
            if not self.simulation_mode:
                code = self.arm.robotiq_close(wait=wait)
                return self.check_code(code, 'close_robotiq_gripper')
            return True
        else:
            print("No gripper configured")
            return False

    def move_gripper_to_stroke(self, stroke, speed=None, force=None, wait=True):
        """Move a stroke-capable gripper to an absolute stroke position."""
        if not self.is_component_enabled('gripper'):
            print("Gripper is not enabled")
            return False

        if not self._validate_gripper_stroke(stroke):
            return False

        if self.simulation_mode:
            self.last_gripper_position = stroke
            if force is not None:
                self.last_gripper_force = force
            print(f"[SIM] {self.gripper_type.title()} gripper moved to stroke {stroke}")
            return True

        if self.gripper_type == 'bio_gen2':
            return self._set_bio_gripper_g2_position_internal(
                stroke, speed=speed, force=force, wait=wait
            )
        elif self.gripper_type == 'standard':
            return self._set_gripper_position_internal(stroke, speed=speed, wait=wait)
        elif self.gripper_type == 'robotiq':
            return self._set_robotiq_position_internal(stroke, speed=speed, force=force, wait=wait)

        print(f"{self.gripper_type} gripper does not support stroke control")
        return False

    def set_gripper_force(self, force):
        """Set gripping force for force-capable grippers."""
        if not self.is_component_enabled('gripper'):
            print("Gripper is not enabled")
            return False

        if not self._validate_gripper_force(force):
            return False

        if self.simulation_mode:
            self.last_gripper_force = force
            print(f"[SIM] {self.gripper_type.title()} gripper force set to {force}")
            return True

        if self.gripper_type == 'bio_gen2':
            force = self._coerce_int(force, 'force')
            code = self.arm.set_bio_gripper_force(force)
            return self.check_code(code, f'set_bio_gripper_force({force})')

        print(f"{self.gripper_type} gripper does not support standalone force control")
        return False

    def get_gripper_position(self):
        """Return the latest gripper position/stroke when available."""
        if self.simulation_mode:
            return self.last_gripper_position

        if self.gripper_type == 'bio_gen2' and hasattr(self.arm, 'get_bio_gripper_g2_position'):
            ret = self.arm.get_bio_gripper_g2_position()
            if ret[0] == 0:
                self.last_gripper_position = ret[1]
                return ret[1]
        elif self.gripper_type == 'standard' and hasattr(self.arm, 'get_gripper_position'):
            ret = self.arm.get_gripper_position()
            if ret[0] == 0:
                self.last_gripper_position = ret[1]
                return ret[1]
        return None

    # =============================================================================
    # LINEAR TRACK CONTROL (Optional)
    # =============================================================================

    def has_track(self):
        """Check if linear track is enabled."""
        return self.enable_track

    def _validate_gripper_stroke(self, stroke):
        """Validate a requested gripper stroke against active config limits."""
        config = getattr(self, 'current_gripper_config', {}) or {}
        if not config.get('has_stroke_control', False):
            print(f"{self.gripper_type} gripper does not support stroke control")
            return False
        stroke_range = config.get('stroke_range', {})
        min_stroke = stroke_range.get('min', 0)
        max_stroke = stroke_range.get('max', 850)
        if stroke < min_stroke or stroke > max_stroke:
            print(f"Gripper stroke {stroke} outside allowed range {min_stroke}-{max_stroke}")
            return False
        return True

    def _validate_gripper_force(self, force):
        """Validate a requested gripper force against active config limits."""
        config = getattr(self, 'current_gripper_config', {}) or {}
        if not config.get('has_force_control', False):
            print(f"{self.gripper_type} gripper does not support force control")
            return False
        force_range = config.get('force_range', {})
        min_force = force_range.get('min', 1)
        max_force = force_range.get('max', 100)
        if force < min_force or force > max_force:
            print(f"Gripper force {force} outside allowed range {min_force}-{max_force}")
            return False
        return True

    def enable_linear_track(self):
        """Enable the linear track."""
        if not self.enable_track:
            print("Linear track is disabled")
            return False
        result = self.arm.set_linear_track_enable(True)
        # Handle both single code and tuple return values
        code = result[0] if isinstance(result, (tuple, list)) else result
        return self.check_code(code, 'enable_linear_track')

    def set_track_speed(self, speed):
        """Set the linear track speed."""
        if not self.is_component_enabled('track'):
            print("Linear track is not enabled")
            return False
        result = self.arm.set_linear_track_speed(speed)
        # Handle both single code and tuple return values
        code = result[0] if isinstance(result, (tuple, list)) else result
        return self.check_code(code, 'set_linear_track_speed')

    def move_track_to_position(self, position, speed=None, wait=True):
        """Move the linear track to a specific position with validation."""
        if not self.is_component_enabled('track'):
            print("Linear track is not enabled")
            return False

        # Validation
        if not self._validate_track_position(position):
            return False

        if speed is None:
            speed = self.track_config.get('Speed', 200)

        # Validate speed
        if not self._validate_track_speed(speed):
            return False

        # Performance tracking
        self._motion_in_progress = True
        start_time = time.time()

        try:
            if self.simulation_mode:
                print(f"[SIM] Linear track moved to position {position}mm at {speed}mm/s")
                self.last_track_position = position
                success = True
            else:
                result = self.arm.set_linear_track_pos(speed=speed, pos=position, wait=wait)
                # Handle both single code and tuple return values
                code = result[0] if isinstance(result, (tuple, list)) else result
                success = self.check_code(code, f'move_track_to_position({position})')

            # Track performance metrics
            cycle_time = time.time() - start_time
            self.performance_metrics['cycle_times'].append(cycle_time)
            self.performance_metrics['command_success_rate'].append(1.0 if success else 0.0)

            if success:
                self._update_track_position()
                # Calculate accuracy for track movement
                if not self.simulation_mode:
                    actual_pos = self.get_track_position()
                    if actual_pos is not None:
                        accuracy_error = abs(actual_pos - position)
                        self.performance_metrics['accuracy_errors'].append(accuracy_error)

            return success

        finally:
            self._motion_in_progress = False

    def move_track_to_named_location(self, location_name: str, speed: Optional[float] = None, wait: bool = True):
        """
        Move the linear track to a pre-configured named location.

        Args:
            location_name (str): The name of the location from linear_track_config.yaml.
            speed (float, optional): Movement speed. Defaults to value from config.
            wait (bool): Wait for movement completion.

        Returns:
            bool: True if successful, False otherwise.
        """
        if not self.is_component_enabled('track'):
            print("Linear track is not enabled")
            return False

        if location_name not in self.track_config.get('locations', {}):
            print(f"Error: Linear track location '{location_name}' not found in configuration.")
            self.last_error = f"Track location '{location_name}' not found."
            return False

        location_data = self.track_config['locations'][location_name]
        
        # Handle both formats: simple integer position or dict with position/speed
        if isinstance(location_data, (int, float)):
            # Simple format: location_name: position_value
            position = location_data
            config_speed = None
        elif isinstance(location_data, dict):
            # Complex format: location_name: {position: value, speed: value}
            position = location_data.get('position')
            config_speed = location_data.get('speed')
        else:
            print(f"Error: Invalid location format for '{location_name}'. Expected number or dict.")
            self.last_error = f"Invalid location format for '{location_name}'."
            return False

        # Use provided speed, then config speed, then default track speed
        if speed is None:
            speed = config_speed or self.track_config.get('Speed', 200)

        if position is None:
            print(f"Error: No position defined for track location '{location_name}'.")
            self.last_error = f"No position for track location '{location_name}'."
            return False

        return self.move_track_to_position(position, speed=speed, wait=wait)

    def _validate_track_position(self, position):
        """Validation for track position."""
        is_valid, message = validate_track_position(
            position,
            (0, 700),
            self.track_config.get('danger_zones', [])
        )

        if not is_valid:
            print(f"Error: {message}")
            self._trigger_callbacks('safety_violation', {
                'type': 'track_position',
                'value': position,
                'message': message
            })
            return False

        return True

    def _validate_track_speed(self, speed):
        """Validation for track speed."""
        is_valid, message = validate_track_speed(speed, (1, 1000))

        if not is_valid:
            print(f"Error: {message}")
            self._trigger_callbacks('safety_violation', {
                'type': 'track_speed',
                'value': speed,
                'message': message
            })
            return False

        return True

    def reset_track(self):
        """Reset the linear track to home position."""
        if not self.is_component_enabled('track'):
            print("Linear track is not enabled")
            return False
        return self.move_track_to_position(0)

    def get_track_position(self):
        """Get current linear track position."""
        if not self.is_component_enabled('track'):
            print("Linear track is not enabled")
            return None

        if self.simulation_mode:
            return getattr(self, 'last_track_position', 0)

        ret = self.arm.get_linear_track_pos()
        if ret[0] == 0:
            self.last_track_position = ret[1]
            return ret[1]
        return None

    # =============================================================================
    # UTILITY METHODS
    # =============================================================================

    def get_current_position(self):
        """Get the current Cartesian position."""
        ret = self.arm.get_position()
        if ret[0] == 0:
            # ret[1] should be the position list [x, y, z, roll, pitch, yaw]
            position = ret[1] if len(ret) > 1 else ret[1:]
            self.last_position = position
            return position
        return None

    def get_current_joints(self):
        """Get the current joint angles."""
        ret = self.arm.get_servo_angle()
        if ret[0] == 0:
            # Handle case where ret[1] might be a list or direct value
            joints = ret[1] if isinstance(ret[1], list) else list(ret[1:])
            self.last_joints = joints
            return joints
        return None

    def go_home(self, speed=None, mvacc=None, wait=True):
        """Move the robot to its home position using the SDK's built-in method."""
        if not self.is_component_enabled('arm'):
            print("Arm is not enabled")
            return False
            
        if speed is None: speed = self.angle_speed
        if mvacc is None: mvacc = self.angle_acc

        # Use the SDK's dedicated go_home method
        code = self.arm.move_gohome(speed=speed, mvacc=mvacc, wait=wait)
        return self.check_code(code, 'go_home')

    def get_named_locations(self):
        """Returns a list of all named locations."""
        if self.position_config and 'positions' in self.position_config:
            return list(self.position_config['positions'].keys())
        return []

    def get_system_info(self):
        """Get information about the configured system."""
        info = {
            'model': self.model,
            'num_joints': self.num_joints,
            'gripper_type': self.gripper_type,
            'has_gripper': self.has_gripper(),
            'has_track': self.has_track(),
            'connected': self.arm.connected if self.arm else False,
            'is_alive': self.is_alive,
            'auto_enable': self.auto_enable,
            'component_states': self.get_component_states()
        }
        return info

    def get_model(self):
        """Get the robot model number."""
        return self.model

    def get_num_joints(self):
        """Get the number of joints for this robot model."""
        return self.num_joints

    def disconnect(self):
        """Disconnects from the robot arm."""
        print("Disconnecting Robot Arm...")
        self.alive = False
        self.stop_monitoring()
        self.states['connection'] = ComponentState.DISABLED
        self.states['arm'] = ComponentState.DISABLED
        if self.arm:
            try:
                self.arm.disconnect()
            except Exception as e:
                print(f"Exception during arm disconnect: {e}")
        print("Robot Arm Disconnected.")

    # Safety validation
    def _validate_target_position(self, position: List[float]) -> bool:
        """Validate target position against safety boundaries."""
        is_valid, error_msg = validate_target_position(position, self.safety_boundaries)
        if not is_valid:
            self._trigger_callbacks('safety_violation', {'type': 'validation_error', 'message': error_msg})
        return is_valid

    def _validate_joint_angles(self, angles: List[float]) -> bool:
        """Validate joint angles against model-specific limits."""
        is_valid, error_msg = validate_joint_angles(angles, self.joint_limits)
        if not is_valid:
            self._trigger_callbacks('safety_violation', {'type': 'joint_validation_error', 'message': error_msg})
        return is_valid

    # Error recovery methods
    def _handle_error_with_recovery(self, error_code: int) -> bool:
        """Handle errors with intelligent recovery strategies."""
        # Track recovery attempts
        if error_code not in self.recovery_attempts:
            self.recovery_attempts[error_code] = 0

        # Check if we've exceeded max recovery attempts
        if self.recovery_attempts[error_code] >= self.max_recovery_attempts:
            print(f"Max recovery attempts ({self.max_recovery_attempts}) exceeded for error {error_code}")
            self._trigger_callbacks('error_occurred', {
                'error_code': error_code,
                'recovery_failed': True,
                'attempts': self.recovery_attempts[error_code]
            })
            return False

        # Increment recovery attempts
        self.recovery_attempts[error_code] += 1

        # Try recovery strategy if available
        if error_code in self.error_recovery_strategies:
            print(f"Attempting recovery for error {error_code} (attempt {self.recovery_attempts[error_code]})")
            try:
                success = self.error_recovery_strategies[error_code]()
                if success:
                    print(f"Successfully recovered from error {error_code}")
                    # Reset recovery attempts on success
                    self.recovery_attempts[error_code] = 0
                    return True
            except Exception as e:
                print(f"Recovery strategy failed for error {error_code}: {e}")

        return False

    def _handle_collision_error(self) -> bool:
        """Handle collision detection errors."""
        print("Handling collision error - stopping motion and clearing error")
        try:
            if self.arm:
                self.arm.emergency_stop()
                time.sleep(0.5)
                self.arm.clean_error()
                self.arm.motion_enable(enable=True)
                return True
        except Exception as e:
            print(f"Failed to handle collision error: {e}")
        return False

    def _handle_joint_limit_error(self) -> bool:
        """Handle joint limit errors."""
        print("Handling joint limit error - moving to safe position")
        try:
            if self.arm:
                self.arm.clean_error()
                # Move to home position as safe recovery
                self.arm.move_gohome(wait=True)
                return True
        except Exception as e:
            print(f"Failed to handle joint limit error: {e}")
        return False

    def _handle_hard_joint_limit_error(self) -> bool:
        """Handle hard joint limit errors."""
        print("Handling hard joint limit error - emergency recovery")
        try:
            if self.arm:
                self.arm.emergency_stop()
                time.sleep(1)
                self.arm.clean_error()
                self.arm.motion_enable(enable=True)
                return True
        except Exception as e:
            print(f"Failed to handle hard joint limit error: {e}")
        return False

    def _handle_joint_speed_error(self) -> bool:
        """Handle joint speed limit errors."""
        print("Handling joint speed error - reducing speed limits")
        try:
            # Reduce speed limits by 20%
            self.angle_speed = int(self.angle_speed * 0.8)
            self.angle_acc = int(self.angle_acc * 0.8)
            print(f"Reduced joint speed to {self.angle_speed} deg/s, acceleration to {self.angle_acc} deg/s^2")

            if self.arm:
                self.arm.clean_error()
                return True
        except Exception as e:
            print(f"Failed to handle joint speed error: {e}")
        return False

    def _handle_tcp_speed_error(self) -> bool:
        """Handle TCP speed limit errors."""
        print("Handling TCP speed error - reducing TCP limits")
        try:
            # Reduce TCP speed limits by 20%
            self.tcp_speed = int(self.tcp_speed * 0.8)
            self.tcp_acc = int(self.tcp_acc * 0.8)
            print(f"Reduced TCP speed to {self.tcp_speed}mm/s, acceleration to {self.tcp_acc}mm/s^2")

            if self.arm:
                self.arm.clean_error()
                return True
        except Exception as e:
            print(f"Failed to handle TCP speed error: {e}")
        return False

    def _handle_communication_error(self) -> bool:
        """Handle communication errors."""
        print("Handling communication error - attempting reconnection")
        try:
            if self.arm and not self.simulation_mode:
                self.arm.disconnect()
                time.sleep(2)
                self.arm.connect()
                return self.arm.connected
        except Exception as e:
            print(f"Failed to handle communication error: {e}")
        return False

    def _handle_state_error(self) -> bool:
        """Handle robot state errors."""
        print("Handling state error - resetting robot state")
        try:
            if self.arm:
                self.arm.clean_error()
                self.arm.clean_warn()
                self.arm.motion_enable(enable=True)
                self.arm.set_mode(0)
                self.arm.set_state(0)
                return True
        except Exception as e:
            print(f"Failed to handle state error: {e}")
        return False

    # Bio Gripper Methods (Internal use - prefer universal methods)
    def _enable_bio_gripper_internal(self):
        """Internal method for enabling bio gripper."""
        if self.gripper_type not in ('bio', 'bio_gen2'):
            return False
        code = self.arm.set_bio_gripper_enable(True)
        return self.check_code(code, 'enable_bio_gripper')

    def _open_bio_gripper_internal(self, speed=None, wait=True):
        """Internal method for opening bio gripper."""
        if speed is None:
            speed = self._gripper_setting('speed', 300)
        code = self.arm.open_bio_gripper(speed=speed, wait=wait)
        return self.check_code(code, 'open_bio_gripper')

    def _close_bio_gripper_internal(self, speed=None, wait=True):
        """Internal method for closing bio gripper."""
        if speed is None:
            speed = self._gripper_setting('speed', 300)
        code = self.arm.close_bio_gripper(speed=speed, wait=wait)
        return self.check_code(code, 'close_bio_gripper')

    def _set_bio_gripper_g2_position_internal(self, position, speed=None, force=None, wait=True):
        """Internal method for BioGripper Gen2 position/stroke control."""
        if self.gripper_type != 'bio_gen2':
            return False
        if speed is None:
            speed = self._gripper_setting('speed', 1000)
        if force is None:
            force = self._gripper_setting('force', 80)
        # Clamp speed to official range 0-4000
        config = getattr(self, 'current_gripper_config', {}) or {}
        speed_range = config.get('speed_range', {'min': 0, 'max': 4000})
        speed = max(speed_range['min'], min(speed_range['max'], speed))
        timeout = self._gripper_setting('timeout', 5)
        position = self._coerce_int(position, 'position')
        speed = self._coerce_int(speed, 'speed')
        force = self._coerce_int(force, 'force')
        timeout = self._coerce_int(timeout, 'timeout')
        code = self.arm.set_bio_gripper_g2_position(
            position, speed=speed, force=force, wait=wait, timeout=timeout
        )
        success = self.check_code(code, f'set_bio_gripper_g2_position({position})')
        if success:
            self.last_gripper_position = position
            self.last_gripper_force = force
            self.last_gripper_speed = speed
        return success

    # Standard Gripper Methods (Internal use - prefer universal methods)
    def _enable_standard_gripper_internal(self):
        """Internal method for enabling standard gripper."""
        if self.gripper_type != 'standard':
            return False
        code = self.arm.set_gripper_enable(True)
        return self.check_code(code, 'enable_standard_gripper')

    def _set_gripper_position_internal(self, position, speed=None, wait=True):
        """Internal method for setting standard gripper position."""
        if speed is None:
            speed = self._gripper_setting('speed', 5000)
        code = self.arm.set_gripper_position(position, speed=speed, wait=wait)
        return self.check_code(code, f'set_gripper_position({position})')

    # RobotIQ Gripper Methods (Internal use - prefer universal methods)
    def _initialize_robotiq_gripper_internal(self):
        """Internal method for initializing RobotIQ gripper."""
        if self.gripper_type != 'robotiq':
            return False
        code1 = self.arm.robotiq_reset()
        if not self.check_code(code1, 'robotiq_reset'):
            return False
        time.sleep(1)
        code2 = self.arm.robotiq_set_activate(True)
        return self.check_code(code2, 'robotiq_set_activate')

    def _set_robotiq_position_internal(self, position, speed=None, force=None, wait=True):
        """Internal method for setting RobotIQ gripper position."""
        if speed is None:
            speed = self._gripper_setting('speed', 255)
        if force is None:
            force = self._gripper_setting('force', 255)
        code = self.arm.robotiq_set_position(position, speed=speed, force=force, wait=wait)
        return self.check_code(code, f'set_robotiq_position({position})')

    # =============================================================================
    # FORCE TORQUE SENSOR METHODS
    # =============================================================================

    def enable_force_torque_sensor(self):
        """Enable the 6-axis force torque sensor."""
        if not self.force_torque_config.get('enable', True):
            print("Force torque sensor is disabled in configuration")
            return False

        if self.simulation_mode:
            print("Force torque sensor enabled in simulation mode")
            self.states['force_torque'] = ComponentState.ENABLED
            return True

        try:
            # Enable force torque sensor on the arm
            code = self.arm.ft_sensor_enable(True)
            if self.check_code(code, 'enable_force_torque_sensor'):
                self.states['force_torque'] = ComponentState.ENABLED
                print("Force torque sensor enabled")
                
                # Auto-calibrate if configured
                if self.force_torque_config.get('calibration', {}).get('auto_calibrate', True):
                    self.calibrate_force_torque_sensor()
                
                return True
            return False
        except Exception as e:
            print(f"Failed to enable force torque sensor: {e}")
            self.states['force_torque'] = ComponentState.ERROR
            return False

    def disable_force_torque_sensor(self):
        """Disable the 6-axis force torque sensor."""
        if self.simulation_mode:
            self.states['force_torque'] = ComponentState.DISABLED
            return True

        try:
            code = self.arm.ft_sensor_enable(False)
            if self.check_code(code, 'disable_force_torque_sensor'):
                self.states['force_torque'] = ComponentState.DISABLED
                print("Force torque sensor disabled")
                return True
            return False
        except Exception as e:
            print(f"Failed to disable force torque sensor: {e}")
            return False

    def calibrate_force_torque_sensor(self, samples=None, delay=None):
        """Calibrate the force torque sensor to zero."""
        if not self.is_component_enabled('force_torque'):
            print("Force torque sensor must be enabled before calibration")
            return False

        config = self.force_torque_config.get('calibration', {})
        samples = samples or config.get('calibration_samples', 100)
        delay = delay or config.get('calibration_delay', 0.1)
        zero_threshold = config.get('zero_threshold', 0.5)

        print(f"Calibrating force torque sensor with {samples} samples...")
        
        if self.simulation_mode:
            # In simulation, just set zero point
            self.force_torque_zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.force_torque_calibrated = True
            print("Force torque sensor calibrated (simulation)")
            return True

        try:
            # Collect samples for calibration
            readings = []
            for i in range(samples):
                ret = self.arm.get_ft_sensor_data()
                if ret[0] == 0:
                    # Get the actual list of 6 values [fx, fy, fz, tx, ty, tz]
                    raw_data = ret[1]  # ret[1] is the list, not ret[1:]
                    if len(raw_data) == 6:
                        readings.append(raw_data)  # Take the 6 values
                    else:
                        print(f"Warning: Expected 6 values, got {len(raw_data)}: {raw_data}")
                time.sleep(delay)

            if len(readings) < samples // 2:
                print("Insufficient readings for calibration")
                return False

            # Calculate average zero point
            self.force_torque_zero = [
                sum(reading[i] for reading in readings) / len(readings)
                for i in range(6)
            ]
            
            self.force_torque_calibrated = True
            print("Force torque sensor calibrated successfully")
            return True

        except Exception as e:
            print(f"Calibration failed: {e}")
            return False

    def get_force_torque_data(self):
        """Get current force torque sensor data."""
        if not self.is_component_enabled('force_torque'):
            return None

        if self.simulation_mode:
            # Return simulated data
            return self.last_force_torque.copy()

        try:
            ret = self.arm.get_ft_sensor_data()
            if ret[0] == 0:
                raw_data = ret[1]  # ret[1] is the list of 6 values
                
                # Apply calibration if available
                if self.force_torque_calibrated:
                    calibrated_data = [
                        raw_data[i] - self.force_torque_zero[i]
                        for i in range(6)
                    ]
                else:
                    calibrated_data = raw_data

                # Update last reading and history
                self.last_force_torque = calibrated_data
                self.force_torque_history.append({
                    'timestamp': time.time(),
                    'data': calibrated_data.copy()
                })

                return calibrated_data
            return None
        except Exception as e:
            print(f"Failed to get force torque data: {e}")
            return None

    def get_force_torque_magnitude(self):
        """Get the magnitude of force and torque vectors."""
        data = self.get_force_torque_data()
        if data is None:
            return None

        # Calculate force magnitude (first 3 values)
        force_magnitude = (data[0]**2 + data[1]**2 + data[2]**2)**0.5
        
        # Calculate torque magnitude (last 3 values)
        torque_magnitude = (data[3]**2 + data[4]**2 + data[5]**2)**0.5

        return {
            'force_magnitude': force_magnitude,
            'torque_magnitude': torque_magnitude,
            'total_magnitude': (force_magnitude**2 + torque_magnitude**2)**0.5
        }

    def get_force_torque_direction(self):
        """Get the direction of force and torque vectors."""
        data = self.get_force_torque_data()
        if data is None:
            return None

        config = self.force_torque_config.get('direction_detection', {})
        dead_zone = config.get('dead_zone', 2.0)

        # Check if force is above dead zone
        force_magnitude = (data[0]**2 + data[1]**2 + data[2]**2)**0.5
        if force_magnitude < dead_zone:
            force_direction = None
        else:
            # Normalize force vector
            force_direction = [data[i] / force_magnitude for i in range(3)]

        # Check if torque is above dead zone
        torque_magnitude = (data[3]**2 + data[4]**2 + data[5]**2)**0.5
        if torque_magnitude < dead_zone:
            torque_direction = None
        else:
            # Normalize torque vector
            torque_direction = [data[i+3] / torque_magnitude for i in range(3)]

        return {
            'force_direction': force_direction,
            'torque_direction': torque_direction,
            'force_magnitude': force_magnitude,
            'torque_magnitude': torque_magnitude
        }

    def check_force_torque_safety(self):
        """Check if force/torque exceeds safety thresholds and trigger alerts."""
        if not self.is_component_enabled('force_torque'):
            return False

        data = self.get_force_torque_data()
        if data is None:
            return False

        thresholds = self.force_torque_config.get('safety_thresholds', {})
        force_thresholds = thresholds.get('force', {})
        torque_thresholds = thresholds.get('torque', {})

        # Check individual force components
        force_violations = []
        for i, axis in enumerate(['x', 'y', 'z']):
            threshold = force_thresholds.get(axis, float('inf'))
            if abs(data[i]) > threshold:
                force_violations.append(f"{axis}: {data[i]:.2f}N > {threshold}N")

        # Check individual torque components
        torque_violations = []
        for i, axis in enumerate(['x', 'y', 'z']):
            threshold = torque_thresholds.get(axis, float('inf'))
            if abs(data[i+3]) > threshold:
                torque_violations.append(f"{axis}: {data[i+3]:.2f}Nm > {threshold}Nm")

        # Check total magnitudes
        magnitudes = self.get_force_torque_magnitude()
        if magnitudes:
            if magnitudes['force_magnitude'] > force_thresholds.get('magnitude', float('inf')):
                force_violations.append(f"total: {magnitudes['force_magnitude']:.2f}N > {force_thresholds.get('magnitude')}N")
            
            if magnitudes['torque_magnitude'] > torque_thresholds.get('magnitude', float('inf')):
                torque_violations.append(f"total: {magnitudes['torque_magnitude']:.2f}Nm > {torque_thresholds.get('magnitude')}Nm")

        # Trigger alerts if violations detected
        if force_violations or torque_violations:
            current_time = time.time()
            alert_cooldown = self.force_torque_config.get('alerts', {}).get('alert_cooldown', 1.0)
            
            if current_time - self.last_alert_time > alert_cooldown:
                self._trigger_force_torque_alert(force_violations, torque_violations, data)
                self.last_alert_time = current_time
                return True

        return False

    def _trigger_force_torque_alert(self, force_violations, torque_violations, data):
        """Trigger force/torque safety alert."""
        alert_config = self.force_torque_config.get('alerts', {})
        
        message = "FORCE/TORQUE SAFETY ALERT!\n"
        if force_violations:
            message += f"Forces: {', '.join(force_violations)}\n"
        if torque_violations:
            message += f"Torques: {', '.join(torque_violations)}\n"
        message += f"Current data: {[f'{x:.2f}' for x in data]}"

        print(f"[ALERT] {message}")

        # Trigger callbacks
        self._trigger_callbacks('safety_violation', {
            'type': 'force_torque',
            'force_violations': force_violations,
            'torque_violations': torque_violations,
            'data': data,
            'message': message
        })

    def move_until_force(self, direction, force_threshold=None, speed=None, timeout=30.0):
        """
        Move in a linear direction until a force threshold is reached.
        
        Args:
            direction: Direction vector [x, y, z] (normalized)
            force_threshold: Force threshold in Newtons (default from config)
            speed: Movement speed in mm/s (default from config)
            timeout: Maximum time to wait in seconds
        
        Returns:
            bool: True if threshold reached, False if timeout or error
        """
        if not self.is_component_enabled('force_torque'):
            print("Force torque sensor must be enabled for force-controlled movement")
            return False

        # Get thresholds from config
        config = self.force_torque_config.get('operation_thresholds', {})
        linear_force_config = config.get('linear_force', {})
        
        # Determine which axis to monitor based on direction
        max_component = max(abs(x) for x in direction)
        if abs(direction[0]) == max_component:
            axis = 'x'
        elif abs(direction[1]) == max_component:
            axis = 'y'
        else:
            axis = 'z'

        force_threshold = force_threshold or linear_force_config.get(axis, 30.0)
        speed = speed or self.tcp_speed

        print(f"Moving in direction {direction} until {axis}-force reaches {force_threshold}N")

        start_time = time.time()
        
        try:
            # Set robot to Cartesian velocity control mode (mode 5)
            code = self.arm.set_mode(5)
            if not self.check_code(code, 'set_mode(5)'):
                return False
            
            # Start velocity control in the specified direction
            # vc_set_cartesian_velocity expects [vx, vy, vz, vrx, vry, vrz]
            velocity = [speed * x for x in direction] + [0, 0, 0]  # Add zero angular velocities
            code = self.arm.vc_set_cartesian_velocity(velocity)
            if not self.check_code(code, 'vc_set_cartesian_velocity'):
                return False

            # Monitor force until threshold is reached
            while time.time() - start_time < timeout:
                data = self.get_force_torque_data()
                if data is None:
                    continue

                # Check if force threshold is exceeded
                if abs(data[0 if axis == 'x' else 1 if axis == 'y' else 2]) > force_threshold:
                    # Stop motion and return to normal mode
                    self.arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0])
                    self.arm.set_mode(0)  # Return to position control mode
                    print(f"Force threshold {force_threshold}N reached in {axis} direction")
                    return True

                time.sleep(0.01)  # 100Hz monitoring

            # Timeout reached
            self.arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0])
            self.arm.set_mode(0)  # Return to position control mode
            print(f"Timeout reached without hitting force threshold")
            return False

        except Exception as e:
            print(f"Error during force-controlled movement: {e}")
            self.arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0])
            self.arm.set_mode(0)  # Return to position control mode
            return False

    def move_joint_until_torque(self, joint_id, target_angle, torque_threshold=None, speed=None, timeout=30.0):
        """
        Move a specific joint until a torque threshold is reached.
        
        Args:
            joint_id: Joint number (1-7)
            target_angle: Target angle in degrees
            torque_threshold: Torque threshold in Nm (default from config)
            speed: Movement speed in deg/s (default from config)
            timeout: Maximum time to wait in seconds
        
        Returns:
            bool: True if threshold reached, False if timeout or error
        """
        if not self.is_component_enabled('force_torque'):
            print("Force torque sensor must be enabled for torque-controlled movement")
            return False

        if not 1 <= joint_id <= self.num_joints:
            print(f"Invalid joint ID {joint_id}. Must be 1-{self.num_joints}")
            return False

        # Get thresholds from config
        config = self.force_torque_config.get('operation_thresholds', {})
        joint_torque_config = config.get('joint_torque', {})
        
        torque_threshold = torque_threshold or joint_torque_config.get(f'j{joint_id}', 2.0)
        speed = speed or self.angle_speed

        print(f"Moving joint {joint_id} to {target_angle} deg until torque reaches {torque_threshold}Nm")

        start_time = time.time()
        
        try:
            # Get current joint angles
            current_joints = self.get_current_joints()
            if current_joints is None:
                return False

            # Determine direction of movement
            angle_diff = target_angle - current_joints[joint_id - 1]
            direction = 1 if angle_diff > 0 else -1

            # Set robot to joint velocity control mode (mode 4)
            code = self.arm.set_mode(4)
            if not self.check_code(code, 'set_mode(4)'):
                return False
            
            # Start joint velocity control
            velocities = [0] * self.num_joints
            velocities[joint_id - 1] = direction * speed
            code = self.arm.vc_set_joint_velocity(velocities)
            if not self.check_code(code, 'vc_set_joint_velocity'):
                return False

            # Monitor torque until threshold is reached
            while time.time() - start_time < timeout:
                data = self.get_force_torque_data()
                if data is None:
                    continue

                # Check if torque threshold is exceeded
                # Map joint to torque axis (simplified mapping)
                torque_axis = min(joint_id - 1, 2)  # Map to x, y, or z torque
                if abs(data[3 + torque_axis]) > torque_threshold:
                    # Stop motion and return to normal mode
                    self.arm.vc_set_joint_velocity([0] * self.num_joints)
                    self.arm.set_mode(0)  # Return to position control mode
                    print(f"Torque threshold {torque_threshold}Nm reached for joint {joint_id}")
                    return True

                # Check if target angle reached
                current_joints = self.get_current_joints()
                if current_joints and abs(current_joints[joint_id - 1] - target_angle) < 1.0:
                    self.arm.vc_set_joint_velocity([0] * self.num_joints)
                    self.arm.set_mode(0)  # Return to position control mode
                    print(f"Target angle {target_angle} deg reached for joint {joint_id}")
                    return True

                time.sleep(0.01)  # 100Hz monitoring

            # Timeout reached
            self.arm.vc_set_joint_velocity([0] * self.num_joints)
            self.arm.set_mode(0)  # Return to position control mode
            print(f"Timeout reached without hitting torque threshold")
            return False

        except Exception as e:
            print(f"Error during torque-controlled movement: {e}")
            self.arm.vc_set_joint_velocity([0] * self.num_joints)
            self.arm.set_mode(0)  # Return to position control mode
            return False

    def get_force_torque_status(self):
        """Get comprehensive force torque sensor status."""
        return {
            'enabled': self.is_component_enabled('force_torque'),
            'calibrated': self.force_torque_calibrated,
            'last_reading': self.last_force_torque,
            'zero_point': self.force_torque_zero,
            'history_length': len(self.force_torque_history),
            'alerts_active': self.force_torque_alerts_active,
            'magnitude': self.get_force_torque_magnitude(),
            'direction': self.get_force_torque_direction()
        }

    def has_force_torque_sensor(self):
        """Check if force torque sensor is available and enabled."""
        return self.force_torque_config.get('enable', True)

    def move_plate_linear(self, target_location, num_steps=1, speed=None, wait_between_steps=0.1):
        """
        Move tool linearly from current position to target position.
        Tool maintains the same absolute orientation throughout the movement.
        
        Args:
            target_location (str): Name of target location from position_config.yaml
            num_steps (int): Number of interpolation steps (default: 10)
            speed (float): Movement speed (default: tcp_speed)
            wait_between_steps (float): Delay between steps in seconds (default: 0.1)
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.is_component_enabled('arm'):
            print("Arm is not enabled")
            return False
            
        # Validate target location exists
        if 'positions' not in self.position_config:
            print("Error: No positions defined in config")
            return False
            
        positions = self.position_config['positions']
        if target_location not in positions:
            print(f"Error: Target location '{target_location}' not found")
            return False
            
        # Get current position as starting point
        start_cartesian = self.get_current_position()
        if not start_cartesian:
            print("Error: Could not get current position")
            return False
            
        # Convert target position to Cartesian
        target_pos = positions[target_location]
        target_cartesian = self._position_to_cartesian(target_location, target_pos, speed)
        if not target_cartesian:
            return False
            
        print(f"Moving linearly from current position to '{target_location}'")
        print(f"Start: {start_cartesian[:3]} (X,Y,Z)")
        print(f"Target: {target_cartesian[:3]} (X,Y,Z)")
        print(f"Tool orientation: {start_cartesian[3:]} (maintained throughout)")
        
        # Perform linear interpolation - POSITION ONLY
        # Tool orientation stays exactly the same as current orientation
        if num_steps == 1:
            # Single step - move directly to target
            steps_to_execute = [1]
        else:
            # Multiple steps - interpolate
            steps_to_execute = range(1, num_steps + 1)
            
        for i in steps_to_execute:
            # Calculate interpolation factor (0 to 1)
            t = i / max(num_steps, 1)  # Avoid division by zero
            
            # Linear interpolation for POSITION only (X, Y, Z)
            interp_x = start_cartesian[0] + t * (target_cartesian[0] - start_cartesian[0])
            interp_y = start_cartesian[1] + t * (target_cartesian[1] - start_cartesian[1])
            interp_z = start_cartesian[2] + t * (target_cartesian[2] - start_cartesian[2])
            
            # Keep CURRENT tool orientation throughout (absolute direction in space)
            interp_roll = start_cartesian[3]
            interp_pitch = start_cartesian[4] 
            interp_yaw = start_cartesian[5]
            
            interp_pos = [interp_x, interp_y, interp_z, interp_roll, interp_pitch, interp_yaw]
            
            # Move to interpolated position
            success = self.move_to_position(
                x=interp_pos[0], y=interp_pos[1], z=interp_pos[2],
                roll=interp_pos[3], pitch=interp_pos[4], yaw=interp_pos[5],
                speed=speed, check_collision=False, wait=True
            )
            
            if not success:
                print(f"Error: Failed at step {i}/{num_steps}")
                return False
                
            print(f"[OK] Step {i}/{num_steps}: {interp_pos[:3]}")
                
            # Wait between steps if specified
            if wait_between_steps > 0 and i < num_steps:
                time.sleep(wait_between_steps)
                
        print(f"[OK] Successfully completed linear movement to '{target_location}'")
        return True

    def _position_to_cartesian(self, location_name, position_data, speed=None):
        """
        Convert any position format to Cartesian coordinates [x, y, z, roll, pitch, yaw].
        
        Supported formats:
        1. Joint angles: [J1, J2, J3, J4, J5] or [J1, J2, J3, J4, J5, J6, J7]
        2. Cartesian list: [x, y, z, roll, pitch, yaw]  
        3. Cartesian dict: {x: 300, y: 0, z: 400, roll: 180, pitch: 0, yaw: 0}
        
        Args:
            location_name (str): Name of the location (for logging)
            position_data: Position in any supported format
            speed (float): Speed for temporary movements (if needed)
            
        Returns:
            list: [x, y, z, roll, pitch, yaw] or None if conversion failed
        """
        if isinstance(position_data, dict):
            # Dictionary format: {x: 300, y: 0, z: 400, roll: 180, pitch: 0, yaw: 0}
            print(f"Using Cartesian dict format for '{location_name}'")
            return [
                position_data['x'], position_data['y'], position_data['z'],
                position_data.get('roll', 180), position_data.get('pitch', 0), position_data.get('yaw', 0)
            ]
            
        elif isinstance(position_data, list):
            if len(position_data) == 6:
                # Already Cartesian: [x, y, z, roll, pitch, yaw]
                print(f"Using Cartesian list format for '{location_name}': {position_data}")
                return position_data
                
            elif len(position_data) <= self.num_joints:
                # Joint angles: [J1, J2, J3, J4, J5] or [J1, ..., J7]
                print(f"Converting joint angles to Cartesian for '{location_name}': {position_data}")
                try:
                    if not self.simulation_mode and hasattr(self.arm, 'get_forward_kinematics'):
                        # Use forward kinematics (preferred - no robot movement)
                        ret = self.arm.get_forward_kinematics(position_data)
                        if ret[0] == 0:
                            cartesian = ret[1][:6]  # [x, y, z, roll, pitch, yaw]
                            print(f"[OK] Forward kinematics result: {cartesian}")
                            return cartesian
                        else:
                            print("Forward kinematics failed, using position sampling")
                    
                    # Fallback: Move robot to get position (less efficient)
                    print("Using position sampling method")
                    temp_current = self.get_current_position()
                    if not self.move_joints(position_data, speed=speed):
                        print(f"Error: Could not move to joint position {position_data}")
                        return None
                    
                    cartesian = self.get_current_position()
                    if not cartesian:
                        print("Error: Could not get Cartesian position after joint movement")
                        return None
                    
                    # Restore to original position
                    if temp_current and not self.move_to_position(
                        x=temp_current[0], y=temp_current[1], z=temp_current[2],
                        roll=temp_current[3], pitch=temp_current[4], yaw=temp_current[5],
                        speed=speed, wait=True
                    ):
                        print("Warning: Could not restore to original position")
                    
                    print(f"[OK] Position sampling result: {cartesian}")
                    return cartesian
                    
                except Exception as e:
                    print(f"Error in joint-to-Cartesian conversion: {e}")
                    return None
            else:
                print(f"Error: Invalid list length {len(position_data)} for '{location_name}'")
                return None
        else:
            print(f"Error: Unsupported position format for '{location_name}': {type(position_data)}")
            return None

# Example usage:
if __name__ == '__main__':
    print("XArmController with streamlined API!")
    print("=" * 50)
    print("[OK] METHODS (Recommended):")
    print("   - move_to_position() - Cartesian movement with collision detection and alternative planning")
    print("   - move_joints() - Joint movement with comprehensive safety validation")
    print("   - open_gripper() / close_gripper() - Universal gripper support")
    print("   - enable_gripper_component() - Component-based gripper control")
    print()
    print("PERFORMANCE TRACKING:")
    print("   - Real-time cycle time monitoring")
    print("   - Position accuracy tracking")
    print("   - Component utilization metrics")
    print("   - Speed limit enforcement")
    print("   - Automatic error recovery")
    print()
    print("Usage Example:")
    print("  # Modern API (recommended):")
    print("  controller = XArmController(auto_enable=True)")
    print("  controller.move_to_position(300, 0, 400)  # With collision detection by default")
    print("  controller.open_gripper()  # Universal gripper support")
    print()
    print("  # Advanced usage:")
    print("  controller.move_to_position(x, y, z, check_collision=True, motion_type=0)")
    print("  controller.move_joints(angles, check_collision=True)")
    