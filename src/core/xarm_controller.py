import time
import os
from collections import deque
from enum import Enum
from typing import List, Optional

from xarm.wrapper import XArmAPI

from core.xarm_utils import (
    SafetyLevel, load_config, get_default_config, validate_target_position,
    validate_joint_angles, validate_track_position, validate_track_speed,
    DEFAULT_SAFETY_BOUNDARIES, DEFAULT_COLLISION_SENSITIVITY,
    get_safety_speed_limits, apply_movement_parameter_limits,
    get_joint_limits_for_model, check_operation_result, validate_and_apply_safety_config
)
# Use relative-then-absolute fallback so the api_server's `from
# .motion_graph` (resolving to src.core.motion_graph in test runs)
# and the controller's import resolve to the SAME class objects.
# Without this, EdgeNotAllowedError raised by the controller is a
# different class from the one the API server's except clause catches.
try:
    from .motion_graph import (
        DEFAULT_PRECONDITIONS, Edge, EdgeNotAllowedError, GraphError,
        GraphMode, GripIntent, GripperTransitionError, MotionGraph, MoveMode,
        NodeMatch, RecoveryMismatch, find_nearest_node,
    )
    from .claims import ClaimManager
    from .events_exporter import EventsExporter
except ImportError:
    from core.motion_graph import (
        DEFAULT_PRECONDITIONS, Edge, EdgeNotAllowedError, GraphError,
        GraphMode, GripIntent, GripperTransitionError, MotionGraph, MoveMode,
        NodeMatch, RecoveryMismatch, find_nearest_node,
    )
    from core.claims import ClaimManager
    from core.events_exporter import EventsExporter


# Coarse xArm SDK controller-state -> STATUS_SPEC-ish label, used only to
# label device-pushed state_transition events. The raw SDK integer always
# rides along in extra.xarm_state; the /status envelope keeps its own
# (richer) derivation in status_builder.
_XARM_STATE_LABELS = {
    1: "busy",      # in motion
    2: "ready",     # sleeping / standby
    3: "busy",      # paused mid-trajectory
    4: "error",     # stop state (error / protective stop)
    5: "unknown",   # system reset
    6: "busy",      # slowing / stopping
}


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
                 safety_level: SafetyLevel = SafetyLevel.MEDIUM):
        """
        Args:
            host (str, optional): The IP address of the xArm. If provided, this
                                overrides any host in the config file. Defaults to None.
            profile_name (str, optional): The name of the connection profile from xarm_config.yaml.
            gripper_type (str): Type of gripper ('bio', 'bio_gen2', 'standard', 'robotiq', or 'none')
            enable_track (bool): Whether to enable the linear track
            auto_enable (bool): Whether to automatically enable components during initialization
            model (int): xArm model (5, 6, 7). If None, will be detected from config
            safety_level (SafetyLevel): Safety level for validation strictness
        """
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
            'position_config': 'joint_config.yaml',
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

        # Motion graph (Phase 1: advisory only, soft-fails to OFF mode
        # when the YAML is missing or invalid so the controller still
        # boots on unmigrated configs).
        self.motion_graph: Optional[MotionGraph] = None
        self.graph_mode: GraphMode = GraphMode.OFF
        graph_path = os.path.join('src', 'settings', 'motion_graph.yaml')
        try:
            self.motion_graph = MotionGraph.from_yaml(
                graph_path, preconditions=DEFAULT_PRECONDITIONS,
            )
            self.graph_mode = GraphMode.ADVISORY
        except FileNotFoundError:
            print(f"Info: {graph_path} not found, motion-graph layer disabled.")
        except GraphError as exc:
            print(f"Warning: motion_graph.yaml failed validation: {exc}. Disabling graph.")

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
        # None (not 0) until a successful encoder read: 0 is a valid-looking
        # rail position (== Home), so a stale 0 would masquerade as Home and
        # make nearest-node/Recover snap to the wrong node. None means
        # "unknown", which find_nearest_node treats as no-match.
        self.last_track_position = None
        self.last_gripper_position = self._gripper_setting('open_position', 0)
        self.last_gripper_force = self._gripper_setting('force', 100)
        self.last_gripper_speed = self._gripper_setting('speed', 300)

        # BIO gripper status/error register cache, surfaced on /status as a
        # plate-transfer verification signal (failed pickup / mid-move slip).
        # Refreshed by refresh_gripper_status() after each Gen2 jaw move;
        # status_builder reads ONLY these cached values so it stays
        # side-effect-free (no live Modbus round-trip from /status).
        self.last_gripper_position_actual = None   # mm, read back from gripper
        self.last_gripper_motion_state = None      # stop|moving|object_detected|fault|unknown
        self.last_gripper_object_detected = None   # bool | None
        self.last_gripper_error_code = 0           # BIO register 0x0F; 0 == OK, 12 == object slipped
        self.last_gripper_error_text = None         # human-readable mapping of the code

        # Force torque sensor tracking
        self.force_torque_history = deque(maxlen=1000)
        self.last_force_torque = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [fx, fy, fz, tx, ty, tz]
        self.force_torque_zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Calibrated zero point
        self.force_torque_calibrated = False
        self.force_torque_alerts_active = False
        self.last_alert_time = 0

        # Motion state tracking
        self._motion_in_progress = False

        # Motion-graph named-coordinate tracking. These are the named values
        # that, combined with the commanded gripper stroke, resolve to a graph
        # node. Any raw cartesian / joint / track move clears the relevant
        # one — re-pin via a named move or recover_to().
        self.last_arm_pose_name: Optional[str] = None
        self.last_rail_location_name: Optional[str] = None
        # Set while move_to_node dispatches the two sub-moves of a
        # cross-rail edge (one graph edge, two physical axes). The edge is
        # validated once at the move_to_node level; the intermediate state
        # between the arm and rail sub-moves is a non-node by design, so
        # per-axis graph consultation must not run during that window.
        self._suppress_graph_consult: bool = False
        # last_gripper_position is the commanded stroke (set on every
        # successful SDK gripper call). Used by _commanded_gripper_stroke() to
        # resolve the current graph node. Distinct from
        # last_gripper_position_actual which is the live hardware readback.

        # Last successful node-to-node transition (Phase 2). Captured by
        # the named-move methods; used by POST /control/graph/record to
        # propose a new edge for the YAML. None until at least one named
        # transition lands between two pinned nodes.
        #   dict: {from_node, to_node, mode, speed, timestamp}
        self.last_transition: Optional[dict] = None

        # STATUS_SPEC v1.1 claim (Phase 3+5). Single in-process holder.
        # Hard enforcement is ON by default so the claim is the single
        # gate on motion: every mutating endpoint (/move/*, /gripper/*,
        # /track/*, ...) requires the active claim's X-Claim-Token or
        # returns HTTP 423 — including when no claim is held at all (a
        # caller must POST /control/claim first). /move/stop and
        # /clear/errors stay ungated (safety floor). Set
        # XARM_ENFORCE_CLAIMS to a falsy value (0/false/no/off) to drop
        # back to advisory mode for demos/dev; POST /control/claim/enforce
        # toggles it at runtime.
        enforce_env = os.environ.get("XARM_ENFORCE_CLAIMS", "").strip().lower()
        enforce = enforce_env not in ("0", "false", "no", "off")
        self.claim_manager = ClaimManager(enforce=enforce)
        if enforce:
            print("[claims] hard enforcement ON (default); set XARM_ENFORCE_CLAIMS=0 for advisory mode")
        else:
            print("[claims] enforcement OFF (advisory) via XARM_ENFORCE_CLAIMS")

        # Device-pushed event rows for the dashboard's history DB (fine-
        # grained state_transition / error events straight from the SDK
        # callbacks; the aggregator's 60 s poll stays the coarse backstop).
        # No-op unless XARM_INGEST_URL is set. See core/events_exporter.py.
        self.events_exporter = EventsExporter.from_env()
        self._last_emitted_state: Optional[str] = None
        if self.events_exporter.enabled:
            print(f"[events] exporter ON -> {self.events_exporter.ingest_url}")
        else:
            print("[events] exporter OFF (set XARM_INGEST_URL to enable)")

        # State tracking
        self.alive = True
        self._ignore_exit_state = False

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

    # =============================================================================
    # INITIALIZATION
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

        print("Initializing Robot Arm...")
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

                    print("xArm Controller Initialized")
                    self._emit_event("startup", message="Controller connected and enabled")
                    self._emit_state_transition("ready", message="Controller initialized")
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

            if self.gripper_type in ('bio', 'bio_gen2'):
                success = self._enable_bio_gripper_internal()
            elif self.gripper_type == 'standard':
                success = self._enable_standard_gripper_internal()
            elif self.gripper_type == 'robotiq':
                success = self._initialize_robotiq_gripper_internal()

            if success:
                self.states['gripper'] = ComponentState.ENABLED
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

            success = self.enable_linear_track()

            if success:
                self.states['track'] = ComponentState.ENABLED
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
            ret = self.arm.get_position()
            if ret[0] == 0:
                self.last_position = ret[1:]

            ret = self.arm.get_servo_angle()
            if ret[0] == 0:
                self.last_joints = ret[1]

        except Exception as e:
            print(f"Warning: Failed to update positions: {e}")

    def _update_track_position(self):
        """Update cached track position.

        Reading the encoder is a *read*, not motion, so it is not gated on
        the track being motion-ENABLED — only on the arm being connected
        and a track being configured. On a non-zero SDK code or exception
        we leave ``last_track_position`` unchanged (it stays None until a
        real read) rather than writing a bogus value, and log why so a
        stale/unknown rail is visible instead of silently masquerading as
        Home (0 mm).
        """
        if not self.arm or not self.enable_track:
            return

        try:
            ret = self.arm.get_linear_track_pos()
            if ret[0] == 0:
                self.last_track_position = ret[1]
            else:
                print(
                    f"Warning: track position read returned code {ret[0]} "
                    f"(track state={self.states.get('track')}); "
                    f"keeping last value {self.last_track_position}"
                )
        except Exception as e:
            print(f"Warning: Failed to update track position: {e}")

    def _current_graph_node(self) -> Optional[str]:
        """Best-effort current motion-graph node id for event context."""
        try:
            if getattr(self, "motion_graph", None) is None:
                return None
            return self.current_node
        except Exception:
            return None

    def _emit_event(self, event, *, from_state=None, to_state=None, message=None, **extra):
        """Push one row to the dashboard's history DB, best-effort.

        Always attaches the standard context keys from the plan's event
        vocabulary (xarm_state, error_code, warn_code, graph_node);
        callers can override or extend via **extra. Never raises.
        """
        exporter = getattr(self, "events_exporter", None)
        if exporter is None or not exporter.enabled:
            return
        context = {
            "xarm_state": getattr(self.arm, "state", None) if self.arm else None,
            "error_code": getattr(self, "last_error_code", 0),
            "warn_code": getattr(self, "last_warn_code", 0),
            "graph_node": self._current_graph_node(),
        }
        context.update(extra)
        exporter.emit(
            event, from_state=from_state, to_state=to_state, message=message, **context
        )

    def _emit_state_transition(self, to_state, *, message=None, **extra):
        """Emit a state_transition row when the coarse state label changes.

        from_state is the last label *this exporter* emitted (None on the
        first transition of the process, mirroring the aggregator's
        first-observation row), so consecutive duplicates are suppressed.
        """
        from_state = self._last_emitted_state
        if to_state == from_state:
            return
        self._last_emitted_state = to_state
        self._emit_event(
            "state_transition", from_state=from_state, to_state=to_state,
            message=message, **extra,
        )

    def _error_warn_callback(self, data):
        """SDK callback: record error/warn codes and flip to error state."""
        if not data:
            return

        if data.get('error_code', 0) != 0:
            error_code = data['error_code']
            self.last_error_code = error_code

            self.error_history.append({
                'timestamp': time.time(),
                'error_code': error_code,
                'warn_code': data.get('warn_code', 0)
            })

            self.alive = False
            self.states['arm'] = ComponentState.ERROR
            print(f'Error {error_code} detected')
            self._emit_event(
                "error",
                message=f"Controller reports error code {error_code}",
                severity="error",
                error_code=error_code,
                warn_code=data.get('warn_code', 0),
            )
            self._emit_state_transition(
                "error", message=f"Error code {error_code} latched",
            )

        if data.get('warn_code', 0) != 0:
            self.last_warn_code = data['warn_code']
            print(f'Warning: {data["warn_code"]}')
            self._emit_event(
                "error",
                message=f"Controller reports warn code {data['warn_code']}",
                severity="warning",
                warn_code=data['warn_code'],
            )

    def _state_changed_callback(self, data):
        """SDK callback: flip to error state on the SDK's emergency state 4."""
        if not data or 'state' not in data:
            return
        state = data['state']
        if not self._ignore_exit_state and state == 4:
            self.alive = False
            self.states['arm'] = ComponentState.ERROR
            print('State 4 detected, stopping operations')
        self._emit_state_transition(
            _XARM_STATE_LABELS.get(state, "unknown"),
            message=f"xArm SDK controller state changed to {state}",
            xarm_state=state,
        )

    def check_code(self, code, operation_name):
        """Check if an SDK operation was successful (None or 0)."""
        is_success = (code is None or code == 0)

        if not self.is_alive or not is_success:
            self.alive = False
            state = self.arm.state if self.arm else None
            error = self.arm.error_code if self.arm else None
            return check_operation_result(code, operation_name, state, error)
        return True

    @property
    def is_alive(self):
        """Check if the robot is in a safe operating state."""
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
            print("Clearing robot errors and warnings...")

            # Clear errors and warnings
            error_clear_code = self.arm.clean_error()
            warn_clear_code = self.arm.clean_warn()

            # BIO Gripper hardware errors (e.g. code 12 "object slipped") live
            # in the gripper's own register; clean_error() doesn't touch them.
            # An unresolved gripper fault can pin the arm in state 4, so clear
            # it here too.
            if self.gripper_type in ('bio', 'bio_gen2') and hasattr(self.arm, 'clean_bio_gripper_error'):
                self.arm.clean_bio_gripper_error()

            # Reset error tracking
            self.error_history.clear()
            self.last_error_code = 0
            self.last_warn_code = 0

            # Reset alive state if errors were cleared successfully
            if error_clear_code == 0 and warn_clear_code == 0:
                self.alive = True
                print("[OK] All errors and warnings cleared successfully")
                self._emit_state_transition("ready", message="Errors cleared")

                # Always re-arm the arm. This is the single recovery button
                # (it replaced the separate "Enable"), so it must re-energize
                # the servos unconditionally — NOT just when auto_enable is on
                # or the arm is flagged ERROR. The SDK parks the arm in state 4
                # after emergency_stop and refuses motion until mode/state are
                # re-asserted; a merely-disabled arm must also come back live.
                print("Re-enabling arm and components...")
                if hasattr(self.arm, 'motion_enable'):
                    self.arm.motion_enable(enable=True)
                if hasattr(self.arm, 'set_mode'):
                    self.arm.set_mode(0)
                if hasattr(self.arm, 'set_state'):
                    self.arm.set_state(0)
                self.states['arm'] = ComponentState.ENABLED

                # BIO gripper faults live in the gripper's own register;
                # re-enable unconditionally so clean_bio_gripper_error +
                # set_bio_gripper_enable(True) actually take effect on the
                # hardware after a slip/overcurrent. Other grippers/track:
                # re-enable when they were in error.
                if self.gripper_type in ('bio', 'bio_gen2'):
                    self.enable_gripper_component()
                elif self.has_gripper() and self.states['gripper'] == ComponentState.ERROR:
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

        # A raw cartesian move invalidates any pinned named arm pose.
        # The named-move wrapper (move_to_named_location / move_plate_linear)
        # restores it after this call on success.
        self.last_arm_pose_name = None

        # Set defaults
        if roll is None: roll = 180
        if pitch is None: pitch = 0
        if yaw is None: yaw = 0
        if speed is None: speed = self.tcp_speed

        target_pos = [x, y, z, roll, pitch, yaw]

        # Pre-motion validation
        if not self._validate_target_position(target_pos):
                return False

        # Collision checking (firmware-side, no-move dry run)
        if check_collision:
            self.arm.set_only_check_type(1)
            check_code = self.arm.set_position(x, y, z, roll, pitch, yaw, speed=speed)
            self.arm.set_only_check_type(0)

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

        # Execute the movement
        self._motion_in_progress = True

        try:
            code = self.arm.set_position(x, y, z, roll, pitch, yaw,
                                       speed=speed, wait=wait, motion_type=motion_type)
            success = self.check_code(code, f'move_to_position({x}, {y}, {z})')

            if success:
                self._update_positions()

            return success

        finally:
            self._motion_in_progress = False

    def move_to_named_location(self, location_name, speed=None):
        """
        Move to a predefined location from the position config.
        Supports both joint-based and Cartesian-based location definitions.

        In STRICT graph mode the edge from current_node to the target's
        node is consulted: edge.mode (linear vs joint) overrides the
        preset's storage format, edge.speed caps the caller's speed,
        and off-whitelist transitions raise EdgeNotAllowedError.
        """
        # Check if positions are defined in config
        if 'positions' not in self.position_config:
            print(f"Error: No 'positions' section found in position config")
            return False

        if location_name not in self.position_config['positions']:
            print(f"Error: Location '{location_name}' not found in config")
            return False

        location = self.position_config['positions'][location_name]

        # === Graph consultation (raises in STRICT mode on off-whitelist) ===
        from_node = self.current_node
        target_node_id = self._predict_target_node_for_arm_pose(location_name)
        edge = self._consult_graph_for_move(target_node_id, location_name)

        # Apply edge.mode override + edge.speed cap (STRICT only).
        speed = self._apply_edge_speed_cap(speed, edge)
        mode_override: Optional[MoveMode] = None
        if self.graph_mode == GraphMode.STRICT and edge is not None:
            mode_override = edge.mode

        # === Dispatch ===
        if mode_override == MoveMode.LINEAR:
            # Edge says linear regardless of how the preset is stored.
            cartesian = self._position_to_cartesian(location_name, location, speed=speed)
            if cartesian is None:
                print(f"Error: could not resolve cartesian coordinates for {location_name!r}")
                return False
            success = self.move_to_position(
                x=cartesian[0], y=cartesian[1], z=cartesian[2],
                roll=cartesian[3], pitch=cartesian[4], yaw=cartesian[5],
                speed=speed,
            )
            mode_used = MoveMode.LINEAR
        elif mode_override == MoveMode.JOINT:
            # Edge says joint. Only meaningful for joint-list presets;
            # cartesian-dict presets would need IK we don't run.
            if not isinstance(location, list):
                print(
                    f"Error: edge requires joint mode but preset {location_name!r} "
                    f"is stored as cartesian; inverse-kinematics dispatch not supported"
                )
                return False
            angles = list(location)
            while len(angles) < self.num_joints:
                angles.append(0.0)
            success = self.move_joints(angles=angles[:self.num_joints], speed=speed)
            mode_used = MoveMode.JOINT
        else:
            # No override (OFF / ADVISORY): fall back to preset's storage format.
            if isinstance(location, list):
                angles = list(location)
                while len(angles) < self.num_joints:
                    angles.append(0.0)
                print(f"Moving to location '{location_name}' using joint angles: {angles[:self.num_joints]}")
                success = self.move_joints(angles=angles[:self.num_joints], speed=speed)
                mode_used = MoveMode.JOINT
            elif isinstance(location, dict):
                print(f"Moving to location '{location_name}' using Cartesian coordinates")
                success = self.move_to_position(
                    x=location['x'], y=location['y'], z=location['z'],
                    roll=location.get('roll'), pitch=location.get('pitch'), yaw=location.get('yaw'),
                    speed=speed,
                )
                mode_used = MoveMode.LINEAR
            else:
                print(f"Error: Invalid location format for '{location_name}'. Expected list (joint angles) or dict (Cartesian coordinates)")
                return False

        # Inner call cleared last_arm_pose_name; restore it on success so
        # the motion graph can pin the new node and capture the transition.
        if success:
            self.last_arm_pose_name = location_name
            to_node = self.current_node
            self._store_transition(from_node, to_node, mode_used, speed)
        return success

    def move_relative(self, dx=0.0, dy=0.0, dz=0.0, droll=0.0, dpitch=0.0, dyaw=0.0, speed=None):
        """
        Move relative to current position (linear movement).
        """
        if not self.is_component_enabled('arm'):
            print("Arm is not enabled")
            return False

        if speed is None:
            speed = self.tcp_speed

        # Jog/relative moves go off-grid by definition.
        self.last_arm_pose_name = None

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

        # Raw joint move invalidates any pinned named pose; the named-move
        # wrapper restores it after this returns successfully.
        self.last_arm_pose_name = None

        # Execute movement
        self._motion_in_progress = True

        try:
            # check=False mirrors the Docker simulator serial-number workaround
            code = self.arm.set_servo_angle(angle=angles, speed=speed, mvacc=acceleration, wait=wait, check=False)
            success = self.check_code(code, f'move_joints({angles})')

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
        # After an emergency stop, the arm froze somewhere between named
        # poses. The graph must report unknown until re-pinned.
        self.last_arm_pose_name = None
        self.last_rail_location_name = None
        code = self.arm.emergency_stop()
        return self.check_code(code, 'emergency_stop')

    def set_manual_mode(self, enable):
        """Enable or disable manual (drag/teach) mode.

        Manual mode releases the joint brakes so the arm can be moved by
        hand -- the same toggle UFACTORY Studio exposes. It maps to xArm
        SDK mode 2 (joint teaching); disabling restores position-control
        mode 0. Both transitions re-assert state 0 so the change takes
        effect (and so toggling straight after a STOP, which leaves the
        arm in state 4, still works).

        WARNING: while manual mode is on the arm is back-drivable and will
        sag under its own weight / payload if unsupported. The caller is
        responsible for warning the operator.
        """
        if not self.arm:
            print("Cannot set manual mode: No arm connection")
            return False

        try:
            target_mode = 2 if enable else 0
            # Manual mode needs motion enabled first.
            if enable and hasattr(self.arm, 'motion_enable'):
                self.arm.motion_enable(enable=True)
            code = self.arm.set_mode(target_mode)
            if not self.check_code(code, f'set_mode({target_mode})'):
                return False
            code = self.arm.set_state(0)
            if not self.check_code(code, 'set_state(0)'):
                return False
            print(f"Manual mode {'enabled' if enable else 'disabled'} (mode {target_mode})")
            return True
        except Exception as e:
            print(f"Failed to set manual mode: {e}")
            return False

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
            code = self.arm.robotiq_open(wait=wait)
            return self.check_code(code, 'open_robotiq_gripper')
        else:
            print("No gripper configured")
            return False

    def close_gripper(self, speed=None, force=None, wait=True):
        """Close the gripper (works with any configured gripper type)."""
        if not self.is_component_enabled('gripper'):
            print("Gripper is not enabled")
            return False

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
            code = self.arm.robotiq_close(wait=wait)
            return self.check_code(code, 'close_robotiq_gripper')
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

        if self.gripper_type == 'bio_gen2':
            force = self._coerce_int(force, 'force')
            code = self.arm.set_bio_gripper_force(force)
            return self.check_code(code, f'set_bio_gripper_force({force})')

        print(f"{self.gripper_type} gripper does not support standalone force control")
        return False

    def get_gripper_position(self):
        """Return the latest gripper position/stroke when available.

        This is a live hardware readback, so it updates
        ``last_gripper_position_actual`` — NOT ``last_gripper_position``,
        which holds the last commanded stroke used for graph
        gripper-state resolution. Overwriting the commanded value here
        would desync the graph state after a grasp (the jaws settle
        above the commanded stroke when a plate is held).
        """
        if self.gripper_type == 'bio_gen2' and hasattr(self.arm, 'get_bio_gripper_g2_position'):
            ret = self.arm.get_bio_gripper_g2_position()
            if ret[0] == 0:
                self.last_gripper_position_actual = ret[1]
                return ret[1]
        elif self.gripper_type == 'standard' and hasattr(self.arm, 'get_gripper_position'):
            ret = self.arm.get_gripper_position()
            if ret[0] == 0:
                self.last_gripper_position_actual = ret[1]
                return ret[1]
        return None

    # BIO Gripper G2 status register (0x00) low 2 bits, per
    # xarm.core.config x_config.XCONF.BioGripperState. Mirrored here so we
    # don't import a deep SDK internal just for four constants.
    _BIO_GRIPPER_MOTION_STATES = {
        0: "stop",             # reached commanded position, nothing held
        1: "moving",
        2: "object_detected",  # caught something between the jaws (good pickup)
        3: "fault",            # detail lives in the error register (0x0F)
    }
    # BIO gripper error register (0x0F) codes we have copy for. Unknown
    # codes fall back to "fault"; extend as new failure modes surface.
    _BIO_GRIPPER_ERROR_TEXT = {
        12: "object slipped",
    }

    def refresh_gripper_status(self):
        """Read the BIO gripper status + error registers and cache them.

        This round-trips to the gripper over Modbus, so it is NOT
        side-effect-free and MUST NOT be called from ``status_builder``
        (which only reads the cached ``last_gripper_*`` attributes set
        here). Call it right after each Gen2 jaw move; the cached values
        then describe that move for the next ``/status`` read.

        No-op for non-BIO grippers, when disconnected, or when the SDK
        build lacks the status getter. Returns the decoded motion-state
        string, or ``None`` when unavailable.
        """
        if self.gripper_type not in ('bio', 'bio_gen2') or not self.arm:
            return None
        if not hasattr(self.arm, 'get_bio_gripper_status'):
            return None
        try:
            code, status = self.arm.get_bio_gripper_status()
            if code != 0 or not isinstance(status, int) or status < 0:
                return None
            motion_state = self._BIO_GRIPPER_MOTION_STATES.get(status & 0x03, "unknown")
            self.last_gripper_motion_state = motion_state
            self.last_gripper_object_detected = (motion_state == "object_detected")
            if motion_state == "fault" and hasattr(self.arm, 'get_bio_gripper_error'):
                ecode, evalue = self.arm.get_bio_gripper_error()
                err_code = evalue if (ecode == 0 and isinstance(evalue, int)) else 0
                self.last_gripper_error_code = err_code
                self.last_gripper_error_text = self._BIO_GRIPPER_ERROR_TEXT.get(
                    err_code, "fault"
                )
            else:
                self.last_gripper_error_code = 0
                self.last_gripper_error_text = None
            # Cache the actual jaw position too (within the same refresh).
            pos = self.get_gripper_position()
            if isinstance(pos, (int, float)) and not isinstance(pos, bool):
                self.last_gripper_position_actual = pos
            return motion_state
        except Exception as e:  # defensive: never let a status read break a move
            print(f"[gripper] status refresh failed: {e}")
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

        # Raw rail move invalidates any pinned named rail location.
        self.last_rail_location_name = None

        self._motion_in_progress = True

        try:
            result = self.arm.set_linear_track_pos(speed=speed, pos=position, wait=wait)
            # Handle both single code and tuple return values
            code = result[0] if isinstance(result, (tuple, list)) else result
            success = self.check_code(code, f'move_track_to_position({position})')

            if success:
                self._update_track_position()

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

        # === Graph consultation (raises in STRICT on off-whitelist) ===
        from_node = self.current_node
        target_node_id = self._predict_target_node_for_rail(location_name)
        edge = self._consult_graph_for_move(target_node_id, location_name)
        speed = self._apply_edge_speed_cap(speed, edge)
        # Rail has no mode question (always linear translation); edge.mode
        # is captured but not used for dispatch on the rail axis.

        success = self.move_track_to_position(position, speed=speed, wait=wait)
        # Inner call cleared the named rail tracker; restore it on success
        # and capture the transition for /control/graph/record.
        if success:
            self.last_rail_location_name = location_name
            to_node = self.current_node
            self._store_transition(
                from_node, to_node, MoveMode.LINEAR, speed,
            )
        return success

    def _validate_track_position(self, position):
        """Validation for track position."""
        is_valid, message = validate_track_position(
            position,
            (0, 700),
            self.track_config.get('danger_zones', [])
        )

        if not is_valid:
            print(f"Error: {message}")
            return False

        return True

    def _validate_track_speed(self, speed):
        """Validation for track speed."""
        is_valid, message = validate_track_speed(speed, (1, 1000))

        if not is_valid:
            print(f"Error: {message}")
            return False

        return True

    def reset_track(self):
        """Reset the linear track to home position."""
        if not self.is_component_enabled('track'):
            print("Linear track is not enabled")
            return False
        return self.move_track_to_position(0)

    def get_track_position(self):
        """Get current linear track position.

        Reading the encoder is a read, not motion, so it is not gated on
        the track being motion-ENABLED — only on the arm being connected
        and a track being configured. Returns None (and leaves the cache
        untouched) when the read is unavailable or fails.
        """
        if not self.arm or not self.enable_track:
            print("Linear track is not available")
            return None

        try:
            ret = self.arm.get_linear_track_pos()
        except Exception as e:
            print(f"Warning: Failed to read track position: {e}")
            return None
        if ret[0] == 0:
            self.last_track_position = ret[1]
            return ret[1]
        print(
            f"Warning: track position read returned code {ret[0]} "
            f"(track state={self.states.get('track')})"
        )
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
        """Move the robot to the named ``robot_home`` preset.

        The SDK's factory home (``move_gohome``, joint zeros) is unsafe on this
        cell, so "home" always routes through the ``robot_home`` definition in
        ``joint_config.yaml`` and travels as a normal named joint move (which
        keeps the motion-graph node tracker correct). If ``robot_home`` is not
        defined we raise rather than fall back to factory home.

        ``mvacc``/``wait`` are accepted for backward compatibility; the named
        move uses the joint-move defaults (blocking).
        """
        if not self.is_component_enabled('arm'):
            print("Arm is not enabled")
            return False

        positions = (self.position_config or {}).get('positions', {})
        if not positions.get('robot_home'):
            raise ValueError(
                "No 'robot_home' position defined in joint_config.yaml; "
                "refusing to fall back to unsafe factory home."
            )

        return self.move_to_named_location('robot_home', speed=speed)

    def get_named_locations(self):
        """Returns a list of all named locations."""
        if self.position_config and 'positions' in self.position_config:
            return list(self.position_config['positions'].keys())
        return []

    # =============================================================================
    # MOTION GRAPH (Phase 1: introspection only — no enforcement)
    # =============================================================================

    def _commanded_gripper_stroke(self) -> Optional[float]:
        """Return the last commanded gripper stroke, or None if unknown.

        ``last_gripper_position`` is set on every successful SDK gripper
        call and serves as the commanded stroke for node resolution.
        """
        stroke = getattr(self, 'last_gripper_position', None)
        if not isinstance(stroke, (int, float)):
            return None
        return float(stroke)

    @property
    def current_node(self) -> Optional[str]:
        """Graph node id matching the controller's arm+rail position, or None.

        Nodes are arm positions only (schema 0.2) — the gripper is a
        separate leaf, exposed via ``current_gripper_state``. Returns
        None when either coordinate is unpinned (e.g., after a raw move
        or before the first named move), or when no node matches.
        """
        if self.motion_graph is None:
            return None
        node = self.motion_graph.find_node(
            arm=self.last_arm_pose_name,
            rail=self.last_rail_location_name,
        )
        return node.id if node else None

    @property
    def current_gripper_state(self) -> Optional[str]:
        """Catalog gripper-state name matching the commanded stroke, or None.

        None when the stroke is unknown or doesn't match any catalog
        state (off-catalog stroke, e.g. after a manual stroke command).
        """
        if self.motion_graph is None:
            return None
        return self.motion_graph.resolve_gripper_state(
            self._commanded_gripper_stroke()
        )

    def reachable_node_ids(self) -> list[str]:
        """Outgoing target node ids traversable with the current gripper
        state (empty if off-grid or the gripper state is off-catalog)."""
        if self.motion_graph is None:
            return []
        return self.motion_graph.allowed_targets_for_state(
            self.current_node, self.current_gripper_state,
        )

    def allowed_gripper_targets(self) -> list[str]:
        """Gripper states reachable via the current node's transition
        whitelist (empty if off-grid or state unknown)."""
        if self.motion_graph is None:
            return []
        return self.motion_graph.allowed_gripper_targets(
            self.current_node, self.current_gripper_state,
        )

    def suggest_current_node(
        self,
        joint_tolerance_deg: float = 10.0,
        rail_tolerance_mm: float = 2.0,
    ) -> NodeMatch:
        """Compute the best-match graph node for the controller's physical
        state. Used by /graph/nearest and as the default safety check for
        /control/graph/recover_to.

        Returns an empty NodeMatch when no graph is loaded.
        """
        if self.motion_graph is None:
            return NodeMatch(
                node_id=None, arm_residual=None, rail_residual=None,
                gripper_state=None, gripper_match=False, within_tolerance=False,
            )
        # Refresh live hardware readings so the match reflects where the
        # robot physically is right now, not a possibly-stale cache — this
        # is the Recover / nearest path, where a wrong rail read snaps to
        # the wrong node.
        if self.arm:
            self._update_positions()
            self._update_track_position()
        # Resolve named arm poses to joint angles (skip cartesian dicts —
        # those need a different recovery path).
        arm_poses: dict[str, list[float]] = {}
        for name, pose in (self.position_config.get('positions') or {}).items():
            if isinstance(pose, list):
                arm_poses[name] = list(pose)
        # Resolve rail location names to mm (handles both `name: 62`
        # and `name: {position: 62, ...}` shapes).
        rail_positions: dict[str, float] = {}
        for name, val in (self.track_config.get('locations') or {}).items():
            if isinstance(val, (int, float)):
                rail_positions[name] = float(val)
            elif isinstance(val, dict):
                p = val.get('position')
                if isinstance(p, (int, float)):
                    rail_positions[name] = float(p)
        return find_nearest_node(
            self.motion_graph,
            current_joints=self.last_joints,
            current_rail_mm=self.last_track_position,
            current_gripper_stroke=self._commanded_gripper_stroke(),
            arm_pose_joints=arm_poses,
            rail_position_mm=rail_positions,
            joint_tolerance_deg=joint_tolerance_deg,
            rail_tolerance_mm=rail_tolerance_mm,
        )

    def recover_to(
        self, node_id: str, force: bool = False,
        gripper_state: Optional[str] = None,
    ) -> dict:
        """Operator-declared re-pin to a known node after off-grid travel.

        Without ``force``, suggest_current_node() must agree with the
        requested node id AND be within tolerance — otherwise raises
        RecoveryMismatch. With ``force=True``, the operator asserts the
        position is correct and the suggestion check is skipped (use
        for cartesian-dict presets the nearest-node algo can't score).

        ``gripper_state`` optionally declares the gripper leaf (must be
        allowed at the node); when given, last_gripper_position is set
        to that state's catalog stroke — *commanded* only, no physical
        gripper move. When omitted, the current commanded stroke is
        kept and simply validated against the node's allowed states
        (unless forced).
        """
        if self.motion_graph is None:
            raise RuntimeError("motion_graph not loaded")
        node = self.motion_graph.node(node_id)  # raises UnknownNodeError

        if gripper_state is not None:
            # An explicitly declared state must be one of the node's
            # leaves even under force — otherwise the pinned pair would
            # violate the data model.
            if not node.allows_gripper(gripper_state):
                raise GripperTransitionError(
                    node_id, self.current_gripper_state, gripper_state,
                    f"state {gripper_state!r} is not allowed at node "
                    f"{node_id!r} (allowed: {list(node.gripper_states)})",
                )

        if not force:
            match = self.suggest_current_node()
            if match.node_id != node_id or not match.within_tolerance:
                raise RecoveryMismatch(
                    requested=node_id,
                    suggested=match.node_id,
                    arm_residual=match.arm_residual,
                    rail_residual=match.rail_residual,
                )
            if gripper_state is None and not node.allows_gripper(match.gripper_state):
                raise GripperTransitionError(
                    node_id, match.gripper_state, "?",
                    f"current gripper stroke resolves to "
                    f"{match.gripper_state!r}, which is not allowed at node "
                    f"{node_id!r} (allowed: {list(node.gripper_states)}); "
                    f"pass gripper_state explicitly or use force",
                )

        self.last_arm_pose_name = node.arm
        self.last_rail_location_name = node.rail
        if gripper_state is not None:
            self.last_gripper_position = (
                self.motion_graph.gripper_state(gripper_state).stroke
            )
        return {
            "recovered_to": node_id,
            "current_node": self.current_node,
            "gripper_state": self.current_gripper_state,
        }

    def _verify_gripper(self, commanded_stroke: float, intent: GripIntent) -> bool:
        """Verify the gripper outcome against the node's intent.

        Reads the actual hardware position and compares it to the
        commanded stroke using tolerances from gripper_config.yaml.

        GRASP:    actual must exceed commanded by at least grasp_min_offset
                  (the object is blocking the jaws open). Returns False
                  and prints an error when the jaws reached commanded
                  (nothing was grabbed).
        POSITION: actual must be within position_tolerance of commanded
                  (free travel confirmed). Returns False when jaws were
                  blocked early.
        NONE:     always returns True without reading hardware.
        """
        if intent == GripIntent.NONE:
            return True

        actual = self.get_gripper_position()
        if actual is None:
            print(f"[motion_graph] _verify_gripper: could not read gripper position; skipping check")
            return True

        config = getattr(self, 'current_gripper_config', {}) or {}

        if intent == GripIntent.GRASP:
            min_offset = float(config.get('grasp_min_offset', 3))
            max_offset = config.get('grasp_max_offset')
            gap = float(actual) - float(commanded_stroke)
            if gap < min_offset:
                print(
                    f"[motion_graph] grasp FAILED: actual={actual}, commanded={commanded_stroke}, "
                    f"gap={gap:.1f} < required {min_offset} — object not held or slipped"
                )
                return False
            if max_offset is not None and gap > float(max_offset):
                print(
                    f"[motion_graph] grasp FAILED: actual={actual}, commanded={commanded_stroke}, "
                    f"gap={gap:.1f} > max_offset {max_offset} — jaws may have been obstructed early"
                )
                return False
            return True

        if intent == GripIntent.POSITION:
            tolerance = float(config.get('position_tolerance', 3))
            delta = abs(float(actual) - float(commanded_stroke))
            if delta > tolerance:
                print(
                    f"[motion_graph] position close FAILED: actual={actual}, commanded={commanded_stroke}, "
                    f"delta={delta:.1f} > tolerance {tolerance} — jaws blocked or did not reach target"
                )
                return False
            return True

        return True

    def move_to_node(self, node_id: str, speed=None) -> bool:
        """Move to a graph node by id, driving the rail when required.

        The gripper stroke is invariant along edges and never touched
        here — grip/release/narrow happens separately via
        ``set_gripper_state`` while parked at a node.

        Same-rail edges are a pure arm move. A cross-rail edge (the
        target sits at a different rail location, e.g.
        ``deck_home -> robot_home``) is one graph edge but two physical
        axes: it is validated once here, then dispatched arm-first,
        rail-second with per-axis graph consultation suppressed (the
        state between the two sub-moves is a non-node by design).

        Returns True on success, False on any sub-move failure. Raises
        EdgeNotAllowedError in STRICT mode for disallowed moves
        (including edges the current gripper state may not ride).
        """
        if self.motion_graph is None:
            print("[motion_graph] move_to_node: motion_graph not loaded")
            return False

        node = self.motion_graph.node(node_id)  # raises UnknownNodeError

        # Same-rail: a pure arm move reaches the node; keep per-axis
        # consultation intact so STRICT still gates the edge as before.
        if node.rail == self.last_rail_location_name:
            return self.move_to_named_location(node.arm, speed=speed)

        # Cross-rail transit: validate the whole edge once (raises in
        # STRICT if the edge is missing, we are off-grid, or the gripper
        # state may not ride it), then dispatch the two axes.
        from_node = self.current_node
        edge = self._consult_graph_for_move(node_id, node_id)
        capped = self._apply_edge_speed_cap(speed, edge)

        self._suppress_graph_consult = True
        try:
            # Arm first: the transit gateway poses are joint-list presets,
            # so the fallback dispatch is joint mode (matching edge.mode).
            if not self.move_to_named_location(node.arm, speed=capped):
                print(
                    f"[motion_graph] move_to_node: arm move to {node.arm!r} "
                    f"failed"
                )
                return False
            # Rail second: no speed passed so the track uses its configured
            # default (edge.speed is an arm speed, not a rail speed).
            if not self.move_track_to_named_location(node.rail):
                print(
                    f"[motion_graph] move_to_node: rail move to {node.rail!r} "
                    f"failed"
                )
                return False
        finally:
            self._suppress_graph_consult = False

        # Both trackers are now set by the sub-moves; record the real edge.
        self._store_transition(
            from_node,
            self.current_node,
            edge.mode if edge is not None else MoveMode.JOINT,
            capped,
        )
        return True

    def _arm_is_moving(self) -> bool:
        """True when the SDK reports the arm in motion (state 1)."""
        arm = getattr(self, 'arm', None)
        return arm is not None and getattr(arm, 'state', None) == 1

    def set_gripper_state(self, state_name: str) -> bool:
        """Change the gripper to a catalog state while parked at a node.

        This is the ONLY graph-sanctioned way to change the gripper:
        the stroke is invariant during arm motion, so transitions are
        gated on (a) the arm not moving, (b) a pinned current node, and
        (c) the transition appearing in that node's whitelist (STRICT
        rejects violations; ADVISORY/OFF warn and proceed).

        Actuates the gripper to the state's stroke, then verifies per
        the state's intent:

        - grasp:    jaws must settle above the commanded stroke (an
                    object is holding them open); reaching the stroke
                    exactly means nothing was gripped → failure.
        - position: jaws must reach the commanded stroke; stalling
                    early means blocked → failure.
        - none:     no verification (empty / fully open).

        Returns True on success, False on actuation or verification
        failure. Raises GripperTransitionError for gating violations.
        """
        if self.motion_graph is None:
            raise RuntimeError("motion_graph not loaded")
        target = self.motion_graph.gripper_state(state_name)  # raises GraphError

        current_state = self.current_gripper_state
        node_id = self.current_node

        # Interlock: never change the gripper while the arm is moving.
        if self._arm_is_moving():
            raise GripperTransitionError(
                node_id, current_state, state_name,
                "arm is moving; gripper state can only change while "
                "parked at a node",
            )

        if current_state == state_name:
            return True  # already there — no-op

        # Whitelist gating against the current node's transitions.
        allowed = self.motion_graph.allowed_gripper_targets(node_id, current_state)
        violation: Optional[str] = None
        if node_id is None:
            violation = (
                "current position is off-grid; recover to a node before "
                "changing the gripper"
            )
        elif current_state is None:
            violation = (
                f"current gripper stroke "
                f"({self._commanded_gripper_stroke()!r}) matches no catalog "
                f"state; recover with an explicit gripper_state first"
            )
        elif state_name not in allowed:
            violation = (
                f"transition {current_state!r} -> {state_name!r} is not "
                f"whitelisted at node {node_id!r} (allowed: {allowed})"
            )

        if violation is not None:
            if self.graph_mode == GraphMode.STRICT:
                raise GripperTransitionError(
                    node_id, current_state, state_name, violation,
                )
            print(f"[motion_graph] advisory: {violation}")

        # ── Actuation ─────────────────────────────────────────────────
        config = getattr(self, 'current_gripper_config', {}) or {}
        if state_name == "empty":
            gripper_ok = self.open_gripper(wait=True)
        else:
            gripper_ok = self.move_gripper_to_stroke(
                target.stroke, force=config.get('force', None), wait=True,
            )
        if not gripper_ok:
            print(
                f"[motion_graph] set_gripper_state: gripper move to "
                f"{target.stroke} ({state_name}) failed"
            )
            return False

        # ── Verification ──────────────────────────────────────────────
        return self._verify_gripper(target.stroke, target.intent)

    def set_graph_mode(self, mode: GraphMode) -> None:
        """Set the motion-graph enforcement mode. Safe at any time."""
        if self.motion_graph is None and mode != GraphMode.OFF:
            raise RuntimeError(
                "cannot enable graph mode: motion_graph.yaml is not loaded"
            )
        self.graph_mode = mode
        print(f"[motion_graph] mode set to {mode.value}")

    def _predict_target_node_for_arm_pose(self, arm_pose_name: str) -> Optional[str]:
        """Predict the node id we'd land on after move_to_named_location.

        A pure-arm move keeps the rail unchanged. Returns None if the
        rail is unpinned or no node matches (arm, rail).
        """
        if self.motion_graph is None:
            return None
        node = self.motion_graph.find_node(
            arm=arm_pose_name,
            rail=self.last_rail_location_name,
        )
        return node.id if node else None

    def _predict_target_node_for_rail(self, rail_location_name: str) -> Optional[str]:
        """Predict the node id we'd land on after move_track_to_named_location."""
        if self.motion_graph is None:
            return None
        node = self.motion_graph.find_node(
            arm=self.last_arm_pose_name,
            rail=rail_location_name,
        )
        return node.id if node else None

    def _consult_graph_for_move(
        self, target_node_id: Optional[str], target_label: str,
    ) -> Optional[Edge]:
        """Look up the edge from current_node to target_node_id.

        Returns the Edge when one exists, None otherwise. In STRICT mode
        raises EdgeNotAllowedError on any failure (target unknown,
        current off-grid, or no whitelisted edge). In ADVISORY mode logs
        a warning and returns None. In OFF mode returns None silently.
        """
        if self.motion_graph is None or self.graph_mode == GraphMode.OFF:
            return None

        # Cross-rail sub-moves: the edge was already validated once at the
        # move_to_node level. The intermediate (arm, rail) state is a
        # non-node by design, so skip the per-axis check here.
        if self._suppress_graph_consult:
            return None

        current_id = self.current_node

        if target_node_id is None:
            msg = (
                f"target {target_label!r} does not resolve to a graph node "
                f"(rail={self.last_rail_location_name!r})"
            )
            if self.graph_mode == GraphMode.STRICT:
                raise EdgeNotAllowedError(current_id, target_label, msg)
            print(f"[motion_graph] advisory: {msg}")
            return None

        if current_id is None:
            msg = (
                f"current position is off-grid; cannot transition to "
                f"{target_node_id!r}. Call a named move that matches "
                f"actual position to re-pin."
            )
            if self.graph_mode == GraphMode.STRICT:
                raise EdgeNotAllowedError(None, target_node_id, msg)
            print(f"[motion_graph] advisory: {msg}")
            return None

        edge = self.motion_graph.find_edge(current_id, target_node_id)
        if edge is None:
            msg = f"no whitelisted edge {current_id!r} -> {target_node_id!r}"
            if self.graph_mode == GraphMode.STRICT:
                raise EdgeNotAllowedError(current_id, target_node_id, msg)
            print(f"[motion_graph] advisory: {msg}")
            return None

        # The gripper stroke is invariant along an edge, so the current
        # state must be allowed at both endpoints (occupancy gating).
        gripper_state = self.current_gripper_state
        if not self.motion_graph.edge_allows_gripper(edge, gripper_state):
            if gripper_state is None:
                msg = (
                    f"gripper stroke "
                    f"({self._commanded_gripper_stroke()!r}) matches no "
                    f"catalog state; recover before moving on the graph"
                )
            else:
                msg = (
                    f"edge {current_id!r} -> {target_node_id!r} is not "
                    f"traversable with gripper state {gripper_state!r}"
                )
            if self.graph_mode == GraphMode.STRICT:
                raise EdgeNotAllowedError(current_id, target_node_id, msg)
            print(f"[motion_graph] advisory: {msg}")
            # Advisory: the edge still informs mode/speed.

        return edge

    def _apply_edge_speed_cap(self, requested: Optional[float], edge: Optional[Edge]) -> Optional[float]:
        """Clamp the caller's speed to edge.speed in STRICT mode.

        edge.speed is the maximum permitted speed for this transition;
        callers can ask for slower (more cautious) but not faster.
        """
        if (self.graph_mode != GraphMode.STRICT
                or edge is None or edge.speed is None):
            return requested
        if requested is None or requested > edge.speed:
            if requested is not None and requested > edge.speed:
                print(
                    f"[motion_graph] clamping speed {requested} -> "
                    f"{edge.speed} (edge.speed cap)"
                )
            return edge.speed
        return requested

    def _store_transition(
        self,
        from_node: Optional[str],
        to_node: Optional[str],
        mode_used: MoveMode,
        speed_used: Optional[float],
    ) -> None:
        """Capture a successful node-to-node transition for graph/record.

        Only stored when both endpoints are pinned nodes — recording an
        off-grid move makes no sense as a graph edge.
        """
        if from_node is None or to_node is None or from_node == to_node:
            return
        self.last_transition = {
            "from_node": from_node,
            "to_node": to_node,
            "mode": mode_used.value,
            "speed": speed_used,
            "timestamp": time.time(),
        }

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
        self._emit_event("shutdown", message="Controller disconnecting")
        self._emit_state_transition("requires_init", message="Controller disconnected")
        self.alive = False
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
            print(f"Safety violation: {error_msg}")
        return is_valid

    def _validate_joint_angles(self, angles: List[float]) -> bool:
        """Validate joint angles against model-specific limits."""
        is_valid, error_msg = validate_joint_angles(angles, self.joint_limits)
        if not is_valid:
            print(f"Joint validation error: {error_msg}")
        return is_valid

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
            # Capture the slip/detect register for /status now that the jaws
            # have settled — this is the move whose outcome we verify.
            self.refresh_gripper_status()
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
        """Print a force/torque safety alert."""
        message = "FORCE/TORQUE SAFETY ALERT!\n"
        if force_violations:
            message += f"Forces: {', '.join(force_violations)}\n"
        if torque_violations:
            message += f"Torques: {', '.join(torque_violations)}\n"
        message += f"Current data: {[f'{x:.2f}' for x in data]}"

        print(f"[ALERT] {message}")

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

    def move_plate_linear(self, target_location, speed=None):
        """
        Move tool linearly from current position to target position.
        Tool maintains the same absolute orientation throughout the movement.
        
        Args:
            target_location (str): Name of target location from joint_config.yaml
            speed (float): Movement speed (default: tcp_speed)
            
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
        
        # Straight-line move to the target X/Y/Z, holding the CURRENT tool
        # orientation throughout (absolute direction in space).
        success = self.move_to_position(
            x=target_cartesian[0], y=target_cartesian[1], z=target_cartesian[2],
            roll=start_cartesian[3], pitch=start_cartesian[4], yaw=start_cartesian[5],
            speed=speed, check_collision=False, wait=True
        )
        if not success:
            print(f"Error: Failed linear movement to '{target_location}'")
            return False

        print(f"[OK] Successfully completed linear movement to '{target_location}'")
        # move_to_position cleared the named pose tracker; we arrived at
        # target_location, so pin it.
        self.last_arm_pose_name = target_location
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
                    if hasattr(self.arm, 'get_forward_kinematics'):
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

