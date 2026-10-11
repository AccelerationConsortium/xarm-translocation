"""Moved to ``robot_motion.trajectory.executor``; kept as an alias."""
from robot_motion.trajectory.executor import *  # noqa: F401,F403
from robot_motion.trajectory.executor import (  # noqa: F401
    CANCELLED, COMPLETED, CREATED, EXPIRED, FAILED, STOPPED, BackendError, RealtimeJointMonitor,
    StopRequested, TrajectoryError, TrajectoryManager, TrajectorySettings, XArmOnlinePlanningBackend,
    XArmServoJBackend, _health_fault, _sample_fault, make_backend,
)
