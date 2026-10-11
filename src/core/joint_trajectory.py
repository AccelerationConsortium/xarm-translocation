"""Moved to ``robot_motion.trajectory.joint_trajectory``; kept as an alias."""
from robot_motion.trajectory.joint_trajectory import *  # noqa: F401,F403
from robot_motion.trajectory.joint_trajectory import (  # noqa: F401
    JOINT_EXECUTION_MODEL, MAX_REPORTED_ERRORS, JointLimits, JointPoint, JointTrajectory,
    estimated_velocities, lead_in, lead_in_duration, stop_deceleration, validate_joint_trajectory,
)
