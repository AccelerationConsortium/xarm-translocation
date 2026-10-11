"""Joint trajectories: interpolation, validation, sessions and the executor.

Robot-independent; each robot supplies a backend (``executor.XArm*Backend``
for the xArm5, ``ur_backend.URServoJBackend`` for UR e-Series). Importing it
never imports a vendor SDK. See src/docs/SERVOJ_TRAJECTORY_PLAN.md and
src/docs/UR5E_TRAJECTORY_PLAN.md.
"""
