# Isolated Windows prototype deployment

Do not install this prerelease into an existing pyxarm or MG400 environment.
Use a separate robot-motion checkout and virtual environment. The pyxarm
compatibility modules retain their old import names; co-installation with the
old distribution in one environment is not supported.

Install selected extras with uv sync --extra ur (UI/API are always included).
The existing pyxarm command remains available after installing --extra xarm,
but is not used by the prototype service.

Create a gitignored .state/robot-motion.local.json with the selected model,
equipment identity, robot_host, observe, and control_enabled=false. No machine
address, calibration, credential or deployment path belongs in a commit.
Observe defaults to false. Enable read-only UR observation only after confirming
network reachability; this never creates an RTDE control connection.

From an elevated PowerShell, invoke deploy/install-prototype.ps1 with the
explicit -BindAddress for the PC's Tailnet interface. First run without -Apply
to verify prerequisites; then run with -Apply to register only the new
robot-motion-prototype NSSM service on port 8075.

The prototype runs as LocalSystem solely under the lab's read-only monitoring
exception: no vendor user profile, COM port, or control interface is used.
This is not approval for a future control deployment under that account.
It binds only the selected Tailnet IP; its firewall rule admits Tailnet IPv4
clients only. There is no public edge route. Logs rotate online at 10 MiB.
The script refuses occupied ports and pre-existing service names.

Verify /health, /status, /drivers, /web/, /graph, /docs and /agent-docs from
the deployment PC and the aggregator. A healthy process does not imply a
reachable or commissioned robot. No /control endpoints are present in this
prototype. Test that a POST to /control/graph/move_to returns 404.

Rollback affects only robot-motion-prototype: stop that service and remove its
dedicated firewall rule if retiring the prototype. Preserve its local config
and source for diagnosis. Do not stop xarm, dobot-mg400, or sdl-lab-hostops.

Existing xArm regression tests run offline with:
python -m pytest -p tests_robot_motion.offline_guard test -m "not integration and not docker"
The plugin forbids all outbound IP socket connections. Prototype tests run
separately because their import-isolation assertion deliberately requires a
fresh process with no xArm modules imported.

