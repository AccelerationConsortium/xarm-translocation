"""The RealSense cameras on the STATUS_SPEC envelope (core/status_builder.py).

One ``components.realsense_<camera_id>`` per configured camera, plus a
``details.realsense`` block holding ``default``, the per-camera ``cameras``
map and the shared ``captures`` summary. They appear iff at least one
configured camera is in the process-wide registry, they never touch
``equipment_status`` (arm motion does not depend on a camera), and a camera
whose reporting raises is dropped rather than breaking ``/status``.
"""

import os
import sys
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core import realsense_camera as rc  # noqa: E402
from src.core.status_builder import build_status  # noqa: E402
from test.test_status_envelope import _fake_controller  # noqa: E402


def _camera(configured=True, component=None, block=None, raises=False,
            camera_id="rs435i", streaming=False, start_on_demand=True):
    cam = MagicMock()
    cam.camera_id = camera_id
    cam.configured = configured
    cam.streaming = streaming
    cam.start_on_demand = start_on_demand
    if raises:
        cam.component_status.side_effect = RuntimeError("usb went away")
        cam.status_block.side_effect = RuntimeError("usb went away")
    else:
        cam.component_status.return_value = component
        cam.status_block.return_value = block
    return cam


@pytest.fixture
def shared():
    """Install one or more cameras in the registry, keyed by their own id.

    Takes cameras (or None / nothing) so the existing single-camera tests read
    the same as before while the multi-camera ones pass two.
    """
    previous = rc.cameras()

    def install(*cameras):
        installed = [c for c in cameras if c is not None]
        rc.set_cameras({c.camera_id: c for c in installed})

    yield install
    rc.set_cameras(previous)


CAM = "realsense_rs435i"


STREAMING_COMPONENT = {"connected": True, "state": "streaming",
                       "message": "xArm depth camera · Intel RealSense D435I · sn 1234 · 29.8 fps"}
STREAMING_BLOCK = {"state": "streaming", "installed": True, "device": {"serial": "1234"},
                   "devices": [{"serial": "1234"}], "streams": {}, "fps_measured": 29.8,
                   "frames_captured": 900, "last_frame_age_s": 0.03, "warnings": [], "reason": None}


def test_absent_when_no_camera_is_registered(shared):
    shared()
    env = build_status(_fake_controller())
    assert not [k for k in env.components if k.startswith("realsense")]
    assert "realsense" not in env.details


def test_absent_when_camera_unconfigured(shared):
    shared(_camera(configured=False, component=STREAMING_COMPONENT, block=STREAMING_BLOCK))
    env = build_status(_fake_controller())
    assert CAM not in env.components
    assert "realsense" not in env.details


def test_present_when_configured(shared):
    shared(_camera(component=STREAMING_COMPONENT, block=STREAMING_BLOCK))
    env = build_status(_fake_controller())
    comp = env.components[CAM]
    assert comp.connected is True and comp.state == "streaming"
    assert comp.message.startswith("xArm depth camera")
    assert env.details["realsense"]["cameras"] == {"rs435i": STREAMING_BLOCK}
    assert env.details["realsense"]["default"] == "rs435i"
    assert env.equipment_status == "ready"


def test_one_component_per_camera(shared):
    """Two cameras fail independently, so they are two components -- a merged
    entry would hide the working one behind the unplugged one."""
    first = _camera(camera_id="rs435i", component=STREAMING_COMPONENT, block=STREAMING_BLOCK)
    second = _camera(
        camera_id="overhead",
        component={"connected": False, "state": "disconnected", "message": "overhead · no device"},
        block={**STREAMING_BLOCK, "state": "off", "device": None, "devices": [],
               "reason": "no RealSense device connected"},
    )
    shared(first, second)
    env = build_status(_fake_controller())
    assert env.components["realsense_rs435i"].state == "streaming"
    assert env.components["realsense_overhead"].state == "disconnected"
    assert set(env.details["realsense"]["cameras"]) == {"rs435i", "overhead"}
    assert env.details["realsense"]["default"] is None      # two cameras, no default
    assert env.equipment_status == "ready"


def test_capture_action_is_advertised_when_any_camera_qualifies(shared, tmp_path):
    """``realsense.capture`` is one verb for a family of routes: one usable
    camera and an enabled store is enough to offer it."""
    import src.core.realsense_captures as rcap

    previous_store = rcap.shared_store()
    rcap.configure_shared({"enabled": True, "root": str(tmp_path)})
    try:
        cold = _camera(camera_id="overhead", start_on_demand=False, streaming=False,
                       component=STREAMING_COMPONENT, block=STREAMING_BLOCK)
        shared(cold)
        assert "realsense.capture" not in build_status(_fake_controller()).allowed_actions

        warm = _camera(camera_id="rs435i", start_on_demand=True,
                       component=STREAMING_COMPONENT, block=STREAMING_BLOCK)
        shared(cold, warm)
        assert "realsense.capture" in build_status(_fake_controller()).allowed_actions

        rcap.set_shared(rcap.CaptureStore({"enabled": False}))
        assert "realsense.capture" not in build_status(_fake_controller()).allowed_actions
    finally:
        rcap.set_shared(previous_store)


def test_unplugged_camera_does_not_degrade_the_arm(shared):
    shared(_camera(
        component={"connected": False, "state": "disconnected",
                   "message": "xArm depth camera · no RealSense device connected"},
        block={**STREAMING_BLOCK, "state": "off", "device": None, "devices": [],
               "fps_measured": None, "reason": "no RealSense device connected"},
    ))
    env = build_status(_fake_controller())
    assert env.equipment_status == "ready"            # §2.2: not a run-blocking subsystem
    assert env.components[CAM].connected is False
    assert env.components[CAM].state == "disconnected"
    assert env.details["realsense"]["cameras"]["rs435i"]["reason"] == "no RealSense device connected"
    assert env.message is None or "realsense" not in env.message.lower()


def test_camera_error_does_not_degrade_the_arm(shared):
    shared(_camera(component={"connected": True, "state": "error", "message": "camera lost"},
                   block={**STREAMING_BLOCK, "state": "error", "reason": "camera lost"}))
    env = build_status(_fake_controller())
    assert env.equipment_status == "ready"
    assert env.components[CAM].state == "error"


def test_reporting_failure_is_dropped_not_raised(shared):
    shared(_camera(raises=True))
    env = build_status(_fake_controller())
    assert CAM not in env.components
    assert "realsense" not in env.details
    assert env.equipment_status == "ready"


def test_status_is_side_effect_free_for_the_camera(shared):
    cam = _camera(component=STREAMING_COMPONENT, block=STREAMING_BLOCK)
    shared(cam)
    build_status(_fake_controller())
    assert not cam.start.called and not cam.stop.called and not cam.ensure_started.called


def test_real_camera_class_integrates(shared):
    """End to end with the real class (no backend): configured but the extra
    'missing' -> a driver_missing component, an envelope that still builds."""
    class NoRS:  # noqa: D401 - a backend whose import fails
        pass

    cam = rc.RealSenseCamera({"enabled": True, "label": "bench cam"},
                             camera_id="rs435i", rs_module=None, np_module=None)
    cam.installed = False
    cam.install_error = "ModuleNotFoundError: No module named 'pyrealsense2'"
    shared(cam)
    env = build_status(_fake_controller())
    comp = env.components[CAM]
    assert comp.state == "driver_missing" and comp.connected is False
    block = env.details["realsense"]["cameras"]["rs435i"]
    assert "uv sync --extra realsense" in block["reason"]
    assert block["camera_id"] == "rs435i"
    assert env.equipment_status == "ready"


def test_present_before_connect(shared):
    """The camera is process-wide: the no-controller envelope carries it too."""
    shared(_camera(component=STREAMING_COMPONENT, block=STREAMING_BLOCK))
    env = build_status(None)
    assert env.equipment_status == "requires_init"
    assert env.components[CAM].state == "streaming"
    assert env.details["realsense"]["cameras"] == {"rs435i": STREAMING_BLOCK}
    assert env.allowed_actions == ["connect"]


def test_absent_before_connect_when_unconfigured(shared):
    shared(_camera(configured=False, component=STREAMING_COMPONENT, block=STREAMING_BLOCK))
    env = build_status(None)
    assert CAM not in env.components
    assert env.details == {}
