"""The RealSense camera on the STATUS_SPEC envelope (core/status_builder.py).

``components.realsense_camera`` + ``details.realsense`` appear iff a
configured camera is installed process-wide, they never touch
``equipment_status`` (arm motion does not depend on the camera), and a
camera whose reporting raises is dropped rather than breaking ``/status``.
"""

import os
import sys
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core import realsense_camera as rc  # noqa: E402
from src.core.status_builder import build_status  # noqa: E402
from test.test_status_envelope import _fake_controller  # noqa: E402


def _camera(configured=True, component=None, block=None, raises=False):
    cam = MagicMock()
    cam.configured = configured
    if raises:
        cam.component_status.side_effect = RuntimeError("usb went away")
        cam.status_block.side_effect = RuntimeError("usb went away")
    else:
        cam.component_status.return_value = component
        cam.status_block.return_value = block
    return cam


@pytest.fixture
def shared():
    previous = rc.shared_camera()
    yield rc.set_shared
    rc.set_shared(previous)


STREAMING_COMPONENT = {"connected": True, "state": "streaming",
                       "message": "xArm depth camera · Intel RealSense D435I · sn 1234 · 29.8 fps"}
STREAMING_BLOCK = {"state": "streaming", "installed": True, "device": {"serial": "1234"},
                   "devices": [{"serial": "1234"}], "streams": {}, "fps_measured": 29.8,
                   "frames_captured": 900, "last_frame_age_s": 0.03, "warnings": [], "reason": None}


def test_absent_when_no_shared_camera(shared):
    shared(None)
    env = build_status(_fake_controller())
    assert "realsense_camera" not in env.components
    assert "realsense" not in env.details


def test_absent_when_camera_unconfigured(shared):
    shared(_camera(configured=False, component=STREAMING_COMPONENT, block=STREAMING_BLOCK))
    env = build_status(_fake_controller())
    assert "realsense_camera" not in env.components
    assert "realsense" not in env.details


def test_present_when_configured(shared):
    shared(_camera(component=STREAMING_COMPONENT, block=STREAMING_BLOCK))
    env = build_status(_fake_controller())
    comp = env.components["realsense_camera"]
    assert comp.connected is True and comp.state == "streaming"
    assert comp.message.startswith("xArm depth camera")
    assert env.details["realsense"] == STREAMING_BLOCK
    assert env.equipment_status == "ready"


def test_unplugged_camera_does_not_degrade_the_arm(shared):
    shared(_camera(
        component={"connected": False, "state": "disconnected",
                   "message": "xArm depth camera · no RealSense device connected"},
        block={**STREAMING_BLOCK, "state": "off", "device": None, "devices": [],
               "fps_measured": None, "reason": "no RealSense device connected"},
    ))
    env = build_status(_fake_controller())
    assert env.equipment_status == "ready"            # §2.2: not a run-blocking subsystem
    assert env.components["realsense_camera"].connected is False
    assert env.components["realsense_camera"].state == "disconnected"
    assert env.details["realsense"]["reason"] == "no RealSense device connected"
    assert env.message is None or "realsense" not in env.message.lower()


def test_camera_error_does_not_degrade_the_arm(shared):
    shared(_camera(component={"connected": True, "state": "error", "message": "camera lost"},
                   block={**STREAMING_BLOCK, "state": "error", "reason": "camera lost"}))
    env = build_status(_fake_controller())
    assert env.equipment_status == "ready"
    assert env.components["realsense_camera"].state == "error"


def test_reporting_failure_is_dropped_not_raised(shared):
    shared(_camera(raises=True))
    env = build_status(_fake_controller())
    assert "realsense_camera" not in env.components
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

    cam = rc.RealSenseCamera({"enabled": True, "label": "bench cam"}, rs_module=None, np_module=None)
    cam.installed = False
    cam.install_error = "ModuleNotFoundError: No module named 'pyrealsense2'"
    shared(cam)
    env = build_status(_fake_controller())
    comp = env.components["realsense_camera"]
    assert comp.state == "driver_missing" and comp.connected is False
    assert "uv sync --extra realsense" in env.details["realsense"]["reason"]
    assert env.equipment_status == "ready"


def test_present_before_connect(shared):
    """The camera is process-wide: the no-controller envelope carries it too."""
    shared(_camera(component=STREAMING_COMPONENT, block=STREAMING_BLOCK))
    env = build_status(None)
    assert env.equipment_status == "requires_init"
    assert env.components["realsense_camera"].state == "streaming"
    assert env.details["realsense"] == STREAMING_BLOCK
    assert env.allowed_actions == ["connect"]


def test_absent_before_connect_when_unconfigured(shared):
    shared(_camera(configured=False, component=STREAMING_COMPONENT, block=STREAMING_BLOCK))
    env = build_status(None)
    assert "realsense_camera" not in env.components
    assert env.details == {}
