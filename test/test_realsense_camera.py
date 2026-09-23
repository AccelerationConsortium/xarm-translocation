"""Tests for the RealSense camera layer (core/realsense_camera.py).

Drives the whole lifecycle -- enumeration, start/stop, the capture thread,
JPEG/PNG encoding, depth lookup with deprojection, the /status blocks --
against a fake ``pyrealsense2`` namespace, so nothing here needs a camera,
the librealsense DLL, or USB. The fake mirrors exactly the surface the
module touches (documented on each class) and nothing more, so a change to
which pyrealsense2 calls the module makes shows up here as a failing test.

numpy + Pillow are real (they are part of the ``realsense`` extra); the
module is skipped when they are missing so the suite still runs on a
machine without the extra.
"""

import os
import sys
import threading
import time

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("PIL")

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

from src.core import realsense_camera as rc  # noqa: E402
from src.core.realsense_camera import (  # noqa: E402
    RealSenseCamera,
    RealSenseError,
    RealSenseNotStreaming,
    RealSenseUnavailable,
)


# ---------------------------------------------------------------------------
# Fake pyrealsense2
# ---------------------------------------------------------------------------

class _Enum:
    """Stand-in for rs.stream / rs.format / rs.camera_info members."""

    def __init__(self, name):
        self.name = name

    def __repr__(self):
        return f"rs.{self.name}"


class FakeDevice:
    """rs.device: get_info / supports / first_depth_sensor."""

    def __init__(self, serial, name="Intel RealSense D435I", usb="3.2", fw="5.16.0.1"):
        self._info = {
            "name": name, "serial_number": serial, "firmware_version": fw,
            "usb_type_descriptor": usb, "product_line": "D400", "product_id": "0B3A",
        }

    def supports(self, member):
        return member.name in self._info

    def get_info(self, member):
        return self._info[member.name]

    def first_depth_sensor(self):
        return self

    def get_depth_scale(self):
        return 0.001


class FakeIntrinsics:
    def __init__(self, w, h):
        self.width, self.height = w, h
        self.fx = self.fy = 600.0
        self.ppx, self.ppy = w / 2.0, h / 2.0
        self.model = _Enum("distortion.brown_conrady")
        self.coeffs = [0.0] * 5


class FakeVideoStreamProfile:
    def __init__(self, w, h):
        self._intr = FakeIntrinsics(w, h)

    def as_video_stream_profile(self):
        return self

    def get_intrinsics(self):
        return self._intr


class FakePipelineProfile:
    def __init__(self, device, w, h):
        self._device, self._w, self._h = device, w, h

    def get_device(self):
        return self._device

    def get_stream(self, stream):
        return FakeVideoStreamProfile(self._w, self._h)


class FakeFrame:
    """rs.video_frame / rs.depth_frame: get_data / frame_number / timestamp."""

    def __init__(self, array, number):
        self._array, self._number = array, number

    def get_data(self):
        return self._array

    def get_frame_number(self):
        return self._number

    def get_timestamp(self):
        return 1000.0 + self._number

    def __bool__(self):
        return True


class FakeFrameset:
    def __init__(self, depth, color):
        self._depth, self._color = depth, color

    def get_depth_frame(self):
        return self._depth

    def get_color_frame(self):
        return self._color


class FakePipeline:
    """rs.pipeline. ``rs_ns.frame_source`` produces the next frameset; a
    ``fail_after`` counter lets a test simulate a yanked cable."""

    def __init__(self, rs_ns):
        self.rs = rs_ns
        self.started = False
        self.stopped = False
        self._n = 0

    def start(self, cfg):
        if self.rs.start_error is not None:
            raise RuntimeError(self.rs.start_error)
        self.started = True
        self.rs.started_pipelines.append(self)
        dev = self.rs.devices[0] if not cfg.serial else next(
            d for d in self.rs.devices if d.get_info(_Enum("serial_number")) == cfg.serial
        )
        return FakePipelineProfile(dev, self.rs.width, self.rs.height)

    def wait_for_frames(self, timeout_ms):
        if self.rs.fail_frames:
            raise RuntimeError("Frame didn't arrive within 5000")
        self._n += 1
        time.sleep(self.rs.frame_interval_s)
        depth = np.full((self.rs.height, self.rs.width), 1500, dtype=np.uint16)
        depth[10, 10] = 0                     # a hole
        depth[20:25, 20:25] = 800             # a 5x5 patch at 0.8 m
        depth[22, 22] = 3000                  # noisy centre pixel
        color = np.zeros((self.rs.height, self.rs.width, 3), dtype=np.uint8)
        color[..., 0] = 255                   # pure blue in BGR
        return FakeFrameset(FakeFrame(depth, self._n), FakeFrame(color, self._n))

    def stop(self):
        self.stopped = True


class FakeConfig:
    def __init__(self):
        self.serial = None
        self.streams = []

    def enable_device(self, serial):
        self.serial = serial

    def enable_stream(self, stream, w, h, fmt, fps):
        self.streams.append((stream.name, w, h, fmt.name, fps))


class FakeAlign:
    def __init__(self, stream):
        self.stream = stream

    def process(self, frames):
        return frames


class FakeColorizer:
    def colorize(self, depth_frame):
        d = depth_frame.get_data()
        rgb = np.zeros(d.shape + (3,), dtype=np.uint8)
        rgb[..., 0] = (d // 16).astype(np.uint8)
        return FakeFrame(rgb, depth_frame.get_frame_number())


class FakeContext:
    def __init__(self, rs_ns):
        self.rs = rs_ns

    def query_devices(self):
        if self.rs.enumeration_error:
            raise RuntimeError(self.rs.enumeration_error)
        return list(self.rs.devices)


class FakeRS:
    """The ``pyrealsense2`` namespace as the module sees it."""

    __version__ = "2.58.4-fake"

    class stream:
        depth = _Enum("depth")
        color = _Enum("color")

    class format:
        z16 = _Enum("z16")
        bgr8 = _Enum("bgr8")

    class camera_info:
        name = _Enum("name")
        serial_number = _Enum("serial_number")
        firmware_version = _Enum("firmware_version")
        usb_type_descriptor = _Enum("usb_type_descriptor")
        product_line = _Enum("product_line")
        product_id = _Enum("product_id")

    def __init__(self, devices=None, width=64, height=48):
        self.devices = devices if devices is not None else [FakeDevice("123456")]
        self.width, self.height = width, height
        self.start_error = None
        self.enumeration_error = None
        self.fail_frames = False
        self.frame_interval_s = 0.002
        self.started_pipelines = []
        self.last_config = None

    def context(self):
        return FakeContext(self)

    def pipeline(self):
        return FakePipeline(self)

    def config(self):
        self.last_config = FakeConfig()
        return self.last_config

    def align(self, stream):
        return FakeAlign(stream)

    def colorizer(self):
        return FakeColorizer()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _config(**overrides):
    cfg = {
        "id": "rs435i",
        "enabled": True,
        "serial": "",
        "label": "test cam",
        "autostart": False,
        "start_on_demand": True,
        "idle_timeout_seconds": 0,
        "color": {"enabled": True, "width": 64, "height": 48, "fps": 30},
        "depth": {"enabled": True, "width": 64, "height": 48, "fps": 30},
        "align_depth_to_color": True,
        "jpeg_quality": 80,
        "frame_timeout_ms": 500,
        "max_consecutive_frame_failures": 3,
    }
    cfg.update(overrides)
    return cfg


@pytest.fixture
def rs():
    return FakeRS()


@pytest.fixture
def camera(rs):
    cam = RealSenseCamera(_config(), camera_id="rs435i", rs_module=rs, np_module=np)
    yield cam
    cam.stop()


def _wait_frames(cam, n=1, timeout=2.0):
    deadline = time.monotonic() + timeout
    while cam.frames_captured < n and time.monotonic() < deadline:
        time.sleep(0.005)
    assert cam.frames_captured >= n, "capture thread produced no frames"


# ---------------------------------------------------------------------------
# Configuration / availability
# ---------------------------------------------------------------------------

class TestConfiguration:
    def test_disabled_config_is_inert(self, rs):
        cam = RealSenseCamera({"enabled": False}, rs_module=rs, np_module=np)
        assert not cam.configured
        assert cam.list_devices() == []
        d = cam.describe()
        assert d["configured"] is False and d["streaming"] is False
        assert "disabled" in d["reason"]
        assert cam.component_status() is None and cam.status_block() is None
        with pytest.raises(RealSenseUnavailable):
            cam.start()

    def test_none_config_is_disabled(self, rs):
        assert not RealSenseCamera(None, rs_module=rs).configured

    def test_missing_backend_reports_not_installed(self, monkeypatch):
        import builtins
        real_import = builtins.__import__

        def fake_import(name, *a, **k):
            if name == "pyrealsense2":
                raise ImportError("No module named 'pyrealsense2'")
            return real_import(name, *a, **k)

        monkeypatch.setattr(builtins, "__import__", fake_import)
        cam = RealSenseCamera(_config(), np_module=np)
        assert cam.configured and not cam.installed
        d = cam.describe()
        assert d["installed"] is False
        assert "uv sync --extra realsense" in d["reason"]
        assert cam.component_status()["state"] == "driver_missing"
        with pytest.raises(RealSenseUnavailable, match="not installed"):
            cam.start()

    def test_camera_id_is_carried_and_reported(self, rs):
        cam = RealSenseCamera(_config(), camera_id="overhead", rs_module=rs, np_module=np)
        assert cam.camera_id == "overhead"
        assert cam.describe()["camera_id"] == "overhead"
        assert cam.status_block()["camera_id"] == "overhead"

    def test_config_file_missing_yields_no_cameras(self, tmp_path, rs):
        config, reason = rc.load_config(str(tmp_path / "nope.yaml"))
        assert config == {} and "nope.yaml" in reason
        built, reason = rc.build_cameras(config, rs_module=rs)
        assert built == {} and reason

    def test_config_file_reads_the_cameras_list(self, tmp_path, rs):
        pytest.importorskip("yaml")
        path = tmp_path / "realsense.yaml"
        path.write_text(
            "enabled: true\n"
            "cameras:\n"
            "  - id: rs435i\n"
            "    serial: 'ABC'\n"
            "    color: {width: 320, height: 240, fps: 15}\n"
        )
        config, reason = rc.load_config(str(path))
        assert reason is None
        built, reason = rc.build_cameras(config, rs_module=rs, np_module=np)
        assert reason is None and list(built) == ["rs435i"]
        cam = built["rs435i"]
        assert cam.configured and cam.serial == "ABC" and cam.camera_id == "rs435i"
        assert cam.color_profile == {"enabled": True, "width": 320, "height": 240, "fps": 15}
        assert cam.depth_profile["width"] == 640  # default kept

    def test_shipped_yaml_parses(self, rs):
        pytest.importorskip("yaml")
        config, reason = rc.load_config(rc.default_config_path())
        assert reason is None
        built, reason = rc.build_cameras(config, rs_module=rs, np_module=np)
        assert reason is None
        assert "rs435i" in built, "the shipped config must define the first camera as rs435i"
        cam = built["rs435i"]
        assert cam.configured
        assert cam.color_profile["enabled"] and cam.depth_profile["enabled"]

    def test_shipped_yaml_defines_the_downward_d405(self, rs):
        pytest.importorskip("yaml")
        config, _ = rc.load_config(rc.default_config_path())
        built, reason = rc.build_cameras(config, rs_module=rs, np_module=np)
        assert reason is None and "rs405" in built
        cam = built["rs405"]
        assert cam.serial == "218622279627"
        assert cam.mount["facing"] == "down"

    def test_mount_is_reported_and_defaults_to_nulls(self, rs):
        cam = RealSenseCamera(_config(mount={"location": " gripper ", "facing": "down"}),
                              camera_id="rs405", rs_module=rs, np_module=np)
        assert cam.mount == {"location": "gripper", "facing": "down"}
        assert cam.describe()["mount"] == cam.mount
        assert cam.status_block()["mount"] == cam.mount
        bare = RealSenseCamera(_config(mount="nonsense"), rs_module=rs, np_module=np)
        assert bare.mount == {"location": None, "facing": None}

    def test_other_camera_on_the_bus_is_not_this_one(self):
        rs = FakeRS(devices=[FakeDevice("OTHER", name="Intel RealSense D405")])
        cam = RealSenseCamera(_config(serial="MINE"), camera_id="rs435i", rs_module=rs, np_module=np)
        d = cam.describe()
        assert d["present"] is False
        assert "MINE" in d["reason"] and "OTHER" in d["reason"]
        comp = cam.component_status()
        assert comp["connected"] is False and comp["state"] == "disconnected"
        assert "OTHER" not in comp["message"].split(" · ")[1:2]
        assert "D405" not in comp["message"]

    def test_matching_serial_is_present(self):
        rs = FakeRS(devices=[FakeDevice("OTHER"), FakeDevice("MINE")])
        cam = RealSenseCamera(_config(serial="MINE"), rs_module=rs, np_module=np)
        assert cam.describe()["present"] is True
        assert "sn MINE" in cam.component_status()["message"]

    def test_bad_values_fall_back_to_defaults(self, rs):
        cam = RealSenseCamera(_config(jpeg_quality="x", frame_timeout_ms=None,
                                      color={"width": "bad"}),
                              camera_id="rs435i", rs_module=rs, np_module=np)
        assert cam.jpeg_quality == 80 and cam.frame_timeout_ms == 5000
        assert cam.color_profile["width"] == 640


class TestEnumeration:
    def test_lists_devices_with_identity(self, camera):
        devs = camera.list_devices()
        assert devs == [{
            "name": "Intel RealSense D435I", "serial": "123456", "firmware": "5.16.0.1",
            "usb_type": "3.2", "product_line": "D400", "product_id": "0B3A",
        }]

    def test_enumeration_is_cached(self, rs, camera):
        camera.list_devices()
        rs.devices.append(FakeDevice("999"))
        assert len(camera.list_devices()) == 1          # TTL cache
        assert len(camera.list_devices(force=True)) == 2

    def test_enumeration_failure_is_empty_not_raised(self, rs, camera):
        rs.enumeration_error = "usb hub exploded"
        assert camera.list_devices(force=True) == []
        assert camera.describe()["reason"] == "no RealSense device connected"

    def test_no_device_component_is_disconnected(self, rs, camera):
        rs.devices.clear()
        block = camera.component_status()
        assert block["connected"] is False and block["state"] == "disconnected"
        assert "no RealSense device connected" in block["message"]

    def test_present_but_idle_component(self, camera):
        block = camera.component_status()
        assert block == {
            "connected": True, "state": "idle",
            "message": "test cam · Intel RealSense D435I · sn 123456 · pipeline stopped (starts on first request)",
        }


# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

class TestLifecycle:
    def test_start_configures_pipeline_and_streams(self, rs, camera):
        info = camera.start()
        assert camera.streaming and info["streaming"] and info["state"] == "streaming"
        assert rs.last_config.serial == "123456"
        assert sorted(rs.last_config.streams) == [
            ("color", 64, 48, "bgr8", 30), ("depth", 64, 48, "z16", 30),
        ]
        assert info["device"]["serial"] == "123456"
        assert info["depth_scale_m"] == 0.001
        intr = camera.intrinsics()
        assert intr["aligned_to"] == "color"
        assert intr["streams"]["color"]["fx"] == 600.0
        assert intr["streams"]["depth"]["stream"] == "depth"

    def test_start_is_idempotent(self, rs, camera):
        camera.start()
        camera.start()
        assert len(rs.started_pipelines) == 1

    def test_stop_releases_pipeline(self, rs, camera):
        camera.start()
        _wait_frames(camera)
        camera.stop()
        assert not camera.streaming and camera.state == "off"
        assert rs.started_pipelines[0].stopped
        camera.stop()  # second stop is harmless

    def test_no_device_raises_unavailable(self, rs, camera):
        rs.devices.clear()
        with pytest.raises(RealSenseUnavailable, match="no RealSense device"):
            camera.start()
        assert camera.state == "off"

    def test_serial_selection(self, rs):
        rs.devices = [FakeDevice("AAA"), FakeDevice("BBB")]
        cam = RealSenseCamera(_config(serial="BBB"), rs_module=rs, np_module=np)
        try:
            cam.start()
            assert rs.last_config.serial == "BBB"
            assert cam.describe()["device"]["serial"] == "BBB"
        finally:
            cam.stop()

    def test_unknown_serial_lists_connected(self, rs):
        cam = RealSenseCamera(_config(serial="ZZZ"), rs_module=rs, np_module=np)
        with pytest.raises(RealSenseUnavailable, match="'ZZZ' not found; connected: 123456"):
            cam.start()

    def test_librealsense_start_failure_is_error_state(self, rs, camera):
        rs.start_error = "Couldn't resolve requests"
        with pytest.raises(RealSenseError, match="Couldn't resolve requests"):
            camera.start()
        d = camera.describe()
        assert d["state"] == "error" and "Couldn't resolve requests" in d["reason"]
        assert camera.component_status()["state"] == "error"

    def test_disabled_streams_are_not_requested(self, rs):
        cam = RealSenseCamera(_config(color={"enabled": False}), rs_module=rs, np_module=np)
        try:
            cam.start()
            assert [s[0] for s in rs.last_config.streams] == ["depth"]
            assert cam.intrinsics()["aligned_to"] is None  # nothing to align to
            _wait_frames(cam)
            with pytest.raises(RealSenseError, match="color stream is disabled"):
                cam.jpeg("color")
        finally:
            cam.stop()

    def test_ensure_started_honours_start_on_demand(self, rs):
        cam = RealSenseCamera(_config(start_on_demand=False), rs_module=rs, np_module=np)
        try:
            with pytest.raises(RealSenseNotStreaming, match="start_on_demand is off"):
                cam.ensure_started()
            assert not cam.streaming
            cam.start_on_demand = True
            cam.ensure_started()
            assert cam.streaming
        finally:
            cam.stop()

    def test_usb2_link_is_a_warning(self, rs):
        rs.devices = [FakeDevice("U2", usb="2.1")]
        cam = RealSenseCamera(_config(), rs_module=rs, np_module=np)
        try:
            d = cam.start()
            assert any("USB 2.1" in w for w in d["warnings"])
        finally:
            cam.stop()


# ---------------------------------------------------------------------------
# Capture thread + frame access
# ---------------------------------------------------------------------------

class TestCapture:
    def test_frames_flow_and_fps_is_measured(self, camera):
        camera.start()
        _wait_frames(camera, n=5)
        d = camera.describe()
        assert d["frames_captured"] >= 5
        assert d["fps_measured"] is not None and d["fps_measured"] > 0
        assert d["last_frame_age_s"] is not None
        bundle = camera.latest()
        assert bundle.color.shape == (48, 64, 3) and bundle.depth.shape == (48, 64)
        assert bundle.depth_color.shape == (48, 64, 3)
        assert bundle.intrinsics["stream"] == "color"  # aligned -> colour intrinsics
        assert bundle.frame_number >= 1

    def test_latest_before_start_raises(self, camera):
        with pytest.raises(RealSenseNotStreaming):
            camera.latest()

    def test_repeated_frame_failures_mark_camera_lost(self, rs, camera):
        camera.start()
        _wait_frames(camera)
        rs.fail_frames = True
        deadline = time.monotonic() + 3.0
        while camera.state == "streaming" and time.monotonic() < deadline:
            time.sleep(0.01)
        assert camera.state == "error"
        d = camera.describe()
        assert "3 consecutive frame failures" in d["reason"]
        assert rs.started_pipelines[0].stopped
        # Recovery: an explicit start clears the error.
        rs.fail_frames = False
        camera.start()
        assert camera.streaming and camera.describe()["last_error"] is None

    def test_idle_timeout_stops_pipeline(self, rs):
        cam = RealSenseCamera(_config(idle_timeout_seconds=0.05), rs_module=rs, np_module=np)
        try:
            cam.start()
            deadline = time.monotonic() + 2.0
            while cam.streaming and time.monotonic() < deadline:
                time.sleep(0.01)
            assert not cam.streaming and cam.state == "off"
        finally:
            cam.stop()

    def test_consumer_keeps_pipeline_alive(self, rs):
        cam = RealSenseCamera(_config(idle_timeout_seconds=0.15), rs_module=rs, np_module=np)
        try:
            cam.start()
            for _ in range(6):
                time.sleep(0.05)
                cam.latest()          # a consumer every 50 ms < 150 ms timeout
            assert cam.streaming
        finally:
            cam.stop()

    def test_wait_for_new_frame_blocks_until_newer(self, camera):
        camera.start()
        _wait_frames(camera)
        seen = camera.frames_captured
        bundle = camera.wait_for_new_frame(seen, timeout_s=1.0)
        assert bundle is not None and camera.frames_captured > seen

    def test_wait_for_new_frame_returns_none_when_stopped(self, camera):
        assert camera.wait_for_new_frame(0, timeout_s=0.1) is None


class TestEncoding:
    def test_color_jpeg(self, camera):
        camera.start()
        _wait_frames(camera)
        data, bundle = camera.jpeg("color")
        assert data[:2] == b"\xff\xd8"          # JPEG SOI
        from PIL import Image
        import io
        img = Image.open(io.BytesIO(data))
        assert img.size == (64, 48)
        r, g, b = img.getpixel((5, 5))
        assert b > 200 and r < 40           # BGR blue came out as RGB blue

    def test_depth_jpeg_uses_colorizer(self, camera):
        camera.start()
        _wait_frames(camera)
        data, _ = camera.jpeg("depth")
        assert data[:2] == b"\xff\xd8"

    def test_depth_png_is_16_bit(self, camera):
        camera.start()
        _wait_frames(camera)
        data, bundle = camera.depth_png()
        assert data[:8] == b"\x89PNG\r\n\x1a\n"
        from PIL import Image
        import io
        img = Image.open(io.BytesIO(data))
        assert img.mode in ("I;16", "I")
        assert img.getpixel((30, 30)) == 1500
        assert img.getpixel((10, 10)) == 0

    def test_bad_kind_rejected(self, camera):
        camera.start()
        with pytest.raises(ValueError, match="stream must be one of"):
            camera.jpeg("infrared")

    def test_mjpeg_parts_are_multipart(self, camera):
        camera.start()
        gen = camera.mjpeg_frames("color", max_fps=1000)
        first = next(gen)
        assert first.startswith(b"--xarm-realsense-frame\r\nContent-Type: image/jpeg\r\n")
        assert b"\r\n\r\n\xff\xd8" in first
        second = next(gen)
        assert second.startswith(b"--xarm-realsense-frame")
        camera.stop()
        assert list(gen) == []                # generator ends when the pipeline stops
        assert camera.mjpeg_content_type() == "multipart/x-mixed-replace; boundary=xarm-realsense-frame"

    def test_mjpeg_is_paced(self, camera):
        camera.start()
        gen = camera.mjpeg_frames("color", max_fps=20)
        next(gen)
        t0 = time.monotonic()
        next(gen)
        assert time.monotonic() - t0 >= 0.04   # >= 1/20 s between parts


class TestDepthAt:
    def test_distance_and_point(self, camera):
        camera.start()
        _wait_frames(camera)
        r = camera.depth_at(32, 24, window=1)
        assert r["distance_m"] == pytest.approx(1.5)
        # centre pixel -> on the optical axis
        assert r["point_m"] == pytest.approx([0.0, 0.0, 1.5])
        assert r["frame"] == "color" and r["valid_samples"] == 1

    def test_off_axis_deprojection(self, camera):
        camera.start()
        _wait_frames(camera)
        r = camera.depth_at(38, 30, window=1)          # +6 px, +6 px from centre
        X, Y, Z = r["point_m"]
        assert Z == pytest.approx(1.5)
        assert X == pytest.approx(6 / 600.0 * 1.5) and Y == pytest.approx(6 / 600.0 * 1.5)

    def test_median_window_rejects_outlier(self, camera):
        camera.start()
        _wait_frames(camera)
        raw = camera.depth_at(22, 22, window=1)
        assert raw["distance_m"] == pytest.approx(3.0)  # the noisy pixel
        med = camera.depth_at(22, 22, window=5)
        assert med["distance_m"] == pytest.approx(0.8) and med["window"] == 5
        assert med["valid_samples"] == 25
        even = camera.depth_at(22, 22, window=4)        # even -> forced odd
        assert even["window"] == 5

    def test_hole_reports_none(self, camera):
        camera.start()
        _wait_frames(camera)
        r = camera.depth_at(10, 10, window=1)
        assert r["distance_m"] is None and r["point_m"] is None and r["valid_samples"] == 0

    def test_out_of_bounds_is_value_error(self, camera):
        camera.start()
        _wait_frames(camera)
        with pytest.raises(ValueError, match="outside the 64x48"):
            camera.depth_at(64, 0)

    def test_not_streaming_raises(self, camera):
        with pytest.raises(RealSenseNotStreaming):
            camera.depth_at(0, 0)


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

class TestReporting:
    def test_status_block_shape(self, camera):
        camera.start()
        _wait_frames(camera)
        block = camera.status_block()
        assert set(block) == {
            "camera_id", "label", "mount", "state", "installed", "device", "devices", "streams",
            "fps_measured", "frames_captured", "last_frame_age_s", "warnings", "reason",
        }
        assert block["state"] == "streaming" and block["reason"] is None
        assert block["streams"]["align_depth_to_color"] is True

    def test_streaming_component(self, camera):
        camera.start()
        _wait_frames(camera, n=3)
        block = camera.component_status()
        assert block["connected"] is True and block["state"] == "streaming"
        assert block["message"].startswith("test cam · Intel RealSense D435I · sn 123456")
        assert "fps" in block["message"]

    def test_describe_never_raises_on_broken_backend(self, rs, camera):
        rs.enumeration_error = "boom"
        d = camera.describe()
        assert d["devices"] == [] and d["streaming"] is False


class TestRegistry:
    """The process-wide registry: what the API server and status_builder read.

    A bad entry is a log line and a skipped camera, never an exception --
    this runs at import time in the arm's own process.
    """

    @staticmethod
    def _entry(camera_id, **overrides):
        entry = dict(_config())
        entry["id"] = camera_id
        entry.pop("enabled", None)
        entry.update(overrides)
        return entry

    def test_configure_from_a_missing_file_leaves_an_empty_registry(self, tmp_path, rs):
        try:
            built = rc.configure_cameras(str(tmp_path / "missing.yaml"), rs_module=rs)
            assert built == {} and rc.cameras() == {}
            assert rc.default_camera_id() is None
            assert "missing.yaml" in rc.configuration_reason()
        finally:
            rc.set_cameras(None)

    def test_set_cameras_installs_and_clears(self, rs):
        try:
            cam = RealSenseCamera(_config(), camera_id="rs435i", rs_module=rs, np_module=np)
            rc.set_cameras({"rs435i": cam})
            assert rc.cameras() == {"rs435i": cam}
            assert rc.camera("rs435i") is cam and rc.camera("nope") is None
            assert rc.default_camera_id() == "rs435i"
            assert rc.configuration_reason() is None
            rc.set_cameras(None)
            assert rc.cameras() == {} and rc.configuration_reason()
        finally:
            rc.set_cameras(None)

    def test_disabled_at_the_top_level_means_no_cameras(self, rs):
        built, reason = rc.build_cameras(
            {"enabled": False, "cameras": [self._entry("rs435i")]}, rs_module=rs, np_module=np)
        assert built == {} and "enabled: false" in reason

    def test_empty_camera_list_means_no_cameras(self, rs):
        built, reason = rc.build_cameras({"enabled": True, "cameras": []}, rs_module=rs)
        assert built == {} and "cameras:" in reason
        built, reason = rc.build_cameras({"enabled": True}, rs_module=rs)
        assert built == {} and "cameras:" in reason

    @pytest.mark.parametrize("bad_id", ["", "RS435i", "../etc", "a/b", "cam.1",
                                        "-lead", "a" * 33, None])
    def test_malformed_ids_are_skipped_not_raised(self, rs, bad_id):
        entry = self._entry("placeholder")
        entry["id"] = bad_id
        built, reason = rc.build_cameras({"enabled": True, "cameras": [entry]},
                                         rs_module=rs, np_module=np)
        assert built == {} and reason

    def test_duplicate_ids_are_refused(self, rs):
        built, reason = rc.build_cameras(
            {"enabled": True,
             "cameras": [self._entry("rs435i", serial="AAA"),
                         self._entry("rs435i", serial="BBB")]},
            rs_module=rs, np_module=np)
        assert list(built) == ["rs435i"]
        assert built["rs435i"].serial == "AAA"      # the first wins, the second is logged

    def test_a_serial_is_required_once_there_are_two_cameras(self, rs):
        """With two cameras on the bus, "the first one enumerated" is not
        stable, so an entry without a serial is skipped rather than guessed."""
        built, reason = rc.build_cameras(
            {"enabled": True,
             "cameras": [self._entry("rs435i", serial="AAA"),
                         self._entry("overhead", serial="")]},
            rs_module=rs, np_module=np)
        assert list(built) == ["rs435i"]

        built, reason = rc.build_cameras(
            {"enabled": True,
             "cameras": [self._entry("rs435i", serial="AAA"),
                         self._entry("overhead", serial="BBB")]},
            rs_module=rs, np_module=np)
        assert sorted(built) == ["overhead", "rs435i"]

    def test_a_single_camera_may_omit_its_serial(self, rs):
        built, reason = rc.build_cameras(
            {"enabled": True, "cameras": [self._entry("rs435i", serial="")]},
            rs_module=rs, np_module=np)
        assert list(built) == ["rs435i"] and built["rs435i"].serial is None

    def test_a_disabled_entry_is_not_registered(self, rs):
        built, reason = rc.build_cameras(
            {"enabled": True, "cameras": [self._entry("rs435i", enabled=False)]},
            rs_module=rs, np_module=np)
        assert built == {} and reason

    def test_default_camera_id_is_none_with_two_cameras(self, rs):
        try:
            built, _ = rc.build_cameras(
                {"enabled": True,
                 "cameras": [self._entry("rs435i", serial="AAA"),
                             self._entry("overhead", serial="BBB")]},
                rs_module=rs, np_module=np)
            rc.set_cameras(built)
            assert rc.default_camera_id() is None
        finally:
            rc.set_cameras(None)

    def test_default_config_path_points_at_settings(self):
        path = rc.default_config_path()
        assert path.endswith(os.path.join("settings", "realsense.yaml"))
        assert os.path.exists(path)


class TestThreadSafety:
    def test_concurrent_readers_do_not_deadlock(self, camera):
        camera.start()
        _wait_frames(camera)
        errors = []

        def reader():
            try:
                for _ in range(20):
                    camera.jpeg("color")
                    camera.depth_at(32, 24)
                    camera.describe()
            except Exception as exc:  # noqa: BLE001
                errors.append(exc)

        threads = [threading.Thread(target=reader) for _ in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=5.0)
        assert not errors and all(not t.is_alive() for t in threads)
