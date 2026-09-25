"""Raw/aligned identity, bounded collection and archive integrity without USB."""
import hashlib
import io
import json
import time
import zipfile
from types import SimpleNamespace as NS

import pytest

np = pytest.importorskip('numpy')
Image = pytest.importorskip('PIL.Image')
from src.core.realsense_camera import RealSenseCamera, RealSenseNotStreaming, RealSenseError
from src.core.realsense_diagnostics import DiagnosticCapture


class Profile:
    def __init__(self, kind):
        self.kind = kind
    def as_video_stream_profile(self): return self
    def format(self): return 'z16' if self.kind != 'color' else 'bgr8'
    def fps(self): return 30
    def stream_type(self): return self.kind
    def stream_index(self): return 0
    def unique_id(self): return 1
    def get_intrinsics(self):
        return NS(width=3, height=2, fx=10, fy=11, ppx=1, ppy=1,
                  model='brown_conrady', coeffs=[0.1, 0, 0, 0, 0])
    def get_extrinsics_to(self, other):
        assert self.kind == 'depth' and other.kind == 'color'
        return NS(rotation=[1, 0, 0, 0, 1, 0, 0, 0, 1], translation=[0.01, 0, 0])


class Frame:
    def __init__(self, kind, number):
        self.profile = Profile(kind)
        self.number = number
        self.data = np.full((2, 3, 3) if kind == 'color' else (2, 3),
                            number, dtype='uint8' if kind == 'color' else 'uint16')
    def get_data(self): return self.data
    def get_frame_number(self): return self.number
    def get_timestamp(self): return self.number * 33.3
    def get_frame_timestamp_domain(self): return 'hardware_clock'
    def supports_frame_metadata(self, key): return key == 1
    def get_frame_metadata(self, key): return 8000


def camera():
    device = NS(query_sensors=lambda: [], first_depth_sensor=lambda: NS(get_depth_scale=lambda: 0.001))
    return NS(camera_id='rs435i', library_version='fake', _np=np,
              _device_dict=lambda d: {'serial': '123', 'firmware': 'fake'},
              _rs=NS(frame_metadata_value=NS(__members__={'actual_exposure': 1, 'sensor_timestamp': 2})),
              _pipeline=NS(get_active_profile=lambda: NS(get_device=lambda: device,
                    get_streams=lambda: [Profile('depth'), Profile('color')])))


def test_native_alignment_pairing_and_archive():
    cam = camera()
    request = DiagnosticCapture(cam)
    request.started = time.monotonic() - 3
    for number in range(1, 51):
        depth, color = Frame('depth', number), Frame('color', number + 1)
        frames = NS(get_depth_frame=lambda: depth, get_color_frame=lambda: color)
        sample = request.before_alignment(cam, frames)
        # Model alignment changing the source buffer to catch late/native reads.
        depth.data[:] = 999
        aligned = Frame('aligned', number)
        aligned.data[:] = number + 100
        request.after_alignment(cam, sample, aligned)
    assert request.done.is_set() and len(request.frames) == 20
    assert request.before_alignment(cam, frames) is None
    with zipfile.ZipFile(io.BytesIO(request.archive())) as archive:
        prefix = request.capture_id + '/'
        manifest = json.loads(archive.read(prefix + 'manifest.json'))
        assert manifest['frame_count'] == 20
        assert manifest['depth_to_color']['translation_m'] == [0.01, 0, 0]
        for i, entry in enumerate(manifest['frames']):
            for info in entry['files'].values():
                data = archive.read(prefix + info['path'])
                assert hashlib.sha256(data).hexdigest() == info['sha256']
            def array(key):
                return np.load(io.BytesIO(archive.read(prefix + entry['files'][key]['path'])))
            assert np.all(array('native_depth') == i + 31)
            assert np.all(array('native_color') == i + 32)
            png = np.array(Image.open(io.BytesIO(archive.read(prefix + entry['files']['aligned_depth_png']['path']))))
            np.testing.assert_array_equal(png, array('aligned_depth'))
            meta = json.loads(archive.read(prefix + entry['files']['metadata']['path']))
            assert meta['depth']['frame_number'] == i + 31
            assert meta['color']['frame_number'] == i + 32
            assert meta['depth']['metadata_sdk_units']['sensor_timestamp'] is None


def test_reject_stopped_and_concurrent_without_start():
    cam = RealSenseCamera({'enabled': False})
    with pytest.raises(RealSenseNotStreaming): cam.diagnostic_export()
    assert cam._pipeline is None and cam._diagnostic is None
    cam._diagnostic_lock.acquire()
    try:
        with pytest.raises(RealSenseError, match='already in progress'): cam.diagnostic_export()
    finally:
        cam._diagnostic_lock.release()


def test_memory_bound_and_partial_frames():
    cam = camera()
    request = DiagnosticCapture(cam)
    request.started -= 3
    request.skipped = 30
    with pytest.raises(RuntimeError, match='Incomplete'):
        request.before_alignment(cam, NS(get_depth_frame=lambda: None, get_color_frame=lambda: None))
    request.max_bytes = 1
    with pytest.raises(RuntimeError, match='budget'):
        request.after_alignment(cam, {'native_depth': np.zeros((2, 3)),
                                     'native_color': np.zeros((2, 3)), 'metadata': {}}, Frame('aligned', 1))
    assert request.frames == []


def test_capture_loop_taps_before_and_after_alignment():
    """Run the real service loop with an aligner that mutates native buffers."""
    cam = RealSenseCamera({'enabled': False})
    cam._np = np
    cam._state = 'streaming'
    cam._intrinsics = {}
    cam._rs = camera()._rs
    cam._device_dict = camera()._device_dict
    cam._colorizer = None
    sequence = 0
    def next_frames(timeout):
        nonlocal sequence
        sequence += 1
        depth, color = Frame('depth', sequence), Frame('color', sequence)
        return NS(get_depth_frame=lambda: depth, get_color_frame=lambda: color)
    def align(frames):
        frames.get_depth_frame().data[:] += 100
        return frames
    cam._pipeline = NS(wait_for_frames=next_frames,
                       get_active_profile=camera()._pipeline.get_active_profile)
    cam._align = NS(process=align)
    request = DiagnosticCapture(cam)
    request.started -= 3
    cam._diagnostic = request
    real_after = request.after_alignment
    def after(camera, sample, frame):
        real_after(camera, sample, frame)
        if request.done.is_set():
            cam._stop_event.set()
    request.after_alignment = after
    cam._capture_loop()
    assert len(request.frames) == 20
    assert sequence == 50
    for i, sample in enumerate(request.frames):
        assert np.all(sample['native_depth'] == i + 31)
        assert np.all(sample['aligned_depth'] == i + 131)


def test_timeout_cleans_up_request(monkeypatch):
    cam = RealSenseCamera({'enabled': False})
    cam._state = 'streaming'
    cam._align = object()
    request = DiagnosticCapture(cam)
    request.done.wait = lambda seconds: False
    monkeypatch.setattr('src.core.realsense_camera.DiagnosticCapture', lambda c: request)
    with pytest.raises(RealSenseError, match='timed out'):
        cam.diagnostic_export()
    assert cam._diagnostic is None
    assert not cam._diagnostic_lock.locked()
    with pytest.raises(RuntimeError, match='incomplete'):
        request.archive()
