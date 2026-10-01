"""USB ownership/discovery tests without native camera drivers or hardware."""
import threading
import time
from types import SimpleNamespace

import pytest

from src.core.usb_camera import CameraManager, CameraUnavailable, OpenCVBackend


def device(index=0, camera_id='usb-test'):
    return dict(id=camera_id, name='USB Camera', path='device-a', index=index,
                backend=700, vid=123, pid=456, persistent_identity=True)


class Capture:
    def __init__(self):
        self.released = False
        self.failed = False
        self.opened = True

    def isOpened(self):
        return self.opened

    def read(self):
        time.sleep(0.005)
        return not self.failed, b'frame'

    def release(self):
        self.released = True


class Backend:
    def __init__(self):
        self.found = [device()]
        self.opens = []
        self.cap = Capture()

    def devices(self):
        return self.found

    def open(self, spec):
        self.opens.append(spec)
        return self.cap

    def encode(self, frame):
        return b'jpeg'


@pytest.fixture
def rig():
    backend = Backend()
    manager = CameraManager(backend)
    yield manager, backend
    manager.stop_all()


def test_discovery_does_not_open_camera_and_refreshes_index(rig):
    manager, backend = rig
    assert manager.discover()[0]['device']['index'] == 0
    assert backend.opens == []
    backend.found = [device(4)]
    cam = manager.get('usb-test')
    cam.start()
    assert cam.jpeg()[0] == b'jpeg'
    assert backend.opens[0]['index'] == 4


def test_multiple_viewers_share_one_owner(rig):
    manager, backend = rig
    cam = manager.get('usb-test')
    threads = [threading.Thread(target=cam.start) for _ in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert cam.jpeg()[0] == b'jpeg'
    assert len(backend.opens) == 1
    cam.stop()
    assert backend.cap.released
    with pytest.raises(CameraUnavailable):
        cam.jpeg()


def test_unplug_replug_and_unknown_device(rig):
    manager, backend = rig
    cam = manager.get('usb-test')
    cam.start()
    cam.jpeg()
    backend.found = []
    assert manager.discover() == []
    assert backend.cap.released
    with pytest.raises(KeyError):
        manager.get('usb-test')
    backend.cap = Capture()
    backend.found = [device(3)]
    assert manager.get('usb-test') is cam
    cam.start()
    cam.jpeg()
    assert backend.opens[-1]['index'] == 3


def test_open_failure_is_reported(rig):
    manager, backend = rig
    backend.cap.opened = False
    cam = manager.get('usb-test')
    cam.start()
    with pytest.raises(CameraUnavailable, match='Cannot open'):
        cam.jpeg()
    assert backend.cap.released


def test_enumeration_failure_does_not_remove_existing_devices(rig):
    manager, backend = rig
    cam = manager.get('usb-test')
    def fail():
        raise OSError('enumeration failed')
    backend.devices = fail
    with pytest.raises(CameraUnavailable, match='enumeration failed'):
        manager.discover()
    assert cam.present


def test_capture_failure_does_not_return_stale_jpeg(rig):
    manager, backend = rig
    cam = manager.get('usb-test')
    cam.start()
    cam.jpeg()
    backend.cap.failed = True
    cam._thread.join(timeout=1)
    with pytest.raises(CameraUnavailable, match='stopped delivering'):
        cam.jpeg()


def test_blocked_native_read_does_not_start_second_owner(rig):
    manager, backend = rig
    gate = threading.Event()
    backend.cap.read = lambda: (gate.wait(5), b'frame')
    cam = manager.get('usb-test')
    try:
        cam.start()
        with pytest.raises(CameraUnavailable, match='Timed out'):
            cam.jpeg(timeout=0.02)
        cam.stop()
        with pytest.raises(CameraUnavailable, match='still stopping'):
            cam.start()
        assert len(backend.opens) == 1
    finally:
        gate.set()
        cam._thread.join(timeout=1)


def test_native_discovery_excludes_realsense_and_id_survives_index_change(monkeypatch):
    monkeypatch.setattr('src.core.usb_camera.platform.system', lambda: 'Windows')
    info = SimpleNamespace(index=1, name='USB Camera', path='unique-path',
                           backend=700, vid=1, pid=2)
    rs = SimpleNamespace(name='Intel RealSense D435i RGB')
    backend = OpenCVBackend.__new__(OpenCVBackend)
    backend.api = 700
    backend.enumerate = lambda api: [rs, info]
    first = backend.devices()
    info.index = 5
    second = backend.devices()
    assert len(first) == 1
    assert first[0]['id'] == second[0]['id']
    assert second[0]['index'] == 5
    assert first[0]['persistent_identity']


def test_idle_camera_releases_device(rig):
    manager, backend = rig
    cam = manager.get('usb-test')
    cam.idle_timeout = 0.03
    cam.start()
    cam.jpeg()
    cam._thread.join(timeout=1)
    assert backend.cap.released
    assert cam.describe()['state'] == 'off'


def test_optional_dependency_missing(monkeypatch):
    import builtins
    original = builtins.__import__
    def unavailable(name, *args, **kwargs):
        if name == 'cv2':
            raise ImportError('not installed')
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', unavailable)
    with pytest.raises(CameraUnavailable, match='usb-camera'):
        CameraManager().discover()
