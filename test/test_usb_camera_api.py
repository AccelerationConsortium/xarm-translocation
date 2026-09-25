"""HTTP contract exercised asynchronously, with no hardware or robot startup."""
import asyncio

import httpx
import pytest
from fastapi import FastAPI, HTTPException

from src.core.usb_camera import CameraUnavailable
from src.core.usb_camera_api import create_router


class Camera:
    def __init__(self):
        self.starts = 0
        self.stops = 0
        self.fail = False

    def describe(self):
        return {'id': 'usb-test', 'capabilities': ['color', 'snapshot', 'mjpeg']}

    def start(self):
        self.starts += 1

    def stop(self):
        self.stops += 1

    def jpeg(self, **kwargs):
        if self.fail:
            raise CameraUnavailable('unplugged')
        if kwargs.get('after'):
            raise CameraUnavailable('stopped')
        return b'jpeg', 1


class Manager:
    def __init__(self):
        self.camera = Camera()
        self.missing = False

    def discover(self):
        if self.missing:
            raise CameraUnavailable('driver missing')
        return [self.camera.describe()]

    def get(self, camera_id):
        if camera_id != 'usb-test':
            raise KeyError(camera_id)
        return self.camera


@pytest.fixture
def api():
    manager = Manager()
    app = FastAPI()
    async def login():
        pass
    app.include_router(create_router(manager, login))
    return app, manager, login


def request(app, method, url, **kwargs):
    async def run():
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://test') as client:
            return await client.request(method, url, **kwargs)
    return asyncio.run(run())


def test_discovery_and_status_do_not_open_camera(api):
    app, manager, _ = api
    response = request(app, 'GET', '/usb/cameras')
    assert response.status_code == 200
    body = response.json()
    assert body['default'] == 'usb-test'
    assert body['cameras'][0]['urls']['snapshot'] == '/usb/usb-test/snapshot.jpg'
    assert 'depth' not in body['cameras'][0]['capabilities']
    assert request(app, 'GET', '/usb/usb-test/status').status_code == 200
    assert manager.camera.starts == 0


def test_missing_driver_and_unknown_device(api):
    app, manager, _ = api
    manager.missing = True
    body = request(app, 'GET', '/usb/cameras').json()
    assert not body['available'] and body['reason'] == 'driver missing'
    assert request(app, 'GET', '/usb/unknown/status').status_code == 404


def test_snapshot_start_stop_and_errors(api):
    app, manager, _ = api
    response = request(app, 'GET', '/usb/usb-test/snapshot.jpg')
    assert response.status_code == 200 and response.content == b'jpeg'
    assert response.headers['content-type'] == 'image/jpeg'
    assert response.headers['cache-control'] == 'no-store'
    assert request(app, 'POST', '/usb/usb-test/start').status_code == 200
    assert request(app, 'POST', '/usb/usb-test/stop').status_code == 200
    assert manager.camera.stops == 1
    manager.camera.fail = True
    assert request(app, 'GET', '/usb/usb-test/snapshot.jpg').status_code == 503
    assert request(app, 'POST', '/usb/usb-test/start').status_code == 503


def test_stream_and_invalid_rate(api):
    app, manager, _ = api
    response = request(app, 'GET', '/usb/usb-test/stream.mjpg?fps=30')
    assert response.status_code == 200
    assert 'boundary=usb-frame' in response.headers['content-type']
    assert b'Content-Length: 4\r\n\r\njpeg\r\n' in response.content
    for fps in ('0', '31', 'nan', 'inf'):
        assert request(app, 'GET', '/usb/usb-test/stream.mjpg?fps='+fps).status_code == 422


def test_login_required_before_any_capture(api):
    app, manager, login = api
    async def denied():
        raise HTTPException(401, 'login_required')
    app.dependency_overrides[login] = denied
    for method, path in [('GET', 'snapshot.jpg'), ('GET', 'stream.mjpg'),
                         ('POST', 'start'), ('POST', 'stop')]:
        assert request(app, method, '/usb/usb-test/'+path).status_code == 401
    assert manager.camera.starts == 0
    assert manager.camera.stops == 0


def test_main_app_combined_discovery_and_existing_routes(monkeypatch):
    from src.core import xarm_api_server as api
    async def realsense():
        return {'cameras': [{'id': 'rs435i', 'urls': {'snapshot': '/realsense/rs435i/snapshot.jpg'}}],
                'reason': None}
    manager = Manager()
    monkeypatch.setattr(api, 'realsense_cameras', realsense)
    monkeypatch.setattr(api.usb_cameras, 'discover', manager.discover)
    response = request(api.app, 'GET', '/cameras')
    assert response.status_code == 200
    rs, usb = response.json()['cameras']
    assert rs['kind'] == 'realsense' and 'depth' in rs['capabilities']
    assert usb['urls']['snapshot'] == '/usb/usb-test/snapshot.jpg'
    assert 'depth' not in usb['capabilities']
    assert request(api.app, 'GET', '/usb/cameras').status_code == 200
    paths = {route.path for route in api.app.routes}
    assert '/realsense/{camera_id}/snapshot.jpg' in paths
    assert '/control/freehand/relative' in paths
    assert manager.camera.starts == 0
