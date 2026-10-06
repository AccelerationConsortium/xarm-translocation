"""The xArm camera routes forward to a fake standalone camera service."""
import json
import time
from types import SimpleNamespace

import httpx
import pytest
from fastapi.testclient import TestClient

from src.core import realsense_camera as registry
from src.core import realsense_captures as captures
from src.core.remote_realsense import RemoteCamera, RemoteService, RemoteStore
from src.core.xarm_api_server import app
import src.core.xarm_api_server as server


class ClaimManager:
    def verify_token(self, token):
        return None

    def claimed_by(self):
        return {"owner": "agent@example.invalid"}


@pytest.fixture
def client(monkeypatch):
    requests = []
    def respond(request):
        requests.append((request.method, request.url.path))
        path = request.url.path
        if path.endswith('/snapshot.jpg'):
            return httpx.Response(200, content=b'jpeg', headers={'X-Frame-Number': '12'})
        if path.endswith('/depth.png'):
            return httpx.Response(200, content=b'png', headers={'X-Frame-Number': '12'})
        if path.endswith('/intrinsics'):
            return httpx.Response(200, json={'depth_scale_m': 0.001})
        if path.endswith('/captures') and request.method == 'POST':
            body = json.loads(request.content)
            meta = {'capture_id': '20260925T000000Z-abcdef12', 'camera_id': 'rs435i',
                    'arm': body['context']['arm'], 'label': body['label'],
                    'files': {'color.jpg': {}, 'depth.png': {}}}
            return httpx.Response(200, json={'capture_id': meta['capture_id'], 'meta': meta})
        if path == '/v1/store':
            return httpx.Response(200, json={'enabled': True, 'count': 0, 'bytes': 0})
        if path == '/v1/cameras':
            return httpx.Response(200, json={'cameras': [{'id': 'rs435i', 'state': 'streaming',
                                                           'streaming': True, 'present': True,
                                                           'installed': True}]})
        return httpx.Response(404)

    remote = RemoteService({'url': 'http://camera.invalid', 'token': 'test', 'cameras': ['rs435i']})
    remote.client.close()
    remote.client = httpx.Client(base_url='http://camera.invalid', transport=httpx.MockTransport(respond))
    remote.states = {'rs435i': {'state': 'streaming', 'streaming': True,
                               'present': True, 'installed': True}}
    remote.sampled = time.monotonic()
    remote.reason = None
    old_cameras = registry.cameras()
    old_store = captures.shared_store()
    registry.set_cameras({'rs435i': RemoteCamera('rs435i', {'label': 'D435i'}, remote)})
    captures.set_shared(RemoteStore(remote))
    monkeypatch.setattr(server, 'controller', SimpleNamespace(
        claim_manager=ClaimManager(), disconnect=lambda: None,
        states={'connection': SimpleNamespace(value='enabled')},
        current_node='deck_1', current_gripper_state='open',
        last_joints=[0, 1, 2, 3, 4], last_position=[1, 2, 3],
        last_track_position=42,
    ))
    try:
        with TestClient(app) as test_client:
            yield test_client, requests
    finally:
        registry.set_cameras(old_cameras)
        captures.set_shared(old_store)
        remote.close()


def test_compatibility_routes_and_agent_docs(client):
    api, requests = client
    assert api.get('/realsense/cameras').status_code == 200
    assert api.get('/cameras').json()['sources']['usb']['available'] is False
    snapshot = api.get('/realsense/rs435i/snapshot.jpg')
    assert snapshot.status_code == 200 and snapshot.content == b'jpeg'
    depth = api.get('/realsense/rs435i/depth.png')
    assert depth.status_code == 200 and depth.content == b'png'
    assert api.get('/realsense/rs435i/intrinsics').json()['depth_scale_m'] == 0.001
    result = api.post('/control/realsense/capture', json={'camera': 'rs435i', 'label': 'test'})
    assert result.status_code == 200, result.text
    assert result.json()['meta']['arm']['node_id'] == 'deck_1'
    assert result.json()['urls']['color'].endswith('/color.jpg')
    assert ('POST', '/v1/cameras/rs435i/captures') in requests
    for path in ('/agent-docs', '/agent-docs/api-reference', '/llms.txt'):
        response = api.get(path)
        assert response.status_code == 200
        assert 'standalone' in response.text.lower() or 'camera service' in response.text.lower()
