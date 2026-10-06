"""Persistent admin graph OFF: authenticated, claim-independent, no hardware."""
import json
import threading
from unittest.mock import AsyncMock, MagicMock

import pytest
from fastapi.testclient import TestClient

from src.core import xarm_api_server as srv
from src.core.claims import ClaimManager
from src.core.motion_graph import GraphMode
from src.core.xarm_controller import XArmController

OFF = '/control/admin/graph/off'
RESTORE = '/control/admin/graph/restore'
ADMIN = {'X-Auth-User': 'admin@lab', 'X-Auth-Role': 'admin', 'X-Edge-Auth': 'test-secret'}


def make_controller():
    c = XArmController.__new__(XArmController)
    c.host = 'test-robot'
    c.profile_name = 'robot'
    c.motion_graph = MagicMock()
    c._graph_mode_lock = threading.RLock()
    c._graph_mode = GraphMode.STRICT
    c._graph_mode_override = None
    c._admin_graph_off = None
    c.claim_manager = ClaimManager(enforce=True)
    c._emit_event = MagicMock()
    return c


@pytest.fixture
def controller(tmp_path, monkeypatch):
    monkeypatch.setenv('XARM_GRAPH_ADMIN_STATE_DIR', str(tmp_path))
    return make_controller()


@pytest.fixture
def client(controller, monkeypatch):
    monkeypatch.setattr(srv, 'controller', controller)
    monkeypatch.setattr(srv, 'EDGE_SHARED_SECRET', 'test-secret')
    monkeypatch.setattr(srv, 'REQUIRE_LOGIN', False)
    monkeypatch.setattr(srv, 'broadcast_status_update', AsyncMock())
    # No lifespan: controller is a bare object; tests never contact devices.
    return TestClient(srv.app)


@pytest.mark.parametrize('path', [OFF, RESTORE])
@pytest.mark.parametrize('headers,expected', [
    ({}, 401),
    ({'X-Auth-User': 'fake@lab', 'X-Auth-Role': 'admin'}, 401),
    ({**ADMIN, 'X-Edge-Auth': 'wrong'}, 401),
    ({**ADMIN, 'X-Auth-Role': 'user'}, 403),
])
def test_admin_gate_cannot_be_bypassed(client, controller, path, headers, expected):
    assert client.post(path, headers=headers).status_code == expected
    assert controller.graph_mode == GraphMode.STRICT
    controller._emit_event.assert_not_called()


@pytest.mark.parametrize('credential', ['cookie', 'api_key'])
def test_admin_sidecar_identity(client, controller, monkeypatch, credential):
    call = MagicMock(return_value=(200, {'identity': {'email': 'admin@lab', 'role': 'admin'}}, None))
    monkeypatch.setattr(srv, '_auth_sidecar_call', call)
    headers = {}
    if credential == 'cookie':
        client.cookies.set(srv.AUTH_COOKIE_NAME, 'test-cookie')
    else:
        headers['X-Api-Key'] = 'test-key'
    assert client.post(OFF, headers=headers).status_code == 200
    assert controller.graph_mode_override_snapshot()['owner'] == 'admin@lab'
    assert call.call_args.args[1] == ('/auth/me' if credential == 'cookie' else '/auth/verify')


def test_auth_failure_is_closed(client, controller, monkeypatch):
    monkeypatch.setattr(srv, '_auth_sidecar_call', MagicMock(side_effect=OSError('offline')))
    assert client.post(OFF, headers={'X-Api-Key': 'key'}).status_code == 503
    assert controller.graph_mode == GraphMode.STRICT


def test_admin_off_survives_claim_changes_and_restart(client, controller, monkeypatch):
    record = controller.claim_manager.acquire(owner='jiaru@lab', session_id='jiaru')
    result = client.post(OFF, headers=ADMIN)
    assert result.status_code == 200, result.text
    override = result.json()['mode_override']
    assert override['scope'] == 'admin'
    assert override['expires_at'] is None
    assert override['claim_bound'] is False
    controller.claim_manager.release(record.token)
    controller.claim_manager.acquire(owner='next@lab', session_id='next')
    monkeypatch.setattr('src.core.xarm_controller.time.monotonic', lambda: 1e15)
    controller.restore_graph_mode('disconnect')
    assert controller.graph_mode == GraphMode.OFF
    assert controller.claim_manager.claimed_by()['session_id'] == 'next'
    reloaded = make_controller()
    reloaded._load_admin_graph_off()
    assert reloaded.graph_mode == GraphMode.OFF
    assert reloaded.graph_mode_override_snapshot()['owner'] == 'admin@lab'
    # Different device/profile cannot inherit the override.
    other = make_controller()
    other.profile_name = 'docker'
    other._load_admin_graph_off()
    assert other.graph_mode == GraphMode.STRICT
    assert client.post(RESTORE, headers=ADMIN).json()['graph_mode'] == 'strict'
    assert controller.claim_manager.claimed_by()['session_id'] == 'next'
    reloaded = make_controller()
    reloaded._load_admin_graph_off()
    assert reloaded.graph_mode == GraphMode.STRICT


def test_ordinary_claimant_cannot_clear_admin_off(client, controller):
    client.post(OFF, headers=ADMIN)
    claim = controller.claim_manager.acquire(owner='user@lab', session_id='user')
    headers = {'X-Claim-Token': claim.token}
    for path, body in [('/control/graph/mode', {'mode': 'strict'}),
                       ('/control/graph/mode/restore', None), ('/control/graph/off', None)]:
        assert client.post(path, headers=headers, json=body).status_code == 409
    assert client.post(RESTORE, headers=headers).status_code == 401
    assert controller.graph_mode == GraphMode.OFF
    # The normal freehand graph gate now permits motion; claim gate remains.
    srv.strict_graph_guard('move.position')
    assert client.post('/control/freehand/position', json={'x': 1, 'y': 2, 'z': 3}).status_code == 423


@pytest.mark.parametrize('body', [{'ttl_seconds': 10}, {'reason': ' '}])
def test_admin_off_rejects_invalid_options(client, controller, body):
    assert client.post(OFF, headers=ADMIN, json=body).status_code == 422
    assert controller.graph_mode == GraphMode.STRICT


def test_persistence_failure_does_not_change_mode(client, controller, monkeypatch):
    monkeypatch.setattr('src.core.xarm_controller.os.replace', MagicMock(side_effect=OSError('disk full')))
    assert client.post(OFF, headers=ADMIN).status_code == 503
    assert controller.graph_mode == GraphMode.STRICT


def test_restore_failure_keeps_off(client, controller, monkeypatch):
    client.post(OFF, headers=ADMIN)
    monkeypatch.setattr('src.core.xarm_controller.os.unlink', MagicMock(side_effect=OSError('read only')))
    assert client.post(RESTORE, headers=ADMIN).status_code == 503
    assert controller.graph_mode == GraphMode.OFF


def test_invalid_persisted_state_refused(controller):
    with open(controller._admin_graph_state_path(), 'w') as handle:
        json.dump({'mode': 'off'}, handle)
    with pytest.raises(ValueError):
        controller._load_admin_graph_off()
    assert controller.graph_mode == GraphMode.STRICT
