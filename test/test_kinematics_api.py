"""Queries must never send motion, settings, or enable commands."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from fastapi.testclient import TestClient
from src.core import xarm_api_server as api


class Arm:
    connected = True
    axis = 5
    device_type = 5
    version_number = (2, 8, 2)
    tcp_offset = [159.5, 0, 59.5, 0, 0, 0]
    world_offset = [0] * 6
    tcp_load = [0.79, [22, 0, 24]]

    def __init__(self):
        self.ik = Mock(return_value=(0, [1, 2, 3, 4, 5, 0, 0]))
        self.get_servo_angle = Mock(return_value=(0, [0, 1, 2, 3, 4, 0, 0]))
        self.get_forward_kinematics = Mock(return_value=(0, [10, 20, 30, 180, 0, 0]))
        self.is_joint_limit = Mock(return_value=(0, False))
        self.get_version = Mock(return_value=(0, '5,5,XF1305,MC1305,v2.8.2'))
        self.get_dh_params = Mock(return_value=(0, [0] * 28))
        self.get_reduced_states = Mock(return_value=(0, [0, [9999, -9999] * 3, 1000, 180, [-360, 360] * 7, 0, 1]))

    def get_inverse_kinematics(self, pose, input_is_radian=None, return_is_radian=None, limited=True, ref_angles=None):
        return self.ik(pose, input_is_radian=input_is_radian, return_is_radian=return_is_radian,
                       limited=limited, ref_angles=ref_angles)


# Avoid application lifespan (which starts cameras and expects a full controller).
@pytest.fixture
def setup(monkeypatch):
    arm = Arm()
    monkeypatch.setattr(api, 'controller', SimpleNamespace(arm=arm))
    yield TestClient(api.app), arm


def test_reference_forwarded(setup):
    client, arm = setup
    body = {'pose': [10, 20, 30, 180, 0, 0], 'reference_angles': [1, 2, 3, 4, 5]}
    r = client.post('/kinematics/ik', json=body)
    assert r.status_code == 200
    assert r.json()['result']['data'] == [1, 2, 3, 4, 5]
    assert r.json()['joint_limit_check']['data'] is False
    assert r.json()['violating_joints'] is None
    arm.ik.assert_called_once_with(body['pose'], input_is_radian=False, return_is_radian=False,
                                  limited=False, ref_angles=body['reference_angles'])
    arm.get_servo_angle.assert_not_called()


def test_default_reference_is_live(setup):
    client, arm = setup
    r = client.post('/kinematics/ik', json={'pose': [1] * 6})
    assert r.json()['reference_angles'] == [0, 1, 2, 3, 4]
    arm.get_servo_angle.assert_called_once_with(is_radian=False)


def test_old_firmware_refused(setup):
    client, arm = setup
    arm.version_number = (2, 6, 0)
    assert client.post('/kinematics/ik', json={'pose': [1] * 6}).status_code == 409
    arm.ik.assert_not_called()


@pytest.mark.parametrize('body', [{'joints': [0] * 6}, {'joints': [0] * 4}, {'joints': ['NaN'] * 5}])
def test_invalid_joints_never_reach_sdk(setup, body):
    client, arm = setup
    assert client.post('/kinematics/fk', json=body).status_code == 422
    arm.get_forward_kinematics.assert_not_called()


def test_fk_and_config(setup):
    client, arm = setup
    assert client.post('/kinematics/fk', json={'joints': [0] * 5}).json()['result']['available']
    config = client.get('/kinematics/config').json()
    assert config['reference_angles_supported']
    assert config['tcp_offset'] == arm.tcp_offset
    assert config['dh_parameters']['available']
    bounds = client.get('/kinematics/limits').json()
    assert bounds['ordinary_joint_limits']['ranges'] is None
    assert bounds['reduced']['enabled'] is False
    assert len(bounds['reduced']['joint_ranges_raw']) == 14


def test_sdk_failure_and_limit_are_distinct(setup):
    client, arm = setup
    arm.ik.return_value = (-7, [99] * 7)
    r = client.post('/kinematics/ik', json={'pose': [1] * 6}).json()
    assert r['result']['data'] is None
    assert r['result']['reason'] == 'JOINT_LIMIT'
    arm.is_joint_limit.assert_not_called()
    arm.ik.return_value = (0, [1] * 7)
    arm.is_joint_limit.return_value = (0, True)
    r = client.post('/kinematics/ik', json={'pose': [1] * 6}).json()
    assert r['result']['available'] and r['joint_limit_check']['data'] is True


def test_disconnected(setup):
    client, arm = setup
    arm.connected = False
    assert client.get('/kinematics/config').status_code == 503
    arm.get_version.assert_not_called()


def test_reference_read_failure_not_ignored(setup):
    client, arm = setup
    arm.get_servo_angle.return_value = (-1, [])
    r = client.post('/kinematics/ik', json={'pose': [1] * 6}).json()
    assert not r['result']['available']
    arm.ik.assert_not_called()


def test_unknown_reference_field_rejected(setup):
    client, arm = setup
    assert client.post('/kinematics/ik', json={'pose': [1] * 6, 'ref_angles': [0] * 5}).status_code == 422
    arm.ik.assert_not_called()


def test_limit_query_failure_is_not_false(setup):
    client, arm = setup
    arm.is_joint_limit.return_value = (-1, False)
    r = client.post('/kinematics/ik', json={'pose': [1] * 6}).json()
    assert r['joint_limit_check']['data'] is None
    assert not r['joint_limit_check']['available']


def test_invalid_reference_length_rejected(setup):
    client, arm = setup
    assert client.post('/kinematics/ik', json={'pose': [1] * 6, 'reference_angles': [0] * 6}).status_code == 422
    arm.ik.assert_not_called()


def _warn_sequence(arm, warns):
    """get_err_warn_code returns these warning codes in order (fresh reads)."""
    arm.get_err_warn_code = Mock(side_effect=[(0, [0, w]) for w in warns])
    arm.clean_warn = Mock(return_value=0)


def test_no_solution_warning_is_not_a_solution_and_is_cleared(setup):
    # The SDK reports code 0 for the controller's warning reply, so the IK
    # result looks like a solution; W14 latched by this query says it is not.
    client, arm = setup
    _warn_sequence(arm, [0, 14, 0])
    r = client.post('/kinematics/ik', json={'pose': [1] * 6}).json()
    assert r['result']['available'] is False
    assert r['result']['data'] is None
    assert r['result']['reason'] == 'controller_warning'
    assert r['controller_warning'] == {
        'warn_code': 14, 'warn_title': 'User instructions and parameters have no solution',
        'raised_by_this_query': True, 'cleared': True}
    arm.clean_warn.assert_called_once_with()
    arm.is_joint_limit.assert_not_called()


def test_clear_is_verified_by_readback(setup):
    client, arm = setup
    _warn_sequence(arm, [0, 14, 14])
    r = client.post('/kinematics/ik', json={'pose': [1] * 6}).json()
    assert r['controller_warning']['cleared'] is False
    assert r['result']['available'] is False


def test_warning_latched_before_the_query_is_left_alone(setup):
    client, arm = setup
    _warn_sequence(arm, [14, 14])
    r = client.post('/kinematics/ik', json={'pose': [1] * 6}).json()
    assert r['controller_warning']['raised_by_this_query'] is False
    assert r['controller_warning']['cleared'] is False
    assert r['result']['available'] is False
    arm.clean_warn.assert_not_called()


def test_unreadable_warning_before_is_not_cleared(setup):
    client, arm = setup
    arm.get_err_warn_code = Mock(side_effect=[(-1, [0, 0]), (0, [0, 14])])
    arm.clean_warn = Mock(return_value=0)
    r = client.post('/kinematics/ik', json={'pose': [1] * 6}).json()
    assert r['result']['available'] is False
    arm.clean_warn.assert_not_called()


def test_no_warning_keeps_the_solution(setup):
    client, arm = setup
    _warn_sequence(arm, [0, 0])
    r = client.post('/kinematics/ik', json={'pose': [1] * 6}).json()
    assert r['result']['available'] is True
    assert r['controller_warning'] is None
    arm.clean_warn.assert_not_called()
