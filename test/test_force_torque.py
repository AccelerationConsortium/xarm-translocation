"""FT snapshot contract, using only a fake SDK; never initialize a connection."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from threading import Event
from unittest.mock import Mock

import pytest
from fastapi.testclient import TestClient

from src.core.xarm_controller import XArmController, ComponentState, force_torque_derived
from src.core import xarm_api_server as server


@pytest.fixture
def ft_controller(mock_config_files, mock_xarm_api, monkeypatch):
    monkeypatch.setattr('src.core.xarm_controller.XArmAPI', lambda *a, **k: mock_xarm_api)
    controller = XArmController(profile_name='test_profile', auto_enable=False)
    controller.states['force_torque'] = ComponentState.ENABLED
    controller.force_torque_config['calibration'] = {'auto_calibrate': False}
    yield controller
    mock_xarm_api.connect.assert_not_called()
    mock_xarm_api.motion_enable.assert_not_called()
    mock_xarm_api.set_position.assert_not_called()
    mock_xarm_api.set_ft_sensor_zero.assert_not_called()
    mock_xarm_api.iden_ft_sensor_load_offset.assert_not_called()


@pytest.fixture
def ft_client(ft_controller, monkeypatch):
    monkeypatch.setattr(server, 'controller', ft_controller)
    # No lifespan: this test must not start background device integrations.
    return TestClient(server.app)


def assert_no_legacy_keys(value):
    if isinstance(value, dict):
        assert not {'calibrated', 'total_magnitude', 'dead_zone'} & value.keys()
        for nested in value.values():
            assert_no_legacy_keys(nested)
    elif isinstance(value, list):
        for nested in value:
            assert_no_legacy_keys(nested)


def test_http_one_read_consistent_snapshot(ft_controller, ft_client):
    ft_controller.arm.get_ft_sensor_data.side_effect = [
        (0, [3, 4, 0, 0, 0, 3]), (0, [99] * 6)]
    response = ft_client.get('/force-torque/data')
    assert response.status_code == 200
    sample = response.json()
    assert sample['wrench'] == [3, 4, 0, 0, 0, 3]
    assert sample['force_magnitude'] == 5
    assert sample['torque_magnitude'] == 3
    assert sample['force_direction'] == [0.6, 0.8, 0]
    assert sample['torque_direction'] == [0, 0, 1]
    assert sample['sensor_sampled_at'] is None
    assert sample['service_received_at'].endswith('+00:00')
    assert sample['service_tare_applied'] is False
    ft_controller.arm.get_ft_sensor_data.assert_called_once_with(is_raw=False)
    config = ft_client.get('/force-torque/config', params={'revision': sample['config_revision']}).json()
    assert config['source']['channel'] == 'controller_compensated_filtered'
    assert config['geometry'] == dict(status='unknown', frame_id=None, axis_convention=None,
                                      torque_reference_point=None, interaction_sign=None)
    assert config['controller_compensation']['validation_status'] == 'unknown'
    assert config['controller_compensation']['configuration_status'] == 'unknown'
    assert config['units'] == {'force': 'N', 'torque': 'N*m'}
    assert ft_client.get('/force-torque/status').json()['last_sample'] == sample
    assert ft_controller.arm.get_ft_sensor_data.call_count == 1
    for obj in (sample, config, ft_client.get('/force-torque/status').json()):
        assert_no_legacy_keys(obj)


@pytest.mark.parametrize('reply', [
    (1, [0] * 6), (0, [0] * 5), (0, [0] * 7), (0, None),
    (0, [float('nan')] * 6), (0, [float('inf')] * 6),
    (0, ['0'] * 6), (0, [True] * 6), (0, [1.7e308] * 6),
    RuntimeError('transport failed'),
])
def test_invalid_read_does_not_publish(ft_controller, ft_client, reply):
    previous = ft_controller.get_force_torque_data()
    count = len(ft_controller.force_torque_history)
    if isinstance(reply, Exception):
        ft_controller.arm.get_ft_sensor_data.side_effect = reply
    else:
        ft_controller.arm.get_ft_sensor_data.return_value = reply
    assert ft_client.get('/force-torque/data').status_code == 500
    assert ft_controller.last_force_torque_sample == previous
    assert len(ft_controller.force_torque_history) == count


@pytest.mark.parametrize('norm,defined', [(0, False), (1.99, False), (2, True), (2.01, True)])
def test_deadband_boundaries(norm, defined):
    result = force_torque_derived([norm, 0, 0, 0, norm, 0], 2, 2)
    assert (result['force_direction'] is not None) is defined
    assert (result['torque_direction'] is not None) is defined


def test_independent_deadbands_and_zero(ft_controller):
    ft_controller.force_torque_config['direction_detection'] = {
        'force_direction_deadband_n': 5, 'torque_direction_deadband_nm': 0.1}
    ft_controller.arm.get_ft_sensor_data.return_value = (0, [3, 0, 0, 0, 0.2, 0])
    sample = ft_controller.get_force_torque_data()
    assert sample['force_direction'] is None
    assert sample['torque_direction'] == [0, 1, 0]
    zero = force_torque_derived([0] * 6, 0, 0)
    assert zero['force_direction'] is zero['torque_direction'] is None


@pytest.mark.parametrize('config', [
    {'dead_zone': 2}, {'force_direction_deadband_n': -1},
    {'torque_direction_deadband_nm': float('nan')},
    {'force_direction_deadband_n': True}, {'torque_direction_deadband_nm': '2'},
])
def test_reject_invalid_or_obsolete_deadbands(ft_controller, config):
    ft_controller.force_torque_config['direction_detection'] = config
    with pytest.raises(ValueError):
        ft_controller.get_force_torque_config()
    assert ft_controller.get_force_torque_data() is None
    ft_controller.arm.get_ft_sensor_data.assert_not_called()


def test_tare_and_old_revision(ft_controller, ft_client):
    ft_controller.arm.get_ft_sensor_data.return_value = (0, [3, 4, 0, 0, 0, 3])
    before = ft_controller.get_force_torque_data()
    old = ft_controller.get_force_torque_config(before['config_revision'])
    assert ft_controller.calibrate_force_torque_sensor(samples=2, delay=0)
    after = ft_controller.get_force_torque_data()
    assert after['wrench'] == [0] * 6
    assert after['service_tare_applied'] is True
    assert after['config_revision'] != before['config_revision']
    assert ft_controller.get_force_torque_config(before['config_revision']) == old
    current = ft_controller.get_force_torque_config(after['config_revision'])
    assert current['service_tare']['offset'] == [3, 4, 0, 0, 0, 3]
    assert current['controller_compensation']['validation_status'] == 'unknown'
    assert ft_client.get('/force-torque/config?revision=missing').status_code == 404
    # Returned objects never alias the stored samples/configs.
    after['wrench'][0] = 999
    current['service_tare']['offset'][0] = 999
    assert ft_controller.get_force_torque_status()['last_sample']['wrench'][0] == 0
    assert ft_controller.get_force_torque_config()['service_tare']['offset'][0] == 3


def test_failed_tare_keeps_previous_revision(ft_controller):
    assert ft_controller.calibrate_force_torque_sensor(samples=1, delay=0)
    before = ft_controller.get_force_torque_config()
    ft_controller.arm.get_ft_sensor_data.return_value = (0, [float('nan')] * 6)
    assert not ft_controller.calibrate_force_torque_sensor(samples=1, delay=0)
    assert ft_controller.get_force_torque_config() == before


def test_revision_retention_follows_history(ft_controller):
    ft_controller.force_torque_history = deque(maxlen=2)
    first = ft_controller.get_force_torque_data()
    for deadband in (4, 6):
        ft_controller.force_torque_config['direction_detection'] = {'force_direction_deadband_n': deadband}
        ft_controller.get_force_torque_data()
    assert ft_controller.get_force_torque_config(first['config_revision']) is None
    for sample in ft_controller.force_torque_history:
        assert ft_controller.get_force_torque_config(sample['config_revision']) is not None
    assert len(ft_controller._ft_configs) == 2


def test_tare_cannot_interleave_with_sample(ft_controller):
    reading = Event()
    release_read = Event()
    tare_started = Event()
    calls = []

    def read(is_raw):
        calls.append(is_raw)
        if len(calls) == 1:
            reading.set()
            assert release_read.wait(3)
        return 0, [3, 4, 0, 0, 0, 0]

    def tare():
        tare_started.set()
        return ft_controller.calibrate_force_torque_sensor(samples=1, delay=0)

    ft_controller.arm.get_ft_sensor_data.side_effect = read
    with ThreadPoolExecutor(2) as pool:
        sample_future = pool.submit(ft_controller.get_force_torque_data)
        assert reading.wait(3)
        tare_future = pool.submit(tare)
        assert tare_started.wait(3)
        try:
            assert not tare_future.done()
            assert calls == [False]
        finally:
            release_read.set()
        sample = sample_future.result(timeout=3)
        assert tare_future.result(timeout=3)
    assert sample['wrench'] == [3, 4, 0, 0, 0, 0]
    assert sample['service_tare_applied'] is False
    assert not ft_controller.get_force_torque_config(sample['config_revision'])['service_tare']['completed']
    assert ft_controller.get_force_torque_data()['wrench'] == [0] * 6


def test_status_config_and_disconnected_reads_never_poll(ft_controller, ft_client):
    ft_controller.arm.connected = False
    assert ft_client.get('/force-torque/config').status_code == 200
    assert ft_client.get('/force-torque/status').json()['last_sample'] is None
    assert ft_client.get('/force-torque/data').status_code == 500
    ft_controller.arm.get_ft_sensor_data.assert_not_called()
    ft_controller.arm.get_version.assert_not_called()
    ft_controller.arm.get_ft_sensor_config.assert_not_called()
    ft_controller.arm.get_ft_sensor_version.assert_not_called()


def test_no_controller_does_not_autoconnect(ft_client, monkeypatch):
    monkeypatch.setattr(server, 'controller', None)
    constructor = Mock(side_effect=AssertionError('must not connect'))
    monkeypatch.setattr(server, 'XArmController', constructor)
    for path in ('data', 'config', 'status'):
        assert ft_client.get('/force-torque/' + path).status_code == 400
    constructor.assert_not_called()


def test_safety_uses_same_reading(ft_controller):
    ft_controller.force_torque_config['safety_thresholds'] = {
        'force': {'magnitude': 4}, 'torque': {'magnitude': 4}}
    ft_controller.arm.get_ft_sensor_data.side_effect = [(0, [3, 4, 0, 0, 0, 0]), (0, [0] * 6)]
    ft_controller._trigger_force_torque_alert = Mock()
    assert ft_controller.check_force_torque_safety()
    ft_controller.arm.get_ft_sensor_data.assert_called_once_with(is_raw=False)
    assert ft_controller._trigger_force_torque_alert.call_args.args[2] == [3, 4, 0, 0, 0, 0]


def test_enable_disable_remain_explicit(ft_controller):
    assert ft_controller.disable_force_torque_sensor()
    assert ft_controller.get_force_torque_data() is None
    assert ft_controller.enable_force_torque_sensor()
    ft_controller.arm.get_ft_sensor_data.assert_not_called()


def test_config_uses_only_populated_firmware_cache(ft_controller):
    class CachedArm:
        _version = 'class-default-must-not-be-used'

        @property
        def version(self):
            raise AssertionError('lazy version getter must never run')

    sdk_arm = CachedArm()
    ft_controller.arm._arm = sdk_arm
    assert ft_controller.get_force_torque_config()['device']['controller_firmware'] is None
    sdk_arm._version = '5,5,XF1305,MC1305,v2.8.2'
    config = ft_controller.get_force_torque_config()
    assert config['device']['controller_firmware'] == sdk_arm._version
    assert config['device']['controller_firmware_source'] == 'sdk_cache'
    ft_controller.arm.get_version.assert_not_called()


def test_read_ids_are_service_acquisitions_not_hardware_samples(ft_controller):
    a = ft_controller.get_force_torque_data()
    b = ft_controller.get_force_torque_data()
    assert a['wrench'] == b['wrench']
    assert a['sample_id'] != b['sample_id']
    assert a['config_revision'] == b['config_revision']
    assert a['sensor_sampled_at'] is b['sensor_sampled_at'] is None


def test_disconnect_invalidates_tare_but_retains_old_sample(ft_controller):
    assert ft_controller.calibrate_force_torque_sensor(samples=1, delay=0)
    sample = ft_controller.get_force_torque_data()
    ft_controller.disconnect()
    status = ft_controller.get_force_torque_status()
    assert status['service_tare_completed'] is False
    assert status['enabled'] is False
    assert status['last_sample'] == sample
    assert ft_controller.get_force_torque_config(sample['config_revision'])['service_tare']['completed']
    assert ft_controller.get_force_torque_config()['service_tare']['offset'] is None
