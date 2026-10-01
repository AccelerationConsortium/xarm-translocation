"""Gripper feedback must keep updating without dashboard viewers."""

import asyncio
from unittest.mock import AsyncMock

import pytest

from src.core import xarm_api_server as api
from src.core.status_builder import _build_gripper_details


def test_poll_tracks_grasp_and_slip_without_websockets(monkeypatch, initialized_controller):
    c = initialized_controller
    c.gripper_type = 'bio_gen2'
    c.arm.get_bio_gripper_status.side_effect = [(0, 9), (0, 10), (0, 11), (0, 8)]
    c.arm.get_bio_gripper_g2_position.return_value = (0, 82)
    c.arm.get_bio_gripper_error.return_value = (0, 12)
    c.last_gripper_position = 71
    monkeypatch.setattr(api, 'controller', c)
    monkeypatch.setattr(api.manager, 'active_connections', [])
    monkeypatch.setattr(api, 'TELEMETRY_HZ', 0)
    snapshots = []

    async def tick(_interval):
        snapshots.append(_build_gripper_details(c))
        if len(snapshots) == 5:
            raise asyncio.CancelledError

    monkeypatch.setattr(api.asyncio, 'sleep', tick)
    asyncio.run(api.gripper_status_loop())

    assert snapshots[1]['motion_state'] == 'moving'
    assert snapshots[1]['object_detected'] is False
    assert snapshots[2]['motion_state'] == 'object_detected'
    assert snapshots[2]['object_detected'] is True
    assert snapshots[2]['position_mm'] == 82
    assert snapshots[3]['motion_state'] == 'fault'
    assert snapshots[3]['error_code'] == 12
    assert snapshots[3]['object_detected'] is False
    assert snapshots[4]['motion_state'] == 'stop'
    assert snapshots[4]['object_detected'] is False
    assert snapshots[4]['error_code'] == 0
    assert c.last_gripper_position == 71


@pytest.mark.parametrize('connected', [False, None])
def test_poll_skips_disconnected_controller(monkeypatch, initialized_controller, connected):
    c = initialized_controller
    c.arm.connected = False
    monkeypatch.setattr(api, 'controller', c if connected is False else None)
    monkeypatch.setattr(api.asyncio, 'sleep', AsyncMock(side_effect=[None, asyncio.CancelledError]))
    asyncio.run(api.gripper_status_loop())
    c.arm.get_bio_gripper_status.assert_not_called()


def test_poll_recovers_after_refresh_exception(monkeypatch, initialized_controller):
    c = initialized_controller
    # A robot fault must not disable connected gripper diagnostics.
    c.arm.error_code = 19
    assert c.is_alive is False
    monkeypatch.setattr(api, 'controller', c)
    refresh = AsyncMock(side_effect=[RuntimeError('temporary read failure'), None])
    monkeypatch.setattr(api.asyncio, 'to_thread', refresh)
    monkeypatch.setattr(api.asyncio, 'sleep', AsyncMock(side_effect=[None, None, asyncio.CancelledError]))
    asyncio.run(api.gripper_status_loop())
    assert refresh.await_count == 2
