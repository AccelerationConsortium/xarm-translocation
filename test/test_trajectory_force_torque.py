"""Offline report fixtures: no hardware, sensor commands, or wall-clock sleeps."""

import json
import math
import struct

import pytest

from robot_motion.trajectory.executor import RealtimeJointMonitor
from robot_motion.trajectory.force_torque import ForceTorqueTelemetry


def report(values=(1, 2, 3, 0.1, 0.2, 0.3), size=135):
    frame = bytearray(size)
    struct.pack_into(">I", frame, 0, size)
    frame[4] = 0x11
    struct.pack_into("<7f", frame, 7, *([0.1] * 7))
    if size >= 135:
        struct.pack_into("<6f", frame, 87, *values)
        # Deliberately different raw values catch the wrong channel/offset.
        struct.pack_into("<6f", frame, 111, *([999.0] * 6))
    return bytes(frame)


def test_tare_timestamps_and_polling_do_not_refresh_a_sample():
    now = [10.0]
    config = {"revision": "cfg:1", "service_tare": {
        "completed": True, "offset": [0.5, 1, 1.5, 0, 0, 0]}}
    telemetry = ForceTorqueTelemetry(enabled=True, configuration=config, clock=lambda: now[0])
    config["service_tare"]["offset"][0] = 999  # snapshot is immutable
    telemetry.start()
    telemetry.receive(report(), 10.0, 1700000000.0)
    first = telemetry.snapshot()
    assert first["valid"]
    assert first["sample"]["wrench"] == pytest.approx([0.5, 1, 1.5, 0.1, 0.2, 0.3])
    assert first["sample"]["sensor_sampled_at"] is None
    assert first["sample"]["service_tare_applied"] is True
    assert first["configuration"]["service_config_revision"] == "cfg:1"
    first["sample"]["wrench"][0] = 999
    first["configuration"]["service_tare"]["offset"][0] = 999
    now[0] += 0.6
    stale = telemetry.snapshot()
    assert not stale["valid"] and stale["reason"] == "stale_report"
    assert stale["age_s"] == pytest.approx(0.6)
    assert stale["sample"]["sequence"] == 1
    assert stale["sample"]["received_s"] == 10.0
    assert stale["sample"]["wrench"][0] == 0.5
    telemetry.receive(report(), now[0], 1700000000.6)  # equal values can be new reports
    assert telemetry.snapshot()["valid"]
    assert telemetry.snapshot()["sample"]["sequence"] == 2
    telemetry.stop()
    assert telemetry.snapshot()["reason"] == "report_stream_stopped"
    assert not telemetry.snapshot()["valid"]


@pytest.mark.parametrize("size", [87, 111, 134])
def test_missing_fields_never_become_zero_measurements(size):
    telemetry = ForceTorqueTelemetry(enabled=True)
    telemetry.start()
    telemetry.receive(report(size=size), 1, 1)
    snapshot = telemetry.snapshot()
    assert snapshot["sample"] is None
    assert snapshot["reason"] == "ft_fields_unavailable"
    assert not snapshot["valid"]


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_invalid_sample_does_not_refresh_previous_good_sample(bad):
    telemetry = ForceTorqueTelemetry(enabled=True, clock=lambda: 2.0)
    telemetry.start()
    telemetry.receive(report(), 1, 1)
    telemetry.receive(report((bad, 2, 3, 4, 5, 6)), 2, 2)
    snapshot = telemetry.snapshot()
    assert snapshot["reason"] == "invalid_ft_values"
    assert snapshot["sample"]["received_s"] == 1
    assert snapshot["invalid_reports"] == 1
    json.dumps(telemetry.record(), allow_nan=False)


def test_disabled_sensor_and_invalid_tare_do_not_publish_wrenches():
    for telemetry in (
        ForceTorqueTelemetry(),
        ForceTorqueTelemetry(enabled=True, configuration={
            "service_tare": {"completed": True, "offset": [1, 2]}}),
    ):
        telemetry.start()
        telemetry.receive(report(), 1, 1)
        assert telemetry.snapshot()["sample"] is None
        assert not telemetry.snapshot()["valid"]


def test_bounded_history_keeps_latest_and_reports_overflow():
    telemetry = ForceTorqueTelemetry(enabled=True, keep=2, clock=lambda: 3)
    telemetry.start()
    for t in (1, 2, 3):
        telemetry.receive(report(), t, t)
    telemetry.stop("report stream closed")
    record = telemetry.record()
    assert record["sequence"] == [2, 3]
    assert record["received_s"] == [2, 3]
    assert record["dropped_samples"] == 1
    assert record["sample"]["sequence"] == 3
    assert record["reason"] == "report stream closed"
    assert not record["valid"]


def test_fragmented_report_socket_decodes_ft_and_joints_without_sending(monkeypatch):
    first, second = report(), report((2, 3, 4, 5, 6, 7))
    chunks = iter([first[:2], first[2:90], first[90:] + second, b""])

    class ReadOnlySocket:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def settimeout(self, value):
            pass

        def recv(self, size):
            return next(chunks)

        # No send methods: any attempt to issue commands fails this test.

    monkeypatch.setattr("robot_motion.trajectory.executor.socket.create_connection",
                        lambda *args, **kwargs: ReadOnlySocket())
    monitor = RealtimeJointMonitor("unused", 5, ft_enabled=True)
    monitor.force_torque.start()
    monitor._run()
    assert len(monitor.samples) == 2
    record = monitor.force_torque.record()
    assert record["received_s"] == [s[0] for s in monitor.samples]
    assert record["wrench"][0] == pytest.approx([1, 2, 3, 0.1, 0.2, 0.3])
    assert record["wrench"][1] == [2, 3, 4, 5, 6, 7]
    assert not record["valid"]  # EOF invalidates immediately
    assert record["reason"] == "report stream closed"
