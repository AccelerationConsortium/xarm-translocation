"""Tests for the dashboard events exporter (core/events_exporter.py).

Covers the exporter in isolation (queue/batch/failure semantics against
an injected transport — no real HTTP) and its wiring into
``XArmController``'s SDK callbacks.
"""

import sys
import os

import pytest

# Ensure src is in the python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

from src.core.events_exporter import EventsExporter
from src.core.xarm_controller import XArmController, ComponentState


class RecordingTransport:
    """Test transport: records payloads, optionally failing first."""

    def __init__(self, fail_times: int = 0):
        self.payloads = []
        self.fail_times = fail_times

    def __call__(self, payload):
        if self.fail_times > 0:
            self.fail_times -= 1
            raise ConnectionError("simulated ingest outage")
        self.payloads.append(payload)


def drained_records(transport):
    """Flatten all delivered payloads into one record list."""
    records = []
    for payload in transport.payloads:
        records.extend(payload["records"])
    return records


class TestEventsExporter:
    def test_disabled_without_url(self):
        exporter = EventsExporter.from_env(environ={})
        assert exporter.enabled is False
        assert exporter.emit("state_transition", to_state="ready") is False
        exporter.close()  # no-op, must not raise

    def test_from_env_reads_url_and_device_id(self):
        exporter = EventsExporter.from_env(environ={
            "XARM_INGEST_URL": "http://dash:8001/api/ingest/events",
            "XARM_INGEST_DEVICE_ID": "xarm_test",
        })
        assert exporter.enabled is True
        assert exporter.ingest_url == "http://dash:8001/api/ingest/events"
        assert exporter.device_id == "xarm_test"
        exporter.close()

    def test_emit_delivers_ingest_shaped_payload(self):
        transport = RecordingTransport()
        exporter = EventsExporter("http://dash/api/ingest/events", transport=transport)
        assert exporter.emit(
            "state_transition",
            from_state="ready",
            to_state="busy",
            message="moving",
            xarm_state=1,
            graph_node="deck_home",
            dropped_none=None,
        )
        exporter.close()

        records = drained_records(transport)
        assert len(records) == 1
        record = records[0]
        assert transport.payloads[0]["device_id"] == "xarm_translocation"
        assert record["event"] == "state_transition"
        assert record["from_state"] == "ready"
        assert record["to_state"] == "busy"
        assert record["message"] == "moving"
        # UTC ISO-8601 with Z suffix per STATUS_SPEC timestamp rules.
        assert record["timestamp"].endswith("Z")
        # extra keys ride through; None values are stripped.
        assert record["extra"] == {"xarm_state": 1, "graph_node": "deck_home"}

    def test_transport_failure_is_swallowed_and_recovers(self):
        transport = RecordingTransport(fail_times=1)
        exporter = EventsExporter("http://dash/api/ingest/events", transport=transport)
        exporter.emit("error", message="first batch — lost to the outage")
        # Let the worker consume the first record before enqueueing the
        # second, so the two land in separate batches deterministically.
        import time
        deadline = time.time() + 5
        while transport.fail_times > 0 and time.time() < deadline:
            time.sleep(0.01)
        exporter.emit("error", message="second batch — delivered")
        exporter.close()

        records = drained_records(transport)
        assert [r["message"] for r in records] == ["second batch — delivered"]

    def test_queue_full_drops_instead_of_blocking(self):
        blocked = RecordingTransport(fail_times=10**6)  # worker never drains
        exporter = EventsExporter(
            "http://dash/api/ingest/events", transport=blocked, queue_size=2,
        )
        results = [exporter.emit("error", message=str(i)) for i in range(20)]
        assert False in results  # overflow was dropped, not blocked on
        exporter.close(timeout_s=0.2)


class TestControllerWiring:
    @pytest.fixture
    def controller(self, mock_config_files, mock_xarm_api, monkeypatch):
        monkeypatch.setattr(
            'src.core.xarm_controller.XArmAPI', lambda *a, **k: mock_xarm_api,
        )
        c = XArmController(profile_name='test_profile', auto_enable=False)
        return c

    def attach_exporter(self, controller):
        transport = RecordingTransport()
        controller.events_exporter = EventsExporter(
            "http://dash/api/ingest/events", transport=transport,
        )
        return transport

    def test_exporter_disabled_by_default(self, controller):
        """No XARM_INGEST_URL -> exporter exists but is a no-op."""
        assert controller.events_exporter.enabled is False
        # Callbacks must not raise with the exporter disabled.
        controller._error_warn_callback({'error_code': 11, 'warn_code': 0})
        controller._state_changed_callback({'state': 4})

    def test_error_callback_emits_error_and_transition(self, controller):
        transport = self.attach_exporter(controller)
        controller._error_warn_callback({'error_code': 11, 'warn_code': 0})
        controller.events_exporter.close()

        events = {r["event"] for r in drained_records(transport)}
        assert events == {"error", "state_transition"}
        error = [r for r in drained_records(transport) if r["event"] == "error"][0]
        assert error["extra"]["error_code"] == 11
        assert error["extra"]["severity"] == "error"
        transition = [
            r for r in drained_records(transport) if r["event"] == "state_transition"
        ][0]
        assert transition["to_state"] == "error"

    def test_warn_only_callback_emits_warning_severity(self, controller):
        transport = self.attach_exporter(controller)
        controller._error_warn_callback({'error_code': 0, 'warn_code': 3})
        controller.events_exporter.close()

        records = drained_records(transport)
        assert len(records) == 1
        assert records[0]["event"] == "error"
        assert records[0]["extra"]["severity"] == "warning"
        assert records[0]["extra"]["warn_code"] == 3

    def test_state_callback_emits_mapped_transitions_once(self, controller):
        transport = self.attach_exporter(controller)
        controller._state_changed_callback({'state': 1})   # busy
        controller._state_changed_callback({'state': 1})   # duplicate — suppressed
        controller._state_changed_callback({'state': 2})   # ready
        controller.events_exporter.close()

        records = drained_records(transport)
        assert [(r["from_state"] if "from_state" in r else None, r["to_state"])
                for r in records] == [(None, "busy"), ("busy", "ready")]
        assert records[0]["extra"]["xarm_state"] == 1

    def test_state_4_still_flips_error_state(self, controller):
        """The new emission path must not change the existing behavior."""
        transport = self.attach_exporter(controller)
        controller._state_changed_callback({'state': 4})
        assert controller.alive is False
        assert controller.states['arm'] == ComponentState.ERROR
        controller.events_exporter.close()
        records = drained_records(transport)
        assert records[0]["to_state"] == "error"

    def test_disconnect_emits_shutdown(self, controller):
        transport = self.attach_exporter(controller)
        controller.disconnect()
        controller.events_exporter.close()
        records = drained_records(transport)
        assert [r["event"] for r in records] == ["shutdown", "state_transition"]
        assert records[1]["to_state"] == "requires_init"
