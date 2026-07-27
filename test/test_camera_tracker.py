"""Tests for the camera tracker (core/camera_tracker.py).

Covers the tracker in isolation: tag -> station -> view resolution, the
dedupe that stops redundant sends within a station, disabled/no-op
configs, view-body construction (preset vs raw ptz vs placeholder), the
optional API-key header, and the contract that nothing here ever raises
into the motion path. An injected sender records calls synchronously, so
there is no real HTTP and no thread timing.
"""

import os
import sys

import pytest

# Ensure src is in the python path (same convention as the other tests).
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

from src.core.camera_tracker import CameraTracker


class FakeNode:
    """Duck-typed stand-in for motion_graph.Node (only .id + .tags used)."""

    def __init__(self, node_id, tags):
        self.id = node_id
        self.tags = tuple(tags)


class RecordingSender:
    """Records (url, payload, headers, timeout); optionally raises."""

    def __init__(self, fail=False):
        self.calls = []
        self.fail = fail

    def __call__(self, url, payload, headers, timeout_s):
        self.calls.append((url, payload, headers, timeout_s))
        if self.fail:
            raise ConnectionError("simulated camera outage")


def _config(**overrides):
    cfg = {
        "enabled": True,
        "dashboard_base_url": "http://dash:8001",
        "camera_id": "cam_hte_tapo_c245",
        "api_key_env": "XARM_CAMERA_API_KEY",
        "request_timeout_seconds": 4.0,
        "views": {
            "left": {"preset_id": "p-left"},
            "middle": {"preset_id": "p-middle"},
            "middle_down": {"preset_id": "p-middledown"},
            "right": {"preset_id": "p-right"},
        },
        "stations": {
            "uplc": "left",
            "opentrons": "middle",
            "deck": "middle_down",
            "shaker": "right",
            "filter": "right",
        },
    }
    cfg.update(overrides)
    return cfg


def _tracker(sender=None, environ=None, **overrides):
    return CameraTracker(_config(**overrides), sender=sender or RecordingSender(),
                         environ=environ if environ is not None else {})


# ---------------------------------------------------------------------------
# Resolution
# ---------------------------------------------------------------------------

class TestResolution:
    @pytest.mark.parametrize("tags,expected", [
        (("uplc", "transit_home"), "left"),
        (("opentrons", "slot2"), "middle"),
        (("deck", "slot1"), "middle_down"),
        (("hood", "shaker"), "right"),
        (("hood", "filter"), "right"),
    ])
    def test_station_tags_map_to_views(self, tags, expected):
        tracker = _tracker()
        assert tracker.resolve_view(tags) == expected

    @pytest.mark.parametrize("tags", [
        (),
        ("safe", "global_home"),
        ("hood", "transit_home"),  # hood alone has no station entry
    ])
    def test_off_station_tags_resolve_to_none(self, tags):
        tracker = _tracker()
        assert tracker.resolve_view(tags) is None

    def test_first_matching_tag_wins_in_config_order(self):
        # stations order is uplc, opentrons, ... so uplc beats deck here.
        tracker = _tracker()
        assert tracker.resolve_view(("deck", "uplc")) == "left"


# ---------------------------------------------------------------------------
# notify_node dispatch + dedupe
# ---------------------------------------------------------------------------

class TestNotify:
    def test_sends_preset_goto_for_station_node(self):
        sender = RecordingSender()
        tracker = _tracker(sender=sender)
        tracker.notify_node(FakeNode("uplc_home", ("uplc", "transit_home")))
        assert len(sender.calls) == 1
        url, payload, headers, timeout = sender.calls[0]
        assert url == "http://dash:8001/api/equipment/cam_hte_tapo_c245/control/preset/goto"
        assert payload == {"preset_id": "p-left"}
        assert headers["Content-Type"] == "application/json"
        assert timeout == 4.0

    def test_dedupe_same_station_sends_once(self):
        sender = RecordingSender()
        tracker = _tracker(sender=sender)
        tracker.notify_node(FakeNode("uplc_home", ("uplc",)))
        tracker.notify_node(FakeNode("uplc_draw_home", ("uplc", "drawer")))
        assert len(sender.calls) == 1  # both are the "left" view

    def test_changing_station_sends_again(self):
        sender = RecordingSender()
        tracker = _tracker(sender=sender)
        tracker.notify_node(FakeNode("uplc_home", ("uplc",)))
        tracker.notify_node(FakeNode("opentrons_home", ("opentrons",)))
        tracker.notify_node(FakeNode("uplc_home", ("uplc",)))
        views = [payload["preset_id"] for _, payload, _, _ in sender.calls]
        assert views == ["p-left", "p-middle", "p-left"]

    def test_shaker_and_filter_share_right_view_deduped(self):
        sender = RecordingSender()
        tracker = _tracker(sender=sender)
        tracker.notify_node(FakeNode("hood_shaker_high", ("hood", "shaker")))
        tracker.notify_node(FakeNode("hood_filter_home", ("hood", "filter")))
        assert len(sender.calls) == 1  # both map to "right"

    def test_off_station_node_does_not_send_and_keeps_last_view(self):
        sender = RecordingSender()
        tracker = _tracker(sender=sender)
        tracker.notify_node(FakeNode("uplc_home", ("uplc",)))
        tracker.notify_node(FakeNode("robot_home", ("safe", "global_home")))
        # No new call, and returning to uplc is still deduped.
        tracker.notify_node(FakeNode("uplc_home", ("uplc",)))
        assert len(sender.calls) == 1

    def test_api_key_header_added_when_env_set(self):
        sender = RecordingSender()
        tracker = _tracker(sender=sender, environ={"XARM_CAMERA_API_KEY": "secret"})
        tracker.notify_node(FakeNode("deck_home", ("deck",)))
        _, _, headers, _ = sender.calls[0]
        assert headers["X-Api-Key"] == "secret"

    def test_ptz_body_view(self):
        sender = RecordingSender()
        cfg = _config(views={"left": {"ptz": {"pan": -0.5, "tilt": 0.0}}})
        tracker = CameraTracker(cfg, sender=sender, environ={})
        tracker.notify_node(FakeNode("uplc_home", ("uplc",)))
        url, payload, _, _ = sender.calls[0]
        assert url.endswith("/control/ptz")
        assert payload == {"pan": -0.5, "tilt": 0.0}

    def test_placeholder_preset_is_skipped(self):
        sender = RecordingSender()
        cfg = _config(views={"left": {"preset_id": "REPLACE_ME"}})
        tracker = CameraTracker(cfg, sender=sender, environ={})
        tracker.notify_node(FakeNode("uplc_home", ("uplc",)))
        assert sender.calls == []


# ---------------------------------------------------------------------------
# Disabled / no-op and failure isolation
# ---------------------------------------------------------------------------

class TestDisabledAndFailures:
    def test_enabled_false_is_noop(self):
        sender = RecordingSender()
        tracker = _tracker(sender=sender, enabled=False)
        assert tracker.configured is False
        tracker.notify_node(FakeNode("uplc_home", ("uplc",)))
        assert sender.calls == []

    def test_missing_base_url_disables(self):
        tracker = _tracker(dashboard_base_url="")
        assert tracker.configured is False

    def test_missing_camera_id_disables(self):
        tracker = _tracker(camera_id="")
        assert tracker.configured is False

    def test_none_config_is_disabled_noop(self):
        tracker = CameraTracker(None)
        assert tracker.configured is False
        tracker.notify_node(FakeNode("uplc_home", ("uplc",)))  # must not raise

    def test_notify_none_node_is_noop(self):
        sender = RecordingSender()
        tracker = _tracker(sender=sender)
        tracker.notify_node(None)
        assert sender.calls == []

    def test_sender_failure_does_not_raise(self):
        sender = RecordingSender(fail=True)
        tracker = _tracker(sender=sender)
        # Should swallow the ConnectionError raised by the sender.
        tracker.notify_node(FakeNode("uplc_home", ("uplc",)))
        assert len(sender.calls) == 1  # attempted once, error swallowed

    def test_missing_config_file_yields_disabled_tracker(self, tmp_path):
        tracker = CameraTracker.from_config_file(str(tmp_path / "nope.yaml"))
        assert tracker.configured is False


# ---------------------------------------------------------------------------
# Following toggle (configured vs following)
# ---------------------------------------------------------------------------

class TestFollowingToggle:
    def test_follow_by_default_true_when_configured(self):
        assert _tracker().following is True

    def test_follow_by_default_false_pauses_sends(self):
        sender = RecordingSender()
        tracker = _tracker(sender=sender, follow_by_default=False)
        assert tracker.following is False
        tracker.notify_node(FakeNode("uplc_home", ("uplc",)))
        assert sender.calls == []  # configured but not following

    def test_set_following_gates_notify(self):
        sender = RecordingSender()
        tracker = _tracker(sender=sender, follow_by_default=False)
        tracker.notify_node(FakeNode("uplc_home", ("uplc",)))
        assert sender.calls == []
        assert tracker.set_following(True) is True
        tracker.notify_node(FakeNode("uplc_home", ("uplc",)))
        assert len(sender.calls) == 1

    def test_set_following_cannot_enable_when_unconfigured(self):
        tracker = _tracker(enabled=False)
        assert tracker.set_following(True) is False
        assert tracker.following is False

    def test_turning_follow_on_reaims_same_station(self):
        # Follow on -> visit uplc (sends) -> pause -> resume -> uplc again
        # should re-send even though it's the same station (dedupe reset).
        sender = RecordingSender()
        tracker = _tracker(sender=sender)
        tracker.notify_node(FakeNode("uplc_home", ("uplc",)))
        assert len(sender.calls) == 1
        tracker.set_following(False)
        tracker.set_following(True)
        tracker.notify_node(FakeNode("uplc_home", ("uplc",)))
        assert len(sender.calls) == 2  # re-aimed, not deduped away


# ---------------------------------------------------------------------------
# Availability probe (reads the camera's live /status via the dashboard)
# ---------------------------------------------------------------------------

class FakeFetcher:
    """Returns a canned /api/equipment payload; optionally raises."""

    def __init__(self, payload=None, fail=False):
        self.payload = payload
        self.fail = fail
        self.calls = 0

    def __call__(self, url, timeout_s):
        self.calls += 1
        if self.fail:
            raise ConnectionError("dashboard down")
        return self.payload


def _equipment_payload(**detail_overrides):
    details = {
        "privacy_mode": False,
        "streaming_enabled": True,
        "go2rtc_reachable": True,
        "lenses": [
            {"id": "wide", "mse_url": "/streams/api/ws?src=cam_hte_tapo_c245_wide",
             "stream_connected": True},
            {"id": "tele", "mse_url": "/streams/api/ws?src=cam_hte_tapo_c245_tele",
             "stream_connected": True},
        ],
    }
    details.update(detail_overrides)
    return [{"equipment_id": "cam_hte_tapo_c245", "equipment_status": "ready",
             "fetch_error": None, "details": details}]


def _avail_tracker(fetcher, **overrides):
    return CameraTracker(_config(**overrides), sender=RecordingSender(),
                         fetcher=fetcher, environ={})


class TestAvailability:
    def test_available_and_stream_url_derived(self):
        tracker = _avail_tracker(FakeFetcher(_equipment_payload()))
        info = tracker.availability()
        assert info["available"] is True
        assert info["reason"] is None
        assert info["configured"] is True
        assert info["stream_url"] == "ws://dash:8001/streams/api/ws?src=cam_hte_tapo_c245_wide"

    def test_lens_selection(self):
        tracker = _avail_tracker(FakeFetcher(_equipment_payload()), lens="tele")
        assert tracker.availability()["stream_url"].endswith("src=cam_hte_tapo_c245_tele")

    def test_privacy_mode_unavailable(self):
        tracker = _avail_tracker(FakeFetcher(_equipment_payload(privacy_mode=True)))
        info = tracker.availability()
        assert info["available"] is False
        assert "privacy" in info["reason"]

    def test_streaming_disabled_unavailable(self):
        tracker = _avail_tracker(FakeFetcher(_equipment_payload(streaming_enabled=False)))
        assert tracker.availability()["available"] is False

    def test_go2rtc_down_unavailable(self):
        tracker = _avail_tracker(FakeFetcher(_equipment_payload(go2rtc_reachable=False)))
        assert tracker.availability()["available"] is False

    def test_camera_not_found_unavailable(self):
        tracker = _avail_tracker(FakeFetcher([{"equipment_id": "other", "details": {}}]))
        info = tracker.availability()
        assert info["available"] is False
        assert "not found" in info["reason"]

    def test_fetch_error_unavailable(self):
        payload = _equipment_payload()
        payload[0]["fetch_error"] = "timeout"
        tracker = _avail_tracker(FakeFetcher(payload))
        assert tracker.availability()["available"] is False

    def test_probe_failure_is_swallowed(self):
        tracker = _avail_tracker(FakeFetcher(fail=True))
        info = tracker.availability()
        assert info["available"] is False
        assert "unreachable" in info["reason"]

    def test_unconfigured_short_circuits_without_fetch(self):
        fetcher = FakeFetcher(_equipment_payload())
        tracker = _avail_tracker(fetcher, enabled=False)
        info = tracker.availability()
        assert info["available"] is False
        assert fetcher.calls == 0  # never hit the dashboard when unconfigured

    def test_result_is_cached(self):
        fetcher = FakeFetcher(_equipment_payload())
        tracker = _avail_tracker(fetcher)
        tracker.availability()
        tracker.availability()
        assert fetcher.calls == 1  # second call served from cache

    def test_force_bypasses_cache(self):
        fetcher = FakeFetcher(_equipment_payload())
        tracker = _avail_tracker(fetcher)
        tracker.availability()
        tracker.availability(force=True)
        assert fetcher.calls == 2

    def test_dict_shaped_payload_supported(self):
        fetcher = FakeFetcher({"equipment": _equipment_payload()})
        tracker = _avail_tracker(fetcher)
        assert tracker.availability()["available"] is True


def _dashboard_payload(**detail_overrides):
    """The dashboard's /api/equipment shape (EquipmentSnapshot): a top-level
    ``id`` + the raw STATUS_SPEC envelope nested under ``status``. Distinct
    from a raw device /status, which carries those fields at the top level.
    """
    raw = _equipment_payload(**detail_overrides)[0]
    return {
        "fetched_at": "2026-07-24T00:00:00Z",
        "equipment": [{
            "id": raw["equipment_id"],
            "fetch_error": raw["fetch_error"],
            "status": {
                "equipment_status": raw["equipment_status"],
                "details": raw["details"],
            },
        }],
    }


class TestDashboardSnapshotShape:
    """The tracker must read the dashboard's decorated snapshot (id + nested
    `status` envelope), not just a raw device /status. Regression for the
    field-shape mismatch found against the live dashboard."""

    def test_available_from_dashboard_snapshot(self):
        tracker = _avail_tracker(FakeFetcher(_dashboard_payload()))
        info = tracker.availability()
        assert info["available"] is True
        assert info["stream_url"] == "ws://dash:8001/streams/api/ws?src=cam_hte_tapo_c245_wide"

    def test_id_keyed_camera_is_found(self):
        # The camera id lives under `id`, not `equipment_id`, on the snapshot.
        tracker = _avail_tracker(FakeFetcher(_dashboard_payload()))
        assert "not found" not in (tracker.availability()["reason"] or "")

    def test_privacy_mode_from_dashboard_snapshot(self):
        tracker = _avail_tracker(FakeFetcher(_dashboard_payload(privacy_mode=True)))
        info = tracker.availability()
        assert info["available"] is False
        assert "privacy" in info["reason"]

    def test_stream_connected_false_does_not_block(self):
        # go2rtc connects to the camera on demand, so stream_connected is False
        # while idle. It must NOT gate availability (verified live: segments
        # flow to an MSE consumer despite stream_connected False).
        payload = _dashboard_payload()
        for lens in payload["equipment"][0]["status"]["details"]["lenses"]:
            lens["stream_connected"] = False
        tracker = _avail_tracker(FakeFetcher(payload))
        info = tracker.availability()
        assert info["available"] is True
        assert info["stream_url"].endswith("src=cam_hte_tapo_c245_wide")


class TestStreamUrl:
    def test_relative_url_anchored_on_dashboard(self):
        tracker = _tracker()
        assert tracker._absolute_ws("/streams/x") == "ws://dash:8001/streams/x"

    def test_https_dashboard_yields_wss(self):
        tracker = _tracker(dashboard_base_url="https://dash.example")
        assert tracker._absolute_ws("/s") == "wss://dash.example/s"

    def test_stream_base_url_override(self):
        tracker = _tracker(stream_base_url="https://stream.example")
        assert tracker._absolute_ws("/s") == "wss://stream.example/s"

    def test_absolute_ws_url_passthrough(self):
        tracker = _tracker()
        assert tracker._absolute_ws("wss://x/y") == "wss://x/y"

    def test_absolute_http_url_converted_to_ws(self):
        tracker = _tracker()
        assert tracker._absolute_ws("https://x/y") == "wss://x/y"
