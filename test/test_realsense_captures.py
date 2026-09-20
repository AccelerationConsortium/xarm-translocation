"""Tests for the capture store (core/realsense_captures.py).

The store is deliberately camera-free -- it handles bytes and directories --
so everything here runs against ``tmp_path`` with no hardware, no
``pyrealsense2`` and no numpy. What is worth pinning down is the behaviour a
later vision phase depends on and that a refactor could quietly break:

* the on-disk layout and id format, because the nightly replication to the
  lab data server and any offline consumer walk it by hand;
* atomicity, because a half-written capture that *looks* complete would be
  read as evidence;
* retention, in both directions and in the right order, including the
  protected exemption that node references rely on.
"""

import json
import os
import sys
from datetime import datetime, timedelta, timezone

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

from src.core import realsense_captures as rcap  # noqa: E402
from src.core.realsense_captures import (  # noqa: E402
    CaptureNotFound,
    CaptureStore,
    CaptureStoreError,
)


COLOR = b"\xff\xd8\xff\xe0fake-jpeg-bytes"
DEPTH = b"\x89PNG\r\n\x1a\nfake-png-bytes"


def _store(tmp_path, **overrides):
    config = {"enabled": True, "root": str(tmp_path / "captures"),
              "keep_days": 30, "keep_max_gb": 20}
    config.update(overrides)
    return CaptureStore(config)


def _write(store, *, at=None, label=None, protected=False, node_id=None, color=COLOR, depth=DEPTH):
    meta = {"label": label, "protected": protected, "arm": {"node_id": node_id}}
    kwargs = {}
    if at is not None:
        kwargs["capture_id"] = rcap.new_capture_id(at)
        kwargs["now"] = at
        meta["captured_at"] = at.replace(microsecond=0).isoformat().replace("+00:00", "Z")
    return store.write(color_jpeg=color, depth_png=depth, meta=meta, **kwargs)


# ---------------------------------------------------------------------------
# Identifiers
# ---------------------------------------------------------------------------

class TestCaptureIds:
    def test_id_is_sortable_and_shards_by_day(self):
        moment = datetime(2026, 9, 20, 1, 42, 33, tzinfo=timezone.utc)
        cid = rcap.new_capture_id(moment)
        assert cid.startswith("20260920T014233Z-")
        assert rcap.day_for_id(cid) == "2026-09-20"
        assert rcap.is_capture_id(cid)

    def test_ids_are_unique_within_one_second(self):
        moment = datetime(2026, 9, 20, 1, 42, 33, tzinfo=timezone.utc)
        assert rcap.new_capture_id(moment) != rcap.new_capture_id(moment)

    def test_lexical_order_is_chronological(self):
        early = rcap.new_capture_id(datetime(2026, 9, 20, 1, 0, 0, tzinfo=timezone.utc))
        late = rcap.new_capture_id(datetime(2026, 9, 20, 2, 0, 0, tzinfo=timezone.utc))
        assert early < late

    @pytest.mark.parametrize("bad", ["", "../etc", "20260920T014233Z", "nope",
                                     "20260920T014233Z-XYZ", "20260920T014233Z-9f3c1a2"])
    def test_malformed_ids_are_rejected(self, bad):
        assert not rcap.is_capture_id(bad)


# ---------------------------------------------------------------------------
# Writing
# ---------------------------------------------------------------------------

class TestWrite:
    def test_layout_is_root_day_id(self, tmp_path):
        store = _store(tmp_path)
        record = _write(store)
        cid = record["capture_id"]
        directory = os.path.join(store.root, rcap.day_for_id(cid), cid)
        assert os.path.isdir(directory)
        assert sorted(os.listdir(directory)) == ["color.jpg", "depth.png", "meta.json"]

    def test_meta_carries_checksums_and_sizes(self, tmp_path):
        import hashlib

        store = _store(tmp_path)
        record = _write(store)
        assert record["files"]["color.jpg"]["bytes"] == len(COLOR)
        assert record["files"]["color.jpg"]["sha256"] == hashlib.sha256(COLOR).hexdigest()
        assert record["files"]["depth.png"]["sha256"] == hashlib.sha256(DEPTH).hexdigest()

    def test_meta_on_disk_matches_returned_record(self, tmp_path):
        store = _store(tmp_path)
        record = _write(store, label="arrival")
        on_disk = json.loads(
            open(store.file_path(record["capture_id"], "meta.json"), encoding="utf-8").read()
        )
        assert on_disk == record

    def test_depth_only_capture_is_allowed(self, tmp_path):
        store = _store(tmp_path)
        record = _write(store, color=None)
        assert "depth.png" in record["files"]
        assert "color.jpg" not in record["files"]

    def test_capture_with_no_frames_is_refused(self, tmp_path):
        store = _store(tmp_path)
        with pytest.raises(CaptureStoreError):
            store.write(color_jpeg=None, depth_png=None, meta={})

    def test_disabled_store_refuses(self, tmp_path):
        store = _store(tmp_path, enabled=False)
        with pytest.raises(CaptureStoreError):
            _write(store)

    def test_partial_directory_is_never_listed_and_is_swept(self, tmp_path):
        """A crashed write leaves .partial debris, not a readable capture."""
        store = _store(tmp_path)
        day = os.path.join(store.root, "2026-09-20")
        os.makedirs(os.path.join(day, "20260920T010000Z-deadbeef.partial"), exist_ok=True)
        assert store.list_captures() == []
        store.prune()
        assert not os.path.exists(os.path.join(day, "20260920T010000Z-deadbeef.partial"))

    def test_capture_missing_meta_is_not_listed(self, tmp_path):
        store = _store(tmp_path)
        record = _write(store)
        os.remove(store.file_path(record["capture_id"], "meta.json"))
        assert store.list_captures() == []


# ---------------------------------------------------------------------------
# Reading
# ---------------------------------------------------------------------------

class TestRead:
    def test_list_is_newest_first(self, tmp_path):
        store = _store(tmp_path)
        base = datetime(2026, 9, 18, 12, 0, 0, tzinfo=timezone.utc)
        for offset in range(3):
            _write(store, at=base + timedelta(hours=offset), label=f"c{offset}")
        labels = [item["label"] for item in store.list_captures()]
        assert labels == ["c2", "c1", "c0"]

    def test_limit_and_filters(self, tmp_path):
        store = _store(tmp_path)
        base = datetime(2026, 9, 18, 12, 0, 0, tzinfo=timezone.utc)
        _write(store, at=base, node_id="deck_1", label="a")
        _write(store, at=base + timedelta(hours=1), node_id="hood_2", label="b")
        assert len(store.list_captures(limit=1)) == 1
        assert [c["label"] for c in store.list_captures(node_id="deck_1")] == ["a"]
        assert [c["label"] for c in store.list_captures(label="b")] == ["b"]

    def test_since_filter_is_inclusive_of_later_captures(self, tmp_path):
        store = _store(tmp_path)
        old = datetime(2026, 9, 18, 12, 0, 0, tzinfo=timezone.utc)
        new = datetime(2026, 9, 19, 12, 0, 0, tzinfo=timezone.utc)
        _write(store, at=old, label="old")
        _write(store, at=new, label="new")
        got = store.list_captures(since="2026-09-19T00:00:00Z")
        assert [c["label"] for c in got] == ["new"]

    def test_get_unknown_raises(self, tmp_path):
        store = _store(tmp_path)
        with pytest.raises(CaptureNotFound):
            store.get(rcap.new_capture_id())

    def test_file_path_whitelists_names(self, tmp_path):
        """Traversal is impossible because the filename is not interpolated."""
        store = _store(tmp_path)
        record = _write(store)
        for bad in ("../../secret", "meta.json.bak", "..", "color.JPG"):
            with pytest.raises(CaptureNotFound):
                store.file_path(record["capture_id"], bad)

    def test_file_path_rejects_malformed_id(self, tmp_path):
        store = _store(tmp_path)
        with pytest.raises(CaptureNotFound):
            store.file_path("../../etc/passwd", "color.jpg")

    def test_delete_removes_and_reports(self, tmp_path):
        store = _store(tmp_path)
        record = _write(store)
        assert store.delete(record["capture_id"]) is True
        assert store.delete(record["capture_id"]) is False
        assert store.list_captures() == []


# ---------------------------------------------------------------------------
# Retention
# ---------------------------------------------------------------------------

class TestRetention:
    def test_age_bound_removes_only_stale_captures(self, tmp_path):
        now = datetime(2026, 9, 20, 12, 0, 0, tzinfo=timezone.utc)
        # Both bounds off while seeding, so the prune that runs on each write
        # (at that capture's own timestamp) cannot pre-empt what is measured.
        store = _store(tmp_path, keep_days=0, keep_max_gb=0)
        _write(store, at=now - timedelta(days=40), label="stale")
        _write(store, at=now - timedelta(days=2), label="fresh")
        store.keep_days = 30
        result = store.prune(now=now)
        assert result.removed_age == 1
        assert [c["label"] for c in store.list_captures()] == ["fresh"]

    def test_write_time_prune_uses_the_capture_moment(self, tmp_path):
        """Retention on write is evaluated at that capture's own timestamp.

        Back-dating a capture therefore ages out anything already older than
        keep_days *relative to it*, which is what makes a replayed or
        imported batch self-trimming instead of accumulating.
        """
        now = datetime(2026, 9, 20, 12, 0, 0, tzinfo=timezone.utc)
        store = _store(tmp_path, keep_days=30, keep_max_gb=0)
        _write(store, at=now - timedelta(days=40), label="stale")
        _write(store, at=now - timedelta(days=2), label="fresh")
        assert [c["label"] for c in store.list_captures()] == ["fresh"]

    def test_size_bound_removes_oldest_first(self, tmp_path):
        now = datetime(2026, 9, 20, 12, 0, 0, tzinfo=timezone.utc)
        # Two captures of ~30 bytes each; a ceiling below that forces eviction.
        store = _store(tmp_path, keep_days=0, keep_max_gb=0)
        _write(store, at=now - timedelta(hours=2), label="older")
        _write(store, at=now - timedelta(hours=1), label="newer")
        store.keep_max_gb = 40 / (1024 ** 3)  # bytes, expressed in GB
        result = store.prune(now=now)
        assert result.removed_size >= 1
        remaining = [c["label"] for c in store.list_captures()]
        assert "older" not in remaining

    def test_protected_captures_survive_both_bounds(self, tmp_path):
        now = datetime(2026, 9, 20, 12, 0, 0, tzinfo=timezone.utc)
        store = _store(tmp_path, keep_days=1, keep_max_gb=0)
        _write(store, at=now - timedelta(days=99), label="reference", protected=True)
        _write(store, at=now - timedelta(days=99), label="ordinary")
        store.keep_max_gb = 1 / (1024 ** 3)
        store.prune(now=now)
        labels = [c["label"] for c in store.list_captures()]
        assert labels == ["reference"]

    def test_zero_bounds_disable_retention(self, tmp_path):
        now = datetime(2026, 9, 20, 12, 0, 0, tzinfo=timezone.utc)
        store = _store(tmp_path, keep_days=0, keep_max_gb=0)
        _write(store, at=now - timedelta(days=9999), label="ancient")
        store.prune(now=now)
        assert [c["label"] for c in store.list_captures()] == ["ancient"]

    def test_write_prunes_without_failing_the_write(self, tmp_path):
        """Retention runs on write; a prune failure must not lose the capture."""
        now = datetime(2026, 9, 20, 12, 0, 0, tzinfo=timezone.utc)
        store = _store(tmp_path, keep_days=30)
        _write(store, at=now - timedelta(days=90), label="stale")
        record = _write(store, at=now, label="fresh")
        labels = [c["label"] for c in store.list_captures()]
        assert "stale" not in labels
        assert record["capture_id"] in [c["capture_id"] for c in store.list_captures()]

    def test_empty_day_directories_are_removed(self, tmp_path):
        now = datetime(2026, 9, 20, 12, 0, 0, tzinfo=timezone.utc)
        store = _store(tmp_path, keep_days=1)
        _write(store, at=now - timedelta(days=30))
        store.prune(now=now)
        assert not os.path.exists(os.path.join(store.root, "2026-08-21"))


# ---------------------------------------------------------------------------
# Configuration and status surface
# ---------------------------------------------------------------------------

class TestConfig:
    def test_defaults_when_block_is_absent(self):
        store = CaptureStore(None)
        assert store.keep_days == rcap.DEFAULT_KEEP_DAYS
        assert store.keep_max_gb == rcap.DEFAULT_KEEP_MAX_GB
        assert store.root.endswith(os.path.normpath(rcap.DEFAULT_ROOT).lstrip(os.sep))

    def test_malformed_values_fall_back(self, tmp_path):
        store = _store(tmp_path, keep_days="not-a-number", keep_max_gb=None)
        assert store.keep_days == rcap.DEFAULT_KEEP_DAYS
        assert store.keep_max_gb == rcap.DEFAULT_KEEP_MAX_GB

    def test_load_captures_config_reads_the_block(self, tmp_path):
        yaml = pytest.importorskip("yaml")
        path = tmp_path / "realsense.yaml"
        path.write_text(yaml.safe_dump({"enabled": True,
                                        "captures": {"keep_days": 7, "keep_max_gb": 1}}))
        assert rcap.load_captures_config(str(path)) == {"keep_days": 7, "keep_max_gb": 1}

    def test_load_captures_config_tolerates_missing_file(self, tmp_path):
        assert rcap.load_captures_config(str(tmp_path / "nope.yaml")) == {}

    def test_summary_tracks_the_newest_capture(self, tmp_path):
        store = _store(tmp_path)
        assert store.summary()["count"] == 0
        record = _write(store)
        summary = store.summary()
        assert summary["count"] == 1
        assert summary["last_id"] == record["capture_id"]
        assert summary["bytes"] > 0

    def test_describe_reports_the_policy(self, tmp_path):
        store = _store(tmp_path, keep_days=30, keep_max_gb=20)
        described = store.describe()
        assert described["keep_days"] == 30
        assert described["keep_max_gb"] == 20
        assert described["root"] == store.root

    def test_shared_store_roundtrip(self, tmp_path):
        store = rcap.configure_shared({"root": str(tmp_path), "keep_days": 5})
        assert rcap.shared_store() is store
        rcap.set_shared(None)
        assert rcap.shared_store() is None
