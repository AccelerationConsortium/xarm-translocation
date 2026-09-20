"""Durable capture records for the RealSense depth camera.

``/realsense/snapshot.jpg`` and friends are *transient*: they answer "what
does the camera see right now" and nothing survives the response. A capture
is the opposite — one aligned frameset written to disk as three files, with
enough metadata beside it that the record still means something months later
without the process that took it:

    <root>/<YYYY-MM-DD>/<capture_id>/
        color.jpg     colour frame, JPEG
        depth.png     raw 16-bit depth, lossless (units of depth_scale metres)
        meta.json     everything needed to interpret the two above

This is the unit every later vision phase consumes (node references,
arrival-verification evidence, training data for plate detection), which is
why the metadata carries the *arm state at capture* — a depth map without the
pose it was taken from is not a measurement, it is a picture.

Design constraints, following the rest of this subsystem:

1. **Outside the repo tree.** Captures are data, not source. The root is
   configured in ``realsense.yaml`` (``captures.root``) and defaults to
   ``C:\\SDL_Data\\xarm\\realsense`` on the device PC.
2. **Bounded, pruned on write.** Retention is by age *and* by total size
   (``keep_days`` / ``keep_max_gb``); both are enforced after every write so
   the store cannot grow without an operator noticing. Captures flagged
   ``protected`` are never pruned — Phase 4 node references live here and
   must outlive ordinary retention.
3. **No hardware, no camera object.** This module only handles bytes and
   the filesystem, so the whole store is unit-testable without
   ``pyrealsense2`` and without a camera.
4. **Never raises into the control path.** Pruning failures are logged and
   swallowed: losing a retention sweep must not fail the capture that
   triggered it.
"""

from __future__ import annotations

import hashlib
import json
import logging
import os
import re
import secrets
import shutil
import time
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)

# One capture is ~0.5 MB at 640x480 (JPEG + 16-bit PNG), so these defaults
# hold roughly 40k captures or 30 days, whichever binds first.
DEFAULT_ROOT = r"C:\SDL_Data\xarm\realsense"
DEFAULT_KEEP_DAYS = 30
DEFAULT_KEEP_MAX_GB = 20.0

COLOR_NAME = "color.jpg"
DEPTH_NAME = "depth.png"
META_NAME = "meta.json"
_FILE_NAMES = (COLOR_NAME, DEPTH_NAME, META_NAME)

# <YYYYMMDD>T<HHMMSS>Z-<8 hex>: lexically sortable == chronologically sorted,
# and the random tail makes two captures in the same second distinct.
_ID_RE = re.compile(r"^\d{8}T\d{6}Z-[0-9a-f]{8}$")
_DAY_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")

_BYTES_PER_GB = 1024 ** 3


class CaptureStoreError(RuntimeError):
    """A capture could not be written. HTTP 500 material."""


class CaptureNotFound(LookupError):
    """No capture with that id. HTTP 404 material."""


@dataclass
class PruneResult:
    """What one retention sweep removed. All counts are captures, not files."""

    removed_age: int = 0
    removed_size: int = 0
    freed_bytes: int = 0
    kept: int = 0
    protected: int = 0

    @property
    def removed(self) -> int:
        return self.removed_age + self.removed_size

    def as_dict(self) -> Dict[str, Any]:
        return {
            "removed": self.removed,
            "removed_age": self.removed_age,
            "removed_size": self.removed_size,
            "freed_bytes": self.freed_bytes,
            "kept": self.kept,
            "protected": self.protected,
        }


def new_capture_id(now: Optional[datetime] = None) -> str:
    """A fresh sortable capture id."""
    moment = now or datetime.now(timezone.utc)
    return f"{moment.strftime('%Y%m%dT%H%M%SZ')}-{secrets.token_hex(4)}"


def day_for_id(capture_id: str) -> str:
    """The ``YYYY-MM-DD`` shard a capture id belongs in."""
    return f"{capture_id[0:4]}-{capture_id[4:6]}-{capture_id[6:8]}"


def is_capture_id(value: str) -> bool:
    """True for a well-formed id. Guards the path joins below."""
    return bool(value) and bool(_ID_RE.match(value))


class CaptureStore:
    """A retention-bounded directory of capture records.

    ``config`` is the ``captures:`` block of ``realsense.yaml``; a missing or
    malformed block yields a store on the defaults above. Construction never
    touches the filesystem, so importing this module on a machine without the
    data volume is safe — the root is created on first write.
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        config = config or {}
        self.enabled = bool(config.get("enabled", True))
        root = str(config.get("root", "") or "").strip() or DEFAULT_ROOT
        self.root = os.path.abspath(os.path.expanduser(root))
        self.keep_days = max(0, int(_as_float(config.get("keep_days"), DEFAULT_KEEP_DAYS)))
        self.keep_max_gb = max(0.0, _as_float(config.get("keep_max_gb"), DEFAULT_KEEP_MAX_GB))

    # ------------------------------------------------------------------
    # Introspection
    # ------------------------------------------------------------------

    @property
    def keep_max_bytes(self) -> int:
        return int(self.keep_max_gb * _BYTES_PER_GB)

    def describe(self) -> Dict[str, Any]:
        """Retention policy + current occupancy, for ``/realsense/status``."""
        captures = self._scan()
        total = sum(entry["bytes"] for entry in captures)
        newest = captures[0] if captures else None
        return {
            "enabled": self.enabled,
            "root": self.root,
            "keep_days": self.keep_days,
            "keep_max_gb": self.keep_max_gb,
            "count": len(captures),
            "bytes": total,
            "last_id": newest["id"] if newest else None,
            "last_at": newest["captured_at"] if newest else None,
        }

    def summary(self) -> Dict[str, Any]:
        """The compact ``details.realsense.captures`` block."""
        described = self.describe()
        return {
            "count": described["count"],
            "bytes": described["bytes"],
            "last_id": described["last_id"],
            "last_at": described["last_at"],
        }

    # ------------------------------------------------------------------
    # Write
    # ------------------------------------------------------------------

    def write(
        self,
        *,
        color_jpeg: Optional[bytes],
        depth_png: Optional[bytes],
        meta: Dict[str, Any],
        capture_id: Optional[str] = None,
        now: Optional[datetime] = None,
    ) -> Dict[str, Any]:
        """Persist one capture and return its ``meta.json`` contents.

        The directory is built under a ``.partial`` name and renamed into
        place, so a reader never sees a half-written capture and a crash
        leaves debris that the next prune collects rather than a record that
        looks complete but is not.
        """
        if not self.enabled:
            raise CaptureStoreError("capture store is disabled in realsense.yaml")
        if color_jpeg is None and depth_png is None:
            raise CaptureStoreError("a capture needs at least one of colour or depth")

        moment = now or datetime.now(timezone.utc)
        cid = capture_id or new_capture_id(moment)
        if not is_capture_id(cid):
            raise CaptureStoreError(f"malformed capture id: {cid!r}")

        day_dir = os.path.join(self.root, day_for_id(cid))
        final_dir = os.path.join(day_dir, cid)
        staging = final_dir + ".partial"

        record = dict(meta)
        record["capture_id"] = cid
        record.setdefault("captured_at", moment.replace(microsecond=0).isoformat().replace("+00:00", "Z"))

        files: Dict[str, Any] = {}
        try:
            if os.path.exists(staging):
                shutil.rmtree(staging, ignore_errors=True)
            os.makedirs(staging, exist_ok=True)
            if color_jpeg is not None:
                files[COLOR_NAME] = _write_blob(os.path.join(staging, COLOR_NAME), color_jpeg)
            if depth_png is not None:
                files[DEPTH_NAME] = _write_blob(os.path.join(staging, DEPTH_NAME), depth_png)
            record["files"] = files
            with open(os.path.join(staging, META_NAME), "w", encoding="utf-8") as handle:
                json.dump(record, handle, indent=2, sort_keys=True, default=str)
            os.makedirs(day_dir, exist_ok=True)
            if os.path.exists(final_dir):
                shutil.rmtree(final_dir, ignore_errors=True)
            os.replace(staging, final_dir)
        except OSError as exc:
            shutil.rmtree(staging, ignore_errors=True)
            raise CaptureStoreError(f"could not write capture {cid}: {exc}") from exc

        # Retention is enforced here rather than on a timer so the bound is
        # true at every observable moment. A failure must not fail the write.
        try:
            self.prune(now=moment)
        except Exception:  # noqa: BLE001 - retention is best-effort
            logger.exception("capture prune failed after writing %s", cid)

        return record

    # ------------------------------------------------------------------
    # Read
    # ------------------------------------------------------------------

    def list_captures(
        self,
        *,
        limit: int = 50,
        node_id: Optional[str] = None,
        since: Optional[str] = None,
        label: Optional[str] = None,
    ) -> List[Dict[str, Any]]:
        """Newest first, optionally filtered. Returns metadata, not bytes."""
        entries = self._scan()
        out: List[Dict[str, Any]] = []
        for entry in entries:
            meta = entry["meta"]
            if node_id is not None and (meta.get("arm") or {}).get("node_id") != node_id:
                continue
            if label is not None and meta.get("label") != label:
                continue
            if since is not None and str(entry["captured_at"] or "") < since:
                continue
            out.append(meta)
            if limit and len(out) >= limit:
                break
        return out

    def get(self, capture_id: str) -> Dict[str, Any]:
        """One capture's ``meta.json``. Raises :class:`CaptureNotFound`."""
        meta_path = self._meta_path(capture_id)
        if meta_path is None or not os.path.isfile(meta_path):
            raise CaptureNotFound(capture_id)
        return _read_meta(meta_path) or {"capture_id": capture_id}

    def file_path(self, capture_id: str, name: str) -> str:
        """Absolute path of one artefact. Raises :class:`CaptureNotFound`.

        ``name`` is checked against a whitelist rather than sanitised: the
        only files a caller may ever fetch are the ones this module writes,
        so there is no traversal surface to get wrong.
        """
        if name not in _FILE_NAMES:
            raise CaptureNotFound(f"{capture_id}/{name}")
        directory = self._capture_dir(capture_id)
        if directory is None:
            raise CaptureNotFound(capture_id)
        path = os.path.join(directory, name)
        if not os.path.isfile(path):
            raise CaptureNotFound(f"{capture_id}/{name}")
        return path

    def delete(self, capture_id: str) -> bool:
        """Remove one capture. False when it was not there."""
        directory = self._capture_dir(capture_id)
        if directory is None or not os.path.isdir(directory):
            return False
        shutil.rmtree(directory, ignore_errors=True)
        self._prune_empty_days()
        return True

    # ------------------------------------------------------------------
    # Retention
    # ------------------------------------------------------------------

    def prune(self, *, now: Optional[datetime] = None) -> PruneResult:
        """Enforce ``keep_days`` then ``keep_max_gb``. Oldest goes first.

        Age is evaluated before size so a store that is over both bounds
        sheds stale records rather than recent ones, and ``protected``
        captures are exempt from both — a node reference that expired
        silently would fail an arrival check months later with no clue why.
        """
        moment = now or datetime.now(timezone.utc)
        result = PruneResult()
        entries = self._scan()

        # Debris from an interrupted write: neither a capture nor worth keeping.
        self._sweep_partials()

        survivors: List[Dict[str, Any]] = []
        if self.keep_days > 0:
            cutoff = moment - timedelta(days=self.keep_days)
            for entry in entries:
                if entry["protected"]:
                    result.protected += 1
                    survivors.append(entry)
                    continue
                stamp = _parse_iso(entry["captured_at"]) or _mtime_utc(entry["path"])
                if stamp is not None and stamp < cutoff:
                    if self._remove(entry):
                        result.removed_age += 1
                        result.freed_bytes += entry["bytes"]
                    continue
                survivors.append(entry)
        else:
            survivors = list(entries)
            result.protected = sum(1 for e in entries if e["protected"])

        if self.keep_max_bytes > 0:
            total = sum(entry["bytes"] for entry in survivors)
            # survivors is newest-first; walk from the oldest end.
            for entry in reversed(survivors):
                if total <= self.keep_max_bytes:
                    break
                if entry["protected"]:
                    continue
                if self._remove(entry):
                    result.removed_size += 1
                    result.freed_bytes += entry["bytes"]
                    total -= entry["bytes"]

        self._prune_empty_days()
        result.kept = len(self._scan())
        return result

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _capture_dir(self, capture_id: str) -> Optional[str]:
        if not is_capture_id(capture_id):
            return None
        return os.path.join(self.root, day_for_id(capture_id), capture_id)

    def _meta_path(self, capture_id: str) -> Optional[str]:
        directory = self._capture_dir(capture_id)
        return None if directory is None else os.path.join(directory, META_NAME)

    def _scan(self) -> List[Dict[str, Any]]:
        """Every complete capture, newest first.

        A full walk is deliberate: at ~0.5 MB a capture, the bound above tops
        out in the low tens of thousands of directories, which is a few
        milliseconds of ``scandir`` — cheaper than keeping an index honest
        across pruning, external deletion and the daily replication sweep.
        """
        entries: List[Dict[str, Any]] = []
        try:
            days = sorted(
                (d.name for d in os.scandir(self.root) if d.is_dir() and _DAY_RE.match(d.name)),
                reverse=True,
            )
        except (FileNotFoundError, NotADirectoryError, PermissionError):
            return entries

        for day in days:
            day_path = os.path.join(self.root, day)
            try:
                names = sorted((d.name for d in os.scandir(day_path) if d.is_dir()), reverse=True)
            except OSError:
                continue
            for name in names:
                if not is_capture_id(name):
                    continue
                path = os.path.join(day_path, name)
                meta_path = os.path.join(path, META_NAME)
                meta = _read_meta(meta_path)
                if meta is None:
                    continue  # incomplete; prune sweeps it
                entries.append(
                    {
                        "id": name,
                        "path": path,
                        "meta": meta,
                        "captured_at": meta.get("captured_at"),
                        "protected": bool(meta.get("protected")),
                        "bytes": _dir_bytes(path),
                    }
                )
        entries.sort(key=lambda e: e["id"], reverse=True)
        return entries

    def _remove(self, entry: Dict[str, Any]) -> bool:
        try:
            shutil.rmtree(entry["path"])
            return True
        except OSError as exc:
            logger.warning("capture prune: could not remove %s: %s", entry["path"], exc)
            return False

    def _sweep_partials(self) -> None:
        try:
            days = [d.path for d in os.scandir(self.root) if d.is_dir() and _DAY_RE.match(d.name)]
        except OSError:
            return
        for day_path in days:
            try:
                for item in os.scandir(day_path):
                    if item.is_dir() and item.name.endswith(".partial"):
                        shutil.rmtree(item.path, ignore_errors=True)
            except OSError:
                continue

    def _prune_empty_days(self) -> None:
        try:
            days = [d.path for d in os.scandir(self.root) if d.is_dir() and _DAY_RE.match(d.name)]
        except OSError:
            return
        for day_path in days:
            try:
                if not any(os.scandir(day_path)):
                    os.rmdir(day_path)
            except OSError:
                continue


# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------


def _write_blob(path: str, payload: bytes) -> Dict[str, Any]:
    with open(path, "wb") as handle:
        handle.write(payload)
    return {
        "bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
    }


def _read_meta(path: str) -> Optional[Dict[str, Any]]:
    try:
        with open(path, "r", encoding="utf-8") as handle:
            loaded = json.load(handle)
    except (OSError, ValueError):
        return None
    return loaded if isinstance(loaded, dict) else None


def _dir_bytes(path: str) -> int:
    total = 0
    try:
        for item in os.scandir(path):
            if item.is_file():
                try:
                    total += item.stat().st_size
                except OSError:
                    continue
    except OSError:
        return 0
    return total


def _parse_iso(value: Any) -> Optional[datetime]:
    if not isinstance(value, str) or not value:
        return None
    text = value[:-1] + "+00:00" if value.endswith("Z") else value
    try:
        parsed = datetime.fromisoformat(text)
    except ValueError:
        return None
    return parsed if parsed.tzinfo else parsed.replace(tzinfo=timezone.utc)


def _mtime_utc(path: str) -> Optional[datetime]:
    try:
        return datetime.fromtimestamp(os.path.getmtime(path), tz=timezone.utc)
    except OSError:
        return None


def _as_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


# ----------------------------------------------------------------------
# Process-wide instance (mirrors realsense_camera.shared_camera)
# ----------------------------------------------------------------------

_shared: Optional[CaptureStore] = None


def configure_shared(config: Optional[Dict[str, Any]]) -> CaptureStore:
    global _shared
    _shared = CaptureStore(config)
    return _shared


def shared_store() -> Optional[CaptureStore]:
    return _shared


def set_shared(store: Optional[CaptureStore]) -> None:
    global _shared
    _shared = store


def load_captures_config(path: str) -> Dict[str, Any]:
    """The ``captures:`` block of ``realsense.yaml``; {} when absent."""
    try:
        import yaml

        with open(path, "r", encoding="utf-8") as handle:
            loaded = yaml.safe_load(handle)
    except FileNotFoundError:
        return {}
    except Exception as exc:  # noqa: BLE001 - never break service boot
        logger.warning("failed to load %s: %s; capture defaults used", path, exc)
        return {}
    if not isinstance(loaded, dict):
        return {}
    block = loaded.get("captures")
    return block if isinstance(block, dict) else {}


__all__ = [
    "CaptureNotFound",
    "CaptureStore",
    "CaptureStoreError",
    "PruneResult",
    "configure_shared",
    "day_for_id",
    "is_capture_id",
    "load_captures_config",
    "new_capture_id",
    "set_shared",
    "shared_store",
]
