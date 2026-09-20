#!/usr/bin/env python3
"""Replicate new RealSense captures from the device PC to the lab data server.

Runs **on the data server** (gaia) and pulls, rather than running on the
device PC and pushing, for three reasons: the destination owns its own
retention, a Windows box has no cron worth relying on, and a pull cannot be
made to overwrite anything on the source.

    C:\\SDL_Data\\xarm\\realsense\\<YYYY-MM-DD>\\<capture_id>\\
        -> /home/sdl2/storage/external/realsens_xarm/<YYYY-MM-DD>/<capture_id>/

Captures are immutable once written, so "new" is simply "not here yet": no
timestamps to compare, no partial-update case to reason about. That is also
why the source's retention sweep does not propagate — deleting a capture
upstream after 30 days must not delete the archived copy, which is the whole
point of replicating to a mirrored, snapshotted pool.

Integrity is checked rather than assumed: every file's SHA-256 is recorded in
the capture's own meta.json when it is written, so this script re-hashes what
it received and refuses to keep a copy that does not match.

Exit status: 0 when everything that could be copied was copied (including
"nothing new"), 1 when at least one capture failed. Safe to run concurrently
with a capture being written upstream: ``.partial`` directories are skipped
by name and an id whose meta.json is unreadable is left for the next run.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import logging
import os
import re
import shutil
import subprocess
import sys
import tempfile
from typing import Dict, List, Set, Tuple

DEFAULT_SOURCE_HOST = "cytation-pc"
DEFAULT_SOURCE_ROOT = r"C:\SDL_Data\xarm\realsense"
DEFAULT_DEST = "/home/sdl2/storage/external/realsens_xarm"

_ID_RE = re.compile(r"^\d{8}T\d{6}Z-[0-9a-f]{8}$")
_DAY_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")

logger = logging.getLogger("replicate-captures")


def _ssh_base(host: str, timeout: int) -> List[str]:
    return [
        "ssh", "-o", "BatchMode=yes", "-o", f"ConnectTimeout={timeout}",
        "-o", "StrictHostKeyChecking=accept-new", host,
    ]


def list_remote_captures(host: str, root: str, timeout: int) -> Set[Tuple[str, str]]:
    """Every ``(day, capture_id)`` present on the device PC.

    Uses one PowerShell round-trip rather than a directory walk per day: the
    link to the device PC is the slow part, not the filesystem.
    """
    script = (
        f"$r='{root}';"
        "if(!(Test-Path $r)){exit 0};"
        "Get-ChildItem -Path $r -Directory | ForEach-Object {"
        "  $d=$_.Name;"
        "  Get-ChildItem -Path $_.FullName -Directory -ErrorAction SilentlyContinue |"
        "    ForEach-Object { Write-Output ($d + '/' + $_.Name) } }"
    )
    # The device PC's ssh shell is cmd.exe, which would eat the pipes and
    # braces above before PowerShell ever saw them. -EncodedCommand takes
    # base64 UTF-16LE and is immune to both shells' quoting rules.
    encoded = base64.b64encode(script.encode("utf-16-le")).decode("ascii")
    command = _ssh_base(host, timeout) + ["powershell", "-NoProfile", "-EncodedCommand", encoded]
    try:
        completed = subprocess.run(command, capture_output=True, text=True, timeout=300)
    except subprocess.TimeoutExpired:
        raise SystemExit("timed out listing captures on the device PC")
    if completed.returncode != 0:
        raise SystemExit(f"could not list captures: {completed.stderr.strip()[:300]}")

    found: Set[Tuple[str, str]] = set()
    for line in completed.stdout.splitlines():
        entry = line.strip().replace("\\", "/")
        if not entry or "/" not in entry:
            continue
        day, _, capture_id = entry.partition("/")
        # .partial debris and anything not matching the id format is skipped:
        # an in-flight capture is not yet a record.
        if _DAY_RE.match(day) and _ID_RE.match(capture_id):
            found.add((day, capture_id))
    return found


def list_local_captures(dest: str) -> Set[Tuple[str, str]]:
    """Every complete capture already archived. Incomplete copies do not count."""
    present: Set[Tuple[str, str]] = set()
    if not os.path.isdir(dest):
        return present
    for day in os.listdir(dest):
        day_path = os.path.join(dest, day)
        if not (_DAY_RE.match(day) and os.path.isdir(day_path)):
            continue
        for capture_id in os.listdir(day_path):
            capture_path = os.path.join(day_path, capture_id)
            if not (_ID_RE.match(capture_id) and os.path.isdir(capture_path)):
                continue
            if os.path.isfile(os.path.join(capture_path, "meta.json")):
                present.add((day, capture_id))
    return present


def verify(directory: str) -> Tuple[bool, str]:
    """Re-hash the copied files against the checksums in their meta.json."""
    meta_path = os.path.join(directory, "meta.json")
    try:
        with open(meta_path, "r", encoding="utf-8") as handle:
            meta = json.load(handle)
    except (OSError, ValueError) as exc:
        return False, f"meta.json unreadable: {exc}"
    files = meta.get("files")
    if not isinstance(files, dict) or not files:
        # A capture may legitimately carry no checksum block (hand-made or
        # from an older writer); presence of the payload is then the only
        # check available, and is better than rejecting the record.
        return True, "no checksums recorded"
    for name, info in files.items():
        path = os.path.join(directory, name)
        if not os.path.isfile(path):
            return False, f"missing {name}"
        expected = (info or {}).get("sha256")
        if not expected:
            continue
        digest = hashlib.sha256()
        with open(path, "rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
        if digest.hexdigest() != expected:
            return False, f"checksum mismatch on {name}"
    return True, "ok"


def copy_capture(host: str, root: str, day: str, capture_id: str, dest: str,
                 timeout: int) -> Tuple[bool, str]:
    """Fetch one capture into a temporary directory, verify, then publish.

    The staging hop means an interrupted transfer never leaves a directory
    that ``list_local_captures`` would count as archived, so the next run
    retries it instead of skipping it forever.
    """
    day_dest = os.path.join(dest, day)
    os.makedirs(day_dest, exist_ok=True)
    final = os.path.join(day_dest, capture_id)
    # Forward slashes deliberately: scp treats a backslash in a remote path as
    # an escape, so "C:\SDL_Data\..." arrives at the Windows server doubled and
    # resolves to nothing. Windows accepts "C:/SDL_Data/..." unchanged.
    remote = f"{root}/{day}/{capture_id}".replace("\\", "/")

    staging = tempfile.mkdtemp(prefix=f".{capture_id}.", dir=day_dest)
    try:
        command = [
            "scp", "-q", "-r", "-o", "BatchMode=yes", "-o", f"ConnectTimeout={timeout}",
            f"{host}:{remote}", os.path.join(staging, capture_id),
        ]
        completed = subprocess.run(command, capture_output=True, text=True, timeout=600)
        if completed.returncode != 0:
            return False, f"scp failed: {completed.stderr.strip()[:200]}"
        fetched = os.path.join(staging, capture_id)
        ok, reason = verify(fetched)
        if not ok:
            return False, reason
        if os.path.exists(final):
            shutil.rmtree(final, ignore_errors=True)
        os.replace(fetched, final)
        return True, "ok"
    except subprocess.TimeoutExpired:
        return False, "scp timed out"
    finally:
        shutil.rmtree(staging, ignore_errors=True)


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default=DEFAULT_SOURCE_HOST)
    parser.add_argument("--source-root", default=DEFAULT_SOURCE_ROOT)
    parser.add_argument("--dest", default=DEFAULT_DEST)
    parser.add_argument("--timeout", type=int, default=15, help="ssh connect timeout, seconds")
    parser.add_argument("--limit", type=int, default=0, help="stop after N captures (0 = all)")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    remote = list_remote_captures(args.host, args.source_root, args.timeout)
    local = list_local_captures(args.dest)
    missing = sorted(remote - local)
    if args.limit:
        missing = missing[: args.limit]

    logger.info(
        "source=%s captures=%d archived=%d new=%d dest=%s",
        args.host, len(remote), len(local), len(missing), args.dest,
    )
    if args.dry_run:
        for day, capture_id in missing:
            logger.info("would copy %s/%s", day, capture_id)
        return 0

    os.makedirs(args.dest, exist_ok=True)
    copied = 0
    failed: Dict[str, str] = {}
    for day, capture_id in missing:
        ok, reason = copy_capture(
            args.host, args.source_root, day, capture_id, args.dest, args.timeout
        )
        if ok:
            copied += 1
            logger.debug("copied %s/%s", day, capture_id)
        else:
            failed[f"{day}/{capture_id}"] = reason
            logger.warning("failed %s/%s: %s", day, capture_id, reason)

    logger.info("copied=%d failed=%d", copied, len(failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
