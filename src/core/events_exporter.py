"""Best-effort event push to the lab dashboard's history DB.

This is the device-side exporter described in ac-organic-lab's
``docs/OBSERVABILITY.md`` (device services push domain events via
``POST /api/ingest/events`` rather than writing the DB directly). It
gives the dashboard *fine-grained* ``state_transition`` / ``error``
rows straight from the xArm SDK callbacks, instead of relying on the
aggregator's 60 s poll — which misses any transition that begins and
ends inside one poll window (latching error/warn codes especially).

Design constraints, in order:

1. **Never block or break the control path.** ``emit()`` only enqueues
   onto a bounded in-memory queue; a daemon thread does the HTTP POST.
   Full queue or unreachable dashboard -> the event is dropped and the
   arm keeps working. The dashboard's own poll remains the coarse
   backstop, so a dropped row degrades timing fidelity, not truth.
2. **Stdlib only.** ``urllib`` instead of a new HTTP dependency.
3. **Disabled unless configured.** Without ``XARM_INGEST_URL`` in the
   environment the exporter is a no-op, so dev machines, unit tests,
   and Docker sims emit nothing.

Configuration (environment):

- ``XARM_INGEST_URL`` — full ingest endpoint URL, e.g.
  ``http://sdl2-server-gaia.tail6a1dd7.ts.net:8001/api/ingest/events``.
  Unset/empty disables the exporter.
- ``XARM_INGEST_DEVICE_ID`` — ``device_id`` stamped on every record;
  defaults to ``xarm_translocation`` (the equipment.yaml id).

Wire format matches the dashboard's ``IngestEventsRequest``: top-level
``{device_id, records: [...]}`` where each record carries ``timestamp``
(UTC ISO-8601), ``event`` (free string), optional ``from_state`` /
``to_state`` / ``message``, and an ``extra`` dict that the ingest
handler folds into the persisted JSON payload verbatim.
"""

from __future__ import annotations

import json
import os
import queue
import threading
import urllib.error
import urllib.request
from datetime import datetime, timezone
from typing import Any, Callable, Optional

DEFAULT_DEVICE_ID = "xarm_translocation"

# Log at most on the first failure of a streak, then every Nth, so an
# unreachable dashboard doesn't flood the service log.
_FAILURE_LOG_EVERY = 50

_SENTINEL = object()


def _utc_now_iso() -> str:
    """UTC ISO-8601 with a Z suffix, per STATUS_SPEC timestamp rules."""
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


class EventsExporter:
    """Queue-and-forward exporter for ``POST /api/ingest/events``.

    ``transport`` is injectable for tests: a callable taking the JSON
    payload dict and raising on delivery failure. The default transport
    POSTs with ``urllib`` and treats any non-2xx / socket error as a
    failure (the batch is dropped, never retried — history fidelity is
    best-effort by design).
    """

    def __init__(
        self,
        ingest_url: Optional[str],
        device_id: str = DEFAULT_DEVICE_ID,
        *,
        timeout_s: float = 3.0,
        queue_size: int = 256,
        batch_max: int = 32,
        transport: Optional[Callable[[dict], None]] = None,
    ):
        self.ingest_url = (ingest_url or "").strip() or None
        self.device_id = device_id
        self._timeout_s = timeout_s
        self._batch_max = batch_max
        self._transport = transport or self._default_transport
        self._consecutive_failures = 0
        self._dropped = 0
        self._queue: Optional[queue.Queue] = None
        self._thread: Optional[threading.Thread] = None
        if self.ingest_url:
            self._queue = queue.Queue(maxsize=queue_size)
            self._thread = threading.Thread(
                target=self._worker, name="xarm-events-exporter", daemon=True
            )
            self._thread.start()

    @classmethod
    def from_env(cls, environ=os.environ) -> "EventsExporter":
        url = environ.get("XARM_INGEST_URL", "").strip()
        device_id = environ.get("XARM_INGEST_DEVICE_ID", "").strip() or DEFAULT_DEVICE_ID
        return cls(url or None, device_id)

    @property
    def enabled(self) -> bool:
        return self._queue is not None

    def emit(
        self,
        event: str,
        *,
        from_state: Optional[str] = None,
        to_state: Optional[str] = None,
        message: Optional[str] = None,
        **extra: Any,
    ) -> bool:
        """Enqueue one event record. Returns False when disabled or dropped.

        Never raises and never blocks: this is called from SDK callbacks
        and motion code paths that must not stall on observability.
        """
        if self._queue is None:
            return False
        record: dict[str, Any] = {"timestamp": _utc_now_iso(), "event": event}
        if from_state is not None:
            record["from_state"] = from_state
        if to_state is not None:
            record["to_state"] = to_state
        if message is not None:
            record["message"] = message
        cleaned = {k: v for k, v in extra.items() if v is not None}
        if cleaned:
            record["extra"] = cleaned
        try:
            self._queue.put_nowait(record)
            return True
        except queue.Full:
            self._dropped += 1
            if self._dropped == 1 or self._dropped % _FAILURE_LOG_EVERY == 0:
                print(f"[events] queue full — dropped {self._dropped} event(s) so far")
            return False

    def close(self, timeout_s: float = 5.0) -> None:
        """Best-effort flush + stop. Safe to call on a disabled exporter."""
        if self._queue is None or self._thread is None:
            return
        try:
            self._queue.put_nowait(_SENTINEL)
        except queue.Full:
            pass
        self._thread.join(timeout=timeout_s)

    # ------------------------------------------------------------------
    # Worker side
    # ------------------------------------------------------------------

    def _worker(self) -> None:
        assert self._queue is not None
        while True:
            item = self._queue.get()
            if item is _SENTINEL:
                return
            batch = [item]
            while len(batch) < self._batch_max:
                try:
                    nxt = self._queue.get_nowait()
                except queue.Empty:
                    break
                if nxt is _SENTINEL:
                    self._send(batch)
                    return
                batch.append(nxt)
            self._send(batch)

    def _send(self, records: list) -> None:
        payload = {"device_id": self.device_id, "records": records}
        try:
            self._transport(payload)
        except Exception as exc:  # noqa: BLE001 - best-effort by contract
            self._consecutive_failures += 1
            if (
                self._consecutive_failures == 1
                or self._consecutive_failures % _FAILURE_LOG_EVERY == 0
            ):
                print(
                    f"[events] ingest POST failed ({self._consecutive_failures} in a row), "
                    f"dropping {len(records)} record(s): {exc}"
                )
        else:
            if self._consecutive_failures:
                print(f"[events] ingest recovered after {self._consecutive_failures} failure(s)")
            self._consecutive_failures = 0

    def _default_transport(self, payload: dict) -> None:
        body = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            self.ingest_url,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        # urllib raises HTTPError on non-2xx and URLError on transport
        # failure; both surface to _send()'s except.
        with urllib.request.urlopen(request, timeout=self._timeout_s):
            pass
