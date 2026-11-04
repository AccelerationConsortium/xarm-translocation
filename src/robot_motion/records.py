"""Diagnoses and refusals by request id, to read back later.

GET /control/diagnostics/{request_id} answers from here, so a caller that
lost a response body never has to resend a move to see why it was refused.
The newest records are kept in memory; anything older, or from before a
restart, is read back from the control audit file, where every one is also
written (control.py event()).
"""

from __future__ import annotations

import json
import logging
import threading
from collections import OrderedDict

log = logging.getLogger(__name__)
# Audit line kinds that carry a request's record, and where in the line.
RECORD_KINDS = {"pose_diagnosis": "record", "arm_refused": "detail", "request_invalid": None}


class RequestRecords:
    def __init__(self, audit_file=None, *, keep=1000):
        self.audit_file = audit_file
        self.keep = keep
        self._records = OrderedDict()
        self._lock = threading.Lock()

    def put(self, request_id, record):
        if not request_id:
            return
        with self._lock:
            self._records.setdefault(request_id, []).append(record)
            self._records.move_to_end(request_id)
            while len(self._records) > self.keep:
                self._records.popitem(last=False)

    def get(self, request_id):
        """Every record for request_id, oldest first ([] when none)."""
        with self._lock:
            found = list(self._records.get(request_id, ()))
        return found or self._from_audit(request_id)

    def _from_audit(self, request_id):
        if not self.audit_file:
            return []
        found = []
        try:
            with open(self.audit_file, encoding="utf-8") as handle:
                for line in handle:
                    if request_id not in line:
                        continue
                    try:
                        event = json.loads(line)
                    except ValueError:
                        continue
                    kind = event.get("kind")
                    if kind not in RECORD_KINDS or event.get("request_id") != request_id:
                        continue
                    field = RECORD_KINDS[kind]
                    record = event.get(field) if field else None
                    found.append({
                        **(record if isinstance(record, dict) else {k: v for k, v in event.items() if k != "kind"}),
                        "kind": kind, "at": event.get("at"), "operator": event.get("operator"),
                        "status": event.get("status"), "source": "audit_file",
                    })
        except OSError:
            log.exception("Reading the control audit file for request %s failed", request_id)
        return found
