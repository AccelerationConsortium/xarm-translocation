"""Passive xArm F/T telemetry from the extended port-30003 report.

No SDK calls or motion decisions belong here. Receipt time describes a report,
not a sensor acquisition: the controller may repeat a sensor value in reports.
"""

from collections import deque
from copy import deepcopy
from datetime import datetime, timezone
import math
import struct
import threading
import time


class ForceTorqueTelemetry:
    def __init__(self, *, enabled=False, configuration=None, keep=65536,
                 stale_after_s=0.5, clock=time.perf_counter):
        self._lock = threading.Lock()
        self._clock = clock
        self._enabled = enabled
        self._active = False
        self._reason = "awaiting_report" if enabled else "sensor_not_enabled_at_start"
        self._latest = None
        self._samples = deque(maxlen=keep)
        self._sequence = 0
        self._dropped = 0
        self._invalid = 0
        self._stale_after_s = stale_after_s
        self._configuration = deepcopy(configuration or {})
        self._configuration["service_config_revision"] = self._configuration.pop("revision", None)
        self._configuration["source"] = {
            "method": "realtime_report", "port": 30003,
            "channel": "controller_compensated_filtered", "is_raw": False,
        }
        self._configuration.update({
            "wrench_order": ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"],
            "units": {"force": "N", "torque": "N*m"},
            "sensor_enabled_at_start": enabled,
        })
        tare = self._configuration.get("service_tare", {})
        self._tare_applied = bool(tare.get("completed", False))
        offset = tare.get("offset") if self._tare_applied else [0.0] * 6
        if (not isinstance(offset, (list, tuple)) or len(offset) != 6
                or not all(isinstance(v, (float, int)) and math.isfinite(v) for v in offset)):
            self._enabled = False
            self._reason = "invalid_tare_configuration"
            offset = [0.0] * 6
        self._offset = tuple(offset)

    def start(self):
        with self._lock:
            self._active = True

    def stop(self, error=None):
        with self._lock:
            self._active = False
            if error:
                self._reason = error

    def reject(self, reason):
        with self._lock:
            if self._enabled:
                self._reason = reason
                self._invalid += 1

    def receive(self, frame, received_s, received_wall):
        if not self._enabled:
            return
        # SDK __handle_report_real: ext [87:111], raw [111:135]. Older
        # frames lack these fields; never substitute zero or cached SDK data.
        if len(frame) < 135:
            self.reject("ft_fields_unavailable")
            return
        values = struct.unpack_from("<6f", frame, 87)
        wrench = tuple(v - offset for v, offset in zip(values, self._offset))
        if not all(math.isfinite(v) for v in (*values, *wrench)):
            self.reject("invalid_ft_values")
            return
        with self._lock:
            self._sequence += 1
            sample = (self._sequence, received_s, received_wall, wrench)
            self._latest = sample
            self._reason = None
            if len(self._samples) == self._samples.maxlen:
                self._dropped += 1
            self._samples.append(sample)

    def snapshot(self):
        with self._lock:
            latest, reason, active = self._latest, self._reason, self._active
            dropped, invalid = self._dropped, self._invalid
        age = max(0.0, self._clock() - latest[1]) if latest else None
        if reason is None:
            if not active:
                reason = "report_stream_stopped"
            elif age is not None and age > self._stale_after_s:
                reason = "stale_report"
        sample = None
        if latest:
            sequence, received_s, received_wall, wrench = latest
            sample = {
                "sequence": sequence, "received_s": received_s,
                "service_received_at": datetime.fromtimestamp(
                    received_wall, timezone.utc).isoformat(),
                "sensor_sampled_at": None,
                "wrench": list(wrench), "service_tare_applied": self._tare_applied,
            }
        return {
            "valid": latest is not None and reason is None,
            "reason": reason, "report_stream_active": active,
            "age_s": age, "stale_after_s": self._stale_after_s,
            "sample": sample, "configuration": deepcopy(self._configuration),
            "dropped_samples": dropped, "invalid_reports": invalid,
        }

    def record(self):
        """Serialize only after the monitor has stopped, outside the servo loop."""
        with self._lock:
            samples = list(self._samples)
        return {
            **self.snapshot(),
            "sequence": [s[0] for s in samples],
            "received_s": [s[1] for s in samples],
            "received_unix_s": [s[2] for s in samples],
            "wrench": [list(s[3]) for s in samples],
        }
