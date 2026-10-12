"""Service log lines carry a UTC ISO 8601 timestamp (core.log_format)."""

import logging
import re

from src.core.log_format import LOG_FORMAT, UtcFormatter, uvicorn_log_config

_ISO_UTC = r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{3}\+00:00"


def _record(name="xarm.events", msg="gripper.state stop -> object_detected", **attrs):
    record = logging.LogRecord(name, logging.INFO, __file__, 1, msg, None, None)
    record.created = 1791773139.25  # 2026-10-12T02:45:39.250Z
    for key, value in attrs.items():
        setattr(record, key, value)
    return record


def test_root_format_is_utc_iso_then_the_old_layout():
    line = UtcFormatter(LOG_FORMAT).format(_record())
    assert line == "2026-10-12T02:45:39.250+00:00 INFO:xarm.events:gripper.state stop -> object_detected"


def test_uvicorn_access_lines_are_timestamped():
    config = uvicorn_log_config()
    formatters = config["formatters"]
    access = formatters["access"]["()"](fmt=formatters["access"]["fmt"], use_colors=False)
    line = access.format(_record(
        name="uvicorn.access", msg='%s - "%s %s HTTP/%s" %d',
        args=("100.64.254.12:33500", "POST", "/gripper/close", "1.1", 200),
    ))
    assert re.match(_ISO_UTC + r' INFO: +100\.64\.254\.12:33500 - "POST /gripper/close HTTP/1\.1" 200', line), line


def test_uvicorn_default_config_is_not_mutated():
    from uvicorn.config import LOGGING_CONFIG

    uvicorn_log_config()
    assert not LOGGING_CONFIG["formatters"]["access"]["fmt"].startswith("%(asctime)s")
