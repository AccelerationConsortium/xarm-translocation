"""Timestamped formats for the service log.

The NSSM service logs (C:\\SDL_Logs\\xarm.err.log / xarm.out.log) had no
timestamps, so an event such as a gripper slip could not be matched to the
request that preceded it. Every line now starts with a UTC ISO 8601 time, the
same clock as ``latched_at`` and the other timestamps on /status.
"""

import logging
from copy import deepcopy
from datetime import datetime, timezone

from uvicorn.config import LOGGING_CONFIG
from uvicorn.logging import AccessFormatter, DefaultFormatter

# Same "LEVEL:name:message" layout as logging's default, prefixed with time.
LOG_FORMAT = "%(asctime)s %(levelname)s:%(name)s:%(message)s"


def _utc_iso(record: logging.LogRecord) -> str:
    return datetime.fromtimestamp(record.created, timezone.utc).isoformat(timespec="milliseconds")


class UtcFormatter(logging.Formatter):
    def formatTime(self, record, datefmt=None):
        return _utc_iso(record)


class UtcDefaultFormatter(DefaultFormatter):
    def formatTime(self, record, datefmt=None):
        return _utc_iso(record)


class UtcAccessFormatter(AccessFormatter):
    def formatTime(self, record, datefmt=None):
        return _utc_iso(record)


def configure_root_logging(level=logging.INFO) -> None:
    """Install a timestamped stderr handler on the root logger.

    Like ``logging.basicConfig`` this does nothing when the root logger
    already has handlers.
    """
    handler = logging.StreamHandler()
    handler.setFormatter(UtcFormatter(LOG_FORMAT))
    logging.basicConfig(level=level, handlers=[handler])


def uvicorn_log_config() -> dict:
    """uvicorn's default log config with a UTC timestamp on every line."""
    config = deepcopy(LOGGING_CONFIG)
    formatters = config["formatters"]
    formatters["default"]["()"] = UtcDefaultFormatter
    formatters["default"]["fmt"] = "%(asctime)s " + formatters["default"]["fmt"]
    formatters["access"]["()"] = UtcAccessFormatter
    formatters["access"]["fmt"] = "%(asctime)s " + formatters["access"]["fmt"]
    return config
