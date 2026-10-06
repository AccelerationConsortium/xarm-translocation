"""Compatibility registry for RealSense cameras owned by the SDL camera service.

No camera SDK or local capture driver is loaded in the xArm process.
"""
from __future__ import annotations

import os
from typing import Any


class RealSenseError(RuntimeError):
    pass


class RealSenseUnavailable(RealSenseError):
    pass


class RealSenseNotStreaming(RealSenseError):
    pass


_cameras: dict[str, Any] = {}
_reason: str | None = None


def load_config(path: str) -> tuple[dict[str, Any], str | None]:
    try:
        import yaml
        with open(path, encoding="utf-8") as handle:
            loaded = yaml.safe_load(handle)
    except FileNotFoundError:
        return {}, f"no RealSense config at {path}"
    except Exception as exc:
        return {}, f"could not read {path}: {exc}"
    if not isinstance(loaded, dict):
        return {}, f"{path} does not contain a mapping"
    return loaded, None


def set_cameras(mapping: dict[str, Any] | None, *, reason: str | None = None) -> None:
    global _cameras, _reason
    _cameras = dict(mapping or {})
    _reason = None if _cameras else (reason or "no RealSense cameras configured")


def cameras() -> dict[str, Any]:
    return dict(_cameras)


def camera(camera_id: str) -> Any | None:
    return _cameras.get(str(camera_id))


def default_camera_id() -> str | None:
    return next(iter(_cameras)) if len(_cameras) == 1 else None


def configuration_reason() -> str | None:
    return _reason if not _cameras else None


def default_config_path() -> str:
    return os.path.join(os.path.dirname(os.path.dirname(__file__)), "settings", "realsense.yaml")
