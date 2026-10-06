"""Compatibility names for captures stored by the SDL camera service."""
from __future__ import annotations
from typing import Any

COLOR_NAME = "color.jpg"
DEPTH_NAME = "depth.png"


class CaptureStoreError(RuntimeError):
    pass


class CaptureNotFound(LookupError):
    pass


_shared: Any | None = None


def set_shared(store: Any | None) -> None:
    global _shared
    _shared = store


def shared_store() -> Any | None:
    return _shared
