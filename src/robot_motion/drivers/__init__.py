"""Driver discovery is metadata-only; vendor SDKs load only on explicit use."""

from importlib.util import find_spec

from ..config import MODELS


def inventory():
    return {
        "ui_included": True,
        "drivers": {
            "xarm": {
                "extra": "xarm",
                "sdk_installed": find_spec("xarm") is not None,
                "implementation": "legacy",
                "control": "existing xArm application only",
            },
            "ur": {
                "extra": "ur",
                "sdk_installed": find_spec("rtde_receive") is not None,
                "implementation": "observe",
                "control": "not yet implemented",
            },
            "mg400": {
                "extra": "mg400",
                "sdk_installed": False,
                "implementation": "planned",
                "control": "not yet implemented",
            },
        },
        "models": MODELS,
    }
