"""Explicit service startup; never kill occupied ports or auto-enable hardware."""

import argparse
import json
from pathlib import Path

from . import __version__
from .config import load_settings
from .drivers import inventory


def main():
    parser = argparse.ArgumentParser(prog="robot-motion")
    parser.add_argument("--version", action="version", version=__version__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser(
        "drivers", help="List model profiles and installed SDKs; no connections"
    )
    serve = commands.add_parser("serve", help="Bundled UI and read-only prototype API")
    serve.add_argument("--config", type=Path)
    serve.add_argument("--host", default="127.0.0.1")
    serve.add_argument("--port", type=int, default=8075)
    legacy = commands.add_parser(
        "legacy-xarm", help="Unchanged xArm API/UI; requires [xarm]"
    )
    legacy.add_argument("--host", default="127.0.0.1")
    legacy.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    if args.command == "drivers":
        print(json.dumps(inventory(), indent=2))
        return
    if not 1 <= args.port <= 65535:
        parser.error("Port must be 1..65535")
    if args.command == "legacy-xarm":
        if not inventory()["drivers"]["xarm"]["sdk_installed"]:
            parser.error('xArm SDK missing: install "robot-motion[xarm]"')
        from core.xarm_api_server import app
    else:
        from .app import create_app

        app = create_app(load_settings(args.config))
    import uvicorn

    uvicorn.run(app, host=args.host, port=args.port, workers=1)


if __name__ == "__main__":
    main()
