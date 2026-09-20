"""Exercise the legacy server's routing without an API or hardware connection."""
from io import BytesIO
from unittest.mock import patch

import pytest

from src.web.server import XArmWebHandler


class RequestSocket:
    def __init__(self, path):
        self.input = BytesIO(f"GET {path} HTTP/1.0\r\n\r\n".encode())
        self.output = BytesIO()

    def makefile(self, *args, **kwargs):
        return self.input

    def sendall(self, data):
        self.output.write(data)


@pytest.mark.parametrize("path", [
    "/camera-player.js?v=20260915a", "/realsense-card.js?v=20260920a", "/graph.js", "/graph.css", "/graph.html",
])
def test_static_assets_are_served_instead_of_proxied(path):
    request = RequestSocket(path)
    with patch.object(XArmWebHandler, "proxy_to_api_server") as proxy:
        XArmWebHandler(request, ("127.0.0.1", 1234), None)
    proxy.assert_not_called()
    assert request.output.getvalue().startswith(b"HTTP/1.0 200")
    if path.startswith("/camera-player.js"):
        assert b"window.setupCameraCard" in request.output.getvalue()
    if path.startswith("/realsense-card.js"):
        assert b"window.setupRealSenseCard" in request.output.getvalue()


@pytest.mark.parametrize("path", [
    "/camera/config", "/graph", "/graph/layout?refresh=1", "/auth/me", "/status",
    "/realsense/cameras", "/realsense/rs435i/status",
    "/realsense/rs435i/snapshot.jpg?stream=depth",
])
def test_api_routes_still_proxy(path):
    with patch.object(XArmWebHandler, "proxy_to_api_server") as proxy:
        XArmWebHandler(RequestSocket(path), ("127.0.0.1", 1234), None)
    proxy.assert_called_once()
