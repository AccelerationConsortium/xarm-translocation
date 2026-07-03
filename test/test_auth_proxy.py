"""Tests for the /auth/* banner proxy (xarm_api_server -> SDL2 Auth sidecar).

The sidecar round-trip (``_auth_sidecar_call``) is monkeypatched — these
tests cover routing, enable/disable behavior, status passthrough, and the
re-issued session cookie on this origin.
"""

import os
import sys

import pytest
from fastapi.testclient import TestClient

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import src.core.xarm_api_server as srv
from src.core.xarm_api_server import app


@pytest.fixture
def client():
    with TestClient(app) as c:
        yield c


@pytest.fixture
def auth_enabled(monkeypatch):
    monkeypatch.setattr(srv, "AUTH_SIDECAR_URL", "http://auth-sidecar:8009")


def fake_call(status, payload, token=None, capture=None):
    def _call(method, path, body=None, cookie_token=None):
        if capture is not None:
            capture.append({
                "method": method, "path": path,
                "body": body, "cookie_token": cookie_token,
            })
        return status, payload, token
    return _call


class TestAuthDisabled:
    def test_config_reports_disabled(self, client, monkeypatch):
        monkeypatch.setattr(srv, "AUTH_SIDECAR_URL", "")
        assert client.get("/auth/config").json() == {"enabled": False}

    def test_me_is_anonymous_without_sidecar(self, client, monkeypatch):
        monkeypatch.setattr(srv, "AUTH_SIDECAR_URL", "")
        assert client.get("/auth/me").json() == {
            "authenticated": False, "identity": None,
        }

    def test_mutating_endpoints_501(self, client, monkeypatch):
        monkeypatch.setattr(srv, "AUTH_SIDECAR_URL", "")
        assert client.post(
            "/auth/request-code", json={"email": "a@b.c"}
        ).status_code == 501
        assert client.post(
            "/auth/verify-code", json={"email": "a@b.c", "code": "1"}
        ).status_code == 501


class TestAuthProxy:
    def test_config_reports_enabled(self, client, auth_enabled):
        assert client.get("/auth/config").json() == {"enabled": True}

    def test_me_without_cookie_is_anonymous_no_roundtrip(
        self, client, auth_enabled, monkeypatch,
    ):
        calls = []
        monkeypatch.setattr(srv, "_auth_sidecar_call", fake_call(200, {}, capture=calls))
        assert client.get("/auth/me").json() == {
            "authenticated": False, "identity": None,
        }
        assert calls == []  # no cookie -> no sidecar call

    def test_me_forwards_cookie(self, client, auth_enabled, monkeypatch):
        calls = []
        identity = {"authenticated": True,
                    "identity": {"email": "op@lab", "role": "user"}}
        monkeypatch.setattr(
            srv, "_auth_sidecar_call", fake_call(200, identity, capture=calls),
        )
        client.cookies.set(srv.AUTH_COOKIE_NAME, "tok123")
        assert client.get("/auth/me").json() == identity
        assert calls[0]["cookie_token"] == "tok123"

    def test_verify_code_sets_origin_cookie(self, client, auth_enabled, monkeypatch):
        monkeypatch.setattr(
            srv, "_auth_sidecar_call",
            fake_call(200, {"ok": True, "email": "op@lab", "role": "user"},
                      token="sess-abc"),
        )
        resp = client.post("/auth/verify-code",
                           json={"email": "op@lab", "code": "123456"})
        assert resp.status_code == 200
        assert resp.json()["email"] == "op@lab"
        set_cookie = resp.headers["set-cookie"]
        assert f"{srv.AUTH_COOKIE_NAME}=sess-abc" in set_cookie
        assert "HttpOnly" in set_cookie

    def test_verify_code_failure_passes_status_through(
        self, client, auth_enabled, monkeypatch,
    ):
        monkeypatch.setattr(
            srv, "_auth_sidecar_call",
            fake_call(401, {"detail": "Invalid or expired code."}),
        )
        resp = client.post("/auth/verify-code",
                           json={"email": "op@lab", "code": "000000"})
        assert resp.status_code == 401
        assert "set-cookie" not in resp.headers

    def test_request_code_throttle_passthrough(self, client, auth_enabled, monkeypatch):
        monkeypatch.setattr(
            srv, "_auth_sidecar_call",
            fake_call(429, {"detail": "A sign-in code was just sent. Try again in 42s."}),
        )
        resp = client.post("/auth/request-code", json={"email": "op@lab"})
        assert resp.status_code == 429

    def test_sidecar_unreachable_maps_to_502(self, client, auth_enabled, monkeypatch):
        def boom(*a, **k):
            raise OSError("no route to host")
        monkeypatch.setattr(srv, "_auth_sidecar_call", boom)
        resp = client.post("/auth/request-code", json={"email": "op@lab"})
        assert resp.status_code == 502

    def test_logout_clears_cookie_even_if_sidecar_down(
        self, client, auth_enabled, monkeypatch,
    ):
        def boom(*a, **k):
            raise OSError("down")
        monkeypatch.setattr(srv, "_auth_sidecar_call", boom)
        client.cookies.set(srv.AUTH_COOKIE_NAME, "tok123")
        resp = client.post("/auth/logout")
        assert resp.status_code == 200
        assert f'{srv.AUTH_COOKIE_NAME}=""' in resp.headers["set-cookie"]


class TestSharedCookieDomain:
    """XARM_AUTH_COOKIE_DOMAIN scopes the session cookie to a parent domain
    so one sign-in covers every *.<domain> lab UI; logout must delete with
    the SAME Domain or the browser keeps the cookie."""

    def test_verify_code_sets_domain_when_configured(
        self, client, auth_enabled, monkeypatch,
    ):
        monkeypatch.setattr(srv, "AUTH_COOKIE_DOMAIN", "tail6a1dd7.ts.net")
        monkeypatch.setattr(
            srv, "_auth_sidecar_call",
            fake_call(200, {"ok": True, "email": "op@lab", "role": "user"},
                      token="sess-abc"),
        )
        resp = client.post("/auth/verify-code",
                           json={"email": "op@lab", "code": "123456"})
        assert resp.status_code == 200
        set_cookie = resp.headers["set-cookie"]
        assert f"{srv.AUTH_COOKIE_NAME}=sess-abc" in set_cookie
        assert "Domain=tail6a1dd7.ts.net" in set_cookie

    def test_verify_code_omits_domain_when_unset(
        self, client, auth_enabled, monkeypatch,
    ):
        monkeypatch.setattr(srv, "AUTH_COOKIE_DOMAIN", None)
        monkeypatch.setattr(
            srv, "_auth_sidecar_call",
            fake_call(200, {"ok": True}, token="sess-abc"),
        )
        resp = client.post("/auth/verify-code",
                           json={"email": "op@lab", "code": "123456"})
        assert "Domain=" not in resp.headers["set-cookie"]

    def test_logout_deletes_with_matching_domain(
        self, client, auth_enabled, monkeypatch,
    ):
        monkeypatch.setattr(srv, "AUTH_COOKIE_DOMAIN", "tail6a1dd7.ts.net")
        monkeypatch.setattr(srv, "_auth_sidecar_call", fake_call(200, {"ok": True}))
        client.cookies.set(srv.AUTH_COOKIE_NAME, "tok123")
        resp = client.post("/auth/logout")
        assert resp.status_code == 200
        set_cookie = resp.headers["set-cookie"]
        assert f'{srv.AUTH_COOKIE_NAME}=""' in set_cookie
        assert "Domain=tail6a1dd7.ts.net" in set_cookie
