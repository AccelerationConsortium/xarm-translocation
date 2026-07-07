"""Tests for single-edge SSO trust (docs/SINGLE_EDGE_SSO_PLAN.md, tasks 3-4).

When the panel is reached through the lab's single Caddy edge, the edge has
already authenticated the human (forward_auth -> ac_auth) and injects
``X-Auth-User`` / ``X-Auth-Role``. The device trusts that identity ONLY when it
arrives with a matching ``X-Edge-Auth`` shared secret, so:

- the banner shows the signed-in user with no second login (``/auth/me``),
- control/claim resolve to the edge identity with NO sidecar round-trip, and
- a direct caller forging the headers (no/ wrong secret, or secret unset) is
  ignored and falls back to the cookie / X-Api-Key path.
"""

from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock

import pytest
from fastapi.testclient import TestClient

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import src.core.xarm_api_server as srv
from src.core.claims import ClaimManager

SECRET = "edge-secret-abc"
EDGE_HEADERS = {"X-Auth-User": "alice@lab", "X-Auth-Role": "admin",
                "X-Edge-Auth": SECRET}


@pytest.fixture
def mock_controller():
    mc = MagicMock()
    mc.claim_manager = ClaimManager(default_ttl_s=30.0)
    return mc


@pytest.fixture
def client(monkeypatch, mock_controller):
    monkeypatch.setattr(srv, "controller", mock_controller)
    with TestClient(srv.app) as c:
        yield c


@pytest.fixture
def edge_on(monkeypatch):
    monkeypatch.setattr(srv, "EDGE_SHARED_SECRET", SECRET)


def make_fake(status, payload, token=None, capture=None):
    def _call(method, path, body=None, cookie_token=None, api_key=None):
        if capture is not None:
            capture.append({"method": method, "path": path,
                            "cookie_token": cookie_token, "api_key": api_key})
        return status, payload, token
    return _call


# ── _edge_identity unit behaviour ────────────────────────────────────


def test_edge_identity_trusts_matching_secret(edge_on):
    req = MagicMock()
    req.headers = {"X-Auth-User": "alice@lab", "X-Auth-Role": "admin",
                   "X-Edge-Auth": SECRET}
    assert srv._edge_identity(req) == {"email": "alice@lab", "role": "admin"}


def test_edge_identity_rejects_wrong_secret(edge_on):
    req = MagicMock()
    req.headers = {"X-Auth-User": "alice@lab", "X-Edge-Auth": "nope"}
    assert srv._edge_identity(req) is None


def test_edge_identity_disabled_when_secret_unset(monkeypatch):
    monkeypatch.setattr(srv, "EDGE_SHARED_SECRET", None)
    req = MagicMock()
    req.headers = {"X-Auth-User": "alice@lab", "X-Edge-Auth": "anything"}
    assert srv._edge_identity(req) is None


def test_edge_identity_requires_user_header(edge_on):
    req = MagicMock()
    req.headers = {"X-Edge-Auth": SECRET}  # secret but no user
    assert srv._edge_identity(req) is None


# ── /auth/config + /auth/me ──────────────────────────────────────────


def test_config_enabled_by_edge_without_sidecar(client, edge_on, monkeypatch):
    monkeypatch.setattr(srv, "AUTH_SIDECAR_URL", "")
    assert client.get("/auth/config").json() == {"enabled": True}


def test_me_reports_edge_identity_without_sidecar_roundtrip(
    client, edge_on, monkeypatch,
):
    monkeypatch.setattr(srv, "AUTH_SIDECAR_URL", "")
    calls = []
    monkeypatch.setattr(srv, "_auth_sidecar_call", make_fake(200, {}, capture=calls))
    body = client.get("/auth/me", headers=EDGE_HEADERS).json()
    assert body["authenticated"] is True
    assert body["identity"] == {"email": "alice@lab", "role": "admin",
                                 "via": "edge"}
    assert calls == []  # edge is header-only; no sidecar hop


def test_me_ignores_forged_headers_without_secret(client, monkeypatch):
    # Secret unset -> edge trust off -> injected headers are ignored.
    monkeypatch.setattr(srv, "EDGE_SHARED_SECRET", None)
    monkeypatch.setattr(srv, "AUTH_SIDECAR_URL", "")
    body = client.get("/auth/me", headers=EDGE_HEADERS).json()
    assert body == {"authenticated": False, "identity": None}


def test_me_ignores_wrong_secret(client, edge_on, monkeypatch):
    monkeypatch.setattr(srv, "AUTH_SIDECAR_URL", "")
    bad = dict(EDGE_HEADERS, **{"X-Edge-Auth": "wrong"})
    body = client.get("/auth/me", headers=bad).json()
    assert body == {"authenticated": False, "identity": None}


# ── control / claim resolve to the edge identity ─────────────────────


@pytest.fixture
def gate_on(monkeypatch):
    monkeypatch.setattr(srv, "REQUIRE_LOGIN", True)
    monkeypatch.setattr(srv, "AUTH_SIDECAR_URL", "http://auth-sidecar:8009")


def test_claim_owner_is_edge_identity_no_roundtrip(
    client, gate_on, edge_on, mock_controller, monkeypatch,
):
    calls = []
    monkeypatch.setattr(srv, "_auth_sidecar_call", make_fake(200, {}, capture=calls))
    resp = client.post("/control/claim",
                       json={"owner": "client-says-anon", "session_id": "s1"},
                       headers=EDGE_HEADERS)
    assert resp.status_code == 200
    holder = mock_controller.claim_manager.claimed_by()
    assert holder["owner"] == "alice@lab"       # edge identity, not client owner
    assert holder["session_id"] == "s1"
    assert calls == []                          # trusted edge -> no sidecar hop


def test_claim_forged_headers_without_secret_401(client, gate_on, monkeypatch):
    # Gate on, sidecar configured, secret unset: the injected headers are not
    # trusted and there is no cookie/key, so the claim is refused.
    monkeypatch.setattr(srv, "EDGE_SHARED_SECRET", None)
    monkeypatch.setattr(srv, "_auth_sidecar_call", make_fake(200, {}))
    resp = client.post("/control/claim",
                       json={"owner": "anon", "session_id": "s1"},
                       headers=EDGE_HEADERS)
    assert resp.status_code == 401
    assert resp.json()["detail"]["error"] == "login_required"


@pytest.fixture
def no_broadcast(monkeypatch):
    async def _noop():
        return None
    monkeypatch.setattr(srv, "broadcast_status_update", _noop)


def test_stop_allowed_via_edge_identity(
    client, gate_on, edge_on, no_broadcast, monkeypatch,
):
    calls = []
    monkeypatch.setattr(srv, "_auth_sidecar_call", make_fake(200, {}, capture=calls))
    resp = client.post("/move/stop", json={}, headers=EDGE_HEADERS)
    assert resp.status_code == 200
    assert calls == []  # edge identity, no sidecar hop
