"""Tests for the server-side login gate on POST /control/claim.

When XARM_REQUIRE_LOGIN_FOR_CLAIM is on (and the auth sidecar is
configured), acquiring a claim requires a verified identity — a signed-in
session cookie (-> sidecar /auth/me) or an X-Api-Key (-> /auth/verify) —
and the verified email OVERRIDES the client-supplied owner. The sidecar
round-trip (``_auth_sidecar_call``) is monkeypatched. A real ClaimManager
is attached to a mock controller so the acquire path runs end-to-end.
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
def gate_on(monkeypatch):
    monkeypatch.setattr(srv, "REQUIRE_LOGIN_FOR_CLAIM", True)
    monkeypatch.setattr(srv, "AUTH_SIDECAR_URL", "http://auth-sidecar:8009")


def make_fake(status, payload, token=None, capture=None):
    def _call(method, path, body=None, cookie_token=None, api_key=None):
        if capture is not None:
            capture.append({
                "method": method, "path": path,
                "cookie_token": cookie_token, "api_key": api_key,
            })
        return status, payload, token
    return _call


# ── gate ON ──────────────────────────────────────────────────────────


def test_claim_without_identity_401(client, gate_on, monkeypatch):
    calls = []
    monkeypatch.setattr(srv, "_auth_sidecar_call", make_fake(200, {}, capture=calls))
    resp = client.post("/control/claim", json={"owner": "anon", "session_id": "s1"})
    assert resp.status_code == 401
    assert resp.json()["detail"]["error"] == "login_required"
    # No credential presented -> no sidecar round-trip at all.
    assert calls == []


def test_claim_with_valid_cookie_overrides_owner(
    client, gate_on, mock_controller, monkeypatch,
):
    calls = []
    monkeypatch.setattr(
        srv, "_auth_sidecar_call",
        make_fake(200, {"authenticated": True,
                        "identity": {"email": "op@lab", "role": "user"}},
                  capture=calls),
    )
    client.cookies.set(srv.AUTH_COOKIE_NAME, "tok123")
    resp = client.post("/control/claim",
                       json={"owner": "client-says-anon", "session_id": "s1"})
    assert resp.status_code == 200
    # Owner is the verified email, NOT the client-supplied "client-says-anon".
    holder = mock_controller.claim_manager.claimed_by()
    assert holder["owner"] == "op@lab"
    assert holder["session_id"] == "s1"  # client session_id preserved
    # Cookie path -> /auth/me, cookie forwarded.
    assert calls[0]["path"] == "/auth/me"
    assert calls[0]["cookie_token"] == "tok123"


def test_claim_with_api_key_succeeds(
    client, gate_on, mock_controller, monkeypatch,
):
    calls = []
    monkeypatch.setattr(
        srv, "_auth_sidecar_call",
        make_fake(200, {"email": "bot@lab"}, capture=calls),
    )
    resp = client.post("/control/claim",
                       json={"owner": "x", "session_id": "s2"},
                       headers={"X-Api-Key": "key123"})
    assert resp.status_code == 200
    assert mock_controller.claim_manager.claimed_by()["owner"] == "bot@lab"
    # X-Api-Key path -> /auth/verify with the key forwarded.
    assert calls[0]["path"] == "/auth/verify"
    assert calls[0]["api_key"] == "key123"


def test_claim_sidecar_down_returns_503(client, gate_on, monkeypatch):
    def boom(*a, **k):
        raise OSError("no route to host")
    monkeypatch.setattr(srv, "_auth_sidecar_call", boom)
    client.cookies.set(srv.AUTH_COOKIE_NAME, "tok123")
    resp = client.post("/control/claim", json={"owner": "anon", "session_id": "s1"})
    # Fail closed: sidecar unreachable is 503, never a silent allow.
    assert resp.status_code == 503


# ── gate OFF (unchanged behaviour) ───────────────────────────────────


def test_claim_gate_off_passes_owner_through(
    client, mock_controller, monkeypatch,
):
    monkeypatch.setattr(srv, "REQUIRE_LOGIN_FOR_CLAIM", False)
    # No auth cookie, no key — with the gate off this is fine.
    resp = client.post("/control/claim", json={"owner": "anon", "session_id": "s1"})
    assert resp.status_code == 200
    assert mock_controller.claim_manager.claimed_by()["owner"] == "anon"


def test_claim_gate_flag_on_but_sidecar_unset_passes_through(
    client, mock_controller, monkeypatch,
):
    """The gate needs BOTH the flag and a configured sidecar; the flag alone
    (no XARM_AUTH_URL) leaves the owner passthrough intact."""
    monkeypatch.setattr(srv, "REQUIRE_LOGIN_FOR_CLAIM", True)
    monkeypatch.setattr(srv, "AUTH_SIDECAR_URL", "")
    resp = client.post("/control/claim", json={"owner": "anon", "session_id": "s1"})
    assert resp.status_code == 200
    assert mock_controller.claim_manager.claimed_by()["owner"] == "anon"
