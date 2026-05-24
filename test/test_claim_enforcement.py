"""Phase 5 tests: claim enforcement on mutating endpoints.

Covers:
- ClaimManager.verify_token() honors the enforce flag + cooperative
  interpretation (no claim held = pass)
- Default (enforce off) behavior unchanged
- enforce on + no claim = moves proceed
- enforce on + claim held + matching token = moves proceed
- enforce on + claim held + wrong/missing token = 423 + body
- /move/stop and /clear/errors are NEVER gated (safety/recovery)
- /control/claim, /heartbeat, /release are NEVER gated (manage the lock)
- POST /control/claim/enforce respects the same dependency
"""

from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock

import pytest
from fastapi.testclient import TestClient

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Import via src.core to match api_server's resolved module path.
from src.core.claims import (
    ClaimManager,
    InvalidClaimToken,
)


# ── Unit: ClaimManager.verify_token ────────────────────────────────


def test_verify_token_noop_when_enforcement_off():
    cm = ClaimManager(enforce=False)
    cm.acquire(owner="a", session_id="s1")
    # Wrong token, but enforcement off — passes silently.
    cm.verify_token("garbage")
    cm.verify_token(None)


def test_verify_token_noop_when_no_claim_held():
    """Cooperative interpretation: no holder = anyone can move."""
    cm = ClaimManager(enforce=True)
    cm.verify_token(None)
    cm.verify_token("garbage")


def test_verify_token_passes_with_matching_token():
    cm = ClaimManager(enforce=True)
    record = cm.acquire(owner="a", session_id="s1")
    cm.verify_token(record.token)


def test_verify_token_raises_on_wrong_token():
    cm = ClaimManager(enforce=True)
    cm.acquire(owner="a", session_id="s1")
    with pytest.raises(InvalidClaimToken):
        cm.verify_token("garbage")


def test_verify_token_raises_on_missing_token():
    cm = ClaimManager(enforce=True)
    cm.acquire(owner="a", session_id="s1")
    with pytest.raises(InvalidClaimToken):
        cm.verify_token(None)


def test_runtime_toggles_change_enforcement():
    cm = ClaimManager(enforce=False)
    assert cm.enforced is False
    cm.enable_enforcement()
    assert cm.enforced is True
    cm.disable_enforcement()
    assert cm.enforced is False


# ── HTTP: enforcement integration ───────────────────────────────────


@pytest.fixture
def mock_controller():
    """Controller mock with a real ClaimManager so enforcement state is live."""
    mc = MagicMock()
    mc.claim_manager = ClaimManager()
    mc.motion_graph = None  # keep simple
    mc.is_connected.return_value = True
    mc.is_alive = True
    mc.host = "127.0.0.1"
    mc.xarm_config = {"port": 18333}
    mc.move_to_named_location.return_value = True
    mc.stop_motion.return_value = True
    mc.clear_errors.return_value = True
    return mc


@pytest.fixture
def client(monkeypatch, mock_controller):
    from src.core.xarm_api_server import app
    monkeypatch.setattr("src.core.xarm_api_server.controller", mock_controller)
    with TestClient(app) as c:
        yield c


# ── Default behavior (enforcement off) ────────────────────────────


def test_named_move_works_without_token_when_enforcement_off(client, mock_controller):
    # Claim held by someone, but enforcement is off (default).
    mock_controller.claim_manager.acquire(owner="a", session_id="s1")
    resp = client.post("/move/location", json={"location_name": "pickup"})
    assert resp.status_code == 200


# ── Enforcement on, no claim held = pass ───────────────────────────


def test_named_move_works_when_enforcement_on_but_no_claim_held(client, mock_controller):
    mock_controller.claim_manager.enable_enforcement()
    # No claim held — cooperative interpretation lets the move through.
    resp = client.post("/move/location", json={"location_name": "pickup"})
    assert resp.status_code == 200


# ── Enforcement on, claim held ─────────────────────────────────────


def test_matching_token_allows_move(client, mock_controller):
    mock_controller.claim_manager.enable_enforcement()
    record = mock_controller.claim_manager.acquire(owner="a", session_id="s1")
    resp = client.post(
        "/move/location",
        json={"location_name": "pickup"},
        headers={"X-Claim-Token": record.token},
    )
    assert resp.status_code == 200


def test_wrong_token_returns_423(client, mock_controller):
    mock_controller.claim_manager.enable_enforcement()
    mock_controller.claim_manager.acquire(owner="alice", session_id="s1")
    resp = client.post(
        "/move/location",
        json={"location_name": "pickup"},
        headers={"X-Claim-Token": "garbage"},
    )
    assert resp.status_code == 423
    detail = resp.json()["detail"]
    assert detail["error"] == "claim_required"
    assert detail["claimed_by"]["owner"] == "alice"
    assert detail["claimed_by"]["session_id"] == "s1"


def test_missing_token_returns_423(client, mock_controller):
    mock_controller.claim_manager.enable_enforcement()
    mock_controller.claim_manager.acquire(owner="a", session_id="s1")
    resp = client.post("/move/location", json={"location_name": "pickup"})
    assert resp.status_code == 423


# ── Always-allowed endpoints (safety/recovery/claim mgmt) ──────────


def test_stop_never_blocked_by_enforcement(client, mock_controller):
    """STOP is the safety floor — must never require a claim token."""
    mock_controller.claim_manager.enable_enforcement()
    mock_controller.claim_manager.acquire(owner="alice", session_id="s1")
    resp = client.post("/move/stop")
    assert resp.status_code == 200


def test_clear_errors_never_blocked_by_enforcement(client, mock_controller):
    mock_controller.claim_manager.enable_enforcement()
    mock_controller.claim_manager.acquire(owner="alice", session_id="s1")
    resp = client.post("/clear/errors")
    assert resp.status_code == 200


def test_claim_endpoint_never_blocked_by_enforcement(client, mock_controller):
    """Acquiring a fresh claim must not require a token. (How would
    you get one in the first place otherwise?)"""
    mock_controller.claim_manager.enable_enforcement()
    # No existing claim; this attempt has no X-Claim-Token.
    resp = client.post("/control/claim", json={"owner": "bob", "session_id": "s2"})
    assert resp.status_code == 200


def test_release_endpoint_never_blocked_by_enforcement(client, mock_controller):
    mock_controller.claim_manager.enable_enforcement()
    record = mock_controller.claim_manager.acquire(owner="a", session_id="s1")
    # Release uses its own X-Claim-Token header, not require_claim.
    resp = client.post("/control/release", headers={"X-Claim-Token": record.token})
    assert resp.status_code == 204


# ── /control/claim/enforce respects require_claim ──────────────────


def test_disable_enforcement_blocked_without_claim_when_already_on(client, mock_controller):
    """Disabling enforcement while it's on requires the current claim,
    or anyone could quietly drop the lock."""
    mock_controller.claim_manager.enable_enforcement()
    mock_controller.claim_manager.acquire(owner="alice", session_id="s1")
    resp = client.post("/control/claim/enforce", json={"enabled": False})
    assert resp.status_code == 423


def test_disable_enforcement_works_with_claim(client, mock_controller):
    mock_controller.claim_manager.enable_enforcement()
    record = mock_controller.claim_manager.acquire(owner="alice", session_id="s1")
    resp = client.post(
        "/control/claim/enforce",
        json={"enabled": False},
        headers={"X-Claim-Token": record.token},
    )
    assert resp.status_code == 200
    assert resp.json()["enforced"] is False


def test_enable_enforcement_open_when_off(client, mock_controller):
    """When enforcement is off, anyone can turn it on (bootstrap)."""
    assert mock_controller.claim_manager.enforced is False
    resp = client.post("/control/claim/enforce", json={"enabled": True})
    assert resp.status_code == 200
    assert resp.json()["enforced"] is True
