"""Phase 5 tests: claim enforcement on mutating endpoints.

Hard enforcement (STATUS_SPEC §5): when enforcement is ON the claim is
the single gate — there is no no-claim free-for-all window.

Covers:
- ClaimManager.verify_token() honors the enforce flag
- Default (enforce off) behavior unchanged
- enforce on + no claim held = motion REFUSED (must claim first)
- enforce on + claim held + matching token = moves proceed
- enforce on + claim held/no claim + wrong/missing token = 423 + body
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


def test_verify_token_raises_when_no_claim_held():
    """Hard enforcement: no holder = motion refused until someone claims."""
    cm = ClaimManager(enforce=True)
    with pytest.raises(InvalidClaimToken):
        cm.verify_token(None)
    with pytest.raises(InvalidClaimToken):
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


# ── Enforcement on, no claim held = refused (hard enforcement) ─────


def test_named_move_refused_when_enforcement_on_and_no_claim_held(client, mock_controller):
    mock_controller.claim_manager.enable_enforcement()
    # No claim held — hard enforcement refuses motion until someone claims.
    resp = client.post("/move/location", json={"location_name": "pickup"})
    assert resp.status_code == 423
    detail = resp.json()["detail"]
    assert detail["error"] == "claim_required"
    # No holder, so claimed_by is null and the hint points at /control/claim.
    assert detail["claimed_by"] is None


def test_claim_then_move_with_token_succeeds(client, mock_controller):
    """The intended flow under hard enforcement: acquire, then move."""
    mock_controller.claim_manager.enable_enforcement()
    claim = client.post(
        "/control/claim", json={"owner": "human@xarm-web", "session_id": "s9"}
    )
    assert claim.status_code == 200
    token = claim.json()["claim_token"]
    resp = client.post(
        "/move/location",
        json={"location_name": "pickup"},
        headers={"X-Claim-Token": token},
    )
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


def test_stop_works_with_enforcement_on_and_no_claim(client, mock_controller):
    """Safety floor under hard enforcement: STOP must work even when no
    claim is held (and would otherwise 423 a move)."""
    mock_controller.claim_manager.enable_enforcement()
    assert client.post("/move/stop").status_code == 200
    assert client.post("/clear/errors").status_code == 200


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
