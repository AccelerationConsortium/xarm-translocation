"""STATUS_SPEC v1.1 claim manager (cooperative single-holder lock).

A claim is a TTL-bounded soft lock on the device. Only one session may
hold the claim at a time. Heartbeats extend the TTL; release drops it;
expiry happens lazily on the next check after ``expires_at`` passes.

This module is pure Python — no FastAPI, no controller, no I/O — so it
is unit-testable without any of the surrounding stack. The api_server
wires it up to HTTP, and the status_builder reads ``claimed_by()`` for
the ``details.claimed_by`` block.

Cooperative, not authenticated: any client that ignores the X-Claim-Token
header could still send a control request. This matches the spec
(``STATUS_SPEC.md`` §5: "claims are cooperative, not authenticated").
Hard enforcement on ``/control/*`` is a separate concern; this module
just owns the state.
"""

from __future__ import annotations

import secrets
import threading
import time
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional


# Default and clamp values for ttl_s requested by clients. The spec
# allows the device to clamp; we keep a generous window.
DEFAULT_TTL_S = 30.0
MIN_TTL_S = 1.0
MAX_TTL_S = 300.0

# Heartbeat interval reported to clients. Spec says clients MUST send
# heartbeats more often than this; we set it to 1/3 of the default TTL
# so a single missed heartbeat doesn't drop the lock.
DEFAULT_HEARTBEAT_INTERVAL_S = 10.0


@dataclass
class ClaimRecord:
    token: str
    owner: str
    session_id: str
    expires_at: float  # epoch seconds (monotonic in UTC)

    def to_claimed_by_dict(self) -> dict:
        """Shape consumed by the ``ClaimedBy`` Pydantic model in /status.details."""
        return {
            "session_id": self.session_id,
            "owner": self.owner,
            "expires_at": datetime.fromtimestamp(self.expires_at, tz=timezone.utc),
        }


class ClaimConflict(Exception):
    """Raised when an acquire would steal a still-valid claim."""
    def __init__(self, holder: ClaimRecord, retry_after_s: float):
        self.holder = holder
        self.retry_after_s = retry_after_s
        super().__init__(
            f"claim held by session {holder.session_id!r} (owner {holder.owner!r}), "
            f"retry in {retry_after_s:.1f}s"
        )


class InvalidClaimToken(Exception):
    """Raised by heartbeat/release when X-Claim-Token doesn't match the
    current holder. Caller MUST treat the claim as lost (per spec)."""


class ClaimManager:
    """Process-local single-holder claim store.

    Thread-safe via an internal lock. Expiry is checked lazily — a
    claim past its ``expires_at`` is treated as released the moment any
    method is called.
    """

    def __init__(
        self,
        default_ttl_s: float = DEFAULT_TTL_S,
        heartbeat_interval_s: float = DEFAULT_HEARTBEAT_INTERVAL_S,
        *,
        clock=time.time,
    ):
        self._default_ttl_s = default_ttl_s
        self._heartbeat_interval_s = heartbeat_interval_s
        self._clock = clock
        self._lock = threading.Lock()
        self._current: Optional[ClaimRecord] = None

    @property
    def heartbeat_interval_s(self) -> float:
        return self._heartbeat_interval_s

    # ── Internal helpers ────────────────────────────────────────

    def _expire_if_due(self) -> None:
        """Drop the current claim if past its TTL. Caller must hold _lock."""
        if self._current and self._current.expires_at <= self._clock():
            self._current = None

    def _clamp_ttl(self, ttl_s: float) -> float:
        return max(MIN_TTL_S, min(MAX_TTL_S, float(ttl_s)))

    # ── Public API ──────────────────────────────────────────────

    def claimed_by(self) -> Optional[dict]:
        """Return the current holder's ``ClaimedBy`` dict, or None when no
        active claim. Used by status_builder to populate
        ``details.claimed_by``."""
        with self._lock:
            self._expire_if_due()
            if self._current is None:
                return None
            return self._current.to_claimed_by_dict()

    def acquire(self, owner: str, session_id: str, ttl_s: float | None = None) -> ClaimRecord:
        """Take the claim. Returns the new ClaimRecord on success.

        Idempotent for the same session_id: if this session already holds
        the claim, the token is rotated and the TTL refreshed (per spec:
        "A second POST /control/claim from the same session_id... is
        idempotent: the device returns 200 with the existing token or
        rotates and returns a fresh one"). We rotate for safety.

        Raises ClaimConflict when another live session holds the claim.
        """
        ttl = self._clamp_ttl(ttl_s if ttl_s is not None else self._default_ttl_s)
        with self._lock:
            self._expire_if_due()
            now = self._clock()
            if self._current is not None and self._current.session_id != session_id:
                retry = max(0.0, self._current.expires_at - now)
                raise ClaimConflict(holder=self._current, retry_after_s=retry)
            # Either no holder, or the same session re-acquiring — rotate the token.
            record = ClaimRecord(
                token=secrets.token_urlsafe(24),
                owner=owner,
                session_id=session_id,
                expires_at=now + ttl,
            )
            self._current = record
            return record

    def heartbeat(self, token: str, ttl_s: float | None = None) -> ClaimRecord:
        """Extend the holder's TTL. Returns the refreshed ClaimRecord.

        Raises InvalidClaimToken when the token is unknown, expired, or
        belongs to a different session (per spec: client MUST treat the
        claim as lost in any of these cases).
        """
        ttl = self._clamp_ttl(ttl_s if ttl_s is not None else self._default_ttl_s)
        with self._lock:
            self._expire_if_due()
            if self._current is None or self._current.token != token:
                raise InvalidClaimToken()
            self._current.expires_at = self._clock() + ttl
            return self._current

    def release(self, token: str) -> bool:
        """Release the claim. Idempotent: returns True if a claim was
        cleared, False if there was nothing to release (already gone /
        wrong token). Per spec, the HTTP layer always returns 204 — the
        bool here is for the controller's own bookkeeping."""
        with self._lock:
            self._expire_if_due()
            if self._current is None or self._current.token != token:
                return False
            self._current = None
            return True

    def _peek_record(self) -> Optional[ClaimRecord]:
        """Test helper: read the current record without expiry."""
        with self._lock:
            return self._current
