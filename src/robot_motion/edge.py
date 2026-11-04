"""Identity injected by the lab's single Caddy edge, or vouched for by the
central auth service for an agent's API key.

The edge authenticates the human (forward_auth -> ac_auth) and proxies with
X-Auth-User / X-Auth-Role plus X-Edge-Auth carrying a shared secret. This
service trusts those headers only when the secret matches its own
ROBOT_MOTION_EDGE_SHARED_SECRET; with no secret configured nothing is trusted,
so a directly reachable port never believes client-supplied identity.

Agents (machine principals using the lab-skills SDK) call the device directly
with their X-Api-Key, as they do the xArm and the OT-2 gateways. With an
auth_url configured, the key is checked by ac_auth's /auth/verify with
?equipment=<this equipment_id>, the same rule the edge applies to the panel,
so the central roster decides; nothing is decided here.
"""

from __future__ import annotations

import hashlib
import hmac
import json
import os
import threading
import time
import urllib.error
import urllib.request
from urllib.parse import urlencode

EDGE_USER_HEADER = "X-Auth-User"
EDGE_ROLE_HEADER = "X-Auth-Role"
EDGE_TRUST_HEADER = "X-Edge-Auth"
API_KEY_HEADER = "X-Api-Key"
SECRET_ENV = "ROBOT_MOTION_EDGE_SHARED_SECRET"


def configured_secret(override=None):
    if override is not None:
        return override or None
    return os.environ.get(SECRET_ENV, "").strip() or None


def edge_identity(request, secret):
    """{"email", "role"} for a request vouched for by the edge, else None."""
    if not secret:
        return None
    presented = request.headers.get(EDGE_TRUST_HEADER)
    if not presented or not hmac.compare_digest(presented, secret):
        return None
    email = (request.headers.get(EDGE_USER_HEADER) or "").strip().lower()
    if not email:
        return None
    return {"email": email, "role": (request.headers.get(EDGE_ROLE_HEADER) or "").strip() or None}


class AuthUnavailable(Exception):
    """The auth service gave no verdict; callers refuse (fail closed)."""


class KeyForbidden(Exception):
    """A valid key whose principal holds no role on this equipment."""


def verify_with_auth(auth_url, equipment_id, key, timeout_s):
    """Ask ac_auth whether `key` may act on `equipment_id`.

    {"email", "role"} when it may; None for an unknown, expired or revoked
    key (401). Raises KeyForbidden on 403 and AuthUnavailable for anything
    that is not a verdict (transport failure, 5xx, an unreadable body).
    """
    url = f"{auth_url}/auth/verify?{urlencode({'equipment': equipment_id})}"
    request = urllib.request.Request(url, headers={API_KEY_HEADER: key})
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as resp:
            payload = json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        if exc.code == 401:
            return None
        if exc.code == 403:
            raise KeyForbidden(equipment_id) from exc
        raise AuthUnavailable(f"auth service answered {exc.code}") from exc
    except (OSError, ValueError) as exc:
        raise AuthUnavailable(str(exc)) from exc
    email = str((payload or {}).get("email") or "").strip().lower()
    if not email:
        raise AuthUnavailable("auth service allowed the key but named no principal")
    return {"email": email, "role": str(payload.get("role") or "").strip() or None}


class ApiKeyVerifier:
    """Checks agent keys against ac_auth for one equipment id.

    An allowing verdict is kept for ttl_s so heartbeats and trajectory polls
    do not each cost a round trip; a revoked key therefore stops working
    within ttl_s. Refusals are never kept. Keys are held only as digests.
    """

    def __init__(self, auth_url, equipment_id, *, ttl_s=30.0, timeout_s=3.0,
                 verify=verify_with_auth, clock=time.monotonic):
        self.auth_url = auth_url.rstrip("/")
        self.equipment_id = equipment_id
        self.ttl_s = ttl_s
        self.timeout_s = timeout_s
        self._verify = verify
        self._clock = clock
        self._lock = threading.Lock()
        self._allowed = {}

    def __call__(self, key):
        digest = hashlib.sha256(key.encode("utf-8")).hexdigest()
        with self._lock:
            held = self._allowed.get(digest)
            if held is not None and held[0] > self._clock():
                return held[1]
            self._allowed.pop(digest, None)
        identity = self._verify(self.auth_url, self.equipment_id, key, self.timeout_s)
        if identity is not None:
            with self._lock:
                self._allowed[digest] = (self._clock() + self.ttl_s, identity)
        return identity
