"""Identity injected by the lab's single Caddy edge.

The edge authenticates the human (forward_auth -> ac_auth) and proxies with
X-Auth-User / X-Auth-Role plus X-Edge-Auth carrying a shared secret. This
service trusts those headers only when the secret matches its own
ROBOT_MOTION_EDGE_SHARED_SECRET; with no secret configured nothing is trusted,
so a directly reachable port never believes client-supplied identity.
"""

from __future__ import annotations

import hmac
import os

EDGE_USER_HEADER = "X-Auth-User"
EDGE_ROLE_HEADER = "X-Auth-Role"
EDGE_TRUST_HEADER = "X-Edge-Auth"
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
