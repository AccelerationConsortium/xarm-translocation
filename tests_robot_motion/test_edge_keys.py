"""ApiKeyVerifier and the ac_auth round trip, against a local stub server."""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import pytest

from robot_motion.config import Settings
from robot_motion.edge import ApiKeyVerifier, AuthUnavailable, KeyForbidden, verify_with_auth

AGENT = "agent:someone@example.invalid"


class StubAuth(BaseHTTPRequestHandler):
    """/auth/verify as ac_auth answers it: 200 + identity, 401, 403, or 500."""

    seen = []

    def do_GET(self):
        url = urlparse(self.path)
        key = self.headers.get("X-Api-Key")
        equipment = parse_qs(url.query).get("equipment")
        StubAuth.seen.append((url.path, equipment, key))
        if key == "agent-key":
            status, body = 200, {"ok": True, "email": AGENT.upper(), "role": "automation"}
        elif key == "forbidden-key":
            status, body = 403, {"detail": "no role on equipment"}
        elif key == "broken-key":
            status, body = 500, {"detail": "boom"}
        elif key == "nameless-key":
            status, body = 200, {"ok": True}
        else:
            status, body = 401, {"detail": "not authenticated"}
        payload = json.dumps(body).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, *args):
        pass


@pytest.fixture
def auth_url():
    server = ThreadingHTTPServer(("127.0.0.1", 0), StubAuth)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    StubAuth.seen = []
    yield f"http://127.0.0.1:{server.server_address[1]}"
    server.shutdown()
    server.server_close()


def test_verify_asks_for_this_equipment_and_reads_the_principal(auth_url):
    identity = verify_with_auth(auth_url, "ur3e_gibbie", "agent-key", 2)
    assert identity == {"email": AGENT, "role": "automation"}
    assert StubAuth.seen == [("/auth/verify", ["ur3e_gibbie"], "agent-key")]


def test_verify_maps_every_answer(auth_url):
    assert verify_with_auth(auth_url, "ligand_ur5e", "unknown-key", 2) is None
    with pytest.raises(KeyForbidden):
        verify_with_auth(auth_url, "ligand_ur5e", "forbidden-key", 2)
    with pytest.raises(AuthUnavailable):
        verify_with_auth(auth_url, "ligand_ur5e", "broken-key", 2)
    with pytest.raises(AuthUnavailable):
        verify_with_auth(auth_url, "ligand_ur5e", "nameless-key", 2)


def test_unreachable_auth_service_is_unavailable_not_a_refusal():
    with pytest.raises(AuthUnavailable):
        verify_with_auth("http://127.0.0.1:9", "ligand_ur5e", "agent-key", 0.5)


def test_allowing_verdicts_are_kept_for_the_ttl_refusals_never():
    now = [0.0]
    asked = []

    def verify(url, equipment, key, timeout):
        asked.append(key)
        return {"email": AGENT, "role": "automation"} if key == "agent-key" else None

    verifier = ApiKeyVerifier("http://auth.invalid/", "ur3e_gibbie", ttl_s=30, verify=verify, clock=lambda: now[0])
    assert verifier.auth_url == "http://auth.invalid"
    assert verifier("agent-key")["email"] == AGENT
    now[0] = 29
    assert verifier("agent-key")["email"] == AGENT
    assert verifier("unknown-key") is None
    assert verifier("unknown-key") is None
    assert asked == ["agent-key", "unknown-key", "unknown-key"]
    now[0] = 31
    verifier("agent-key")
    assert asked[-1] == "agent-key" and len(asked) == 4


def test_auth_url_is_an_origin():
    assert Settings(auth_url="http://100.64.254.6:8009").auth_url == "http://100.64.254.6:8009"
    for bad in ["100.64.254.6:8009", "http://host/auth", "http://host?x=1"]:
        with pytest.raises(ValueError):
            Settings(auth_url=bad)
