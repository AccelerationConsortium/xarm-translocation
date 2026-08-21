"""The sash interlock over HTTP: refusal shape, coverage, and the override.

Three things are checked here that lower-level tests cannot see:

* **The 412 shape**, on every motion endpoint. The interlock refuses by raising
  through the controller, so what a client actually receives depends on FastAPI
  exception handling -- including an app-level handler that exists precisely so
  a motion endpoint added *later* inherits the right status code instead of
  leaking a 500.
* **That ``/assistant/execute`` does not flatten it.** That route catches
  ``GraphError``, and ``SashInterlockError`` is a ``GraphError`` subclass (so a
  broad catch degrades to a refusal rather than a silent allow), which means
  without an explicit clause ahead of it the rich 412 would be re-labelled a
  generic 409 ``step_invalid`` and the operator would lose the observed sash
  position and the way out.
* **Refusal ordering.** A claim conflict must surface as 423 *before* the
  interlock's 412, matching how the plateloc reference device orders them.
"""

from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock

import pytest
from fastapi.testclient import TestClient

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core.claims import ClaimManager
from src.core.xarm_controller import ComponentState
from src.core.motion_graph import DEFAULT_PRECONDITIONS, GraphMode, MotionGraph
from src.core.sash_interlock import SashInterlock, SashInterlockError

GRAPH_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "settings", "motion_graph.yaml",
)


def _sash(position):
    return {
        "equipment_status": "ready",
        "message": f"Sash parked at position {position}",
        "components": {
            "actuator": {"connected": True, "state": "idle"},
            "sash": {"connected": True, "state": f"position_{position}"},
        },
        "metrics": {"sash_position": {"value": position, "unit": "preset"}},
    }


def _make_controller(position, *, current_node="robot_home", current_rail="Home"):
    """A mock controller whose sash interlock is real but network-free.

    The interlock itself is the genuine article (so the refusal really does
    originate where it would in production); only the arm and the HTTP fetch
    are doubles.
    """
    graph = MotionGraph.from_yaml(GRAPH_PATH, preconditions=DEFAULT_PRECONDITIONS)
    interlock = SashInterlock(
        {"base_url": "http://sash.invalid", "enabled": True},
        fetcher=lambda url, timeout: _sash(position),
    )

    mc = MagicMock()
    mc.is_simulated = False
    mc.is_docker_target = False
    mc.is_controller_simulating = False
    mc.is_real_box_simulating = False
    mc.claim_manager = ClaimManager(default_ttl_s=30.0, enforce=True)
    mc.motion_graph = graph
    mc.graph_mode = GraphMode.ADVISORY      # so STRICT is never what refuses
    mc.current_node = current_node
    mc.current_gripper_state = "empty"
    mc.last_arm_pose_name = current_node
    mc.last_rail_location_name = current_rail
    mc._motion_in_progress = False
    mc.is_connected.return_value = True
    mc.is_alive = True
    mc.sash_interlock = interlock
    # Pinned because an unset MagicMock attribute is truthy: without these the
    # envelope reads as `error`, allowed_actions collapses to the recovery pair,
    # and the mirroring assertions would pass for the wrong reason.
    mc.alive = True
    mc.last_error_code = 0
    mc.last_error = None
    mc.last_warn_code = 0
    mc.last_transition = None
    mc.states = {
        'connection': ComponentState.ENABLED,
        'arm': ComponentState.ENABLED,
        'gripper': ComponentState.ENABLED,
        'track': ComponentState.ENABLED,
        'force_torque': ComponentState.DISABLED,
    }
    mc.has_gripper.return_value = True
    mc.has_track.return_value = True
    mc.has_force_torque_sensor.return_value = False
    mc.last_position = [300, 0, 300, 180, 0, 0]
    mc.last_joints = [0, 0, 0, 0, 0]

    # Route the real gate through the mock, as the controller does.
    def gate(target_node=None, target_pose=None, action="graph.move_to"):
        return interlock.gate_move(
            action=action, target_node=target_node, target_pose=target_pose,
            current_node=mc.current_node, current_rail=mc.last_rail_location_name,
            zone=interlock.zone(graph),
        )

    def move_to_node(node_id, speed=None):
        gate(target_node=node_id)
        return True

    def travel_to_node(node_id, speed=None):
        gate(target_node=node_id, action="graph.travel_to")
        return {"success": True, "path": [node_id], "completed": [node_id],
                "failed_hop": None, "speed_clamps": []}

    def move_to_named_location(location_name, speed=None):
        gate(target_pose=location_name, action="move.location")
        return True

    mc.move_to_node.side_effect = move_to_node
    mc.travel_to_node.side_effect = travel_to_node
    mc.move_to_named_location.side_effect = move_to_named_location
    mc.filter_sash_gated_targets.side_effect = lambda targets: interlock.filter_targets(
        targets, zone=interlock.zone(graph),
        current_node=mc.current_node, current_rail=mc.last_rail_location_name,
    )
    return mc


@pytest.fixture
def closed_client(monkeypatch):
    """Sash at 3, arm outside the region."""
    mc = _make_controller(3)
    from src.core.xarm_api_server import app
    monkeypatch.setattr("src.core.xarm_api_server.controller", mc)
    with TestClient(app) as c:
        yield c, mc


@pytest.fixture
def open_client(monkeypatch):
    mc = _make_controller(5)
    from src.core.xarm_api_server import app
    monkeypatch.setattr("src.core.xarm_api_server.controller", mc)
    with TestClient(app) as c:
        yield c, mc


@pytest.fixture
def inside_client(monkeypatch):
    """Sash at 3, arm parked *inside* the hood -- the stuck-arm case."""
    mc = _make_controller(3, current_node="hood_filter_high", current_rail="Hood")
    from src.core.xarm_api_server import app
    monkeypatch.setattr("src.core.xarm_api_server.controller", mc)
    with TestClient(app) as c:
        yield c, mc


def _claim(client):
    resp = client.post("/control/claim", json={"owner": "t", "session_id": "s1"})
    return {"X-Claim-Token": resp.json()["claim_token"]}


# ── The 412, on every motion surface ─────────────────────────────────

MOTION_CALLS = [
    ("/control/graph/move_to", {"node_id": "hood_home"}),
    ("/control/graph/travel_to", {"node_id": "hood_home"}),
    ("/move/location", {"location_name": "hood_home"}),
]


@pytest.mark.parametrize("path,body", MOTION_CALLS)
def test_gated_move_returns_412_with_the_full_body(closed_client, path, body):
    client, _ = closed_client
    headers = _claim(client)
    resp = client.post(path, json=body, headers=headers)
    assert resp.status_code == 412, resp.text
    detail = resp.json()["detail"]
    assert detail["error"] == "interlock_not_satisfied"
    assert detail["interlock"] == "fume_hood_sash"
    assert detail["required"] == "sash_position=5"
    assert detail["observed"] == 3
    assert detail["state"] == "blocked"
    assert "override" in detail["hint"].lower()
    # Recovery is operator-driven (someone must move the sash), so no elapsed
    # time clears it -- §6.1 wants retry_after_s null in exactly this case.
    assert detail["retry_after_s"] is None


@pytest.mark.parametrize("path,body", MOTION_CALLS)
def test_the_same_calls_succeed_with_the_sash_open(open_client, path, body):
    client, _ = open_client
    headers = _claim(client)
    resp = client.post(path, json=body, headers=headers)
    assert resp.status_code == 200, resp.text


def test_non_gated_target_still_moves_while_the_sash_is_closed(closed_client):
    client, _ = closed_client
    headers = _claim(client)
    resp = client.post(
        "/control/graph/move_to", json={"node_id": "cytation_home"}, headers=headers
    )
    assert resp.status_code == 200, resp.text


def test_assistant_execute_returns_412_not_a_flattened_409(closed_client):
    """The route catches GraphError; SashInterlockError must be caught first.

    Without the explicit clause the operator would get 409 "step_invalid" with
    a stringified reason -- no observed position, no hint, no way out.
    """
    client, _ = closed_client
    headers = _claim(client)
    resp = client.post(
        "/assistant/execute",
        json={"steps": [{"kind": "move", "to": "hood_home"}]},
        headers=headers,
    )
    assert resp.status_code == 412, resp.text
    detail = resp.json()["detail"]
    assert detail["error"] == "interlock_not_satisfied"
    assert detail["observed"] == 3
    assert detail["failed_step"] == 1


def test_app_level_handler_covers_a_route_with_no_explicit_clause():
    """A motion endpoint added later must inherit the 412, not leak a 500.

    Registered against a throwaway app so this asserts the handler itself, not
    any particular route's except clauses.
    """
    from fastapi import FastAPI
    from src.core.xarm_api_server import sash_interlock_handler

    app = FastAPI()
    app.add_exception_handler(SashInterlockError, sash_interlock_handler)

    @app.post("/some/future/move")
    async def future_move():
        raise SashInterlockError(
            action="move.future", reason="sash is at 3", state="blocked",
            required_position=5, observed_position=3, hint="open it",
        )

    with TestClient(app, raise_server_exceptions=False) as c:
        resp = c.post("/some/future/move")
    assert resp.status_code == 412
    assert resp.json()["detail"]["error"] == "interlock_not_satisfied"


def test_plate_linear_refuses_synchronously(closed_client):
    """It answers before the move runs, so the 412 must not be deferred.

    ``/move/plate_linear`` dispatches its move as a background task and returns
    "command accepted" immediately. The controller's own gate would therefore
    raise *after* the response was sent -- the operator would read 200 for a
    move that never happened, with the refusal visible only in the log. So the
    endpoint gates up front as well.

    This path matters more than most: move_plate_linear never consults the
    motion graph, and it is the motion used to descend into the hood
    (hood_shaker_low / hood_filter_low in demo_workflow.py).
    """
    client, _ = closed_client
    headers = _claim(client)
    resp = client.post(
        "/move/plate_linear", json={"target_location": "hood_shaker_low"},
        headers=headers,
    )
    assert resp.status_code == 412, resp.text
    assert resp.json()["detail"]["error"] == "interlock_not_satisfied"


def test_plate_linear_is_accepted_with_the_sash_open(open_client):
    client, _ = open_client
    headers = _claim(client)
    resp = client.post(
        "/move/plate_linear", json={"target_location": "hood_shaker_low"},
        headers=headers,
    )
    assert resp.status_code == 200, resp.text


def test_plate_linear_to_a_non_gated_pose_is_accepted_while_closed(closed_client):
    client, _ = closed_client
    headers = _claim(client)
    resp = client.post(
        "/move/plate_linear", json={"target_location": "cytation_home"},
        headers=headers,
    )
    assert resp.status_code == 200, resp.text


# ── Freehand coverage ────────────────────────────────────────────────

FREEHAND_CALLS = [
    ("/move/position", {"x": 100, "y": 0, "z": 200}),
    ("/move/joints", {"angles": [0, 0, 0, 0, 0]}),
    ("/move/relative", {"dx": 5}),
    ("/velocity/cartesian", {"vx": 5}),
    ("/track/move", {"position": 100}),
]


@pytest.mark.parametrize("path,body", FREEHAND_CALLS)
def test_freehand_motion_refused_while_inside_with_the_sash_closed(
    inside_client, path, body
):
    """These carry no node id, so the controller gate cannot see them.

    Entry is not gateable (we cannot predict where a freehand move ends), but
    an arm already inside the region can be refused -- and that is the case
    where the glass is in its path right now.
    """
    client, _ = inside_client
    headers = _claim(client)
    resp = client.post(path, json=body, headers=headers)
    assert resp.status_code == 412, resp.text
    assert resp.json()["detail"]["error"] == "interlock_not_satisfied"


@pytest.mark.parametrize("path,body", FREEHAND_CALLS)
def test_freehand_motion_allowed_outside_the_region(closed_client, path, body):
    """A closed sash must not stop freehand work elsewhere."""
    client, _ = closed_client
    headers = _claim(client)
    resp = client.post(path, json=body, headers=headers)
    assert resp.status_code != 412, resp.text


@pytest.mark.parametrize("path,body", [
    ("/gripper/open", None),
    ("/gripper/close", None),
    ("/gripper/move/stroke", {"stroke": 100}),
])
def test_gripper_actions_are_a_documented_carve_out(inside_client, path, body):
    """Not gated, on purpose -- and asserted so it reads as a decision.

    The hazard is the arm envelope against the sash glass; a ~40mm jaw stroke
    does not extend that envelope toward the sash. If the sash has already
    closed onto the arm, opening the jaws is harmless -- while gating them
    would add a way to trap a plate for no safety gain, and setting a plate
    down is the one useful thing left to an arm that cannot move.
    """
    client, _ = inside_client
    headers = _claim(client)
    resp = client.post(path, json=body, headers=headers)
    assert resp.status_code != 412, resp.text


# ── Ordering: claims come first ──────────────────────────────────────

def test_a_tokenless_gated_move_is_423_not_412(closed_client):
    """Claim enforcement is a dependency, so it fires before the handler body.

    Same ordering the plateloc reference device documents: 423 ahead of the
    412s. Asserted because a future refactor moving the gate into a dependency
    could silently invert it.
    """
    client, _ = closed_client
    resp = client.post("/control/graph/move_to", json={"node_id": "hood_home"})
    assert resp.status_code == 423, resp.text
    assert resp.json()["detail"]["error"] == "claim_required"


# ── allowed_actions mirroring on the wire ────────────────────────────

def test_status_withholds_gated_targets_while_closed(closed_client):
    from src.core.status_builder import build_status

    client, mc = closed_client
    mc.sash_interlock.evaluate()        # prime the cache; /status never fetches
    mc.graph_mode = MagicMock(value='strict')   # move.<node> enumeration is STRICT-only
    mc.reachable_node_ids.side_effect = lambda: mc.filter_sash_gated_targets(
        ["hood_home", "opentrons_home", "cytation_home"]
    )
    envelope = build_status(mc)
    assert "move.cytation_home" in envelope.allowed_actions
    assert "move.hood_home" not in envelope.allowed_actions
    assert "move.opentrons_home" not in envelope.allowed_actions


def test_status_carries_the_interlock_block_and_message_prefix(closed_client):
    from src.core.status_builder import build_status

    client, mc = closed_client
    mc.sash_interlock.evaluate()
    envelope = build_status(mc)
    block = envelope.details["interlocks"]["fume_hood_sash"]
    assert block["configured"] is True
    assert block["state"] == "blocked"
    assert block["observed_position"] == 3
    assert block["required_position"] == 5
    assert envelope.message.startswith("[SASH-CLOSED]")
    # A neighbouring device's sash is not this arm's ill health (§2.2), and
    # under fail-open its capability is not even reduced -- only supervised.
    assert envelope.equipment_status != "degraded"


def test_status_never_fetches(monkeypatch):
    """build_status is side-effect-free and polled every 2-3s by the aggregator.

    An interlock that probed from there would turn every dashboard poll into an
    outbound call on another device -- so this asserts the fetcher is never
    reached, rather than merely that the call is fast.
    """
    from src.core.status_builder import build_status

    def explode(url, timeout):
        raise AssertionError("build_status must never fetch the sash device")

    mc = _make_controller(5)
    mc.sash_interlock = SashInterlock(
        {"base_url": "http://sash.invalid"}, fetcher=explode
    )
    mc.reachable_node_ids.side_effect = lambda: ["hood_home"]
    envelope = build_status(mc)        # must not raise
    assert envelope.details["interlocks"]["fume_hood_sash"]["configured"] is True


# ── Operator endpoints ───────────────────────────────────────────────

def test_interlocks_sash_is_readable_without_a_claim(closed_client):
    client, _ = closed_client
    resp = client.get("/interlocks/sash")
    assert resp.status_code == 200, resp.text
    body = resp.json()
    assert body["configured"] is True
    assert body["required_position"] == 5


def test_interlocks_sash_refresh_forces_a_probe(closed_client):
    client, mc = closed_client
    resp = client.get("/interlocks/sash", params={"refresh": "true"})
    assert resp.status_code == 200, resp.text
    assert resp.json()["observed_position"] == 3


def test_interlocks_sash_answers_before_connect(monkeypatch):
    """An operator must be able to test the link with no arm attached.

    Otherwise diagnosing "why was my move refused" requires a connected arm.
    """
    from src.core.xarm_api_server import app
    monkeypatch.setattr("src.core.xarm_api_server.controller", None)
    monkeypatch.setattr("src.core.xarm_api_server._standalone_sash_interlock", None)
    with TestClient(app) as c:
        resp = c.get("/interlocks/sash")
    assert resp.status_code == 200, resp.text
    assert resp.json()["connected"] is False


def test_override_unblocks_a_stuck_arm_then_expires(inside_client):
    """The full operator round-trip: stuck -> override -> move -> re-locked."""
    client, mc = inside_client
    headers = _claim(client)

    blocked = client.post(
        "/control/graph/move_to", json={"node_id": "robot_home"}, headers=headers
    )
    assert blocked.status_code == 412, blocked.text

    granted = client.post(
        "/control/interlocks/sash/override",
        json={"reason": "walking the arm out of the hood", "ttl_seconds": 60},
        headers=headers,
    )
    assert granted.status_code == 200, granted.text
    assert granted.json()["granted_seconds"] == 60

    moved = client.post(
        "/control/graph/move_to", json={"node_id": "robot_home"}, headers=headers
    )
    assert moved.status_code == 200, moved.text

    cleared = client.post(
        "/control/interlocks/sash/override/clear", headers=headers
    )
    assert cleared.status_code == 200, cleared.text
    again = client.post(
        "/control/graph/move_to", json={"node_id": "robot_home"}, headers=headers
    )
    assert again.status_code == 412, "clearing the override did not restore the gate"


def test_override_requires_a_reason(inside_client):
    client, _ = inside_client
    headers = _claim(client)
    for body in ({"reason": ""}, {}):
        resp = client.post(
            "/control/interlocks/sash/override", json=body, headers=headers
        )
        assert resp.status_code == 422, resp.text


def test_override_is_capped(inside_client):
    client, mc = inside_client
    headers = _claim(client)
    resp = client.post(
        "/control/interlocks/sash/override",
        json={"reason": "long one", "ttl_seconds": 99999},
        headers=headers,
    )
    assert resp.status_code == 200, resp.text
    assert resp.json()["granted_seconds"] == mc.sash_interlock.override_max_seconds


def test_override_requires_a_claim(inside_client):
    """It enables motion, so unlike /control/stop it must not be claim-free."""
    client, _ = inside_client
    resp = client.post(
        "/control/interlocks/sash/override", json={"reason": "no claim held"}
    )
    assert resp.status_code == 423, resp.text


def test_override_reports_in_status(inside_client):
    from src.core.status_builder import build_status

    client, mc = inside_client
    headers = _claim(client)
    client.post(
        "/control/interlocks/sash/override",
        json={"reason": "audited reason", "ttl_seconds": 60},
        headers=headers,
    )
    envelope = build_status(mc)
    block = envelope.details["interlocks"]["fume_hood_sash"]
    assert block["state"] == "overridden"
    assert block["override"]["reason"] == "audited reason"
    assert envelope.message.startswith("[SASH-OVERRIDE]")
