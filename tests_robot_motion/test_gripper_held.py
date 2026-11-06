"""The held-part guard (held.py): one record for routines, Graph Drive and
freehand gripper commands on a graph with held_part_guard: true."""

import json

from fastapi.testclient import TestClient

from robot_motion.app import create_app
from tests_robot_motion.test_motion import HEADERS, OPERATOR, SECRET, connected
from tests_robot_motion.test_routines import CLOSE, OPEN, Bench, gripping_graph

AGENT = "agent:bob@utoronto.ca"


def guarded_graph():
    """gripping_graph (vial states at pan and pick) plus a safe waypoint, hub,
    that may narrow but never let go, and the guard on."""
    graph = gripping_graph()
    graph["held_part_guard"] = True
    for n in graph["nodes"]:
        if n["id"] == "hub":
            n["gripper_states"], n["gripper_release"] = ["vial_open"], False
    return graph


def guarded_bench(tmp_path, **kw):
    bench = Bench(tmp_path, graph=guarded_graph(), **kw)
    config = bench.rig.config
    bench.rig.config = config.model_copy(
        update={"control": config.control.model_copy(update={"audit_file": str(tmp_path / "audit.jsonl")})}
    )
    return bench


def held(client):
    return client.get("/status").json()["details"]["gripper_held"]


def allowed(client):
    return client.get("/status").json()["allowed_actions"]


def confirm(client, headers):
    return client.post("/gripper/confirm_empty", json={"note": "looked"}, headers=headers)


def opened(client, headers):
    return client.post("/control/freehand/gripper/open", headers=headers)


def test_with_no_saved_state_the_gripper_counts_as_holding_until_a_person_confirms(tmp_path):
    bench = guarded_bench(tmp_path)
    bench.gripper.position = 217  # closed to the vial grip, as after the 2026-11-06 run
    with bench.client() as client:
        h = connected(client)
        state = held(client)
        assert state["held"] is True and state["source"] == "unknown" and "first start" in state["reason"]
        assert "gripper.open" not in allowed(client) and "gripper.move" in allowed(client)

        refused = opened(client, h)
        assert refused.status_code == 422, refused.text
        assert refused.json()["detail"]["error"] == "gripper_release_not_allowed"
        routine = client.post("/routines/vial/balance_to_node", json={"target": "hub"}, headers=h)
        assert routine.status_code == 412 and routine.json()["detail"]["error"] == "gripper_holding"
        assert routine.json()["detail"]["gripper_held"]["source"] == "unknown"
        assert bench.gripper.moves == []

        done = confirm(client, h)
        assert done.status_code == 200, done.text
        assert done.json()["gripper_held"]["held"] is False
        assert done.json()["gripper_held"]["cleared_by"] == OPERATOR
        assert "gripper.open" in allowed(client)
        assert opened(client, h).status_code == 200
    events = [json.loads(line) for line in (tmp_path / "audit.jsonl").read_text().splitlines()]
    attest = [e for e in events if e["kind"] == "gripper.confirm_empty"]
    assert attest and attest[0]["operator"] == OPERATOR and attest[0]["position_raw"] == 217
    assert attest[0]["was"] == "unknown" and attest[0]["note"] == "looked"


def test_a_routine_grip_holds_and_only_a_release_node_lets_go(tmp_path):
    bench = guarded_bench(tmp_path)
    with bench.client() as client:
        h = connected(client)
        confirm(client, h)
        done = client.post("/routines/vial/balance_to_node", json={"target": "hub"}, headers=h)
        assert done.status_code == 200, done.text
        state = held(client)
        assert state["held"] is True and state["source"] == "routine" and state["set_by"] == OPERATOR

        # At the safe waypoint: no letting go, by any route.
        bench.gripper.mm_to_raw = lambda mm: 50
        moves = list(bench.gripper.moves)
        graph = client.get("/status").json()["details"]["motion_graph"]
        assert graph["current_node"] == "hub" and graph["allowed_gripper_targets"] == []
        for refused in (
            opened(client, h),
            client.post("/control/freehand/gripper/stroke", json={"stroke": 60}, headers=h),
            client.post("/control/graph/gripper", json={"state": "vial_open"}, headers=h),
            client.post("/component/enable", json={"component": "gripper"}, headers=h),
        ):
            assert refused.status_code == 422, refused.text
            assert refused.json()["detail"]["error"] == "gripper_release_not_allowed"
        assert "gripper.open" not in allowed(client)
        assert bench.gripper.moves == moves

        # Narrowing is still allowed and keeps it held.
        assert client.post("/control/freehand/gripper/close", headers=h).status_code == 200
        assert held(client)["held"] is True

        # At the pan, a release node, a freehand open lets go and clears it.
        assert client.post("/graph/move", json={"target": "pan"}, headers=h).status_code == 200
        assert "gripper.open" in allowed(client)
        assert opened(client, h).status_code == 200
        state = held(client)
        assert state["held"] is False and state["source"] == "freehand" and state["cleared_by"] == OPERATOR


def test_after_a_confirmation_the_safe_node_offers_vial_open_again(tmp_path):
    bench = guarded_bench(tmp_path)
    with bench.client() as client:
        h = connected(client)
        confirm(client, h)
        client.post("/routines/vial/balance_to_node", json={"target": "hub"}, headers=h)
        bench.gripper.holding = False  # the person took the vial out by hand
        assert confirm(client, h).status_code == 200
        graph = client.get("/status").json()["details"]["motion_graph"]
        assert graph["current_node"] == "hub"
        assert graph["allowed_gripper_targets"] == ["vial_open"]
        assert client.post("/control/graph/gripper", json={"state": "vial_open"}, headers=h).status_code == 200


def test_a_rack_routine_put_down_clears_it(tmp_path):
    bench = guarded_bench(tmp_path)
    with bench.client() as client:
        h = connected(client)
        confirm(client, h)
        client.post("/routines/vial/balance_to_node", json={"target": "hub"}, headers=h)
        assert held(client)["held"] is True
        done = client.post("/routines/vial/node_to_rack", json={"slot": "A1"}, headers=h)
        assert done.status_code == 200, done.text
        state = held(client)
        assert state["held"] is False and state["source"] == "routine"


def test_the_record_survives_a_restart_and_an_unreadable_one_counts_as_holding(tmp_path):
    bench = guarded_bench(tmp_path)
    with bench.client() as client:
        h = connected(client)
        confirm(client, h)
        client.post("/routines/vial/balance_to_node", json={"target": "hub"}, headers=h)
    with bench.client() as client:
        assert held(client)["held"] is True and held(client)["source"] == "routine"
        h = connected(client)
        confirm(client, h)
    with bench.client() as client:
        assert held(client)["held"] is False
    (tmp_path / "gripper-held.json").write_text("{not json")
    with bench.client() as client:
        state = held(client)
        assert state["held"] is True and state["source"] == "unknown" and "unreadable" in state["reason"]


def test_only_a_person_through_the_edge_may_confirm_empty(tmp_path):
    bench = guarded_bench(tmp_path)

    def verify(key):
        return {"email": AGENT, "role": "operator"} if key == "agent-key" else None

    app = create_app(
        bench.rig.config, observer=bench.rig.observer, control_session_factory=bench.rig.session,
        edge_secret=SECRET, gripper=bench.gripper, balance_door_reader=bench.read_door, key_verifier=verify,
    )
    with TestClient(app) as client:
        panel = connected(client)
        assert client.post("/control/release", headers=panel).status_code in (200, 204)
        token = client.post(
            "/control/claim", json={"owner": "x", "session_id": "agent", "ttl_s": 60},
            headers={"X-Api-Key": "agent-key"},
        ).json()["claim_token"]
        agent = {"X-Api-Key": "agent-key", "X-Claim-Token": token}
        refused = confirm(client, agent)
        assert refused.status_code == 403 and refused.json()["detail"]["error"] == "person_required"
        # An "agent:" principal is refused even if it arrived with edge headers.
        spoof = {"X-Auth-User": AGENT, "X-Edge-Auth": SECRET, "X-Claim-Token": token}
        assert confirm(client, spoof).status_code == 403
        assert held(client)["held"] is True
        # Not under /control, so the dashboard passthrough cannot reach it.
        paths = client.get("/openapi.json").json()["paths"]
        assert "/gripper/confirm_empty" in paths
        assert not any(p.startswith("/control/") and "confirm_empty" in p for p in paths)


def test_without_the_guard_nothing_changes(tmp_path):
    bench = Bench(tmp_path, graph=gripping_graph())
    with bench.client() as client:
        h = connected(client)
        assert held(client) is None
        assert opened(client, h).status_code == 200
        assert bench.gripper.moves == [3]
        assert client.post("/gripper/confirm_empty", headers=h).status_code == 404
        assert OPEN < CLOSE
