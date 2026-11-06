"""Offline tests for graph moves and the vial routines (routines.py).

The arm is test_motion's fake (toy linear kinematics: TCP x/y/z follow
J1-J3 at 300 mm per radian). The graph and the rack are fixture geometry
inside that toy workspace, not the Gibbie bench; the committed Gibbie files
are only checked for validity and for the paths they produce. The gripper is
a fake that reports a part between the fingers when told one is there.
"""

import json
import math

import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from robot_motion.app import create_app
from robot_motion.drivers.robotiq import GripperSettings, RobotiqGripper
from robot_motion.graph import Graph
from robot_motion.routines import RoutineSettings
from tests_robot_motion.test_motion import HEADERS, SECRET, Rig, connected, settings, wait_for

pytest.importorskip("scipy")

DEPLOY = __import__("pathlib").Path(__file__).resolve().parents[1] / "deploy" / "ur3e_gibbie"
MM_PER_DEG = 300 * math.pi / 180


def node(name, j1, j2, j3):
    joints = [j1, j2, j3, 0.0, 0.0, 0.0]
    tcp = [300 + MM_PER_DEG * j1, MM_PER_DEG * j2, 300 + MM_PER_DEG * j3, 0.0, 0.0, 0.0]
    return {"id": name, "joints_deg": joints, "tcp_mm_rpy_deg": tcp}


GRAPH = {
    "schema_version": "robot-motion/1",
    "robot_model": "ur5e",
    "nodes": [
        node("home", 0, 0, 0),
        node("approach", 5, 0, 0),
        node("hub", 0, 10, 0),
        node("pan_above", 0, 10, -5),
        node("pan", 0, 10, -10),
        node("pick_above", 0, 12, -5),
        node("pick", 0, 12, -10),
    ],
    "edges": [
        {"source": a, "target": b, "mode": m}
        for a, b, m in [
            ("home", "approach", "linear"), ("approach", "home", "joint"),
            ("home", "hub", "joint"), ("hub", "home", "joint"),
            ("hub", "pan_above", "linear"), ("pan_above", "pan", "linear"),
            ("pan", "pan_above", "linear"), ("pan_above", "hub", "linear"),
            ("hub", "pick_above", "linear"), ("pick_above", "pick", "linear"),
            ("pick", "pick_above", "linear"), ("pick_above", "hub", "linear"),
        ]
    ],
}
A1 = [340.0, 0.0, 300.0, 0.0, 0.0, 0.0]
ROUTINES = {
    "home_node": "home",
    "vial_rack": {
        "approach_node": "approach", "a1_tcp_mm_rpy_deg": A1, "columns": 2, "rows": 3,
        "column_pitch_mm": 10, "row_pitch_mm": 10, "pick_depth_mm": 20, "place_depth_mm": 15,
    },
    "vial_grip": {"open_raw": 207, "close_raw": 242},
    "balance": {"drop_node": "pan", "pick_node": "pick", "status_url": "http://balance.invalid/status"},
}
OPEN, CLOSE = 207, 242


class FakeGripper:
    """Just what GripperControl and /status read. `part` is whether a vial
    sits between the fingers where they close."""

    def __init__(self):
        self.settings = GripperSettings(
            model="robotiq_2f85", open_raw=3, closed_raw=228, default_speed_pct=30, default_force_pct=10,
            max_force_pct=20,
        )
        self.part = True
        self.holding = False
        self.moves = []
        self.busy = False
        self.requested = None
        self.forces = []
        self.touch_below = None  # a raw the vial stops the fingers at (stroke grips)
        self.position = 3  # where the fingers are (raw), as the Robotiq reports POS

    pct_to_raw = staticmethod(RobotiqGripper.pct_to_raw)

    def start(self):
        pass

    def close(self):
        pass

    def request_stop(self):
        return False

    def summary(self):
        return {
            "model": self.settings.model, "name": self.settings.name, "reachable": True, "error": None,
            "busy": False, "state": "active", "activated": True, "fault_code": 0, "fault": None,
            "object": "contact_closing" if self.holding else "at_requested_position",
            "requested_raw": self.requested,
            "position_raw": self.position,
        }

    def move(self, position_raw, speed_raw, force_raw):
        self.moves.append(position_raw)
        self.forces.append(force_raw)
        self.requested = position_raw
        if self.touch_below is not None and position_raw >= self.touch_below and self.part:
            self.holding = True
            self.position = self.touch_below - 5
            return {"position_raw": self.touch_below - 5, "requested_raw": position_raw, "opening_mm": 2.0,
                    "object": "contact_closing", "object_detected": True, "elapsed_s": 0.01}
        closing = position_raw >= CLOSE
        self.holding = closing and self.part
        self.position = 200 if self.holding else position_raw
        return {
            "position_raw": 200 if self.holding else position_raw,
            "requested_raw": position_raw,
            "opening_mm": 0.0,
            "object": "contact_closing" if self.holding else "at_requested_position",
            "object_detected": self.holding,
            "elapsed_s": 0.01,
        }


class Bench:
    def __init__(self, tmp_path, routines=ROUTINES, graph=GRAPH):
        (tmp_path / "graph.json").write_text(json.dumps(graph))
        (tmp_path / "routines.json").write_text(json.dumps(routines))
        self.rig = Rig(
            settings(
                graph_file=str(tmp_path / "graph.json"),
                routines_file=str(tmp_path / "routines.json"),
                gripper={"model": "robotiq_2f85"},
            )
        )
        self.gripper = FakeGripper()
        self.door = "open"
        self.door_reads = 0

    async def read_door(self, port):
        self.door_reads += 1
        if isinstance(self.door, Exception):
            raise self.door
        return self.door

    def client(self):
        return TestClient(
            create_app(
                self.rig.config,
                observer=self.rig.observer,
                control_session_factory=self.rig.session,
                edge_secret=SECRET,
                gripper=self.gripper,
                balance_door_reader=self.read_door,
            )
        )

    def targets(self):
        """Commanded moves as (kind, TCP mm rounded) in order."""
        out = []
        for kind, target, *_ in self.rig.control.moves:
            if kind == "moveL":
                out.append(("L", [round(v * 1000, 1) for v in target[:3]]))
            else:
                joints = [round(math.degrees(v), 3) for v in target]
                out.append(("J", joints[:3]))
        return out


def mm(name):
    n = next(n for n in GRAPH["nodes"] if n["id"] == name)
    return [round(v, 1) for v in n["tcp_mm_rpy_deg"][:3]]


def joints(name):
    n = next(n for n in GRAPH["nodes"] if n["id"] == name)
    return [round(v, 3) for v in n["joints_deg"][:3]]


# ── graph moves ──────────────────────────────────────────────────────
def test_graph_move_walks_the_taught_edges_one_arm_move_each(tmp_path):
    bench = Bench(tmp_path)
    with bench.client() as client:
        held = connected(client)
        status = client.get("/status").json()
        assert "graph.move" in status["allowed_actions"]
        assert "routine.vial_rack_to_balance" in status["allowed_actions"]
        assert status["details"]["routines"]["nodes"][0] == "home"

        plan = client.post("/graph/move", json={"target": "pan", "dry_run": True}, headers=held)
        assert plan.status_code == 200 and [s["to"] for s in plan.json()["plan"]] == ["hub", "pan_above", "pan"]
        assert bench.rig.control.moves == []

        moved = client.post("/graph/move", json={"target": "pan"}, headers=held)
        assert moved.status_code == 200, moved.text
        assert moved.json()["steps"] == ["hub", "pan_above", "pan"]
        assert bench.targets() == [("J", joints("hub")), ("L", mm("pan_above")), ("L", mm("pan"))]
        # Back the way it was taught, and the arm is then at home again.
        assert client.post("/graph/move", json={"target": "home"}, headers=held).status_code == 200
        assert client.post("/graph/move", json={"target": "home", "dry_run": True}, headers=held).json()["plan"] == []


def test_graph_move_needs_the_arm_at_a_node_and_a_known_target(tmp_path):
    bench = Bench(tmp_path)
    with bench.client() as client:
        held = connected(client)
        assert client.post("/graph/move", json={"target": "nowhere"}, headers=held).status_code == 422
        client.post("/control/freehand/joints", json={"angles": [3, 3, 3, 0, 0, 0]}, headers=held)
        sent = len(bench.rig.control.moves)
        refused = client.post("/graph/move", json={"target": "pan"}, headers=held)
        assert refused.status_code == 412
        assert refused.json()["detail"]["error"] == "not_at_node"
        assert refused.json()["detail"]["nearest"] in {"home", "approach"}
        assert len(bench.rig.control.moves) == sent


def test_graph_move_needs_a_claim_and_a_session(tmp_path):
    bench = Bench(tmp_path)
    with bench.client() as client:
        wait_for(lambda: client.get("/status").json()["details"].get("robotmode"))
        assert client.post("/graph/move", json={"target": "pan"}, headers=HEADERS).status_code == 423
        token = client.post(
            "/control/claim", json={"owner": "x", "session_id": "s", "ttl_s": 60}, headers=HEADERS
        ).json()["claim_token"]
        no_session = client.post("/graph/move", json={"target": "pan"}, headers={**HEADERS, "X-Claim-Token": token})
        assert no_session.status_code == 409 and no_session.json()["detail"]["error"] == "no_control_session"


# ── vial routines ────────────────────────────────────────────────────
def test_rack_to_balance_picks_from_the_slot_and_leaves_the_vial_on_the_pan(tmp_path):
    bench = Bench(tmp_path)
    with bench.client() as client:
        held = connected(client)
        done = client.post("/routines/vial/rack_to_balance", json={"slot": "B2"}, headers=held)
        assert done.status_code == 200, done.text
        b2 = [330.0, -10.0, 300.0]
        assert bench.targets() == [
            ("L", mm("approach")),
            ("L", b2), ("L", [330.0, -10.0, 280.0]), ("L", b2),  # down 20, grip, up
            ("L", mm("approach")),
            ("J", joints("home")), ("J", joints("hub")), ("L", mm("pan_above")), ("L", mm("pan")),
            ("L", mm("pan_above")), ("L", mm("hub")), ("J", joints("home")),
        ]
        assert bench.gripper.moves == [OPEN, CLOSE, OPEN]
        assert bench.door_reads == 1
        assert done.json()["slot"] == "B2"


def test_balance_to_rack_returns_the_vial_and_lifts_straight_out(tmp_path):
    bench = Bench(tmp_path)
    with bench.client() as client:
        held = connected(client)
        done = client.post("/routines/vial/balance_to_rack", json={"slot": "A1"}, headers=held)
        assert done.status_code == 200, done.text
        a1 = A1[:3]
        assert bench.targets() == [
            ("J", joints("hub")), ("L", mm("pick_above")), ("L", mm("pick")),
            ("L", mm("pick_above")), ("L", mm("hub")), ("J", joints("home")), ("L", mm("approach")),
            ("L", a1), ("L", [340.0, 0.0, 285.0]), ("L", a1),  # down 15, release, straight up
            ("L", mm("approach")), ("J", joints("home")),
        ]
        assert bench.gripper.moves == [OPEN, CLOSE, OPEN]


@pytest.mark.parametrize(
    "setup, status, error",
    [
        ("off_home", 412, "not_at_node"),
        ("door_closed", 412, "balance_door_not_open"),
        ("door_unreadable", 412, "balance_door_unknown"),
        ("holding", 412, "gripper_holding"),
    ],
)
def test_routine_preconditions_refuse_before_anything_moves(tmp_path, setup, status, error):
    bench = Bench(tmp_path)
    with bench.client() as client:
        held = connected(client)
        if setup == "off_home":
            client.post("/graph/move", json={"target": "hub"}, headers=held)
        elif setup == "door_closed":
            bench.door = "closed"
        elif setup == "door_unreadable":
            bench.door = OSError("connection refused")
        elif setup == "holding":
            bench.gripper.holding = True
        sent = len(bench.rig.control.moves)
        refused = client.post("/routines/vial/rack_to_balance", json={"slot": "A1"}, headers=held)
        assert refused.status_code == status
        assert refused.json()["detail"]["error"] == error
        assert len(bench.rig.control.moves) == sent and bench.gripper.moves == []


def test_unknown_slot_is_422_and_lists_the_rack(tmp_path):
    bench = Bench(tmp_path)
    with bench.client() as client:
        held = connected(client)
        refused = client.post("/routines/vial/rack_to_balance", json={"slot": "C1"}, headers=held)
        assert refused.status_code == 422
        assert refused.json()["detail"]["slots"] == ["A1", "A2", "A3", "B1", "B2", "B3"]
        plan = client.post("/routines/vial/rack_to_balance", json={"slot": "A3", "dry_run": True}, headers=held)
        assert plan.status_code == 200 and plan.json()["plan"][2]["pose"][:3] == [340.0, -20.0, 300.0]
        assert bench.rig.control.moves == [] and bench.door_reads == 0


def test_closing_on_nothing_stops_the_routine_where_it_gripped(tmp_path):
    bench = Bench(tmp_path)
    bench.gripper.part = False
    with bench.client() as client:
        held = connected(client)
        failed = client.post("/routines/vial/rack_to_balance", json={"slot": "A1"}, headers=held)
        assert failed.status_code == 500
        body = failed.json()
        assert body["error"] == "routine_failed" and body["cause"] == "no_object_gripped"
        assert body["step"] == "vial_close" and body["arm_may_have_moved"] is True
        assert body["completed_steps"] == ["vial_open", "approach", "slot_A1", "slot_A1_grip"]
        # Nothing moved after the empty grip.
        assert bench.targets()[-1] == ("L", [340.0, 0.0, 280.0])
        assert bench.gripper.moves == [OPEN, CLOSE]


def test_a_refused_move_mid_routine_reports_what_already_ran(tmp_path):
    bench = Bench(tmp_path)
    control = bench.rig.control
    # The controller's joint limits refuse the first joint move (planning a
    # joint move is what consults them): the walk back to home.
    control.on_plan = lambda: setattr(control, "joints_ok", len(control.moves) < 3)
    with bench.client() as client:
        held = connected(client)
        refused = client.post("/routines/vial/rack_to_balance", json={"slot": "A1"}, headers=held)
        detail = refused.json()["detail"]
        assert detail["error"] == "outside_controller_limits"
        assert detail["routine"] == "routine.vial_rack_to_balance"
        assert detail["step"] == "home" and detail["arm_may_have_moved"] is True
        assert detail["completed_steps"] == [
            "vial_open", "approach", "slot_A1", "slot_A1_grip", "vial_close", "slot_A1", "approach",
        ]
        assert len(control.moves) == 5  # nothing sent for the refused step or after it
        assert client.get("/status").json()["details"]["routines"]["running"] is None


# ── balance to a node, and from there to the rack ────────────────────
def test_balance_to_node_carries_the_vial_to_the_target_and_keeps_holding_it(tmp_path):
    bench = Bench(tmp_path)
    with bench.client() as client:
        held = connected(client)
        actions = client.get("/status").json()["allowed_actions"]
        assert {"routine.vial_balance_to_node", "routine.vial_node_to_rack"} <= set(actions)
        plan = client.post("/routines/vial/balance_to_node", json={"target": "hub", "dry_run": True}, headers=held)
        assert [s["to"] for s in plan.json()["plan"]] == [
            "vial_open", "hub", "pick_above", "pick", "vial_close", "pick_above", "hub",
        ]
        assert bench.rig.control.moves == [] and bench.door_reads == 0

        done = client.post("/routines/vial/balance_to_node", json={"target": "hub"}, headers=held)
        assert done.status_code == 200, done.text
        assert done.json()["target"] == "hub"
        assert bench.targets() == [
            ("J", joints("hub")), ("L", mm("pick_above")), ("L", mm("pick")),
            ("L", mm("pick_above")), ("L", mm("hub")),
        ]
        assert bench.gripper.moves == [OPEN, CLOSE] and bench.gripper.holding
        # Holding a vial away from home: the home-start routines refuse.
        refused = client.post("/routines/vial/rack_to_balance", json={"slot": "A1"}, headers=held)
        assert refused.status_code == 412 and refused.json()["detail"]["error"] == "not_at_node"


def test_node_to_rack_puts_the_held_vial_in_the_slot_from_where_the_arm_is(tmp_path):
    bench = Bench(tmp_path)
    with bench.client() as client:
        held = connected(client)
        assert client.post("/routines/vial/balance_to_node", json={"target": "hub"}, headers=held).status_code == 200
        sent = len(bench.rig.control.moves)
        done = client.post("/routines/vial/node_to_rack", json={"slot": "A2"}, headers=held)
        assert done.status_code == 200, done.text
        assert done.json()["source"] == "hub" and done.json()["slot"] == "A2"
        a2 = [340.0, -10.0, 300.0]
        assert bench.targets()[sent:] == [
            ("J", joints("home")), ("L", mm("approach")),
            ("L", a2), ("L", [340.0, -10.0, 285.0]), ("L", a2),  # down 15, release, straight up
            ("L", mm("approach")), ("J", joints("home")),
        ]
        assert bench.gripper.moves == [OPEN, CLOSE, OPEN] and not bench.gripper.holding


def test_balance_to_rack_is_the_pick_then_node_to_rack_from_the_pick_node(tmp_path):
    bench = Bench(tmp_path)
    with bench.client() as client:
        held = connected(client)
        whole = client.post("/routines/vial/balance_to_rack", json={"slot": "B3", "dry_run": True}, headers=held)
        to_pick = client.post("/routines/vial/balance_to_node", json={"target": "pick_above", "dry_run": True}, headers=held)
        pick = [s["to"] for s in to_pick.json()["plan"]][:5]
        assert [s["to"] for s in whole.json()["plan"]][:5] == pick


@pytest.mark.parametrize(
    "route, body, setup, status, error",
    [
        ("balance_to_node", {"target": "pick"}, None, 422, "target_is_pick_node"),
        ("balance_to_node", {"target": "nowhere"}, None, 422, "unknown_node"),
        ("balance_to_node", {"target": "hub"}, "door_closed", 412, "balance_door_not_open"),
        ("balance_to_node", {"target": "hub"}, "holding", 412, "gripper_holding"),
        ("node_to_rack", {"slot": "A1"}, None, 412, "gripper_empty"),
        ("node_to_rack", {"slot": "A1"}, "off_node", 412, "not_at_node"),
        ("node_to_rack", {"slot": "Z9"}, "holding", 422, "unknown_slot"),
        ("node_to_rack", {"slot": "A1"}, "holding_door_closed", 412, "balance_door_not_open"),
    ],
)
def test_balance_to_node_and_node_to_rack_refuse_before_anything_moves(tmp_path, route, body, setup, status, error):
    bench = Bench(tmp_path)
    with bench.client() as client:
        held = connected(client)
        if setup == "off_node":
            client.post("/control/freehand/joints", json={"angles": [3, 3, 3, 0, 0, 0]}, headers=held)
        if setup in ("holding", "holding_door_closed", "off_node"):
            bench.gripper.holding = True
        if setup in ("door_closed", "holding_door_closed"):
            bench.door = "closed"
        sent = len(bench.rig.control.moves)
        refused = client.post(f"/routines/vial/{route}", json=body, headers=held)
        assert refused.status_code == status, refused.text
        assert refused.json()["detail"]["error"] == error
        assert len(bench.rig.control.moves) == sent and bench.gripper.moves == []


# ── configuration ────────────────────────────────────────────────────
def test_routines_must_name_graph_nodes(tmp_path):
    with pytest.raises(ValueError, match="missing from the graph"):
        Bench(tmp_path, routines={**ROUTINES, "home_node": "nowhere"}).client()


def test_routines_file_needs_a_graph(tmp_path):
    (tmp_path / "routines.json").write_text(json.dumps(ROUTINES))
    with pytest.raises(ValueError, match="needs a graph_file"):
        create_app(settings(routines_file=str(tmp_path / "routines.json")), edge_secret=SECRET)


def test_a_grip_must_close_past_where_it_opens():
    with pytest.raises(ValidationError):
        RoutineSettings.model_validate({**ROUTINES, "vial_grip": {"open_raw": 242, "close_raw": 207}})


def test_the_gibbie_files_reproduce_the_workflows_paths():
    graph = Graph.model_validate_json((DEPLOY / "graph.json").read_text())
    routines = RoutineSettings.model_validate_json((DEPLOY / "routines.json").read_text())
    routines.check_against(graph)
    # URController.vial_2_balance and balance_2_vial, move for move.
    assert graph.path("vial_stock_prep", "drop_vial") == ["safe_rack", "home_prep_bal", "safe_bal", "prep_viap_drop", "drop_vial"]
    assert graph.path("drop_vial", "safe_rack") == ["prep_viap_drop", "safe_bal", "home_prep_bal", "safe_rack"]
    assert graph.path("safe_rack", "pick_up_vial") == ["home_prep_bal", "safe_bal", "prep_pick_up_vial", "pick_up_vial"]
    assert graph.path("pick_up_vial", "vial_stock_prep") == ["prep_pick_up_vial", "safe_bal", "home_prep_bal", "safe_rack", "vial_stock_prep"]
    rack = routines.vial_rack
    assert len(rack.slots()) == 24 and rack.slot_pose("A1") == list(rack.a1_tcp_mm_rpy_deg)
    assert rack.slot_pose("D6")[:2] == pytest.approx([483.7 - 60, 122.41 - 100])
    # The workflow's open (0.81 of 255); the close is the 4 mm stroke grip.
    assert (routines.vial_grip.open_raw, routines.vial_grip.close_raw) == (207, 217)


def test_a_joint_a_whole_turn_off_the_node_is_refused_with_the_turns_named(tmp_path):
    """Wrist 3 on the UR3e turns without limit, so 360° past home folds to the
    same pose; a moveJ to the taught angle would spin it the whole way round."""
    bench = Bench(tmp_path)
    with bench.client() as client:
        held = connected(client)
        bench.rig.world.q[5] += 2 * math.pi
        probe = lambda: client.post("/graph/move", json={"target": "pan", "dry_run": True}, headers=held)
        wait_for(lambda: probe().status_code == 412, what="the feedback to see the extra turn")
        sent = len(bench.rig.control.moves)
        for route, body in (
            ("/graph/move", {"target": "pan"}),
            ("/routines/vial/rack_to_balance", {"slot": "A1"}),
        ):
            refused = client.post(route, json=body, headers=held)
            assert refused.status_code == 412, refused.text
            detail = refused.json()["detail"]
            assert detail["error"] == "joint_turn_offset"
            assert detail["at"] is None and detail["nearest"] == "home"
            assert detail["turns"] == {"j6": 1}
            assert detail["nearest_deviation_deg"] == pytest.approx(0.0, abs=1e-6)
            assert "j6 by +1 turn" in detail["reason"]
        assert len(bench.rig.control.moves) == sent
        # Unwound, the same pose is home again and the move goes ahead.
        bench.rig.world.q[5] -= 2 * math.pi
        wait_for(lambda: probe().status_code == 200, what="the feedback to see the joint unwound")
        assert probe().json()["from"] == "home"


# ── edge speeds ──────────────────────────────────────────────────────
def speeded(**speeds):
    """GRAPH with speeds on some edges, keyed "a>b"."""
    graph = json.loads(json.dumps(GRAPH))
    for e in graph["edges"]:
        key = f"{e['source']}>{e['target']}"
        if key in speeds:
            e["speed"] = speeds[key]
    return graph


def commanded_speeds(bench):
    """deg/s for moveJ, mm/s for moveL, in order."""
    return [
        round(math.degrees(speed), 3) if kind == "moveJ" else round(speed * 1000, 3)
        for kind, _target, speed, *_ in bench.rig.control.moves
    ]


def test_each_edge_runs_at_its_own_speed_and_others_at_the_default(tmp_path):
    bench = Bench(tmp_path, graph=speeded(**{"home>hub": 8, "hub>pan_above": 30}))
    with bench.client() as client:
        held = connected(client)
        plan = client.post("/graph/move", json={"target": "pan", "dry_run": True}, headers=held).json()["plan"]
        assert [s.get("speed") for s in plan] == [8, 30, None]
        assert client.post("/graph/move", json={"target": "pan"}, headers=held).status_code == 200
        # MOTION's defaults are a quarter of its caps: 5 deg/s, 25 mm/s.
        assert commanded_speeds(bench) == [8.0, 30.0, 25.0]


def test_an_edge_faster_than_the_motion_block_is_refused_at_load(tmp_path):
    with pytest.raises(ValueError, match=r"home -> hub: speed 21 deg/s is above the motion block's 20 deg/s"):
        Bench(tmp_path, graph=speeded(**{"home>hub": 21})).client()
    with pytest.raises(ValueError, match="above the motion block's 100 mm/s"):
        Bench(tmp_path, graph=speeded(**{"hub>pan_above": 101})).client()


def test_the_gibbie_edges_carry_the_chosen_speeds():
    graph = Graph.model_validate_json((DEPLOY / "graph.json").read_text())
    final = {("prep_viap_drop", "drop_vial"), ("drop_vial", "prep_viap_drop"),
             ("prep_pick_up_vial", "pick_up_vial"), ("pick_up_vial", "prep_pick_up_vial")}
    for e in graph.edges:
        # User's speeds: joints 5 deg/s, pan approaches 5 mm/s, transit 15 mm/s
        # (10 until 2026-11-05).
        want = 5 if e.mode == "joint" or (e.source, e.target) in final else 15
        assert e.speed == want, (e.source, e.target)
    # Inside the UR3e's placeholder motion block (10 deg/s, 50 mm/s).
    assert max(e.speed for e in graph.edges if e.mode == "joint") <= 10
    assert max(e.speed for e in graph.edges if e.mode == "linear") <= 50


# ── the shared panel's Graph Drive ───────────────────────────────────
def test_status_reports_the_node_and_where_the_graph_goes_without_a_session(tmp_path):
    bench = Bench(tmp_path)
    observer, read = bench.rig.observer, bench.rig.observer.read

    def with_joints():
        # The live poll's joints; the fixture observer has none of its own.
        observed = read()
        observed["details"]["current_joints"] = [math.degrees(q) for q in bench.rig.world.q]
        return observed

    observer.read = with_joints
    with bench.client() as client:
        wait_for(lambda: (client.get("/status").json()["details"].get("motion_graph") or {}).get("current_node"),
                 what="the observed node")
        graph = client.get("/status").json()["details"]["motion_graph"]
        assert graph["current_node"] == "home" and graph["graph_mode"] == "off"
        assert graph["reachable_nodes"] == ["approach", "hub"]
        assert graph["travel_targets"] == ["approach", "hub", "pan_above", "pan", "pick_above", "pick"]
        assert graph["allowed_gripper_targets"] == [] and graph["mode_override"] is None

        held = connected(client)
        client.post("/control/freehand/joints", json={"angles": [3, 3, 3, 0, 0, 0]}, headers=held)
        wait_for(lambda: client.get("/status").json()["details"]["motion_graph"]["current_node"] is None,
                 what="the arm off the graph")
        graph = client.get("/status").json()["details"]["motion_graph"]
        assert graph["reachable_nodes"] == [] and graph["travel_targets"] == []
        assert graph["nearest_node"] in {"home", "approach"}


def test_move_to_takes_one_taught_edge_only(tmp_path):
    bench = Bench(tmp_path, graph=speeded(**{"home>hub": 8}))
    with bench.client() as client:
        held = connected(client)
        refused = client.post("/control/graph/move_to", json={"node_id": "pan"}, headers=held)
        assert refused.status_code == 422, refused.text
        assert refused.json()["detail"]["error"] == "not_adjacent"
        assert refused.json()["detail"]["reachable_nodes"] == ["approach", "hub"]
        assert bench.rig.control.moves == []
        moved = client.post("/control/graph/move_to", json={"node_id": "hub"}, headers=held)
        assert moved.status_code == 200, moved.text
        assert moved.json()["path"] == ["home", "hub"]
        assert bench.targets() == [("J", joints("hub"))] and commanded_speeds(bench) == [8.0]


def test_travel_to_walks_the_path_and_a_speed_can_only_slow_it(tmp_path):
    bench = Bench(tmp_path, graph=speeded(**{"home>hub": 8, "hub>pan_above": 30}))
    with bench.client() as client:
        held = connected(client)
        mode = client.post("/control/graph/mode", json={"mode": "strict"}, headers=held)
        assert mode.status_code == 200 and mode.json()["graph_mode"] == "off"
        done = client.post("/control/graph/travel_to", json={"node_id": "pan", "speed": 6}, headers=held)
        assert done.status_code == 200, done.text
        assert done.json()["path"] == ["home", "hub", "pan_above", "pan"]
        assert bench.targets() == [("J", joints("hub")), ("L", mm("pan_above")), ("L", mm("pan"))]
        assert commanded_speeds(bench) == [6.0, 6.0, 6.0]
        back = client.post("/control/graph/travel_to", json={"node_id": "home", "speed": 50}, headers=held)
        assert back.status_code == 200 and back.json()["path"] == ["pan", "pan_above", "hub", "home"]
        # A cap above the edge's speed (or the default) changes nothing.
        assert commanded_speeds(bench)[3:] == [25.0, 25.0, 5.0]


def test_graph_drive_needs_a_claim(tmp_path):
    bench = Bench(tmp_path)
    with bench.client() as client:
        wait_for(lambda: client.get("/status").json()["details"].get("robotmode"))
        for path, body in (
            ("/control/graph/move_to", {"node_id": "hub"}),
            ("/control/graph/travel_to", {"node_id": "pan"}),
            ("/control/graph/recover_to", {"node_id": "home"}),
            ("/control/graph/mode", {"mode": "strict"}),
        ):
            assert client.post(path, json=body, headers=HEADERS).status_code == 423, path


def test_recover_moves_onto_the_nearest_node_only_when_close(tmp_path):
    bench = Bench(tmp_path)
    with bench.client() as client:
        held = connected(client)
        client.post("/control/freehand/joints", json={"angles": [-3, 0, 0, 0, 0, 0]}, headers=held)
        wait_for(lambda: client.get("/graph/nearest").json()["arm_residual_deg"] > 2, what="the arm off home")
        nearest = client.get("/graph/nearest").json()
        assert nearest["suggested_node"] == "home" and nearest["within_tolerance"] is False
        assert nearest["recoverable"] is True and nearest["recover_max_deg"] == 15
        assert nearest["reason"] is None
        assert nearest["closest"][0]["node"] == "home"
        assert nearest["closest"][0]["move_by_deg"] == pytest.approx([3, 0, 0, 0, 0, 0], abs=0.2)

        wrong = client.post("/control/graph/recover_to", json={"node_id": "hub"}, headers=held)
        assert wrong.status_code == 422 and wrong.json()["detail"]["error"] == "not_nearest"
        sent = len(bench.rig.control.moves)
        done = client.post("/control/graph/recover_to", json={"node_id": "home", "force": True}, headers=held)
        assert done.status_code == 200, done.text
        assert done.json()["recovered_to"] == "home" and done.json()["moved"] is True
        assert bench.targets()[sent:] == [("J", joints("home"))]
        again = client.post("/control/graph/recover_to", json={"node_id": "home"}, headers=held)
        assert again.status_code == 200 and again.json()["moved"] is False

        client.post("/control/freehand/joints", json={"angles": [-20, 0, 0, 0, 0, 0]}, headers=held)
        wait_for(lambda: client.get("/graph/nearest").json()["arm_residual_deg"] > 19, what="the arm far off home")
        far_off = client.get("/graph/nearest").json()
        # Nothing within reach: no node is named, but the least far are
        # listed with the move per joint that would bring the arm onto them.
        assert far_off["recoverable"] is False and far_off["suggested_node"] is None
        assert "No node within 15 deg" in far_off["reason"] and "'home'" in far_off["reason"]
        assert [c["node"] for c in far_off["closest"]] == ["home", "hub", "pan_above"]  # 20, 20, 20 deg
        assert far_off["closest"][0]["move_by_deg"] == pytest.approx([20, 0, 0, 0, 0, 0], abs=0.2)
        assert far_off["closest"][0]["worst_joint_deg"] == pytest.approx(20, abs=0.2)
        sent = len(bench.rig.control.moves)
        far = client.post("/control/graph/recover_to", json={"node_id": "home", "force": True}, headers=held)
        assert far.status_code == 412, far.text
        assert far.json()["detail"]["error"] == "too_far_to_recover"
        assert len(bench.rig.control.moves) == sent


def test_recover_refuses_a_joint_a_whole_turn_off(tmp_path):
    bench = Bench(tmp_path)
    with bench.client() as client:
        held = connected(client)
        bench.rig.world.q[5] += 2 * math.pi
        wait_for(lambda: client.get("/graph/nearest").json().get("turns") == {"j6": 1}, what="the extra turn")
        nearest = client.get("/graph/nearest").json()
        assert nearest["recoverable"] is False and nearest["suggested_node"] is None
        assert "j6 +1 turn" in nearest["reason"]
        # Folded, the pose is home: the ranking says so.
        assert nearest["closest"][0]["node"] == "home" and nearest["closest"][0]["worst_joint_deg"] == pytest.approx(0, abs=0.01)
        refused = client.post("/control/graph/recover_to", json={"node_id": "home"}, headers=held)
        assert refused.status_code == 412 and refused.json()["detail"]["error"] == "joint_turn_offset"
        assert bench.rig.control.moves == []


# ── gripper states on the graph ──────────────────────────────────────
def gripping_graph():
    """GRAPH with the vial states settable only at pan and pick."""
    graph = json.loads(json.dumps(GRAPH))
    graph["gripper_catalog"] = {"vial_open": {"raw": OPEN}, "vial_closed": {"raw": CLOSE, "expect_object": True}}
    for n in graph["nodes"]:
        if n["id"] in ("pan", "pick"):
            n["gripper_states"] = ["vial_open", "vial_closed"]
    return graph


def test_the_gripper_changes_only_where_the_graph_allows(tmp_path):
    bench = Bench(tmp_path, graph=gripping_graph())
    with bench.client() as client:
        held = connected(client)
        graph = client.get("/status").json()["details"]["motion_graph"]
        assert graph["current_node"] == "home" and graph["allowed_gripper_targets"] == []
        assert graph["gripper_state"] is None
        assert graph["gripper_catalog"]["vial_closed"] == {"raw": CLOSE, "expect_object": True, "grasp": False}
        refused = client.post("/control/graph/gripper", json={"state": "vial_open"}, headers=held)
        assert refused.status_code == 422, refused.text
        assert refused.json()["detail"]["error"] == "gripper_change_not_allowed"
        assert refused.json()["detail"]["allowed_gripper_states"] == []
        assert bench.gripper.moves == []

        assert client.post("/graph/move", json={"target": "pick"}, headers=held).status_code == 200
        graph = client.get("/status").json()["details"]["motion_graph"]
        assert graph["current_node"] == "pick"
        assert graph["allowed_gripper_targets"] == ["vial_open", "vial_closed"]

        opened = client.post("/control/graph/gripper", json={"state": "vial_open"}, headers=held)
        assert opened.status_code == 200, opened.text
        assert opened.json()["gripper_state"] == "vial_open" and bench.gripper.moves == [OPEN]
        graph = client.get("/status").json()["details"]["motion_graph"]
        assert graph["gripper_state"] == "vial_open" and graph["allowed_gripper_targets"] == ["vial_closed"]

        closed = client.post("/control/graph/gripper", json={"state": "vial_closed"}, headers=held)
        assert closed.status_code == 200, closed.text
        assert bench.gripper.moves == [OPEN, CLOSE] and bench.gripper.holding
        assert client.get("/status").json()["details"]["motion_graph"]["gripper_state"] == "vial_closed"


def test_closing_on_no_vial_at_a_node_is_a_failure(tmp_path):
    bench = Bench(tmp_path, graph=gripping_graph())
    bench.gripper.part = False
    with bench.client() as client:
        held = connected(client)
        client.post("/graph/move", json={"target": "pan"}, headers=held)
        failed = client.post("/control/graph/gripper", json={"state": "vial_closed"}, headers=held)
        assert failed.status_code == 500
        assert failed.json()["cause"] == "no_object_gripped"
        unknown = client.post("/control/graph/gripper", json={"state": "dose_closed"}, headers=held)
        assert unknown.status_code == 422 and unknown.json()["detail"]["error"] == "gripper_change_not_allowed"


def test_a_safe_node_opens_an_empty_gripper_but_never_drops_a_vial(tmp_path):
    graph = gripping_graph()
    for n in graph["nodes"]:
        if n["id"] == "hub":
            n["gripper_states"], n["gripper_release"] = ["vial_open"], False
    bench = Bench(tmp_path, graph=graph)
    with bench.client() as client:
        held = connected(client)

        def allowed():
            return client.get("/status").json()["details"]["motion_graph"]["allowed_gripper_targets"]

        def grip(state):
            return client.post("/control/graph/gripper", json={"state": state}, headers=held)

        assert client.post("/graph/move", json={"target": "hub"}, headers=held).status_code == 200
        assert allowed() == ["vial_open"]
        assert grip("vial_open").status_code == 200 and bench.gripper.moves == [OPEN]

        client.post("/graph/move", json={"target": "pick"}, headers=held)
        assert grip("vial_closed").status_code == 200 and bench.gripper.holding
        client.post("/graph/move", json={"target": "hub"}, headers=held)
        assert allowed() == []
        refused = grip("vial_open")
        assert refused.status_code == 422, refused.text
        assert refused.json()["detail"]["error"] == "gripper_release_not_allowed"
        assert refused.json()["detail"]["allowed_gripper_states"] == []
        assert bench.gripper.moves == [OPEN, CLOSE] and bench.gripper.holding

        # Where the graph allows a release, it still opens.
        client.post("/graph/move", json={"target": "pan"}, headers=held)
        assert grip("vial_open").status_code == 200 and not bench.gripper.holding


def test_a_node_state_missing_from_the_catalog_fails_the_load(tmp_path):
    graph = gripping_graph()
    graph["nodes"][0]["gripper_states"] = ["dose_closed"]
    with pytest.raises(ValueError, match=r"Node home: gripper states not in gripper_catalog: \['dose_closed'\]"):
        Bench(tmp_path, graph=graph).client()


def test_the_gibbie_vial_grip_is_a_4mm_stroke_at_20_percent():
    """The workflow closed to raw 242 (0.95 of 255), past where the fingers
    meet (228): at 50 % that clamp went through the vial into the pan and
    overloaded the balance (2026-11-05). The fingertips cage the vial, so the
    grip is a stroke at no more than 20 %: 2 mm first, then 4 mm (user,
    2026-11-05)."""
    graph = Graph.model_validate_json((DEPLOY / "graph.json").read_text())
    routines = RoutineSettings.model_validate_json((DEPLOY / "routines.json").read_text())
    catalog = graph.gripper_catalog
    assert catalog["vial_open"].raw == routines.vial_grip.open_raw == 207
    assert catalog["vial_closed"].raw == routines.vial_grip.close_raw == 217
    assert catalog["vial_closed"].grasp and not catalog["vial_closed"].expect_object
    assert routines.vial_grip.close_check == "stroke" and routines.grip_force_pct == 20
    # pc-04's calibration (open_raw 3, closed_raw 228 over 85 mm): about 4 mm.
    assert (228 - 217) / (228 - 3) * 85 == pytest.approx(4.0, abs=0.2)
    settable = {n.id: n.gripper_states for n in graph.nodes if n.gripper_states}
    assert settable == {
        "pick_up_vial": ["vial_open", "vial_closed"], "drop_vial": ["vial_open", "vial_closed"],
        # Narrow the fingers before an approach (user, 2026-11-05); never a drop there.
        "safe_rack": ["vial_open"], "safe_bal": ["vial_open"],
    }
    assert {n.id for n in graph.nodes if not n.gripper_release} == {"safe_rack", "safe_bal"}


def test_a_grip_force_above_the_gripper_cap_fails_the_load(tmp_path):
    # FakeGripper's cap is 20 %.
    with pytest.raises(ValueError, match="grip_force_pct 30 is above the gripper's commissioned 20"):
        Bench(tmp_path, routines={**ROUTINES, "grip_force_pct": 30}).client()


def test_vial_grips_use_the_force_set_in_the_routines(tmp_path):
    bench = Bench(tmp_path, routines={**ROUTINES, "grip_force_pct": 20}, graph=gripping_graph())
    with bench.client() as client:
        held = connected(client)
        done = client.post("/routines/vial/balance_to_node", json={"target": "hub"}, headers=held)
        assert done.status_code == 200, done.text
        # Graph Drive's gripper route grips the same way.
        client.post("/graph/move", json={"target": "pick"}, headers=held)
        assert client.post("/control/graph/gripper", json={"state": "vial_open"}, headers=held).status_code == 200
    assert bench.gripper.moves == [OPEN, CLOSE, OPEN]
    assert bench.gripper.forces == [RobotiqGripper.pct_to_raw(20)] * 3




# ── stroke grips (the fingertips cage the vial) ──────────────────────
STROKE = 223
STROKE_ROUTINES = {**ROUTINES, "vial_grip": {"open_raw": OPEN, "close_raw": STROKE, "close_check": "stroke"}}


@pytest.mark.parametrize("touches", [False, True], ids=["fingers_reach_the_stroke", "fingers_stop_on_the_vial"])
def test_a_stroke_grip_is_a_grip_whether_or_not_the_fingers_touch(tmp_path, touches):
    bench = Bench(tmp_path, routines=STROKE_ROUTINES)
    bench.gripper.touch_below = STROKE if touches else None
    with bench.client() as client:
        held = connected(client)
        done = client.post("/routines/vial/balance_to_node", json={"target": "hub"}, headers=held)
        assert done.status_code == 200, done.text
        assert bench.gripper.moves == [OPEN, STROKE]
        # Held at a node, the stroke grip counts as holding: home routines
        # refuse, node_to_rack accepts.
        sent = len(bench.rig.control.moves)
        put_away = client.post("/routines/vial/node_to_rack", json={"slot": "A1"}, headers=held)
        assert put_away.status_code == 200, put_away.text
        assert bench.gripper.moves == [OPEN, STROKE, OPEN] and len(bench.rig.control.moves) > sent


def test_a_stroke_grip_at_home_blocks_the_home_routines(tmp_path):
    bench = Bench(tmp_path, routines=STROKE_ROUTINES)
    with bench.client() as client:
        held = connected(client)
        assert client.post("/routines/vial/balance_to_node", json={"target": "home"}, headers=held).status_code == 200
        refused = client.post("/routines/vial/rack_to_balance", json={"slot": "A1"}, headers=held)
        assert refused.status_code == 412 and refused.json()["detail"]["error"] == "gripper_holding"


# ── rack speeds, catalog names, /control routes ──────────────────────
RACK_SPEEDS = {**ROUTINES, "vial_rack": {**ROUTINES["vial_rack"], "transit_speed_mm_s": 30, "slot_speed_mm_s": 6}}


def test_rack_moves_run_at_the_rack_speeds(tmp_path):
    bench = Bench(tmp_path, routines=RACK_SPEEDS)
    with bench.client() as client:
        held = connected(client)
        plan = client.post("/routines/vial/rack_to_balance", json={"slot": "A1", "dry_run": True}, headers=held)
        steps = [(s["to"], s.get("speed")) for s in plan.json()["plan"][1:7] if s["kind"] != "gripper"]
        # The graph edge home -> approach has no speed (default); then the rack.
        assert steps == [("approach", None), ("slot_A1", 30), ("slot_A1_grip", 6), ("slot_A1", 6), ("approach", 30)]
        done = client.post("/routines/vial/rack_to_balance", json={"slot": "A1"}, headers=held)
        assert done.status_code == 200, done.text
        # approach (graph edge, default 25), above A1 30, down 6, up 6, back to approach 30, ...
        assert commanded_speeds(bench)[:5] == [25.0, 30.0, 6.0, 6.0, 30.0]


def test_a_rack_speed_above_the_motion_block_fails_the_load(tmp_path):
    routines = {**ROUTINES, "vial_rack": {**ROUTINES["vial_rack"], "slot_speed_mm_s": 101}}
    with pytest.raises(ValueError, match="vial_rack slot_speed_mm_s 101 mm/s is above the motion block's 100 mm/s"):
        Bench(tmp_path, routines=routines).client()


def test_the_gibbie_rack_speeds():
    routines = RoutineSettings.model_validate_json((DEPLOY / "routines.json").read_text())
    assert (routines.vial_rack.transit_speed_mm_s, routines.vial_rack.slot_speed_mm_s) == (15, 5)


def test_allowed_actions_carry_the_catalog_names(tmp_path):
    bench = Bench(tmp_path)
    with bench.client() as client:
        connected(client)
        actions = set(client.get("/status").json()["allowed_actions"])
        assert {"graph.move_to", "graph.travel_to", "graph.gripper", "graph.recover_to", "graph.move"} <= actions
        assert {
            "routine.vial_rack_to_balance", "routine.vial_balance_to_rack",
            "routine.vial_balance_to_node", "routine.vial_node_to_rack",
        } <= actions


def test_the_routines_and_graph_move_answer_under_control(tmp_path):
    """The dashboard's passthrough forwards only /control/*, so the catalog
    skills call the /control aliases."""
    bench = Bench(tmp_path)
    with bench.client() as client:
        held = connected(client)
        for path, body in (
            ("/control/routines/vial/rack_to_balance", {"slot": "A1", "dry_run": True}),
            ("/control/routines/vial/balance_to_rack", {"slot": "A1", "dry_run": True}),
            ("/control/routines/vial/balance_to_node", {"target": "hub", "dry_run": True}),
            ("/control/graph/move", {"target": "pan", "dry_run": True}),
        ):
            answer = client.post(path, json=body, headers=held)
            assert answer.status_code == 200 and answer.json()["dry_run"] is True, (path, answer.text)
        assert bench.rig.control.moves == []
        refused = client.post("/control/routines/vial/node_to_rack", json={"slot": "A1"}, headers=held)
        assert refused.status_code == 412 and refused.json()["detail"]["error"] == "gripper_empty"
        assert client.get("/routines").json()["routines"]["vial_balance_to_node"].startswith(
            "POST /control/routines/vial/balance_to_node")
