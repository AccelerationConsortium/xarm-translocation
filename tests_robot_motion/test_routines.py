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
        }

    def move(self, position_raw, speed_raw, force_raw):
        self.moves.append(position_raw)
        closing = position_raw >= CLOSE
        self.holding = closing and self.part
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
    assert (routines.vial_grip.open_raw, routines.vial_grip.close_raw) == (207, 242)


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
