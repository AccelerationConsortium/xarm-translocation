"""The sash interlock against the real motion graph.

Where test_sash_interlock.py tests the decision, this tests *what it applies
to*: which nodes count as inside the fume hood / Opentrons region, and which
moves are refused when the sash is not parked.

The load-bearing test here is ``test_gated_set_is_exactly``. The gated set is
derived from tags and rail names, so an ordinary graph edit -- adding a node,
retagging one, wiring a new edge -- can silently widen or narrow the region a
physical guard covers. Pinning it to an explicit literal means any such change
fails CI and a human has to look at it. That is the actual defence; the config
is only the mechanism.
"""

from __future__ import annotations

import copy
import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core.motion_graph import DEFAULT_PRECONDITIONS, MotionGraph
from src.core.sash_interlock import SashInterlock, SashInterlockError

GRAPH_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "settings", "motion_graph.yaml",
)

# Every node the fume hood sash is allowed to veto. 7 `hood` + 8 `opentrons`.
# If this literal needs editing, stop and ask whether the *physical* region
# changed -- and whether `deck` should now be in gated_tags too (the Hood and
# Deck rail locations are both 550mm).
EXPECTED_GATED = {
    "hood_home",
    "hood_shaker_high",
    "hood_shaker_low",
    "hood_filter_home",
    "hood_filter_high",
    "hood_filter_low",
    "hood_filter_top_plate",
    "opentrons_home",
    "opentrons_2_high",
    "opentrons_2_low",
    "opentrons_4_high",
    "opentrons_4_low_top",
    "opentrons_4_low",
    "opentrons_6_high",
    "opentrons_6_low",
}

SASH_OPEN = {
    "equipment_status": "ready",
    "message": "Sash parked at position 5",
    "components": {
        "actuator": {"connected": True, "state": "idle"},
        "sash": {"connected": True, "state": "position_5"},
    },
    "metrics": {"sash_position": {"value": 5, "unit": "preset"}},
}


def _sash_at(position):
    p = copy.deepcopy(SASH_OPEN)
    p["metrics"]["sash_position"]["value"] = position
    p["components"]["sash"]["state"] = f"position_{position}"
    p["message"] = f"Sash parked at position {position}"
    return p


@pytest.fixture(scope="module")
def graph():
    return MotionGraph.from_yaml(GRAPH_PATH, preconditions=DEFAULT_PRECONDITIONS)


def interlock_for(payload, **cfg):
    config = {"base_url": "http://sash.invalid", "enabled": True}
    config.update(cfg)
    return SashInterlock(config, fetcher=lambda url, timeout: payload)


@pytest.fixture
def closed(graph):
    """An interlock reading a sash at position 3, plus the live zone."""
    interlock = interlock_for(_sash_at(3))
    return interlock, interlock.zone(graph)


@pytest.fixture
def open_sash(graph):
    interlock = interlock_for(SASH_OPEN)
    return interlock, interlock.zone(graph)


# ── The gated set ────────────────────────────────────────────────────

def test_gated_set_is_exactly(graph):
    """Pin the region. See the module docstring for why this one matters."""
    zone = interlock_for(SASH_OPEN).zone(graph)
    assert set(zone.node_ids) == EXPECTED_GATED


def test_gated_set_is_derived_from_config_not_hard_coded(graph):
    """Adding `deck` must be a one-line config change, not a code change.

    The bench check that decides this is open: `Hood` and `Deck` are the same
    550mm rail position, and joint_config notes the arm at Deck can reach the
    fumehood positions.
    """
    zone = interlock_for(SASH_OPEN, gated_tags=["hood", "opentrons", "deck"]).zone(graph)
    deck_nodes = set(graph.nodes_with_tag("deck"))
    assert deck_nodes, "the graph has no deck-tagged nodes; this test is stale"
    assert deck_nodes <= set(zone.node_ids)


def test_rail_clause_catches_an_untagged_node_at_a_gated_rail(graph):
    """POST /control/graph/node takes `tags` as optional.

    So an operator can create a node at rail Hood carrying no `hood` tag. The
    tag clause alone would leave it silently ungated, which is why membership
    is also keyed on the rail.
    """
    raw = {
        "schema_version": "0.2",
        "gripper_states": {"empty": {"stroke": 150, "intent": "none"}},
        "nodes": [
            # transit_home only because the loader requires it on both ends of
            # a cross-rail edge. Neither node carries `hood` -- that is the
            # point of the test.
            {"id": "somewhere", "arm": "somewhere", "rail": "Home",
             "tags": ["transit_home"]},
            {"id": "sneaky", "arm": "sneaky_pose", "rail": "Hood",
             "tags": ["transit_home"]},
        ],
        "edges": [{"from": "somewhere", "to": "sneaky", "mode": "joint"}],
    }
    small = MotionGraph.from_dict(raw, preconditions=DEFAULT_PRECONDITIONS)
    zone = interlock_for(SASH_OPEN).zone(small)
    assert "sneaky" in zone.node_ids


def test_exempt_nodes_are_removed(graph):
    zone = interlock_for(SASH_OPEN, exempt_nodes=["hood_home"]).zone(graph)
    assert "hood_home" not in zone.node_ids
    assert "hood_filter_low" in zone.node_ids


def test_zone_is_recomputed_when_the_graph_is_replaced(graph):
    """The graph object is *replaced* on a hot reload.

    POST /control/graph/{node,edge,edge/create,edge/delete} rewrite the YAML
    and reload, so a memo keyed on anything but object identity would leave a
    newly added hood node ungated until the service restarted.
    """
    interlock = interlock_for(SASH_OPEN)
    first = interlock.zone(graph)
    assert interlock.zone(graph) is first, "should be memoized for one graph"
    reloaded = MotionGraph.from_yaml(GRAPH_PATH, preconditions=DEFAULT_PRECONDITIONS)
    second = interlock.zone(reloaded)
    assert second is not first, "stale zone reused after a graph reload"
    assert set(second.node_ids) == EXPECTED_GATED


# ── Refusals while the sash is closed ────────────────────────────────

@pytest.mark.parametrize("target", ["hood_home", "opentrons_home"])
def test_ingress_is_refused(closed, target):
    """The two edges that cross into the region from outside."""
    interlock, zone = closed
    with pytest.raises(SashInterlockError) as excinfo:
        interlock.gate_move(
            action="graph.move_to", target_node=target,
            current_node="robot_home", zone=zone,
        )
    assert excinfo.value.observed_position == 3


@pytest.mark.parametrize("current,target", [
    ("hood_home", "hood_filter_home"),
    ("hood_filter_home", "hood_filter_high"),
    ("hood_filter_high", "hood_filter_low"),
    ("opentrons_home", "opentrons_4_high"),
    ("opentrons_4_high", "opentrons_4_low"),
])
def test_moving_deeper_inside_is_refused(closed, current, target):
    interlock, zone = closed
    with pytest.raises(SashInterlockError):
        interlock.gate_move(
            action="graph.move_to", target_node=target,
            current_node=current, zone=zone,
        )


@pytest.mark.parametrize("current,target", [
    ("hood_home", "robot_home"),
    ("opentrons_home", "robot_home"),
])
def test_egress_is_also_refused(closed, current, target):
    """No automatic retreat -- deliberately, and this test says so.

    A blind retreat could drag the arm through a descending sash, which is the
    collision the guard exists to prevent, and the interlock cannot see where
    the sash is along the retreat path. So an arm caught inside stays put until
    a human looks at it and overrides.

    This is the behaviour most likely to be "fixed" by a later well-meaning
    change, so it is asserted explicitly rather than left implicit.
    """
    interlock, zone = closed
    with pytest.raises(SashInterlockError) as excinfo:
        interlock.gate_move(
            action="graph.move_to", target_node=target,
            current_node=current, zone=zone,
        )
    assert "all motion is refused" in excinfo.value.reason
    assert "only way to move it" in excinfo.value.hint


def test_override_permits_egress_then_expires(graph):
    """The other half of the pair above: the arm is not permanently trapped."""
    clock_now = [1000.0]
    interlock = SashInterlock(
        {"base_url": "http://sash.invalid", "override_max_seconds": 120},
        fetcher=lambda url, timeout: _sash_at(3),
        clock=lambda: clock_now[0],
    )
    zone = interlock.zone(graph)

    def try_egress():
        return interlock.gate_move(
            action="graph.move_to", target_node="robot_home",
            current_node="hood_filter_low", zone=zone,
        )

    with pytest.raises(SashInterlockError):
        try_egress()

    interlock.grant_override(reason="walking the arm out", ttl_seconds=60, actor="tester")
    decision = try_egress()
    assert decision is not None and decision.allowed is True

    clock_now[0] += 61
    with pytest.raises(SashInterlockError):
        try_egress()


def test_freehand_motion_inside_the_region_is_refused(closed):
    """What interlock_freehand_guard relies on.

    A raw cartesian/joint/velocity move carries no node id, so entry cannot be
    gated -- but an arm already inside the region can still be refused, which
    is the case that matters.
    """
    interlock, zone = closed
    with pytest.raises(SashInterlockError):
        interlock.gate_move(
            action="move.joints", target_node=None, target_pose=None,
            current_node="hood_filter_high", zone=zone,
        )


def test_gated_pose_is_refused_when_the_node_does_not_resolve(closed):
    """The OFF/ADVISORY hole: a named move whose target resolves to no node.

    move_to_named_location('hood_shaker_high') while the rail is elsewhere
    yields target_node=None, so without the pose fallback there would be
    nothing to gate on.
    """
    interlock, zone = closed
    with pytest.raises(SashInterlockError):
        interlock.gate_move(
            action="move.location", target_node=None,
            target_pose="hood_shaker_high",
            current_node="robot_home", zone=zone,
        )


def test_rail_alone_marks_the_arm_as_inside(closed):
    """An off-grid arm at rail Hood is inside, even with no node pin."""
    interlock, zone = closed
    with pytest.raises(SashInterlockError):
        interlock.gate_move(
            action="move.joints", target_node=None,
            current_node=None, current_rail="Hood", zone=zone,
        )


# ── What must keep working ───────────────────────────────────────────

def test_non_gated_targets_are_unaffected_while_closed(closed):
    """A closed sash must not halt work elsewhere in the lab."""
    interlock, zone = closed
    for target in ("uplc_home", "cytation_home", "plateloc_home", "deck_home"):
        assert interlock.gate_move(
            action="graph.move_to", target_node=target,
            current_node="robot_home", zone=zone,
        ) is None, f"{target} should be unaffected by the sash"


def test_everything_is_permitted_while_the_sash_is_open(open_sash, graph):
    """No node anywhere in the graph is refused with the sash parked.

    gate_move returns None for a target it need not evaluate (not gated, arm
    not inside) and an allowed decision for one it did evaluate; neither may
    raise.
    """
    interlock, zone = open_sash
    for node in graph.nodes:
        decision = interlock.gate_move(
            action="graph.move_to", target_node=node.id,
            current_node="robot_home", zone=zone,
        )
        assert decision is None or decision.allowed is True
        # The gated ones must actually have been evaluated, or this test would
        # pass just as well with a no-op interlock.
        if node.id in EXPECTED_GATED:
            assert decision is not None, f"{node.id} was not evaluated at all"


def test_no_journey_between_non_gated_nodes_transits_the_region(graph):
    """The cul-de-sac property, locked in.

    Only two edges cross into the region, both from robot_home, so no route to
    UPLC/deck/cytation/plateloc has any reason to pass through the hood. If a
    future edge (say hood_home -> deck_home) broke that, this interlock would
    start refusing unrelated travel -- so fail here instead, loudly.
    """
    gated = EXPECTED_GATED
    outside = [n.id for n in graph.nodes if n.id not in gated]
    for src in outside:
        for dst in outside:
            if src == dst:
                continue
            try:
                path = graph.plan_path(src, dst, "empty")
            except Exception:
                continue          # unreachable with this gripper state; not our concern
            transited = gated & set(path)
            assert not transited, (
                f"path {src} -> {dst} transits the gated region via {sorted(transited)}"
            )


# ── allowed_actions mirroring (STATUS_SPEC 6.2) ──────────────────────

def test_filter_targets_withholds_gated_nodes_while_closed(closed):
    interlock, zone = closed
    interlock.evaluate()               # prime the cache; the filter never fetches
    targets = ["hood_home", "opentrons_home", "cytation_home", "uplc_home"]
    kept = interlock.filter_targets(
        targets, zone=zone, current_node="robot_home",
    )
    assert kept == ["cytation_home", "uplc_home"]


def test_filter_targets_withholds_everything_while_inside(closed):
    """Mirrors the endpoint: inside + closed refuses all motion, so advertise none."""
    interlock, zone = closed
    interlock.evaluate()
    kept = interlock.filter_targets(
        ["hood_home", "robot_home"], zone=zone, current_node="hood_filter_low",
    )
    assert kept == []


def test_filter_targets_keeps_everything_while_open(open_sash):
    interlock, zone = open_sash
    interlock.evaluate()
    targets = ["hood_home", "cytation_home"]
    assert interlock.filter_targets(
        targets, zone=zone, current_node="robot_home"
    ) == targets


def test_filter_targets_keeps_everything_while_blind(graph):
    """Fail-open consistency.

    Withholding here would produce "endpoint allows, /status withholds",
    stalling exactly the well-behaved workflows that consult allowed_actions
    while a client POSTing blindly sailed through.
    """
    def boom(url, timeout):
        raise OSError("no route to host")

    interlock = SashInterlock(
        {"base_url": "http://sash.invalid", "require_initial_contact": False},
        fetcher=boom,
    )
    zone = interlock.zone(graph)
    interlock.evaluate()
    targets = ["hood_home", "cytation_home"]
    assert interlock.filter_targets(
        targets, zone=zone, current_node="robot_home"
    ) == targets


def test_filter_targets_never_fetches(graph):
    calls = []

    def counting(url, timeout):
        calls.append(url)
        return _sash_at(3)

    interlock = SashInterlock({"base_url": "http://sash.invalid"}, fetcher=counting)
    zone = interlock.zone(graph)
    interlock.filter_targets(["hood_home"], zone=zone, current_node="robot_home")
    assert calls == []


def test_allowed_actions_and_the_endpoint_agree(graph):
    """The 6.2 property, over every gated/non-gated target and both sash states.

    For each target: it is advertised iff a move to it would not be refused.
    """
    for position in (5, 3):
        interlock = interlock_for(_sash_at(position))
        zone = interlock.zone(graph)
        interlock.evaluate()          # prime the cache
        all_targets = [n.id for n in graph.nodes]
        advertised = set(
            interlock.filter_targets(all_targets, zone=zone, current_node="robot_home")
        )
        for target in all_targets:
            try:
                interlock.gate_move(
                    action="graph.move_to", target_node=target,
                    current_node="robot_home", zone=zone,
                )
                refused = False
            except SashInterlockError:
                refused = True
            assert (target in advertised) is (not refused), (
                f"sash at {position}: {target} advertised={target in advertised} "
                f"but refused={refused}"
            )
