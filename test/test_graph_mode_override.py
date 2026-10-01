"""The motion-graph mode override: bounded, audited, self-reverting.

Lowering enforcement below STRICT used to be unbounded process-wide state —
`POST /control/graph/mode {advisory}` and nothing but operator discipline
brought it back, while every other client of the arm silently inherited the
relaxation. These tests pin the replacement contract:

* lowering needs a **reason** (and only lowering — raising is free);
* it runs for a **clamped window** and then restores STRICT on its own;
* it also restores when the **claim that bought it** goes away, and on
  **disconnect**, because a relaxation belongs to an operator, not to a
  process that outlives them;
* the restore is observed through the ordinary `graph_mode` read, so no
  caller has to remember to ask — which is the whole reason the expiry is
  lazy in the property rather than a timer thread.

Time is driven by monkeypatching ``time.monotonic`` rather than sleeping:
the point under test is the deadline arithmetic, not the clock.
"""

from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core.motion_graph import (  # noqa: E402
    DEFAULT_PRECONDITIONS,
    MODE_OVERRIDE_DEFAULT_SECONDS,
    MODE_OVERRIDE_MAX_SECONDS,
    GraphMode,
    MotionGraph,
)


def _graph_dict():
    return {
        "schema_version": "0.2",
        "gripper_states": {"empty": {"stroke": 150, "intent": "none"}},
        "nodes": [
            {"id": "n_home", "arm": "home", "rail": "Home", "gripper_states": ["empty"]},
            {"id": "n_pickup", "arm": "pickup", "rail": "Home", "gripper_states": ["empty"]},
        ],
        "edges": [
            {"from": "n_home", "to": "n_pickup", "mode": "joint", "speed": 30},
            {"from": "n_pickup", "to": "n_home", "mode": "joint", "speed": 30},
        ],
    }


@pytest.fixture
def c(initialized_controller):
    """A controller in STRICT with a small in-memory graph."""
    ctl = initialized_controller
    ctl.motion_graph = MotionGraph.from_dict(
        _graph_dict(), preconditions=DEFAULT_PRECONDITIONS,
    )
    ctl.set_graph_mode(GraphMode.STRICT)
    return ctl


@pytest.fixture
def clock(monkeypatch):
    """A monotonic clock the test advances by hand."""
    state = {"t": 1_000.0}

    def _now():
        return state["t"]

    monkeypatch.setattr("src.core.xarm_controller.time.monotonic", _now)
    return state


# ── A reason is required, in one direction only ──────────────────────


def test_lowering_without_a_reason_is_refused(c):
    with pytest.raises(ValueError, match="requires a reason"):
        c.set_graph_mode(GraphMode.ADVISORY)
    assert c.graph_mode == GraphMode.STRICT, "the refusal must not half-apply"


def test_blank_reason_counts_as_no_reason(c):
    with pytest.raises(ValueError):
        c.set_graph_mode(GraphMode.OFF, reason="   ")
    assert c.graph_mode == GraphMode.STRICT


def test_raising_to_strict_needs_no_reason(c):
    c.set_graph_mode(GraphMode.ADVISORY, reason="calibrating")
    assert c.set_graph_mode(GraphMode.STRICT) is None
    assert c.graph_mode == GraphMode.STRICT


def test_no_reason_needed_when_nothing_was_being_enforced(initialized_controller):
    """No graph => nothing to revert TO, so demanding a reason is theatre."""
    ctl = initialized_controller
    ctl.motion_graph = None
    ctl.graph_mode = GraphMode.OFF
    assert ctl.set_graph_mode(GraphMode.OFF) is None
    assert ctl.graph_mode_override_snapshot() is None


# ── The window ───────────────────────────────────────────────────────


def test_default_window_is_the_configured_default(c, clock):
    granted = c.set_graph_mode(GraphMode.ADVISORY, reason="camera survey")
    assert granted == MODE_OVERRIDE_DEFAULT_SECONDS


def test_ttl_is_clamped_to_the_cap(c, clock):
    granted = c.set_graph_mode(
        GraphMode.ADVISORY, reason="camera survey", ttl_seconds=99_999,
    )
    assert granted == MODE_OVERRIDE_MAX_SECONDS


def test_ttl_comes_from_the_graph_when_the_yaml_overrides_it(c, clock):
    c.motion_graph.mode_override_default_seconds = 30.0
    c.motion_graph.mode_override_max_seconds = 45.0
    assert c.set_graph_mode(GraphMode.ADVISORY, reason="x") == 30.0
    assert c.set_graph_mode(GraphMode.ADVISORY, reason="x", ttl_seconds=999) == 45.0


def test_mode_reverts_to_strict_when_the_window_lapses(c, clock):
    c.set_graph_mode(GraphMode.ADVISORY, reason="camera survey", ttl_seconds=60)
    clock["t"] += 59
    assert c.graph_mode == GraphMode.ADVISORY
    clock["t"] += 2
    assert c.graph_mode == GraphMode.STRICT
    assert c.graph_mode_override_snapshot() is None


def test_expiry_is_observed_by_an_ordinary_mode_read(c, clock):
    """No endpoint, no timer, no explicit poll — just reading the attribute.

    This is the property that makes the revert unconditional: every guard in
    the API layer and every controller move path reads `graph_mode`, so none
    of them can be the one that forgot to check.
    """
    c.set_graph_mode(GraphMode.OFF, reason="bench work", ttl_seconds=10)
    clock["t"] += 11
    assert c.graph_mode is GraphMode.STRICT


def test_reissuing_grants_a_fresh_window(c, clock):
    c.set_graph_mode(GraphMode.ADVISORY, reason="first", ttl_seconds=60)
    clock["t"] += 50
    c.set_graph_mode(GraphMode.ADVISORY, reason="still going", ttl_seconds=60)
    clock["t"] += 50
    assert c.graph_mode == GraphMode.ADVISORY, "re-issue should have extended"
    assert c.graph_mode_override_snapshot()["reason"] == "still going"


def test_advisory_to_off_keeps_the_strict_baseline(c, clock):
    """Stepping down twice must not re-baseline; the way back is STRICT."""
    c.set_graph_mode(GraphMode.ADVISORY, reason="first", ttl_seconds=60)
    c.set_graph_mode(GraphMode.OFF, reason="need raw moves", ttl_seconds=60)
    assert c.graph_mode_override_snapshot()["restores_to"] == "strict"
    clock["t"] += 61
    assert c.graph_mode == GraphMode.STRICT


# ── Non-TTL revert triggers ──────────────────────────────────────────


def test_releasing_the_claim_restores_strict(c, clock):
    record = c.claim_manager.acquire(owner="op@lab", session_id="s1")
    c.set_graph_mode(
        GraphMode.ADVISORY, reason="survey", ttl_seconds=600,
        owner="op@lab", session_id="s1",
    )
    assert c.graph_mode == GraphMode.ADVISORY
    c.claim_manager.release(record.token)
    assert c.graph_mode == GraphMode.STRICT, (
        "a relaxation is granted to an operator, not to the device"
    )


def test_another_session_taking_the_claim_restores_strict(c, clock):
    record = c.claim_manager.acquire(owner="op@lab", session_id="s1")
    c.set_graph_mode(
        GraphMode.ADVISORY, reason="survey", ttl_seconds=600,
        owner="op@lab", session_id="s1",
    )
    c.claim_manager.release(record.token)
    c.claim_manager.acquire(owner="other@lab", session_id="s2")
    assert c.graph_mode == GraphMode.STRICT


def test_no_claim_at_grant_time_does_not_revert_instantly(c, clock):
    """With claims unenforced there is no session to lose; absence of one
    must not read as a loss, or the window would close on the first read."""
    c.set_graph_mode(GraphMode.ADVISORY, reason="survey", ttl_seconds=600)
    assert c.graph_mode == GraphMode.ADVISORY
    assert c.graph_mode == GraphMode.ADVISORY


def test_disconnect_restores_strict(c, clock):
    c.set_graph_mode(GraphMode.ADVISORY, reason="survey", ttl_seconds=600)
    c.arm = MagicMock()
    c.disconnect()
    assert c.graph_mode == GraphMode.STRICT, (
        "reconnecting boots STRICT from the YAML; a live override would "
        "silently re-lower the next session's floor"
    )


def test_explicit_restore_clears_the_window(c, clock):
    c.set_graph_mode(GraphMode.ADVISORY, reason="survey", ttl_seconds=600)
    assert c.restore_graph_mode("explicit") == "strict"
    assert c.graph_mode_override_snapshot() is None
    assert c.restore_graph_mode("explicit") is None, "idempotent"


# ── The snapshot ─────────────────────────────────────────────────────


def test_snapshot_is_none_while_enforcing(c):
    assert c.graph_mode_override_snapshot() is None


def test_snapshot_carries_the_countdown_and_the_audit(c, clock):
    c.claim_manager.acquire(owner="op@lab", session_id="s1")
    c.set_graph_mode(
        GraphMode.ADVISORY, reason="freehand camera survey",
        ttl_seconds=120, owner="op@lab", session_id="s1",
    )
    clock["t"] += 30
    snap = c.graph_mode_override_snapshot()
    assert snap["active"] is True
    assert snap["mode"] == "advisory"
    assert snap["restores_to"] == "strict"
    assert snap["reason"] == "freehand camera survey"
    assert snap["owner"] == "op@lab"
    assert snap["granted_seconds"] == 120.0
    assert snap["remaining_seconds"] == 90.0
    assert snap["expires_at"].endswith("Z")


# ── Audit trail ──────────────────────────────────────────────────────


def _events(controller):
    """Swap in a recording exporter. ``is_simulated`` is a read-only
    property and already False on the fixture (no docker profile), which is
    what _emit_event gates on."""
    assert controller.is_simulated is False
    controller.events_exporter = MagicMock()
    controller.events_exporter.enabled = True
    return controller.events_exporter


def test_lowering_emits_an_audit_row(c, clock):
    exporter = _events(c)
    c.set_graph_mode(
        GraphMode.ADVISORY, reason="survey", ttl_seconds=60, owner="op@lab",
    )
    event, kwargs = exporter.emit.call_args[0][0], exporter.emit.call_args[1]
    assert event == "graph_mode_override"
    assert kwargs["from_state"] == "strict"
    assert kwargs["to_state"] == "advisory"
    assert kwargs["message"] == "survey"
    assert kwargs["ttl_s"] == 60.0
    assert kwargs["owner"] == "op@lab"


@pytest.mark.parametrize(
    "advance, trigger",
    [(61, "ttl_expired"), (0, "explicit")],
)
def test_restore_emits_its_trigger(c, clock, advance, trigger):
    c.set_graph_mode(GraphMode.ADVISORY, reason="survey", ttl_seconds=60)
    exporter = _events(c)
    if advance:
        clock["t"] += advance
        _ = c.graph_mode
    else:
        c.restore_graph_mode("explicit")
    event, kwargs = exporter.emit.call_args[0][0], exporter.emit.call_args[1]
    assert event == "graph_mode_restored"
    assert kwargs["trigger"] == trigger
    assert kwargs["to_state"] == "strict"


# ── /status surfaces ─────────────────────────────────────────────────


def _status(controller):
    from src.core.status_builder import build_status
    return build_status(controller)


def test_status_is_unmarked_while_enforcing(c):
    envelope = _status(c)
    assert not envelope.message.startswith("[GRAPH-")
    assert envelope.details["motion_graph"]["mode_override"] is None


@pytest.mark.parametrize(
    "mode, prefix",
    [(GraphMode.ADVISORY, "[GRAPH-ADVISORY]"), (GraphMode.OFF, "[GRAPH-OFF]")],
)
def test_status_message_is_prefixed_while_lowered(c, clock, mode, prefix):
    """The mode is process-wide state, so a client that never touched it
    still inherits the relaxation — worth the tile space."""
    c.set_graph_mode(mode, reason="survey", ttl_seconds=60)
    assert _status(c).message.startswith(prefix)


def test_status_carries_the_countdown(c, clock):
    c.set_graph_mode(GraphMode.ADVISORY, reason="survey", ttl_seconds=60)
    clock["t"] += 20
    block = _status(c).details["motion_graph"]["mode_override"]
    assert block["remaining_seconds"] == 40.0
    assert block["mode"] == "advisory"
    assert block["restores_to"] == "strict"


def test_status_prefix_clears_itself_when_the_window_lapses(c, clock):
    """No request, no restart — the next poll is enough."""
    c.set_graph_mode(GraphMode.ADVISORY, reason="survey", ttl_seconds=60)
    assert _status(c).message.startswith("[GRAPH-ADVISORY]")
    clock["t"] += 61
    envelope = _status(c)
    assert not envelope.message.startswith("[GRAPH-")
    assert envelope.details["motion_graph"]["mode_override"] is None


# ── What the window is FOR: freehand motion ──────────────────────────


def test_freehand_is_unblocked_while_lowered_and_reblocked_after(c, clock, monkeypatch):
    """The end-to-end point of the feature.

    ``strict_graph_guard`` is what refuses /control/freehand/* — moving the
    arm by raw coordinates to survey it with the camera. Lowering the mode
    opens that up; the window closing shuts it again with no request, no
    restart and nobody remembering to.
    """
    from fastapi import HTTPException

    from src.core import xarm_api_server as api
    monkeypatch.setattr(api, "get_controller", lambda: c)

    with pytest.raises(HTTPException) as refused:
        api.strict_graph_guard("move.position")
    assert refused.value.detail["error"] == "graph_mode_strict"

    c.set_graph_mode(GraphMode.ADVISORY, reason="camera survey", ttl_seconds=60)
    api.strict_graph_guard("move.position")   # allowed: no raise

    clock["t"] += 61
    with pytest.raises(HTTPException):
        api.strict_graph_guard("move.position")
