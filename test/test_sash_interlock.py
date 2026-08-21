"""The fume hood sash interlock's decision surface, in isolation.

The arm reaches into the fume hood and over the Opentrons deck; the sash that
closes over that space belongs to a *different* device. This module tests the
part that decides — the composite predicate, the asymmetric cache, the two
failure policies, the override — with an injected fetcher and an injected
clock, so nothing here touches the network or sleeps.

Two clusters carry most of the weight:

* **The composite predicate.** ``position == 5`` alone is not enough, because
  we do not yet know whether the device's ``sash_position`` is a true readback
  or the last *commanded* preset. If it is the latter, a sash travelling 5 -> 3
  reports 5 for the whole trip, so requiring ``actuator.state == "idle"`` is
  what keeps the check honest. Each conjunct gets its own test: they are the
  difference between a collision guard and a command audit.
* **The cache.** This is the only blocking outbound HTTP call on a motion path
  in this repo, so "a dead Pi does not add its timeout to every move" is a
  correctness property, not an optimisation.
"""

from __future__ import annotations

import copy
import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core.sash_interlock import (
    BLIND,
    BLOCKED,
    DISABLED,
    MALFORMED,
    OVERRIDDEN,
    SATISFIED,
    SashInterlock,
    SashInterlockError,
    SashReading,
)

# The live envelope from the deployed fume hood device, verbatim.
LIVE_PAYLOAD = {
    "protocol_version": "1.1",
    "equipment_id": "fume_hood_actuator",
    "equipment_name": "Fume Hood Actuator",
    "equipment_kind": "fume_hood",
    "equipment_status": "ready",
    "message": "Sash parked at position 5",
    "components": {
        "actuator": {"connected": True, "state": "idle", "message": None},
        "sash": {"connected": True, "state": "position_5", "message": None},
    },
    "metrics": {"sash_position": {"value": 5, "unit": "preset", "timestamp": None}},
    "last_error": None,
    "allowed_actions": ["sash.move", "sash.stop"],
    "details": {"claimed_by": None},
}


class FakeClock:
    """A monotonic clock we can advance, so TTL tests never sleep."""

    def __init__(self, start: float = 1000.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class CountingFetcher:
    """Returns a canned payload (or raises) and counts calls."""

    def __init__(self, *results):
        # Each result is a payload dict or an Exception instance. The last one
        # repeats once exhausted.
        self.results = list(results)
        self.calls = 0
        self.timeouts: list[float] = []

    def __call__(self, url, timeout_s):
        self.calls += 1
        self.timeouts.append(timeout_s)
        result = self.results[min(self.calls - 1, len(self.results) - 1)]
        if isinstance(result, Exception):
            raise result
        return result


def make(*results, clock=None, **cfg):
    """A configured interlock over a CountingFetcher."""
    config = {"base_url": "http://sash.invalid", "enabled": True}
    config.update(cfg)
    fetcher = CountingFetcher(*(results or (LIVE_PAYLOAD,)))
    interlock = SashInterlock(config, fetcher=fetcher, clock=clock or FakeClock())
    return interlock, fetcher


def payload_with(**overrides):
    """LIVE_PAYLOAD with a nested field changed, e.g. position=3."""
    p = copy.deepcopy(LIVE_PAYLOAD)
    if "position" in overrides:
        p["metrics"]["sash_position"]["value"] = overrides.pop("position")
    if "sash_state" in overrides:
        p["components"]["sash"]["state"] = overrides.pop("sash_state")
    if "sash_connected" in overrides:
        p["components"]["sash"]["connected"] = overrides.pop("sash_connected")
    if "actuator_state" in overrides:
        p["components"]["actuator"]["state"] = overrides.pop("actuator_state")
    if "equipment_status" in overrides:
        p["equipment_status"] = overrides.pop("equipment_status")
    assert not overrides, f"unhandled overrides: {overrides}"
    return p


# ── The happy path ───────────────────────────────────────────────────

def test_live_payload_is_satisfied():
    """The real device's envelope, unmodified, must read as satisfied.

    Pinned verbatim so a change to the composite predicate that happens to
    reject production reality fails here rather than in the lab.
    """
    interlock, _ = make(LIVE_PAYLOAD)
    decision = interlock.evaluate()
    assert decision.state == SATISFIED
    assert decision.allowed is True
    assert decision.observed_position == 5


def test_unconfigured_is_an_inert_no_op():
    """No base_url -> allows everything and never reaches the network."""
    fetcher = CountingFetcher(LIVE_PAYLOAD)
    interlock = SashInterlock({"enabled": True}, fetcher=fetcher)
    assert interlock.configured is False
    assert interlock.evaluate().state == DISABLED
    assert interlock.evaluate().allowed is True
    assert fetcher.calls == 0


def test_disabled_flag_is_an_inert_no_op():
    interlock, fetcher = make(LIVE_PAYLOAD, enabled=False)
    assert interlock.configured is False
    assert interlock.evaluate().allowed is True
    assert fetcher.calls == 0


def test_missing_config_file_yields_an_inert_no_op(tmp_path):
    interlock = SashInterlock.from_config_file(str(tmp_path / "nope.yaml"))
    assert interlock.configured is False
    assert interlock.evaluate().allowed is True


def test_configured_timeout_is_passed_to_the_fetcher():
    interlock, fetcher = make(LIVE_PAYLOAD, request_timeout_seconds=1.0)
    interlock.evaluate()
    assert fetcher.timeouts == [1.0]


# ── The composite predicate, one conjunct at a time ───────────────────
# Each of these would read as "satisfied" under a bare position == 5 check.

def test_wrong_position_blocks():
    interlock, _ = make(payload_with(position=3, sash_state="position_3"))
    decision = interlock.evaluate()
    assert decision.state == BLOCKED
    assert decision.allowed is False
    assert decision.observed_position == 3
    assert "position 3" in decision.reason and "required 5" in decision.reason


def test_component_state_disagreeing_with_the_metric_blocks():
    """position says 5, components.sash.state says otherwise -> do not trust it."""
    interlock, _ = make(payload_with(sash_state="position_4"))
    decision = interlock.evaluate()
    assert decision.state == BLOCKED
    assert "position_4" in decision.reason


def test_sash_component_disconnected_blocks():
    interlock, _ = make(payload_with(sash_connected=False))
    assert interlock.evaluate().state == BLOCKED


def test_actuator_in_motion_blocks():
    """The conjunct that matters most if sash_position is last-commanded.

    A sash travelling 5 -> 3 reports its *target* for the whole trip if the
    metric is the commanded preset, so a position check alone would happily
    wave the arm into a closing sash. Requiring an idle actuator closes it.
    """
    interlock, _ = make(payload_with(actuator_state="moving"))
    decision = interlock.evaluate()
    assert decision.state == BLOCKED
    assert "not idle" in decision.reason


def test_unhealthy_fume_hood_device_blocks():
    interlock, _ = make(payload_with(equipment_status="error"))
    decision = interlock.evaluate()
    assert decision.state == BLOCKED
    assert "not 'ready'" in decision.reason


def test_required_position_is_configurable():
    """Nothing hard-codes 5; the preset mapping is still unconfirmed."""
    interlock, _ = make(
        payload_with(position=2, sash_state="position_2"), required_position=2
    )
    assert interlock.evaluate().state == SATISFIED


# ── Malformed: present but unintelligible ────────────────────────────

@pytest.mark.parametrize("payload", [
    {},
    {"metrics": {}},
    {"metrics": {"sash_position": {}}},
    {"metrics": {"sash_position": {"value": "five"}}},
    {"metrics": {"sash_position": {"value": None}}},
    "not a dict",
], ids=["empty", "no-metric", "no-value", "string-value", "null-value", "not-json-object"])
def test_unparseable_answers_are_malformed_and_fail_closed(payload):
    """A device that answers but makes no sense is NOT an outage.

    It is present and saying something we do not understand -- a renamed field
    after a firmware bump, say. Treating that as ``blind`` would let it inherit
    fail-open and silently disable the guard forever, indistinguishable from
    the Pi being down.
    """
    interlock, _ = make(payload)
    decision = interlock.evaluate()
    assert decision.state == MALFORMED
    assert decision.allowed is False


def test_missing_component_blocks_are_malformed():
    p = copy.deepcopy(LIVE_PAYLOAD)
    del p["components"]["actuator"]
    interlock, _ = make(p)
    assert interlock.evaluate().state == MALFORMED


def test_malformed_can_be_configured_to_fail_open():
    interlock, _ = make({"metrics": {}}, malformed_fails_closed=False)
    decision = interlock.evaluate()
    assert decision.state == MALFORMED
    assert decision.allowed is True


# ── Unreachable: fail open, with the misconfiguration carve-out ───────

def test_unreachable_after_contact_fails_open():
    """The decided policy: an outage must not halt arm work."""
    clock = FakeClock()
    interlock, fetcher = make(
        LIVE_PAYLOAD, OSError("no route to host"), clock=clock,
        require_initial_contact=True,
    )
    assert interlock.evaluate().state == SATISFIED          # establishes contact
    clock.advance(60)
    decision = interlock.evaluate()
    assert decision.state == BLIND
    assert decision.allowed is True
    assert interlock._blind_since_wall is not None


def test_never_contacted_fails_closed():
    """A device we have never reached is misconfiguration, not an outage.

    Wrong base_url, re-imaged Pi, changed tailnet ACL -- none of these are the
    transient outage fail-open was chosen for, and all of them would otherwise
    yield a permanently, silently unguarded arm.
    """
    interlock, _ = make(OSError("no route to host"), require_initial_contact=True)
    decision = interlock.evaluate()
    assert decision.state == BLIND
    assert decision.allowed is False
    assert "never been reached" in decision.reason


def test_never_contacted_can_be_configured_to_fail_open():
    interlock, _ = make(OSError("down"), require_initial_contact=False)
    decision = interlock.evaluate()
    assert decision.state == BLIND
    assert decision.allowed is True


def test_fail_open_false_blocks_when_blind():
    clock = FakeClock()
    interlock, _ = make(LIVE_PAYLOAD, OSError("down"), clock=clock, fail_open=False)
    interlock.evaluate()
    clock.advance(60)
    decision = interlock.evaluate()
    assert decision.state == BLIND
    assert decision.allowed is False


def test_recovery_clears_the_blind_marker():
    clock = FakeClock()
    interlock, _ = make(
        LIVE_PAYLOAD, OSError("down"), LIVE_PAYLOAD, clock=clock,
    )
    interlock.evaluate()
    clock.advance(60)
    assert interlock.evaluate().state == BLIND
    clock.advance(60)
    assert interlock.evaluate().state == SATISFIED
    assert interlock._blind_since_wall is None
    assert interlock._consecutive_failures == 0


# ── The cache: a dead Pi must not cost a timeout per move ─────────────

def test_satisfied_reading_is_reused_within_its_ttl():
    clock = FakeClock()
    interlock, fetcher = make(LIVE_PAYLOAD, clock=clock, cache_ttl_satisfied_seconds=2.0)
    for _ in range(5):
        interlock.evaluate()
    assert fetcher.calls == 1
    clock.advance(2.5)
    interlock.evaluate()
    assert fetcher.calls == 2


def test_blocked_reading_expires_fast_so_reopening_the_sash_is_noticed():
    """Long TTL here would mean "open the sash, click, still refused"."""
    clock = FakeClock()
    blocked = payload_with(position=3, sash_state="position_3")
    interlock, fetcher = make(
        blocked, LIVE_PAYLOAD, clock=clock, cache_ttl_blocked_seconds=1.0
    )
    assert interlock.evaluate().state == BLOCKED
    clock.advance(1.1)
    assert interlock.evaluate().state == SATISFIED
    assert fetcher.calls == 2


def test_unreachable_is_not_reprobed_for_a_long_time():
    """The property that keeps a dead Pi from taxing every hop of a journey.

    Under fail-open, re-probing an unreachable device cannot change the
    decision -- only how fast recovery is noticed -- so the failure side of the
    cache is where the long TTL belongs.
    """
    clock = FakeClock()
    interlock, fetcher = make(
        LIVE_PAYLOAD, OSError("down"), clock=clock,
        cache_ttl_unreachable_seconds=10.0, unreachable_backoff_after=99,
    )
    interlock.evaluate()
    clock.advance(60)
    interlock.evaluate()                     # goes blind; 2 calls so far
    calls_after_going_blind = fetcher.calls
    for _ in range(10):                      # a 10-hop journey
        clock.advance(0.5)
        interlock.evaluate()
    assert fetcher.calls == calls_after_going_blind, "re-probed a device known down"
    clock.advance(11)
    interlock.evaluate()
    assert fetcher.calls == calls_after_going_blind + 1


def test_repeated_failures_back_off_further():
    clock = FakeClock()
    interlock, fetcher = make(
        OSError("down"), clock=clock,
        require_initial_contact=False,
        cache_ttl_unreachable_seconds=10.0,
        unreachable_backoff_after=3,
        unreachable_backoff_seconds=30.0,
    )
    for _ in range(3):
        interlock.evaluate()
        clock.advance(10.5)
    assert fetcher.calls == 3
    # Now in backoff: 10s is no longer enough to trigger a re-probe.
    clock.advance(10.5)
    interlock.evaluate()
    assert fetcher.calls == 3
    clock.advance(30.5)
    interlock.evaluate()
    assert fetcher.calls == 4


def test_allow_fetch_false_never_touches_the_network():
    """What /status relies on: a decision with no outbound call, ever.

    build_status is contractually side-effect-free and polled every 2-3s by the
    dashboard aggregator, so this is the property that keeps the interlock out
    of that path.
    """
    interlock, fetcher = make(LIVE_PAYLOAD)
    decision = interlock.evaluate(allow_fetch=False)
    assert fetcher.calls == 0
    assert decision.source == "none"


def test_refresh_forces_a_live_probe():
    clock = FakeClock()
    interlock, fetcher = make(LIVE_PAYLOAD, clock=clock)
    interlock.evaluate()
    interlock.refresh()
    assert fetcher.calls == 2


# ── Override ─────────────────────────────────────────────────────────

def test_override_allows_while_active_and_expires():
    clock = FakeClock()
    blocked = payload_with(position=3, sash_state="position_3")
    interlock, _ = make(blocked, clock=clock)
    assert interlock.evaluate().allowed is False

    granted = interlock.grant_override(reason="walking the arm out", ttl_seconds=60)
    assert granted == 60
    decision = interlock.evaluate()
    assert decision.state == OVERRIDDEN
    assert decision.allowed is True

    clock.advance(61)
    assert interlock.evaluate().allowed is False


def test_override_is_capped():
    interlock, _ = make(LIVE_PAYLOAD, override_max_seconds=120)
    assert interlock.grant_override(reason="x", ttl_seconds=99999) == 120


def test_override_requires_a_reason():
    interlock, _ = make(LIVE_PAYLOAD)
    for bad in ("", "   ", None):
        with pytest.raises(ValueError):
            interlock.grant_override(reason=bad)


def test_override_is_reissuable():
    """A second grant extends rather than erroring.

    Load-bearing: the override is the only way to walk a stuck arm out, and an
    expiry part-way through a multi-hop retreat would re-lock it half out.
    """
    clock = FakeClock()
    interlock, _ = make(payload_with(position=3, sash_state="position_3"), clock=clock)
    interlock.grant_override(reason="first", ttl_seconds=60)
    clock.advance(50)
    interlock.grant_override(reason="still going", ttl_seconds=60)
    clock.advance(50)
    assert interlock.evaluate().allowed is True, "extension did not take effect"


def test_clear_override_restores_enforcement():
    interlock, _ = make(payload_with(position=3, sash_state="position_3"))
    interlock.grant_override(reason="x", ttl_seconds=60)
    assert interlock.evaluate().allowed is True
    interlock.clear_override()
    assert interlock.evaluate().allowed is False


# ── Observability ────────────────────────────────────────────────────

def test_snapshot_counts_fail_open_bypasses():
    """The counter is the only in-envelope evidence the policy fired."""
    clock = FakeClock()
    interlock, _ = make(OSError("down"), clock=clock, require_initial_contact=False)
    zone = _zone_over({"hood_home"})
    for _ in range(3):
        clock.advance(60)
        interlock.gate_move(
            action="graph.move_to", target_node="hood_home",
            current_node="robot_home", zone=zone,
        )
    snap = interlock.snapshot()
    assert snap["moves_allowed_while_blind"] == 3
    assert snap["state"] == BLIND
    assert snap["fail_open"] is True
    assert snap["blind_since"] is not None


def test_snapshot_counts_refusals():
    interlock, _ = make(payload_with(position=3, sash_state="position_3"))
    zone = _zone_over({"hood_home"})
    for _ in range(2):
        with pytest.raises(SashInterlockError):
            interlock.gate_move(
                action="graph.move_to", target_node="hood_home",
                current_node="robot_home", zone=zone,
            )
    assert interlock.snapshot()["moves_blocked"] == 2


def test_snapshot_reports_the_position_under_override():
    """``sash_position`` exists so the UI has a readout in every state.

    ``observed_position`` is decision-scoped, and ``evaluate`` short-circuits
    on an active override before it looks at any reading — so that field goes
    None exactly when an operator is walking the arm out and most wants to see
    where the sash is. This is the reading-scoped twin that does not.
    """
    interlock, _ = make(payload_with(position=3, sash_state="position_3"))
    interlock.evaluate()                       # prime the cache
    interlock.grant_override(reason="walking the arm out", ttl_seconds=60)

    snap = interlock.snapshot()
    assert snap["state"] == OVERRIDDEN
    assert snap["observed_position"] is None   # decision-scoped, as designed
    assert snap["sash_position"] == 3          # reading-scoped: still there


def test_snapshot_position_is_none_when_blind():
    """No reading means no position. The last good number must not be shown
    as current — 'unknown' is the honest readout during an outage."""
    interlock, _ = make(OSError("down"), require_initial_contact=False)
    interlock.evaluate()
    snap = interlock.snapshot()
    assert snap["state"] == BLIND
    assert snap["sash_position"] is None


def test_snapshot_position_when_satisfied():
    interlock, _ = make()
    interlock.evaluate()
    snap = interlock.snapshot()
    assert snap["state"] == SATISFIED
    assert snap["sash_position"] == 5
    assert snap["observed_position"] == 5


def test_snapshot_never_fetches():
    interlock, fetcher = make(LIVE_PAYLOAD)
    interlock.snapshot()
    assert fetcher.calls == 0


def test_snapshot_of_an_unconfigured_interlock_is_minimal():
    interlock = SashInterlock(None)
    assert interlock.snapshot() == {"configured": False, "state": DISABLED}


@pytest.mark.parametrize("payload,expected", [
    (LIVE_PAYLOAD, None),
    (payload_with(position=3, sash_state="position_3"), "[SASH-CLOSED]"),
    ({"metrics": {}}, "[SASH-BLIND]"),
])
def test_status_prefix_reflects_state(payload, expected):
    interlock, _ = make(payload)
    interlock.evaluate()          # prime the cache; the prefix never fetches
    assert interlock.status_prefix() == expected


def test_status_prefix_reports_an_override():
    interlock, _ = make(LIVE_PAYLOAD)
    interlock.grant_override(reason="x", ttl_seconds=60)
    assert interlock.status_prefix() == "[SASH-OVERRIDE]"


def test_412_body_carries_the_diagnosis():
    interlock, _ = make(payload_with(position=2, sash_state="position_2"))
    zone = _zone_over({"hood_shaker_low"})
    with pytest.raises(SashInterlockError) as excinfo:
        interlock.gate_move(
            action="graph.move_to", target_node="hood_shaker_low",
            current_node="hood_home", zone=zone,
        )
    detail = excinfo.value.to_detail()
    assert detail["error"] == "interlock_not_satisfied"
    assert detail["interlock"] == "fume_hood_sash"
    assert detail["required"] == "sash_position=5"
    assert detail["observed"] == 2
    assert detail["state"] == BLOCKED
    # The way out must be discoverable at the moment of refusal, or people
    # reach for graph_mode:off instead.
    assert "override" in detail["hint"].lower()


# ── The watchdog ─────────────────────────────────────────────────────
# ``_maybe_trip`` is exercised directly rather than by starting the thread:
# the thread only supplies timing, and asserting on it would make these tests
# nondeterministic for no added coverage.

class FakeArm:
    """Just enough controller surface for the watchdog."""

    def __init__(self, *, moving=True, simulated=False):
        self._motion_in_progress = moving
        self.is_simulated = simulated
        self.motion_graph = None
        self.current_node = "hood_filter_high"
        self.last_rail_location_name = "Hood"
        self.stops = 0
        self.events = []

    def stop_motion(self):
        self.stops += 1
        return True

    def _emit_event(self, event, **extra):
        self.events.append((event, extra))


def test_watchdog_stops_a_moving_arm_when_the_sash_leaves_position():
    """The second requirement: not just refuse, but act.

    An arm mid-move inside the hood with the sash coming down is the case a
    pure refusal gate cannot address, because no new request is being made.
    """
    interlock, _ = make(LIVE_PAYLOAD)
    arm = FakeArm(moving=True)
    interlock._controller = arm
    interlock._ever_contacted = True

    interlock._maybe_trip(interlock._interpret(LIVE_PAYLOAD))
    assert arm.stops == 0, "stopped the arm while the sash was parked"

    interlock._maybe_trip(interlock._interpret(payload_with(position=3, sash_state="position_3")))
    assert arm.stops == 1
    assert interlock.watchdog_stops == 1
    assert any(e[0] == "interlock_stop" for e in arm.events)


def test_watchdog_stops_only_once_per_episode():
    """Repeated polls of a still-closed sash must not re-issue stops."""
    interlock, _ = make(LIVE_PAYLOAD)
    arm = FakeArm(moving=True)
    interlock._controller = arm
    interlock._ever_contacted = True
    closed = interlock._interpret(payload_with(position=3, sash_state="position_3"))
    for _ in range(5):
        interlock._maybe_trip(closed)
    assert arm.stops == 1


def test_watchdog_rearms_after_the_sash_reopens():
    interlock, _ = make(LIVE_PAYLOAD)
    arm = FakeArm(moving=True)
    interlock._controller = arm
    interlock._ever_contacted = True
    closed = interlock._interpret(payload_with(position=3, sash_state="position_3"))
    interlock._maybe_trip(closed)
    interlock._maybe_trip(interlock._interpret(LIVE_PAYLOAD))   # reopened
    interlock._maybe_trip(closed)                                # closed again
    assert arm.stops == 2


def test_watchdog_records_the_event_even_when_the_arm_is_idle():
    """Nothing to stop, but the sash-loss still belongs in the record."""
    interlock, _ = make(LIVE_PAYLOAD)
    arm = FakeArm(moving=False)
    interlock._controller = arm
    interlock._ever_contacted = True
    interlock._maybe_trip(interlock._interpret(payload_with(position=3, sash_state="position_3")))
    assert arm.stops == 0
    assert interlock.watchdog_stops == 1
    assert any(e[0] == "interlock_stop" for e in arm.events)


def test_watchdog_does_not_stop_the_arm_merely_for_going_blind():
    """Losing sight of the sash is not evidence the sash moved.

    Under fail-open, halting a plate transfer because another device's Pi
    became unreachable would be the wrong trade -- and inconsistent with the
    refusal path, which allows in exactly this situation.
    """
    interlock, _ = make(LIVE_PAYLOAD)
    arm = FakeArm(moving=True)
    interlock._controller = arm
    interlock._ever_contacted = True
    interlock._maybe_trip(
        SashReading(state=BLIND, position=None, reason="unreachable", fetched_at=0.0)
    )
    assert arm.stops == 0


def test_watchdog_respects_an_active_override():
    """Otherwise the override could not be used to walk the arm out."""
    interlock, _ = make(payload_with(position=3, sash_state="position_3"))
    arm = FakeArm(moving=True)
    interlock._controller = arm
    interlock._ever_contacted = True
    interlock.grant_override(reason="walking out", ttl_seconds=60)
    interlock._maybe_trip(interlock._interpret(payload_with(position=3, sash_state="position_3")))
    assert arm.stops == 0


def test_watchdog_is_not_started_in_simulation():
    """A simulated arm cannot hit a real sash, and a dev box should not poll it.

    The refusal path still evaluates in simulation (so a dry run exercises the
    interlock, as the Docker-sim contract promises); only the background poller
    and the stop command are suppressed.
    """
    interlock, fetcher = make(LIVE_PAYLOAD)
    interlock.start(FakeArm(simulated=True))
    assert interlock._thread is None
    assert fetcher.calls == 0


def test_watchdog_respects_the_backoff_when_the_arm_is_outside():
    """A permanently dead fume hood Pi must not draw a probe every tick.

    Outside the region the watchdog is only a cache warmer, so it goes through
    the cache and inherits the unreachable backoff. Without this, the
    asymmetric TTL would protect the motion path while the background thread
    hammered a dead host forever.
    """
    clock = FakeClock()
    interlock, fetcher = make(
        OSError("down"), clock=clock,
        cache_ttl_unreachable_seconds=10.0, unreachable_backoff_after=99,
    )
    for _ in range(5):
        interlock._fetch_for_watch(inside=False)
        clock.advance(2.0)          # the outside poll interval
    assert fetcher.calls == 1, "watchdog ignored the unreachable backoff"


def test_watchdog_always_probes_live_while_inside():
    """Inside the region, detection latency beats politeness.

    The faster cadence is the entire mechanism for noticing a sash coming down
    on the arm, so it must not be short-circuited by a cached reading.
    """
    clock = FakeClock()
    interlock, fetcher = make(LIVE_PAYLOAD, clock=clock)
    for _ in range(4):
        interlock._fetch_for_watch(inside=True)
        clock.advance(0.5)
    assert fetcher.calls == 4


def test_close_is_safe_when_never_started():
    interlock, _ = make(LIVE_PAYLOAD)
    interlock.close()      # must not raise


# ── Helpers ──────────────────────────────────────────────────────────

def _zone_over(node_ids):
    """A minimal stand-in for GatedZone over an explicit id set.

    Zone *computation* from a real graph is covered in test_sash_gating.py;
    here we only need membership so the decision logic is what is under test.
    """

    class _Zone:
        def contains_node(self, node_id):
            return node_id in node_ids

        def contains_pose(self, pose):
            return pose in node_ids

        def contains_rail(self, rail):
            return False

    return _Zone()
