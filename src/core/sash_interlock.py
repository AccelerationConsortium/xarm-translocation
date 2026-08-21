"""Refuse arm motion into the fume hood / Opentrons region unless the fume
hood sash is parked open.

The xArm reaches into the fume hood (``hood``-tagged motion-graph nodes) and
over the Opentrons deck (``opentrons``-tagged nodes). The sash that closes over
that space belongs to a *different* device — ``fume_hood_actuator``, a Pi
serving STATUS_SPEC v1.1 — so nothing in this service could previously know
whether the arm was about to swing into laminated glass.

This module closes that gap with two behaviours:

1. **Refuse entry.** A move whose target is inside the gated region is refused
   (HTTP 412 at the API layer) unless the sash reports the required preset.
2. **Stop on loss.** A watchdog thread notices the sash leaving that preset
   while the arm is *already* inside the region and calls
   ``controller.stop_motion()``. Afterwards **all** motion is refused — there is
   deliberately no automatic retreat, because a blind retreat could drag the arm
   through a descending sash. An operator clears it with a time-boxed override.

Design constraints, mirroring ``core/camera_tracker.py`` and
``core/events_exporter.py``:

1. **Stdlib only.** ``urllib`` rather than a new HTTP dependency.
2. **Inert unless configured.** ``enabled: false``, a missing
   ``src/settings/interlocks.yaml``, or a missing ``base_url`` makes this an
   inert no-op that allows everything, so dev machines and unit tests do
   nothing and reach no network.
3. **Injectable transport and clock.** ``fetcher`` and ``clock`` are
   constructor arguments so the whole decision surface — including cache TTLs
   and backoff — is unit-testable with no HTTP and no sleeping.

Unlike the two modules above, this one is **not** fire-and-forget: it is the
first thing in this repo that can block a motion path on an outbound HTTP call.
That is what the asymmetric cache is for — see :meth:`_ttl_for`. The long TTL
lives on the *failure* side, because under ``fail_open`` re-probing a dead
device cannot change the decision, only how quickly recovery is noticed.

``/status`` must never fetch (it is contractually side-effect-free and polled
every 2-3 s by the dashboard aggregator), so every read path used by the status
builder passes ``allow_fetch=False`` and is served from the cache the watchdog
thread keeps warm.

Configuration lives in ``src/settings/interlocks.yaml``; see that file for the
field documentation.
"""

from __future__ import annotations

import json
import logging
import threading
import urllib.error
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timezone
from time import monotonic
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Set

from .motion_graph import GraphError

# A fetcher takes (url, timeout_s) and returns parsed JSON, raising on failure.
Fetcher = Callable[[str, float], Any]
# A clock returns monotonic seconds. Injectable so TTL tests need not sleep.
Clock = Callable[[], float]

# Records land here rather than on the API server's module logger so this
# module stays importable without it. The server attaches its
# WebSocketLogHandler to this same logger name, so WARNINGs reach the operator
# panel's log stream instead of only stdout -- which matters: a fail-open
# bypass nobody can see is not observable.
logger = logging.getLogger("xarm.interlock")

# Cap the response body. urllib's ``timeout=`` bounds each socket operation,
# not total elapsed time, so a slow-drip response could otherwise hang a move
# for far longer than request_timeout_seconds.
_MAX_BODY_BYTES = 65536

# Probe-failure log throttling, matching events_exporter's idiom: log the first
# failure of a streak, then every Nth. Fail-open *decisions* are never
# throttled -- only this repeated "still can't reach it" chatter.
_FAILURE_LOG_EVERY = 50

# Decision / reading states.
SATISFIED = "satisfied"      # sash confirmed parked at the required preset
BLOCKED = "blocked"          # sash reachable and definitely not where we need it
BLIND = "blind"              # cannot reach the sash device at all
MALFORMED = "malformed"      # reachable, answering, but not in a shape we trust
OVERRIDDEN = "overridden"    # an operator override is in force
DISABLED = "disabled"        # not configured; inert

# States in which the device answered us. Used to decide whether a failure is
# an outage (retry slowly) or a device that is present but unhappy.
_REACHABLE_STATES = (SATISFIED, BLOCKED, MALFORMED)


class SashInterlockError(GraphError):
    """Raised when the sash interlock refuses a motion.

    Subclasses ``GraphError`` so that any pre-existing broad ``except
    GraphError`` degrades to *a refusal* rather than to a silent allow.
    Deliberately **not** an ``EdgeNotAllowedError``: that one means "no
    whitelisted edge" and is mapped to HTTP 409 all over the API layer, whereas
    this is an unmet external precondition and must surface as 412.
    """

    def __init__(
        self,
        *,
        action: str,
        reason: str,
        state: str,
        required_position: int,
        observed_position: Optional[int] = None,
        from_node: Optional[str] = None,
        to_node: Optional[str] = None,
        retry_after_s: Optional[float] = None,
        hint: Optional[str] = None,
    ) -> None:
        super().__init__(reason)
        self.action = action
        self.reason = reason
        self.state = state
        self.required_position = required_position
        self.observed_position = observed_position
        self.from_node = from_node
        self.to_node = to_node
        self.retry_after_s = retry_after_s
        self.hint = hint

    def to_detail(self) -> Dict[str, Any]:
        """The HTTP 412 body, shaped like ``box_sim_guard``'s refusal."""
        return {
            "error": "interlock_not_satisfied",
            "interlock": "fume_hood_sash",
            "action": self.action,
            "required": f"sash_position={self.required_position}",
            "observed": self.observed_position,
            "state": self.state,
            "retry_after_s": self.retry_after_s,
            "hint": self.hint,
            "message": self.reason,
        }


@dataclass(frozen=True)
class SashReading:
    """One immutable observation of the sash device.

    Stored in a single attribute so the control path and ``/status`` can read
    it without taking the fetch lock -- an attribute read is atomic in CPython,
    and a ``/status`` call that had to wait on a 1 s fetch would violate the
    side-effect-free contract by the back door.
    """

    state: str
    position: Optional[int]
    reason: str
    fetched_at: float                        # monotonic
    observed_at: Optional[str] = None        # ISO-8601 UTC, for details
    component_state: Optional[str] = None
    actuator_state: Optional[str] = None
    equipment_status: Optional[str] = None
    device_message: Optional[str] = None
    error: Optional[str] = None

    @property
    def reachable(self) -> bool:
        return self.state in _REACHABLE_STATES


@dataclass(frozen=True)
class InterlockDecision:
    """The verdict for one prospective motion."""

    allowed: bool
    state: str
    reason: str
    required_position: int
    observed_position: Optional[int] = None
    retry_after_s: Optional[float] = None
    source: str = "none"                     # live | cache | none | disabled
    age_seconds: Optional[float] = None
    hint: Optional[str] = None

    @property
    def blind(self) -> bool:
        return self.state in (BLIND, MALFORMED)


class GatedZone:
    """The set of graph nodes the sash can collide with.

    Membership is the union of three config-driven clauses, minus an exemption
    list:

    * ``gated_tags``  -- nodes carrying e.g. ``hood`` / ``opentrons``
    * ``gated_rails`` -- **every** node at a gated rail location, tagged or not
    * ``gated_nodes`` -- explicit ids, for anything the first two miss

    The rail clause is not redundant. ``POST /control/graph/node`` accepts
    ``tags`` as optional, so a claim-holding operator can create a node at rail
    ``Hood`` with no ``hood`` tag; without the rail clause that node would be
    silently ungated. The real backstop is the test pinning the computed set to
    an explicit literal, so any graph edit that changes it fails CI.

    ``arm_poses`` exists for the OFF/ADVISORY case where a named move's target
    does not resolve to a node (wrong rail, off-grid): the *pose name* is still
    recognisable even when the node is not.
    """

    def __init__(
        self,
        graph: Any,
        *,
        gated_tags: Sequence[str] = (),
        gated_rails: Sequence[str] = (),
        gated_nodes: Sequence[str] = (),
        exempt_nodes: Sequence[str] = (),
    ) -> None:
        tags = {str(t) for t in gated_tags}
        rails = {str(r) for r in gated_rails}
        exempt = {str(n) for n in exempt_nodes}

        node_ids: Set[str] = {str(n) for n in gated_nodes}
        arm_poses: Set[str] = set()

        for node in getattr(graph, "nodes", ()) or ():
            node_id = getattr(node, "id", None)
            if node_id is None:
                continue
            if (set(getattr(node, "tags", ()) or ()) & tags) or (
                getattr(node, "rail", None) in rails
            ):
                node_ids.add(node_id)

        node_ids -= exempt

        # Map the surviving ids back to their arm-pose names.
        for node in getattr(graph, "nodes", ()) or ():
            if getattr(node, "id", None) in node_ids:
                pose = getattr(node, "arm", None)
                if pose:
                    arm_poses.add(str(pose))

        self.node_ids = frozenset(node_ids)
        self.arm_poses = frozenset(arm_poses)
        self.rails = frozenset(rails)

    def contains_node(self, node_id: Optional[str]) -> bool:
        return bool(node_id) and node_id in self.node_ids

    def contains_pose(self, pose_name: Optional[str]) -> bool:
        return bool(pose_name) and pose_name in self.arm_poses

    def contains_rail(self, rail_name: Optional[str]) -> bool:
        return bool(rail_name) and rail_name in self.rails

    def __len__(self) -> int:  # pragma: no cover - convenience
        return len(self.node_ids)


class SashInterlock:
    """Read the fume hood sash and gate motion into the hood/Opentrons region.

    ``fetcher`` is injectable for tests: a callable taking ``(url, timeout_s)``
    and returning parsed JSON or raising. ``clock`` is a monotonic source, also
    injectable, so cache-TTL and backoff behaviour can be tested without
    sleeping.
    """

    def __init__(
        self,
        config: Optional[Dict[str, Any]],
        *,
        fetcher: Optional[Fetcher] = None,
        clock: Optional[Clock] = None,
        environ: Optional[Dict[str, str]] = None,
    ) -> None:
        cfg = dict(config or {})
        env = environ if environ is not None else {}

        self._fetcher: Fetcher = fetcher or self._http_get_json
        self._clock: Clock = clock or monotonic

        base_url = env.get("XARM_SASH_STATUS_URL") or cfg.get("base_url") or ""
        self.base_url = str(base_url).rstrip("/")
        status_path = str(cfg.get("status_path", "/status"))
        if not status_path.startswith("/"):
            status_path = "/" + status_path
        self.status_url = f"{self.base_url}{status_path}" if self.base_url else ""

        self.enabled = bool(cfg.get("enabled", True))
        self.required_position = int(cfg.get("required_position", 5))

        self.request_timeout_s = float(cfg.get("request_timeout_seconds", 1.0))
        self._ttl_satisfied = float(cfg.get("cache_ttl_satisfied_seconds", 2.0))
        self._ttl_blocked = float(cfg.get("cache_ttl_blocked_seconds", 1.0))
        self._ttl_unreachable = float(cfg.get("cache_ttl_unreachable_seconds", 10.0))
        self._backoff_after = int(cfg.get("unreachable_backoff_after", 3))
        self._backoff_s = float(cfg.get("unreachable_backoff_seconds", 30.0))

        self.poll_interval_s = float(cfg.get("poll_interval_seconds", 2.0))
        self.poll_interval_inside_s = float(cfg.get("poll_interval_inside_seconds", 0.5))

        self.fail_open = bool(cfg.get("fail_open", True))
        self.require_initial_contact = bool(cfg.get("require_initial_contact", True))
        self.malformed_fails_closed = bool(cfg.get("malformed_fails_closed", True))

        self.gated_tags = tuple(cfg.get("gated_tags", ("hood", "opentrons")))
        self.gated_rails = tuple(cfg.get("gated_rails", ("Hood",)))
        self.gated_nodes = tuple(cfg.get("gated_nodes", ()))
        self.exempt_nodes = tuple(cfg.get("exempt_nodes", ()))

        self.recheck_before_rail = bool(cfg.get("recheck_before_rail", True))
        self.override_max_seconds = float(cfg.get("override_max_seconds", 120.0))

        # Mutable state. `_reading` is the single atomic hand-off between the
        # fetcher and every reader; `_fetch_lock` is held only across a fetch.
        self._reading: Optional[SashReading] = None
        self._fetch_lock = threading.Lock()
        self._ever_contacted = False
        self._consecutive_failures = 0
        self._blind_since_wall: Optional[str] = None
        self._last_contact_wall: Optional[str] = None
        self._last_satisfied_wall: Optional[str] = None

        # Counters. These are the auditable record that fail-open actually
        # happened; they do not survive a restart, which is why the events
        # exporter also gets a row for each.
        self.moves_allowed_while_blind = 0
        self.moves_blocked = 0
        self.watchdog_stops = 0

        self._override_until: Optional[float] = None
        self._override_reason: Optional[str] = None
        self._override_by: Optional[str] = None

        # Watchdog / warmer.
        self._controller: Any = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._tripped = False

        # GatedZone memo, keyed on graph object identity: the graph object is
        # *replaced* on a hot reload (POST /control/graph/node etc.), and a memo
        # keyed on anything else would leave a newly added hood node ungated
        # until the service restarted.
        self._zone: Optional[GatedZone] = None
        self._zone_graph_id: Optional[int] = None

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    @classmethod
    def from_config_file(
        cls,
        path: str,
        *,
        key: str = "fume_hood_sash",
        fetcher: Optional[Fetcher] = None,
        clock: Optional[Clock] = None,
        environ: Optional[Dict[str, str]] = None,
    ) -> "SashInterlock":
        """Build from a YAML file. Missing/invalid file -> inert no-op."""
        config: Dict[str, Any] = {}
        try:
            import yaml  # local import: keeps the module importable without PyYAML

            with open(path, "r") as handle:
                loaded = yaml.safe_load(handle)
            if isinstance(loaded, dict):
                if not loaded.get("enabled", True):
                    # Master switch off: build an explicitly disabled instance
                    # rather than reading the per-interlock block.
                    config = {"enabled": False}
                else:
                    block = loaded.get(key)
                    config = dict(block) if isinstance(block, dict) else {}
        except FileNotFoundError:
            logger.info(f"[interlock] no config at {path}; sash interlock disabled")
        except Exception as exc:  # noqa: BLE001 - never break controller boot
            logger.warning(
                f"[interlock] failed to load {path}: {exc}; sash interlock disabled"
            )
        return cls(config, fetcher=fetcher, clock=clock, environ=environ)

    @property
    def configured(self) -> bool:
        """True when this interlock is live. False makes it an inert no-op."""
        return bool(self.enabled and self.status_url)

    # ------------------------------------------------------------------
    # Gated zone
    # ------------------------------------------------------------------

    def zone(self, graph: Any) -> Optional[GatedZone]:
        """The gated node set for ``graph``, memoized on its object identity."""
        if graph is None:
            return None
        if self._zone is not None and self._zone_graph_id == id(graph):
            return self._zone
        zone = GatedZone(
            graph,
            gated_tags=self.gated_tags,
            gated_rails=self.gated_rails,
            gated_nodes=self.gated_nodes,
            exempt_nodes=self.exempt_nodes,
        )
        self._zone = zone
        self._zone_graph_id = id(graph)
        return zone

    # ------------------------------------------------------------------
    # Reading the sash
    # ------------------------------------------------------------------

    def _ttl_for(self, reading: SashReading) -> float:
        """How long ``reading`` may be reused before we re-probe.

        Asymmetric on purpose. A *satisfied* reading is bounded by physics (a
        preset step takes seconds, so a shorter TTL buys nothing actionable). A
        *blocked* one expires fast so "open the sash, click again" works. An
        *unreachable* one is held for a long time and then longer still, because
        under fail-open re-probing a dead device cannot change the decision --
        it only affects how fast recovery is noticed -- and re-probing per hop
        would add a timeout to every move of a multi-hop journey.
        """
        if reading.state == SATISFIED:
            return self._ttl_satisfied
        if reading.state in (BLOCKED, MALFORMED):
            return self._ttl_blocked
        if self._consecutive_failures >= self._backoff_after:
            return self._backoff_s
        return self._ttl_unreachable

    def _is_fresh(self, reading: Optional[SashReading]) -> bool:
        if reading is None:
            return False
        return (self._clock() - reading.fetched_at) < self._ttl_for(reading)

    def _interpret(self, payload: Any) -> SashReading:
        """Turn a sash ``/status`` envelope into a reading.

        The predicate is deliberately composite rather than a bare
        ``position == 5``. We do not yet know whether ``sash_position`` is a
        true readback or the last *commanded* preset; if it is the latter, a
        sash moving 5 -> 3 reports 5 for the whole trip, so requiring
        ``actuator.state == "idle"`` is what keeps the check honest. Every
        conjunct catches a different failure and costs nothing.

        A value we cannot parse is ``malformed``, not ``blocked``: the device is
        answering in a shape we do not recognise (a firmware rename, say), which
        must be distinguishable from an outage rather than silently disabling
        the guard forever.
        """
        now = self._clock()
        observed_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

        if not isinstance(payload, dict):
            return SashReading(
                state=MALFORMED,
                position=None,
                reason="sash /status did not return a JSON object",
                fetched_at=now,
                observed_at=observed_at,
            )

        components = payload.get("components")
        components = components if isinstance(components, dict) else {}
        sash = components.get("sash") if isinstance(components.get("sash"), dict) else None
        actuator = (
            components.get("actuator")
            if isinstance(components.get("actuator"), dict)
            else None
        )
        metrics = payload.get("metrics")
        metrics = metrics if isinstance(metrics, dict) else {}
        entry = metrics.get("sash_position")
        entry = entry if isinstance(entry, dict) else {}
        raw_value = entry.get("value")

        equipment_status = payload.get("equipment_status")
        device_message = payload.get("message")
        component_state = sash.get("state") if sash else None
        actuator_state = actuator.get("state") if actuator else None

        def malformed(reason: str) -> SashReading:
            return SashReading(
                state=MALFORMED,
                position=None,
                reason=reason,
                fetched_at=now,
                observed_at=observed_at,
                component_state=component_state,
                actuator_state=actuator_state,
                equipment_status=(
                    str(equipment_status) if equipment_status is not None else None
                ),
                device_message=str(device_message) if device_message is not None else None,
            )

        if isinstance(raw_value, bool) or not isinstance(raw_value, (int, float)):
            return malformed(
                "sash /status has no numeric metrics.sash_position.value "
                f"(got {raw_value!r})"
            )
        if sash is None:
            return malformed("sash /status has no components.sash block")
        if actuator is None:
            return malformed("sash /status has no components.actuator block")

        position = int(round(float(raw_value)))
        expected_component = f"position_{self.required_position}"

        def reading(state: str, reason: str) -> SashReading:
            return SashReading(
                state=state,
                position=position,
                reason=reason,
                fetched_at=now,
                observed_at=observed_at,
                component_state=str(component_state) if component_state is not None else None,
                actuator_state=str(actuator_state) if actuator_state is not None else None,
                equipment_status=(
                    str(equipment_status) if equipment_status is not None else None
                ),
                device_message=str(device_message) if device_message is not None else None,
            )

        if position != self.required_position:
            return reading(
                BLOCKED,
                f"fume hood sash is at position {position}, "
                f"required {self.required_position}",
            )
        if sash.get("connected") is not True:
            return reading(BLOCKED, "fume hood sash component reports not connected")
        if component_state != expected_component:
            return reading(
                BLOCKED,
                f"fume hood sash component state is {component_state!r}, "
                f"expected {expected_component!r}",
            )
        if actuator_state != "idle":
            return reading(
                BLOCKED,
                f"fume hood sash actuator is {actuator_state!r}, not idle "
                "(the sash may be in motion)",
            )
        if equipment_status != "ready":
            return reading(
                BLOCKED,
                f"fume hood device reports equipment_status {equipment_status!r}, "
                "not 'ready'",
            )
        return reading(
            SATISFIED, f"fume hood sash parked at position {self.required_position}"
        )

    def _fetch(self) -> SashReading:
        """Probe the sash device once. Never raises."""
        try:
            payload = self._fetcher(self.status_url, self.request_timeout_s)
        except Exception as exc:  # noqa: BLE001 - an outage is an expected outcome
            self._consecutive_failures += 1
            if (
                self._consecutive_failures == 1
                or self._consecutive_failures % _FAILURE_LOG_EVERY == 0
            ):
                logger.warning(
                    f"[interlock] sash probe failed "
                    f"({self._consecutive_failures} in a row) "
                    f"at {self.status_url}: {exc}"
                )
            reading = SashReading(
                state=BLIND,
                position=None,
                reason=f"cannot reach the fume hood sash device at {self.status_url}",
                fetched_at=self._clock(),
                error=str(exc),
            )
            if self._blind_since_wall is None:
                self._blind_since_wall = (
                    datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
                )
            self._reading = reading
            return reading

        reading = self._interpret(payload)
        if self._consecutive_failures:
            logger.info(
                f"[interlock] sash probe recovered after "
                f"{self._consecutive_failures} failure(s)"
            )
        self._consecutive_failures = 0
        self._blind_since_wall = None
        self._ever_contacted = True
        self._last_contact_wall = reading.observed_at
        if reading.state == SATISFIED:
            self._last_satisfied_wall = reading.observed_at
        self._reading = reading
        return reading

    def _current_reading(self, *, allow_fetch: bool) -> Optional[SashReading]:
        """The freshest reading available, fetching only if allowed.

        Single-flight: the lock is taken only around a fetch, and the cache is
        re-checked inside it so concurrent callers do not stack up probes.
        """
        cached = self._reading
        if self._is_fresh(cached):
            return cached
        if not allow_fetch:
            return cached
        with self._fetch_lock:
            cached = self._reading
            if self._is_fresh(cached):
                return cached
            return self._fetch()

    def refresh(self) -> Optional[SashReading]:
        """Force a live probe (used by the watchdog and ``?refresh=true``)."""
        if not self.configured:
            return None
        with self._fetch_lock:
            return self._fetch()

    # ------------------------------------------------------------------
    # Override
    # ------------------------------------------------------------------

    def override_active(self) -> bool:
        until = self._override_until
        return until is not None and self._clock() < until

    def override_remaining_s(self) -> Optional[float]:
        if not self.override_active():
            return None
        assert self._override_until is not None
        return max(0.0, self._override_until - self._clock())

    def grant_override(
        self, *, reason: str, ttl_seconds: Optional[float] = None, actor: Optional[str] = None
    ) -> float:
        """Suspend the interlock for a bounded window. Returns seconds granted.

        Re-issuable on purpose: because there is no automatic retreat, this is
        the *only* way to move an arm stuck inside the region, and an expiry
        part-way through a multi-hop retreat would re-lock it half out. A second
        call extends rather than erroring.
        """
        if not str(reason or "").strip():
            raise ValueError("an override requires a non-empty reason")
        requested = float(ttl_seconds if ttl_seconds is not None else self.override_max_seconds)
        granted = max(0.0, min(requested, self.override_max_seconds))
        self._override_until = self._clock() + granted
        self._override_reason = str(reason).strip()
        self._override_by = actor
        # The watchdog should be able to trip again after the override lapses.
        self._tripped = False
        logger.warning(
            f"[interlock] sash interlock OVERRIDDEN for {granted:.0f}s by "
            f"{actor or 'unknown'}: {self._override_reason}"
        )
        self._emit("interlock_override", message=self._override_reason, ttl_s=granted)
        return granted

    def clear_override(self) -> None:
        if self._override_until is not None:
            logger.warning("[interlock] sash interlock override cleared")
        self._override_until = None
        self._override_reason = None
        self._override_by = None

    # ------------------------------------------------------------------
    # Decisions
    # ------------------------------------------------------------------

    def evaluate(self, *, allow_fetch: bool = True) -> InterlockDecision:
        """The sash-only verdict, independent of any particular node."""
        if not self.configured:
            return InterlockDecision(
                allowed=True,
                state=DISABLED,
                reason="sash interlock is not configured",
                required_position=self.required_position,
                source="disabled",
            )

        if self.override_active():
            return InterlockDecision(
                allowed=True,
                state=OVERRIDDEN,
                reason=(
                    f"sash interlock overridden by {self._override_by or 'operator'}: "
                    f"{self._override_reason}"
                ),
                required_position=self.required_position,
                retry_after_s=self.override_remaining_s(),
                source="cache",
            )

        before = self._reading
        reading = self._current_reading(allow_fetch=allow_fetch)
        source = "none" if reading is None else ("live" if reading is not before else "cache")
        age = None if reading is None else max(0.0, self._clock() - reading.fetched_at)

        # Never contacted since boot: that is misconfiguration (wrong URL,
        # re-imaged Pi, changed ACL), not the outage fail_open was chosen for,
        # so it fails closed however fail_open is set.
        if reading is None or (reading.state == BLIND and not self._ever_contacted):
            if self.require_initial_contact:
                return InterlockDecision(
                    allowed=False,
                    state=BLIND,
                    reason=(
                        "the fume hood sash device has never been reached since "
                        f"startup ({self.status_url}); refusing rather than "
                        "assuming the sash is open"
                    ),
                    required_position=self.required_position,
                    source=source,
                    age_seconds=age,
                    hint=(
                        "Check base_url in src/settings/interlocks.yaml and that "
                        "the fume hood service is up, or override."
                    ),
                )
            return InterlockDecision(
                allowed=self.fail_open,
                state=BLIND,
                reason=(
                    "cannot determine the fume hood sash position "
                    f"({self.status_url})"
                ),
                required_position=self.required_position,
                source=source,
                age_seconds=age,
            )

        if reading.state == SATISFIED:
            return InterlockDecision(
                allowed=True,
                state=SATISFIED,
                reason=reading.reason,
                required_position=self.required_position,
                observed_position=reading.position,
                source=source,
                age_seconds=age,
            )

        if reading.state == MALFORMED:
            return InterlockDecision(
                allowed=not self.malformed_fails_closed,
                state=MALFORMED,
                reason=reading.reason,
                required_position=self.required_position,
                source=source,
                age_seconds=age,
                hint=(
                    "The fume hood device is reachable but its /status shape is "
                    "unrecognised; check whether its firmware or contract changed."
                ),
            )

        if reading.state == BLIND:
            return InterlockDecision(
                allowed=self.fail_open,
                state=BLIND,
                reason=reading.reason,
                required_position=self.required_position,
                source=source,
                age_seconds=age,
            )

        return InterlockDecision(
            allowed=False,
            state=BLOCKED,
            reason=reading.reason,
            required_position=self.required_position,
            observed_position=reading.position,
            source=source,
            age_seconds=age,
            hint=(
                f"Move the sash to preset {self.required_position} "
                "(POST /control/sash/move on fume_hood_actuator), then retry."
            ),
        )

    def gate_move(
        self,
        *,
        action: str,
        target_node: Optional[str] = None,
        target_pose: Optional[str] = None,
        current_node: Optional[str] = None,
        current_rail: Optional[str] = None,
        zone: Optional[GatedZone] = None,
        allow_fetch: bool = True,
    ) -> Optional[InterlockDecision]:
        """Refuse a motion the sash could collide with. Raises or returns.

        Two clauses, both only consulted once the sash is known *not* to be
        parked:

        * the move's **target** is inside the region -> refuse (this is what
          blocks entry, and also blocks moving around inside);
        * the arm's **current position** is inside the region -> refuse *any*
          move, egress included.

        The second clause is the "no automatic retreat" decision: an arm caught
        inside with the sash down stays put until an operator overrides, because
        a blind retreat could drag it through a descending sash. The override is
        therefore load-bearing, not a convenience.
        """
        if not self.configured:
            return None

        target_gated = bool(
            zone
            and (zone.contains_node(target_node) or zone.contains_pose(target_pose))
        )
        inside = self._inside(zone, current_node, current_rail)
        if not (target_gated or inside):
            return None

        decision = self.evaluate(allow_fetch=allow_fetch)
        if decision.allowed:
            if decision.blind:
                # Fail-open actually happening. Unthrottled by design: each one
                # is a discrete safety-relevant decision, and a bypass nobody
                # can see is the whole risk of this policy.
                self.moves_allowed_while_blind += 1
                logger.warning(
                    f"[interlock] ALLOWING {action} -> {target_node or target_pose!r} "
                    f"UNGUARDED: {decision.reason} "
                    f"(fail-open bypass #{self.moves_allowed_while_blind})"
                )
                self._emit(
                    "interlock_bypass",
                    message=decision.reason,
                    action=action,
                    to_node=target_node or target_pose,
                    bypass_count=self.moves_allowed_while_blind,
                )
            return decision

        self.moves_blocked += 1
        if inside and not target_gated:
            reason = (
                f"{decision.reason}; the arm is inside the fume hood/Opentrons "
                "region, so all motion is refused until the sash is parked or an "
                "operator overrides"
            )
        else:
            reason = (
                f"'{action}' to {target_node or target_pose!r} is refused: "
                f"{decision.reason}"
            )
        # The override is named in *every* refusal, not just the stuck-inside
        # one. It is the documented way out, and if it is not discoverable at
        # the moment of refusal people reach for POST /control/graph/mode
        # {off} or `enabled: false` instead -- both of which disable the guard
        # entirely and silently.
        hint = decision.hint or f"Move the sash to preset {self.required_position}."
        if inside:
            hint = (
                f"{hint} While the arm is inside the region, "
                "POST /control/interlocks/sash/override is the only way to move it."
            )
        else:
            hint = f"{hint} Or POST /control/interlocks/sash/override."
        logger.warning(f"[interlock] REFUSED {action}: {reason}")
        self._emit(
            "interlock_blocked",
            message=reason,
            action=action,
            to_node=target_node or target_pose,
            observed_position=decision.observed_position,
        )
        raise SashInterlockError(
            action=action,
            reason=reason,
            state=decision.state,
            required_position=self.required_position,
            observed_position=decision.observed_position,
            from_node=current_node,
            to_node=target_node or target_pose,
            retry_after_s=decision.retry_after_s,
            hint=hint,
        )

    def filter_targets(
        self,
        targets: Iterable[str],
        *,
        zone: Optional[GatedZone] = None,
        current_node: Optional[str] = None,
        current_rail: Optional[str] = None,
    ) -> List[str]:
        """Drop targets ``gate_move`` would refuse. Never fetches.

        STATUS_SPEC §6.2 requires ``/status.allowed_actions`` to withhold
        exactly what the endpoint would refuse, and ``build_status`` is
        contractually side-effect-free, so this mirrors the endpoint's decision
        using only the cached reading.
        """
        targets = list(targets)
        if not self.configured or zone is None:
            return targets

        inside = self._inside(zone, current_node, current_rail)
        gated = [t for t in targets if zone.contains_node(t)]
        if not gated and not inside:
            return targets

        decision = self.evaluate(allow_fetch=False)
        if decision.allowed:
            return targets
        if inside:
            # All motion is refused, so advertise none of it.
            return []
        return [t for t in targets if not zone.contains_node(t)]

    def _inside(
        self,
        zone: Optional[GatedZone],
        current_node: Optional[str],
        current_rail: Optional[str],
    ) -> bool:
        if zone is None:
            return False
        return zone.contains_node(current_node) or zone.contains_rail(current_rail)

    # ------------------------------------------------------------------
    # Watchdog
    # ------------------------------------------------------------------

    def start(self, controller: Any) -> None:
        """Start the cache warmer / watchdog thread.

        Double duty: it keeps the cache warm so the control path is almost
        always a cache hit (zero added latency), and while the arm is *inside*
        the region it polls faster and stops the arm if the sash leaves the
        required preset.
        """
        if not self.configured or self._thread is not None:
            return
        if getattr(controller, "is_simulated", False):
            logger.info("[interlock] simulated session; sash watchdog not started")
            return
        self._controller = controller
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._watch, name="xarm-sash-interlock", daemon=True
        )
        self._thread.start()
        logger.info(
            f"[interlock] sash watchdog started ({self.status_url}, "
            f"required_position={self.required_position})"
        )

    def close(self, timeout_s: float = 3.0) -> None:
        """Stop the watchdog. Safe to call when it never started."""
        self._stop_event.set()
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=timeout_s)

    def _watch(self) -> None:
        while not self._stop_event.is_set():
            inside = False
            try:
                inside = self._controller_inside()
                reading = self._fetch_for_watch(inside=inside)
                if inside and reading is not None:
                    self._maybe_trip(reading)
                elif reading is not None and reading.state == SATISFIED:
                    self._tripped = False
            except Exception as exc:  # noqa: BLE001 - the watchdog must not die
                logger.warning(f"[interlock] watchdog iteration failed: {exc}")
            interval = self.poll_interval_inside_s if inside else self.poll_interval_s
            self._stop_event.wait(max(0.05, interval))

    def _fetch_for_watch(self, *, inside: bool) -> Optional[SashReading]:
        """Refresh the cached reading for one watchdog tick.

        Inside the gated region, force a live probe: the sash moving is an
        emergency there, and detection latency is the whole point of the faster
        cadence. Outside, go through the cache so the unreachable backoff
        applies -- otherwise a permanently dead fume hood Pi would draw a
        connection attempt every poll interval forever, which is exactly what
        the asymmetric TTL exists to prevent.
        """
        if not inside:
            return self._current_reading(allow_fetch=True)
        with self._fetch_lock:
            return self._fetch()

    def _controller_inside(self) -> bool:
        controller = self._controller
        if controller is None:
            return False
        zone = self.zone(getattr(controller, "motion_graph", None))
        return self._inside(
            zone,
            getattr(controller, "current_node", None),
            getattr(controller, "last_rail_location_name", None),
        )

    def _maybe_trip(self, reading: SashReading) -> None:
        """Stop the arm when the sash leaves the required preset mid-operation."""
        if self.override_active():
            return
        satisfied = reading.state == SATISFIED
        if satisfied:
            self._tripped = False
            return
        if reading.state == BLIND and self.fail_open and self._ever_contacted:
            # Losing sight of the sash is not evidence the sash moved; under
            # fail-open that must not stop the arm mid-plate.
            return
        if self._tripped:
            return
        self._tripped = True
        self.watchdog_stops += 1
        controller = self._controller
        moving = bool(getattr(controller, "_motion_in_progress", False))
        logger.warning(
            f"[interlock] SASH LEFT position {self.required_position} while the arm "
            f"is inside the region ({reading.reason}); "
            f"{'stopping the arm' if moving else 'arm is idle, nothing to stop'}"
        )
        self._emit(
            "interlock_stop",
            message=reading.reason,
            observed_position=reading.position,
            was_moving=moving,
            stop_count=self.watchdog_stops,
        )
        if not moving:
            return
        try:
            controller.stop_motion()
        except Exception as exc:  # noqa: BLE001 - report, never re-raise
            logger.error(f"[interlock] stop_motion() failed after sash loss: {exc}")

    # ------------------------------------------------------------------
    # Observability
    # ------------------------------------------------------------------

    def snapshot(self) -> Dict[str, Any]:
        """The ``details.interlocks.fume_hood_sash`` block. Never fetches."""
        if not self.configured:
            return {"configured": False, "state": DISABLED}

        decision = self.evaluate(allow_fetch=False)
        reading = self._reading
        return {
            "configured": True,
            "state": decision.state,
            "required_position": self.required_position,
            "observed_position": decision.observed_position,
            "sash_component_state": reading.component_state if reading else None,
            "sash_actuator_state": reading.actuator_state if reading else None,
            "sash_equipment_status": reading.equipment_status if reading else None,
            "sash_message": reading.device_message if reading else None,
            "source": decision.source,
            "age_seconds": (
                round(decision.age_seconds, 3) if decision.age_seconds is not None else None
            ),
            "last_contact_at": self._last_contact_wall,
            "last_satisfied_at": self._last_satisfied_wall,
            "blind_since": self._blind_since_wall,
            "consecutive_failures": self._consecutive_failures,
            "fail_open": self.fail_open,
            "require_initial_contact": self.require_initial_contact,
            "ever_contacted": self._ever_contacted,
            "moves_allowed_while_blind": self.moves_allowed_while_blind,
            "moves_blocked": self.moves_blocked,
            "watchdog_stops": self.watchdog_stops,
            "override": (
                {
                    "active": True,
                    "reason": self._override_reason,
                    "by": self._override_by,
                    "expires_in_s": round(self.override_remaining_s() or 0.0, 1),
                }
                if self.override_active()
                else None
            ),
            "gated_tags": list(self.gated_tags),
            "gated_rails": list(self.gated_rails),
            "device_url": self.status_url,
            "message": decision.reason,
        }

    def status_prefix(self) -> Optional[str]:
        """A ``message`` prefix for ``/status``, like ``[SIMULATION]``.

        The dashboard renders ``message`` on the equipment tile, which makes
        this the highest-visibility place to say "this guard is not currently
        guarding" without misreporting ``equipment_status`` (a neighbouring Pi
        being down is not *this* device's ill health -- see STATUS_SPEC §2.2).
        """
        if not self.configured:
            return None
        decision = self.evaluate(allow_fetch=False)
        if decision.state == OVERRIDDEN:
            return "[SASH-OVERRIDE]"
        if decision.state == BLOCKED:
            return "[SASH-CLOSED]"
        if decision.state in (BLIND, MALFORMED):
            return "[SASH-BLIND]"
        return None

    def _emit(self, event: str, **extra: Any) -> None:
        """Best-effort history-DB row via the controller's exporter hook."""
        controller = self._controller
        emit = getattr(controller, "_emit_event", None)
        if emit is None:
            return
        try:
            emit(event, interlock="fume_hood_sash", **extra)
        except Exception:  # noqa: BLE001 - observability must never break motion
            pass

    # ------------------------------------------------------------------
    # Transport
    # ------------------------------------------------------------------

    @staticmethod
    def _http_get_json(url: str, timeout_s: float) -> Any:
        request = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            return json.loads(response.read(_MAX_BODY_BYTES).decode("utf-8"))
