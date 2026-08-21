"""Build a STATUS_SPEC v1.2 ``EquipmentStatus`` envelope from controller state.

This module is the single, side-effect-free source of truth for the spec
``GET /status`` response. It MUST NOT call any controller method that
mutates hardware state (no ``connect``, no ``motion_enable``, no movement
queries that round-trip to the arm). All readings come from cached
attributes the controller updates as part of its normal operation.

See ``docs/STATUS_SPEC.md`` in ac-organic-lab for the contract.
"""

from __future__ import annotations

import socket
import time
from datetime import datetime, timezone
from typing import TYPE_CHECKING, Any

from .models import (
    PROTOCOL_VERSION,
    ComponentStatus,
    EquipmentStatus,
    ErrorInfo,
    MetricValue,
)

if TYPE_CHECKING:  # pragma: no cover - import-time only for type hints
    from .xarm_controller import XArmController


EQUIPMENT_ID = "xarm_translocation"
EQUIPMENT_NAME = "UFactory xArm5"
EQUIPMENT_KIND = "robot_arm"


def _resolve_equipment_version() -> str | None:
    """Installed pyxarm version for the envelope's ``equipment_version``.

    Package metadata first (the deployed service is uv-installed), then
    the CLI's ``__version__`` for from-source runs; ``None`` only when
    both fail rather than guessing.
    """
    try:
        from importlib.metadata import version

        return version("pyxarm")
    except Exception:
        pass
    try:
        from cli import __version__

        return __version__
    except Exception:
        return None


EQUIPMENT_VERSION = _resolve_equipment_version()


# Process start time used for ``uptime_seconds``. Captured at import so each
# new uvicorn worker reports its own uptime.
_PROCESS_START_TIME = time.time()


def _component_state(controller: XArmController, key: str) -> str:
    """Read a ComponentState enum value defensively as a snake_case string."""
    state = controller.states.get(key)
    if state is None:
        return "unknown"
    value = getattr(state, "value", None)
    if value is None:
        return str(state)
    return str(value)


def _observe_activity(
    controller: XArmController,
) -> tuple[str, datetime | None]:
    """Observed activity and the instant it last changed (spec §2.3).

    ``activity`` is read from the controller's motion bookkeeping — the flag
    every motion primitive brackets its SDK call with — and never derived
    from ``equipment_status``, which §2.3 forbids because it would add no
    information. ``activity_since`` is the latch the controller writes when
    that flag flips, so a reader can recover an in-progress move's true
    elapsed time instead of the timestamp of its own poll.

    Both reads are defensive: a controller predating the latch (or a test
    double) yields ``idle`` / ``None`` rather than raising.
    """
    running = bool(getattr(controller, "_motion_in_progress", False))
    since = getattr(controller, "_activity_since", None)
    if not isinstance(since, datetime):
        since = None
    return ("running" if running else "idle"), since


def _safe_hostname() -> str | None:
    try:
        return socket.gethostname()
    except OSError:
        return None


def _disconnected_envelope() -> EquipmentStatus:
    """Envelope returned when no controller object exists yet.

    The FastAPI process is up but no ``XArmController`` has been
    instantiated, so the only honest answer is ``requires_init`` with
    ``required_actions: ["connect"]`` (matching the SDK's pre-migration
    ``LegacyXArmAdapter`` mapping).
    """
    return EquipmentStatus(
        protocol_version=PROTOCOL_VERSION,
        equipment_id=EQUIPMENT_ID,
        equipment_name=EQUIPMENT_NAME,
        equipment_kind=EQUIPMENT_KIND,
        equipment_version=EQUIPMENT_VERSION,
        host=_safe_hostname(),
        equipment_status="requires_init",
        # §2.3 invariant: requires_init ⇒ idle. Nothing is instantiated, so
        # no primary operation can be in flight — "idle" is both the
        # required value and the honest one. ``activity_since`` is null
        # because there is no observed transition to timestamp.
        activity="idle",
        activity_since=None,
        message="Controller not instantiated. POST /connect to initialize.",
        required_actions=["connect"],
        # /connect is honored in this state (it is the one action that gets
        # out of it), so advertise it — matching the connection-down branch
        # of build_status, which reports the same equipment_status.
        allowed_actions=["connect"],
        device_time=datetime.now(timezone.utc),
        uptime_seconds=time.time() - _PROCESS_START_TIME,
        components={
            "arm": ComponentStatus(connected=False, state="disabled"),
            "gripper": ComponentStatus(connected=False, state="disabled"),
            "track": ComponentStatus(connected=False, state="disabled"),
            "force_torque": ComponentStatus(connected=False, state="disabled"),
        },
    )


def build_status(controller: XArmController | None) -> EquipmentStatus:
    """Map controller state to an ``EquipmentStatus`` envelope.

    Side-effect-free: only cached attributes are read. The controller MUST
    NOT be re-initialized, polled, or otherwise mutated by this call.

    ``equipment_status`` answers "is this arm healthy and suitable for a
    run"; ``activity`` answers "is it moving right now". They are derived
    independently (spec §2.3) — precedence below applies to the first only:

    1. ``controller is None`` -> ``requires_init``.
    2. controller has an active error code or string -> ``error``.
    3. connection not enabled -> ``requires_init``.
    4. ``controller.alive`` and arm enabled -> ``busy`` while a motion is in
       flight, else ``ready``.
    5. otherwise -> ``degraded`` (with ``activity`` still reporting whether
       a motion is in flight).

    **Simulation** (``controller.is_simulated``): either the connection
    targets the Docker simulator *or* the control box itself reports
    simulation mode — UFACTORY Studio's Real/Sim toggle, which can be
    flipped mid-session by anyone with the panel open. Both are observed,
    not declared; see the ``is_simulated`` docstring for why the box's own
    report bit is the authority.

    The healthy states (``ready`` / ``busy``) are reported as ``dry_run`` —
    the spec's first-class "simulation mode" state, which the dashboard
    already projects to ``simulated: true`` — with ``activity`` unchanged
    (the §2.3 invariant table deliberately allows any activity for
    ``dry_run``). Fault states (``error`` / ``degraded`` /
    ``requires_init``) keep their honest value so simulated failure paths
    stay testable. Every state gets a ``[SIMULATION]`` message prefix,
    ``details.simulated: true``, and ``details.simulation_source`` naming
    which of the two mechanisms is in force, so no reader — human or
    machine — can mistake a sim session for the real arm.

    Reporting ``dry_run`` here is what keeps §2.2 honest: in Studio-Sim the
    SDK short-circuits track and gripper commands to *success* without
    moving anything, so a device that kept reporting ``ready`` would be
    claiming it can perform its primary operation when it cannot.
    """
    if controller is None:
        return _disconnected_envelope()

    arm_state = _component_state(controller, "arm")
    gripper_state = _component_state(controller, "gripper")
    track_state = _component_state(controller, "track")
    force_torque_state = _component_state(controller, "force_torque")
    connection_state = _component_state(controller, "connection")
    arm_connected = connection_state == "enabled"

    last_error_code = getattr(controller, "last_error_code", 0) or 0
    last_error_text = getattr(controller, "last_error", None)
    if isinstance(last_error_text, int):
        last_error_text = None

    activity, activity_since = _observe_activity(controller)
    alive = bool(getattr(controller, "alive", False))

    # State derivation.
    last_error: ErrorInfo | None = None
    required_actions: list[str] = []

    if last_error_code != 0 or last_error_text:
        equipment_status = "error"
        message = (
            f"Controller reports error: {last_error_text}"
            if last_error_text
            else f"Controller reports error code {last_error_code}."
        )
        required_actions = ["clear_errors"]
        last_error = ErrorInfo(
            code=str(last_error_code) if last_error_code else None,
            message=str(last_error_text) if last_error_text else f"error_code={last_error_code}",
            severity="error",
            timestamp=datetime.now(timezone.utc),
        )
    elif connection_state != "enabled":
        equipment_status = "requires_init"
        message = "Controller not connected. POST /connect to initialize."
        required_actions = ["connect"]
        # §2.3 invariant: requires_init ⇒ idle. The connection dropping
        # mid-move can leave the motion flag latched; a move that can no
        # longer be executing must not be reported as a run.
        activity, activity_since = "idle", None
    elif alive and arm_state == "enabled":
        # Healthy. §2.3 makes ``busy`` definitionally healthy + running, so
        # deriving it here from the observed activity keeps both invariants
        # (busy ⇒ running, ready ⇒ idle) true by construction.
        if activity == "running":
            equipment_status = "busy"
            message = "Robot motion in progress."
        else:
            equipment_status = "ready"
            message = "Idle"
    else:
        # Connected but unhealthy. §2.3 forbids busy + degraded, so a move
        # in flight here is reported as degraded + activity "running" — the
        # health fault and the run are independent facts, and neither
        # suppresses the other.
        equipment_status = "degraded"
        message = "Controller connected but not fully alive."

    simulated = bool(getattr(controller, "is_simulated", False))
    if simulated:
        # Simulator session: healthy states become the spec's first-class
        # "dry_run"; faults keep their honest value (see the docstring).
        if equipment_status in ("ready", "busy"):
            equipment_status = "dry_run"
        message = f"[SIMULATION] {message}"

    # Sash interlock visibility. Deliberately a message prefix rather than a
    # push to `degraded`: STATUS_SPEC §2.2 scopes degraded to an unhealthy
    # subsystem *of this device*, and a neighbouring fume hood Pi being
    # unreachable is not this arm's ill health — under fail-open its
    # capability is not even reduced, only its supervision. But the dashboard
    # renders `message` on the tile, so this is the highest-visibility place
    # to say "the guard is not currently guarding" without misreporting the
    # envelope. The machine-readable twin is details.interlocks (as
    # [SIMULATION] pairs with details.simulated).
    sash_prefix = _sash_status_prefix(controller)
    if sash_prefix:
        message = f"{sash_prefix} {message}"

    # Components.
    components: dict[str, ComponentStatus] = {
        "arm": ComponentStatus(connected=arm_connected, state=arm_state),
    }
    if controller.has_gripper():
        gripper_message = str(getattr(controller, "gripper_type", None) or "")
        # Reflect a BIO gripper hardware fault (e.g. "object slipped") into
        # the component message so it's visible without opening details.
        grip_err_code = getattr(controller, "last_gripper_error_code", 0)
        grip_err_text = getattr(controller, "last_gripper_error_text", None)
        if isinstance(grip_err_code, int) and grip_err_code != 0:
            label = grip_err_text if isinstance(grip_err_text, str) else "fault"
            gripper_message = f"{gripper_message} FAULT: {label} (code {grip_err_code})".strip()
        components["gripper"] = ComponentStatus(
            connected=arm_connected,
            state=gripper_state,
            message=gripper_message or None,
        )
    if controller.has_track():
        components["track"] = ComponentStatus(
            connected=arm_connected,
            state=track_state,
        )
    if controller.has_force_torque_sensor():
        components["force_torque"] = ComponentStatus(
            connected=arm_connected,
            state=force_torque_state,
            message=(
                "calibrated"
                if getattr(controller, "force_torque_calibrated", False)
                else "uncalibrated"
            ),
        )

    # Metrics (numeric values with units).
    metrics: dict[str, MetricValue] = {}
    if controller.has_track():
        track_pos = getattr(controller, "last_track_position", None)
        if isinstance(track_pos, (int, float)):
            metrics["track_position"] = MetricValue(value=float(track_pos), unit="mm")

    last_ft = getattr(controller, "last_force_torque", None)
    if (
        controller.has_force_torque_sensor()
        and isinstance(last_ft, (list, tuple))
        and len(last_ft) >= 3
    ):
        try:
            fx, fy, fz = (float(last_ft[0]), float(last_ft[1]), float(last_ft[2]))
            magnitude = (fx * fx + fy * fy + fz * fz) ** 0.5
            metrics["force_magnitude"] = MetricValue(value=magnitude, unit="N")
        except (TypeError, ValueError):
            pass

    tcp_speed = getattr(controller, "tcp_speed", None)
    if isinstance(tcp_speed, (int, float)):
        metrics["tcp_speed"] = MetricValue(value=float(tcp_speed), unit="mm/s")
    angle_speed = getattr(controller, "angle_speed", None)
    if isinstance(angle_speed, (int, float)):
        metrics["angle_speed"] = MetricValue(value=float(angle_speed), unit="deg/s")

    # Details (debug-only, free-form).
    details: dict[str, Any] = {
        "current_position": list(getattr(controller, "last_position", []) or []) or None,
        "current_joints": list(getattr(controller, "last_joints", []) or []) or None,
        "model_name": getattr(controller, "model_name", None),
        "num_joints": getattr(controller, "num_joints", None),
        "gripper_type": getattr(controller, "gripper_type", None),
        # Manual (drag/teach) mode == xArm SDK mode 2. Read-only here:
        # the toggle lives on POST /robot/manual.
        "manual_mode": getattr(getattr(controller, "arm", None), "mode", None) == 2,
    }
    if simulated:
        # Machine-readable twin of the dry_run state / message prefix. The
        # panel keys its banner on this; workflows can branch on it without
        # string-matching the message.
        details["simulated"] = True
        # Which mechanism is in force. Worth distinguishing: the profile term
        # is fixed for the session, while "controller" can appear or vanish
        # mid-session as the Studio toggle is flipped — so an operator seeing
        # a tile change to SIMULATION without a reconnect needs to know it
        # was the box, not the connection.
        sources = []
        if getattr(controller, "is_docker_target", False):
            sources.append("docker_profile")
        if getattr(controller, "is_controller_simulating", False):
            sources.append("controller")
        details["simulation_source"] = "+".join(sources) or "unknown"
    # Carry connection details for the local web UI's panel. Not contracted.
    details["connection_details"] = _build_connection_details(controller)
    # Motion-graph state (Phase 1: introspection only; absent when no graph
    # is loaded so the field doesn't pollute /status for unmigrated configs).
    motion_graph_block = _build_motion_graph_details(controller)
    if motion_graph_block is not None:
        details["motion_graph"] = motion_graph_block

    # Cross-device interlock state. Carries the counters that make a fail-open
    # bypass auditable (moves_allowed_while_blind, watchdog_stops) — without
    # them the policy is invisible, since the log line does not survive a
    # restart and the envelope would look identical either way.
    interlocks_block = _build_sash_interlock_details(controller)
    if interlocks_block is not None:
        details["interlocks"] = interlocks_block

    # BIO gripper slip/detect register snapshot (plate-transfer
    # verification aid). Absent for non-BIO grippers and before the first
    # gripper move of the session. Populated from cached values only.
    gripper_block = _build_gripper_details(controller)
    if gripper_block is not None:
        details["gripper"] = gripper_block

    # v1.1: ``details.claimed_by`` while a claim is active; absent
    # otherwise. The status envelope is unchanged for v1.0 readers
    # because the field lives under ``details`` (free-form blob).
    claim_block = _build_claimed_by(controller)
    if claim_block is not None:
        details["claimed_by"] = claim_block
    else:
        # Spec section 9 example shows ``"claimed_by": null`` for the
        # no-claim case. Keep parity to make the field explicit rather
        # than missing.
        details["claimed_by"] = None

    return EquipmentStatus(
        protocol_version=PROTOCOL_VERSION,
        equipment_id=EQUIPMENT_ID,
        equipment_name=EQUIPMENT_NAME,
        equipment_kind=EQUIPMENT_KIND,
        equipment_version=EQUIPMENT_VERSION,
        host=_safe_hostname(),
        equipment_status=equipment_status,  # type: ignore[arg-type]
        activity=activity,  # type: ignore[arg-type]
        activity_since=activity_since,
        message=message,
        required_actions=required_actions,
        allowed_actions=_build_allowed_actions(
            controller, equipment_status, last_error is not None, activity,
        ),
        device_time=datetime.now(timezone.utc),
        uptime_seconds=time.time() - _PROCESS_START_TIME,
        components=components,
        metrics=metrics,
        last_error=last_error,
        details=details,
    )


def build_telemetry(controller: XArmController | None) -> dict[str, Any]:
    """Compact live-telemetry payload pushed at motion rate over the WS.

    Carries only the fields that change while the arm moves (joints, pose,
    track, manual mode, coarse state) so the high-frequency push stays small
    and the browser can apply it with a cheap field-diff instead of running
    the full status render. The complete ``EquipmentStatus`` envelope is still
    sent as the ``status_update`` message on connect and after every action.

    Side-effect-free: derived from ``build_status`` so the state/manual/joint
    values can never drift from the authoritative envelope.
    """
    if controller is None:
        return {
            "equipment_status": "requires_init",
            "is_alive": False,
            "current_joints": None,
            "current_position": None,
            "track_position": None,
            "manual_mode": False,
            "num_joints": None,
        }

    full = build_status(controller)
    details = full.details or {}
    track_metric = full.metrics.get("track_position")
    return {
        "equipment_status": full.equipment_status,
        "is_alive": full.equipment_status in ("ready", "busy", "degraded", "e_stop", "dry_run"),
        "current_joints": details.get("current_joints"),
        "current_position": details.get("current_position"),
        "track_position": (track_metric.value if track_metric is not None else None),
        "manual_mode": bool(details.get("manual_mode")),
        "num_joints": details.get("num_joints"),
    }


def _filter_sash_gated(controller: XArmController, targets) -> list[str]:
    """Drop node targets the sash interlock would refuse. Never fetches.

    Tolerant of a controller without the attribute (test doubles, an older
    controller) so status building can never fail on its absence.
    """
    helper = getattr(controller, "filter_sash_gated_targets", None)
    if helper is None:
        return list(targets)
    try:
        filtered = helper(targets)
    except Exception:  # noqa: BLE001 - /status must never fail on an interlock
        return list(targets)
    # Only trust a real sequence of strings. Anything else (a test double, a
    # half-built interlock) falls back to the unfiltered list rather than
    # putting a foreign object into the envelope.
    if isinstance(filtered, (list, tuple)) and all(isinstance(t, str) for t in filtered):
        return list(filtered)
    return list(targets)


def _build_sash_interlock_details(controller: XArmController) -> dict[str, Any] | None:
    """The ``details.interlocks`` block, or None when nothing is configured.

    Absent rather than empty for an unconfigured interlock, so /status is
    unchanged for anyone who has not set one up.
    """
    interlock = getattr(controller, "sash_interlock", None)
    if interlock is None or not getattr(interlock, "configured", False):
        return None
    try:
        snapshot = interlock.snapshot()
    except Exception:  # noqa: BLE001 - observability must not break /status
        return None
    return {"fume_hood_sash": snapshot} if isinstance(snapshot, dict) else None


def _sash_status_prefix(controller: XArmController) -> str | None:
    """A ``[SASH-*]`` message prefix, or None when the sash is parked.

    Validates the type before it reaches ``message``: that field is part of
    the contract, and a subsystem returning something unexpected must not be
    able to splice an arbitrary object into it. Same discipline as
    ``_observe_activity`` type-checking ``activity_since``.
    """
    interlock = getattr(controller, "sash_interlock", None)
    if interlock is None:
        return None
    try:
        prefix = interlock.status_prefix()
    except Exception:  # noqa: BLE001
        return None
    return prefix if isinstance(prefix, str) and prefix.startswith("[") else None


def _build_claimed_by(controller: XArmController) -> dict[str, Any] | None:
    """Read the active claim (if any) from the controller's ClaimManager.

    Returns the ``ClaimedBy`` dict shape from STATUS_SPEC v1.1 §2, or
    None when no claim is held. Lazily-expired claims are reported as
    None — the manager checks TTL on every read.
    """
    cm = getattr(controller, "claim_manager", None)
    if cm is None:
        return None
    return cm.claimed_by()


def _build_allowed_actions(
    controller: XArmController,
    equipment_status: str,
    has_error: bool,
    activity: str = "idle",
) -> list[str]:
    """Populate the v1.1 ``allowed_actions`` list.

    Four sources:
    1. State-driven defaults (connect / clear_errors / stop) based on
       ``equipment_status`` — always present so workflow clients have
       *something* to act on even when the graph isn't enforcing.
    2. Graph-driven move targets **and** gripper states — only when
       ``graph_mode == STRICT``, since ADVISORY/OFF modes don't actually
       constrain either one and claiming a list of allowed actions there
       would be misleading.
    3. ``activity``: while a motion is in flight, every move target and
       gripper state is withheld (spec §2.3 — no second concurrent run).
       ``stop`` stays, so an abort is always reachable.
    4. Catalog family names (``graph.move_to``, ``graph.gripper``,
       ``graph.recover_to``, ``graph.mode``, ``graph.record``) — the
       ``Skill.name`` strings the lab-skills ``robot_arm`` catalog
       registers, advertised whenever the corresponding endpoint would
       honor a POST. These are what STATUS_SPEC's ``allowed_actions``
       contract actually asks for; the per-target names from source 2
       are this device's finer-grained enumeration on top.

    Sources 3 and the move endpoints' HTTP 409 are the same rule on two
    surfaces, and §6.2 requires them never to disagree: both read the
    controller's motion state, so a client that sees ``move.<node>`` listed
    and immediately POSTs it cannot be refused for being busy.

    The same contract governs the fume hood sash interlock, which withholds
    gated ``move.<node>`` targets via ``reachable_node_ids()`` — one method
    feeding this list, ``details.motion_graph``, and ``GET /graph`` so they
    cannot drift. Two properties of that arrangement are worth stating because
    they look like violations and are not:

    * **It reads a cached sash observation, never a live one.** ``build_status``
      is contractually side-effect-free and polled every 2-3s, so it must not
      make an outbound HTTP call. The cache can therefore be a couple of
      seconds stale, and a client could in principle read this list, have the
      sash close, and eat a 412. That is not a §6.2 disagreement: §6.2
      constrains the two surfaces' *rules*, not the world between two HTTP
      calls, and both surfaces run the identical decision function over the
      identical reading. It is structurally the same race this file already
      accepts for ``motion_in_progress`` — another client can take the motion
      slot between a poll and a POST — and when it does bite, the 412 is the
      correct answer.
    * **When the interlock is blind it advertises the gated targets.** That
      mirrors what the endpoint would do (the configured policy is fail-open),
      which is the point. Withholding instead would produce "endpoint allows,
      /status withholds", stalling exactly the well-behaved workflows that
      consult this list while a client POSTing blindly sailed through.

    The formats ``"move.<node_id>"`` and ``"gripper.<state>"`` mirror the
    dotted convention from other v1.1 devices (e.g. ``"seal.start"``,
    ``"stage.in"``).
    """
    actions: list[str] = []

    if equipment_status == "requires_init":
        actions.append("connect")
        return actions

    if has_error:
        # Error state: the only meaningful actions are recovery ones.
        actions.extend(["clear_errors", "stop"])
        return actions

    # dry_run is the simulator session's "ready"/"busy" (see build_status):
    # the sim honors exactly the same actions, so it advertises them too.
    if equipment_status in ("ready", "busy", "degraded", "dry_run"):
        # Stop is always available while the device is reachable; spec
        # treats it as the safety floor.
        actions.append("stop")

        if activity == "running":
            # A motion is in flight. Starting a second one would be a
            # collision, and the move endpoints refuse it with 409, so the
            # list must not offer it either.
            return actions

        if getattr(controller, "is_real_box_simulating", False):
            # The real box is in Studio-Sim: every motion and gripper
            # endpoint returns 412 (box_sim_guard), because the SDK would
            # report success without moving. §6.2 requires this surface to
            # agree, so no move target is offered. `stop` stays — it is the
            # safety floor and is never gated.
            #
            # Note this branch is NOT reached for the Docker simulator,
            # which reports the same underlying bit but stays fully
            # actuable as the supported dry-run path.
            return actions

        graph = getattr(controller, "motion_graph", None)
        graph_mode = getattr(controller, "graph_mode", None)
        # Compare on .value to avoid having to import GraphMode here
        # (which would risk a second module load under test conditions).
        graph_mode_value = getattr(graph_mode, "value", graph_mode)
        if graph is not None:
            strict = graph_mode_value == "strict"
            has_gripper = bool(controller.has_gripper())
            move_targets = list(controller.reachable_node_ids()) if strict else []
            gripper_targets = (
                list(controller.allowed_gripper_targets())
                if strict and has_gripper
                else []
            )

            # Source 4: catalog family names. Advertised iff a POST to the
            # endpoint would not be *state*-refused (§6.2); refusals that
            # depend on the request's arguments (unknown node, off-whitelist
            # transition, recovery mismatch) are 409/422s that a flat action
            # list cannot and need not predict.
            #
            # * ``graph.move_to`` / ``graph.gripper`` — in STRICT, only
            #   while at least one whitelisted target exists (with none,
            #   every request 409s). In ADVISORY/OFF the endpoints honor
            #   any target (warn + proceed), so the family name is
            #   advertised there even though no per-target enumeration is
            #   possible — closing the understatement that previously left
            #   these modes advertising nothing at all.
            # * ``graph.recover_to`` / ``graph.mode`` — honored whenever a
            #   graph is loaded.
            # * ``graph.record`` — mirrors the endpoint's own gates: 412
            #   for any simulator (a simulated move validates no geometry,
            #   so the edge must not be recorded) and 409 when there is no
            #   last transition to record.
            if not strict or move_targets:
                actions.append("graph.move_to")
            if has_gripper and (not strict or gripper_targets):
                actions.append("graph.gripper")
            actions.append("graph.recover_to")
            actions.append("graph.mode")
            if (
                not getattr(controller, "is_simulated", False)
                and getattr(controller, "last_transition", None) is not None
            ):
                actions.append("graph.record")

            # Source 2: per-target enumeration, STRICT only. ADVISORY/OFF
            # would still allow off-whitelist moves, so a per-target list
            # there would understate capability and mislead SDK callers.
            if strict:
                for node_id in move_targets:
                    actions.append(f"move.{node_id}")

                # Gripper transitions are whitelisted per (node, current state)
                # exactly as move targets are, so enumerate them the same way:
                # one action per reachable catalog state. The list is then
                # precisely what POST /control/graph/gripper would honor —
                # §6.2's requirement — because both surfaces read the same
                # ``allowed_gripper_targets()``, so they cannot drift. A single
                # ``gripper.set`` action could not express *which* states are
                # legal here, so a caller reading the list would still have to
                # guess and eat a 409.
                #
                # Every gate above applies unchanged. In particular the
                # motion-in-flight early return covers the gripper too, and must:
                # the stroke is invariant during arm motion, which is why the
                # endpoint itself requires a stationary arm.
                for state in gripper_targets:
                    actions.append(f"gripper.{state}")

    return actions


def _build_gripper_details(controller: XArmController) -> dict[str, Any] | None:
    """Cached BIO gripper status/error register snapshot for /status.

    This is the device-side surface of the plate-transfer verification
    signal: ``object_detected`` distinguishes a real pickup from the jaws
    closing on empty deck, and ``error_code`` 12 ("object slipped")
    flags a mid-move drop. All values come from the controller's
    ``refresh_gripper_status`` cache, so this reader is side-effect-free.

    Returns ``None`` when nothing concrete has been cached yet (non-BIO
    gripper, or the gripper hasn't moved this session) so the field
    doesn't pollute /status for arms that never grip.

    Field shape::

        {
          "motion_state":    "stop" | "moving" | "object_detected" | "fault",
          "object_detected": bool,
          "position_mm":     float,   # actual read-back jaw position
          "error_code":      int,     # BIO register 0x0F; 0 == OK, 12 == slipped
          "error_text":      str      # present only when error_code != 0
        }
    """
    block: dict[str, Any] = {}

    motion_state = getattr(controller, "last_gripper_motion_state", None)
    if isinstance(motion_state, str):
        block["motion_state"] = motion_state

    detected = getattr(controller, "last_gripper_object_detected", None)
    if isinstance(detected, bool):
        block["object_detected"] = detected

    position = getattr(controller, "last_gripper_position_actual", None)
    if isinstance(position, (int, float)) and not isinstance(position, bool):
        block["position_mm"] = float(position)

    error_code = getattr(controller, "last_gripper_error_code", None)
    if isinstance(error_code, int) and not isinstance(error_code, bool):
        block["error_code"] = error_code
        error_text = getattr(controller, "last_gripper_error_text", None)
        if error_code != 0 and isinstance(error_text, str):
            block["error_text"] = error_text

    return block or None


def _build_motion_graph_details(controller: XArmController) -> dict[str, Any] | None:
    """Read-only snapshot of the motion-graph layer for the dashboard.

    Returns None when no graph is loaded (legacy / unmigrated configs)
    so the field doesn't appear in /status.details at all. When loaded:

        current_node             graph node id matching the controller's
                                 arm+rail position, or None when off-grid
                                 (post-STOP, after raw moves)
        reachable_nodes          outgoing target ids traversable with the
                                 current gripper state (empty list when
                                 off-grid or state unknown)
        travel_targets           every node reachable in >= 1 hops with the
                                 current gripper state (multi-hop superset
                                 of reachable_nodes; feeds travel_to)
        graph_mode               "off" | "advisory" | "strict"
        gripper_stroke           last commanded gripper stroke (float | None)
        gripper_state            catalog state name resolved from the stroke,
                                 or None when off-catalog
        allowed_gripper_targets  gripper states reachable via the current
                                 node's transition whitelist
    """
    graph = getattr(controller, "motion_graph", None)
    if graph is None:
        return None
    return {
        "current_node": controller.current_node,
        # Both target lists are filtered by the sash interlock (a no-op when
        # it is unconfigured or satisfied). travel_targets needs the same
        # treatment as reachable_nodes or the panel's Travel dropdown would
        # still offer hood nodes the endpoint refuses. Neither call fetches.
        "reachable_nodes": controller.reachable_node_ids(),
        "travel_targets": _filter_sash_gated(
            controller,
            graph.reachable_set(
                controller.current_node, controller.current_gripper_state,
            ),
        ),
        "graph_mode": getattr(controller, "graph_mode").value,
        "gripper_stroke": getattr(controller, "last_gripper_position", None),
        "gripper_state": controller.current_gripper_state,
        "allowed_gripper_targets": controller.allowed_gripper_targets(),
        "arm_pose_name": getattr(controller, "last_arm_pose_name", None),
        "rail_location_name": getattr(controller, "last_rail_location_name", None),
    }


def _build_connection_details(controller: XArmController) -> dict[str, Any] | None:
    """Best-effort connection panel data for the local web UI.

    Lives under ``details`` (not contracted by STATUS_SPEC) to keep the
    UI's "Host: ... Port: ..." widget working without breaking the spec
    envelope.
    """
    host = getattr(controller, "host", None)
    if host is None:
        return None
    cfg = getattr(controller, "xarm_config", {}) or {}
    return {
        "host": host,
        "port": cfg.get("port", 18333),
        "profile_name": getattr(controller, "profile_name", None) or "unknown",
        "gripper_type": getattr(controller, "gripper_type", None) or "N/A",
        "gripper_config": getattr(controller, "current_gripper_config", {}),
    }
