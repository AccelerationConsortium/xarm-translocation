"""Natural-language command layer for the motion graph.

Turns a free-text operator utterance (e.g. "go to deck home", "pick up
the tray from deck slot 1") into a **structured, graph-validated plan**
that reuses the existing graph-gated control surface (``move_to_node`` /
``set_gripper_state`` under STRICT enforcement).

Two-stage design, with a hard wall between them:

1. **Interpretation (LLM).** :class:`NLCommandInterpreter` asks Claude
   Haiku to translate language into a structured :class:`NLIntent`
   (``move_to`` a node, ``set_gripper`` to a catalog state, or ``none``).
   The LLM *never* commands the robot — it only names an intent, using a
   forced tool-use schema so the reply is always structured, never prose.

2. **Planning / validation (deterministic).** :func:`plan_from_intent`
   takes that intent plus a live snapshot of the graph state and decides
   whether it is executable, producing an :class:`NLPlan` of concrete
   steps. This is pure Python — no LLM, fully unit-testable.

v1 scope: a plan is executable only when its single step is *directly*
available from the current state (the target node is an outgoing neighbor,
or the gripper target is a whitelisted transition at the current node).
The :class:`NLPlan` step list is intentionally a sequence so a future
multi-hop planner (BFS over the graph) can drop into
:func:`plan_from_intent` without touching the LLM layer or the API/UI.
"""

from __future__ import annotations

import json
import logging
import os
from dataclasses import dataclass, field
from typing import Any, Optional

logger = logging.getLogger(__name__)

# Default model. Overridable via XARM_LLM_MODEL so the model can be bumped
# without a code change. The alias tracks the latest Haiku point release.
DEFAULT_MODEL = "claude-haiku-4-5"

# Structured-output tool the LLM is forced to call. Keeping the schema here
# (not inline in the request) makes it easy to extend later.
_ROBOT_COMMAND_TOOL: dict[str, Any] = {
    "name": "robot_command",
    "description": (
        "Report the single robot action that best matches the operator's "
        "natural-language command, given the robot's current state and the "
        "catalog of known positions. Always call this tool exactly once."
    ),
    "input_schema": {
        "type": "object",
        "properties": {
            "action": {
                "type": "string",
                "enum": ["move_to", "set_gripper", "none"],
                "description": (
                    "move_to: drive the arm to a named graph node. "
                    "set_gripper: change the gripper to a catalog state while "
                    "parked. none: the command could not be mapped to an action."
                ),
            },
            "target": {
                "type": ["string", "null"],
                "description": (
                    "For move_to: the exact graph node id (e.g. 'deck_home'). "
                    "For set_gripper: the exact gripper-state name (e.g. "
                    "'grip_120'). Null when action is 'none'. Always use an id "
                    "from the provided catalogs — never invent one."
                ),
            },
            "explanation": {
                "type": "string",
                "description": (
                    "One short sentence, addressed to the operator, explaining "
                    "how the command was interpreted."
                ),
            },
            "error": {
                "type": ["string", "null"],
                "description": (
                    "When action is 'none', a short reason the command could "
                    "not be understood or mapped. Null otherwise."
                ),
            },
        },
        "required": ["action", "explanation"],
    },
}


# ── Domain types ─────────────────────────────────────────────────────


@dataclass
class NLIntent:
    """A structured interpretation of one natural-language command.

    Produced by the LLM (or by a fallback path when the LLM is
    unavailable). This is a *claim about intent*, not a validated action —
    :func:`plan_from_intent` decides whether it can actually run.
    """

    action: str = "none"          # "move_to" | "set_gripper" | "none"
    target: Optional[str] = None
    explanation: str = ""
    error: Optional[str] = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "action": self.action,
            "target": self.target,
            "explanation": self.explanation,
            "error": self.error,
        }


@dataclass
class NLStep:
    """One concrete, executable step of an :class:`NLPlan`.

    ``type`` mirrors the control endpoints: ``move_to`` -> node move,
    ``set_gripper`` -> gripper transition.
    """

    type: str          # "move_to" | "set_gripper"
    target: str

    def to_dict(self) -> dict[str, Any]:
        return {"type": self.type, "target": self.target}


@dataclass
class NLPlan:
    """A validated plan derived from an :class:`NLIntent`.

    ``steps`` is an ordered list so multi-hop planning can be added later
    without changing the shape the API/UI consume. In v1 a valid plan has
    exactly one step. ``valid`` gates execution; ``reason`` explains a
    rejection (or narrates a valid plan) for display to the operator.
    """

    steps: list[NLStep] = field(default_factory=list)
    explanation: str = ""
    valid: bool = False
    reason: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "steps": [s.to_dict() for s in self.steps],
            "explanation": self.explanation,
            "valid": self.valid,
            "reason": self.reason,
        }


# ── Context snapshot ─────────────────────────────────────────────────


def build_context(controller: Any) -> dict[str, Any]:
    """Assemble the read-only graph snapshot the interpreter and planner need.

    Pulls the live state (current node, gripper state, what's directly
    reachable) plus the full node/gripper catalogs so the LLM can
    fuzzy-match loose phrasing ("deck slot 1", "the shaker") to exact
    node ids. Safe to call whether or not a graph is loaded.
    """
    graph = getattr(controller, "motion_graph", None)
    if graph is None:
        return {
            "current_node": None,
            "gripper_state": None,
            "reachable_nodes": [],
            "allowed_gripper_targets": [],
            "nodes": [],
            "gripper_states": [],
        }

    nodes = [
        {"id": n.id, "tags": list(n.tags), "gripper_states": list(n.gripper_states)}
        for n in graph.nodes
    ]
    gripper_states = [
        {"name": gs.name, "stroke": gs.stroke, "intent": gs.intent.value}
        for gs in graph.gripper_states
    ]
    return {
        "current_node": controller.current_node,
        "gripper_state": controller.current_gripper_state,
        "reachable_nodes": controller.reachable_node_ids(),
        "allowed_gripper_targets": controller.allowed_gripper_targets(),
        "nodes": nodes,
        "gripper_states": gripper_states,
    }


# ── Deterministic planner / validator ────────────────────────────────


def plan_from_intent(intent: NLIntent, context: dict[str, Any]) -> NLPlan:
    """Validate an :class:`NLIntent` against the live graph snapshot.

    Pure, LLM-free, deterministic. This is the single source of truth for
    "can this run right now?" and the seam where a future BFS multi-hop
    planner will expand a distant target into a sequence of steps. Today it
    only accepts targets that are *directly* available from the current
    state.
    """
    node_ids = {n["id"] for n in context.get("nodes", [])}
    gripper_names = {gs["name"] for gs in context.get("gripper_states", [])}
    current_node = context.get("current_node")
    reachable = set(context.get("reachable_nodes") or [])
    allowed_gripper = set(context.get("allowed_gripper_targets") or [])

    if intent.action == "none":
        return NLPlan(
            steps=[],
            explanation=intent.explanation,
            valid=False,
            reason=intent.error or "Could not interpret the command.",
        )

    if intent.action == "move_to":
        target = intent.target
        if not target or target not in node_ids:
            return NLPlan(
                steps=[], explanation=intent.explanation, valid=False,
                reason=f"Unknown location: {target!r}.",
            )
        if current_node is None:
            return NLPlan(
                steps=[], explanation=intent.explanation, valid=False,
                reason=(
                    "Current position is off-grid; recover to a known node "
                    "before issuing moves."
                ),
            )
        if target == current_node:
            return NLPlan(
                steps=[], explanation=intent.explanation, valid=False,
                reason=f"Already at {target!r}.",
            )
        if target not in reachable:
            # v1 limitation: only direct neighbors. This is exactly where
            # the future multi-hop planner would compute a path instead.
            return NLPlan(
                steps=[], explanation=intent.explanation, valid=False,
                reason=(
                    f"{target!r} is not directly reachable from "
                    f"{current_node!r}. Reachable now: "
                    f"{sorted(reachable) or 'none'}."
                ),
            )
        return NLPlan(
            steps=[NLStep(type="move_to", target=target)],
            explanation=intent.explanation,
            valid=True,
            reason=f"Move from {current_node!r} to {target!r}.",
        )

    if intent.action == "set_gripper":
        target = intent.target
        if not target or target not in gripper_names:
            return NLPlan(
                steps=[], explanation=intent.explanation, valid=False,
                reason=f"Unknown gripper state: {target!r}.",
            )
        if current_node is None:
            return NLPlan(
                steps=[], explanation=intent.explanation, valid=False,
                reason=(
                    "Current position is off-grid; recover to a known node "
                    "before changing the gripper."
                ),
            )
        if target == context.get("gripper_state"):
            return NLPlan(
                steps=[], explanation=intent.explanation, valid=False,
                reason=f"Gripper already at {target!r}.",
            )
        if target not in allowed_gripper:
            return NLPlan(
                steps=[], explanation=intent.explanation, valid=False,
                reason=(
                    f"Gripper transition to {target!r} is not allowed at "
                    f"{current_node!r}. Allowed now: "
                    f"{sorted(allowed_gripper) or 'none'}."
                ),
            )
        return NLPlan(
            steps=[NLStep(type="set_gripper", target=target)],
            explanation=intent.explanation,
            valid=True,
            reason=f"Set gripper to {target!r} at {current_node!r}.",
        )

    return NLPlan(
        steps=[], explanation=intent.explanation, valid=False,
        reason=f"Unsupported action: {intent.action!r}.",
    )


# ── LLM interpreter ──────────────────────────────────────────────────


class NLCommandError(Exception):
    """Raised when interpretation cannot be performed (LLM unavailable or
    the API call failed)."""


class NLCommandInterpreter:
    """Translates natural language into an :class:`NLIntent` via Claude Haiku.

    The Anthropic client is created lazily and only when an API key is
    present, so importing this module (and running the rest of the server)
    never requires the ``anthropic`` package or a key. Check
    :attr:`available` before calling :meth:`interpret`.
    """

    def __init__(
        self,
        model: Optional[str] = None,
        api_key: Optional[str] = None,
    ):
        self.model = model or os.environ.get("XARM_LLM_MODEL", DEFAULT_MODEL)
        self._api_key = api_key or os.environ.get("ANTHROPIC_API_KEY")
        self._client: Any = None

    @property
    def available(self) -> bool:
        """True when an API key is configured and the SDK is importable."""
        if not self._api_key:
            return False
        try:
            import anthropic  # noqa: F401
        except ImportError:
            return False
        return True

    def _get_client(self) -> Any:
        if self._client is not None:
            return self._client
        if not self._api_key:
            raise NLCommandError(
                "ANTHROPIC_API_KEY is not set; natural-language commands are "
                "disabled."
            )
        try:
            import anthropic
        except ImportError as exc:
            raise NLCommandError(
                "the 'anthropic' package is not installed; run "
                "`pip install anthropic`."
            ) from exc
        self._client = anthropic.Anthropic(api_key=self._api_key)
        return self._client

    def _system_prompt(self, context: dict[str, Any]) -> str:
        nodes = context.get("nodes", [])
        gripper_states = context.get("gripper_states", [])
        return (
            "You are the command interpreter for a lab robot arm on a linear "
            "rail with a gripper. You translate an operator's natural-language "
            "instruction into ONE structured action by calling the "
            "'robot_command' tool. You never chat.\n\n"
            "Model:\n"
            "- A 'node' is a named arm position (arm pose + rail location).\n"
            "- 'move_to' drives the arm to a node.\n"
            "- 'set_gripper' changes the gripper state (e.g. grip a tray, "
            "release) while parked at a node.\n\n"
            "Rules:\n"
            "- 'target' MUST be an exact id from the catalogs below. Match "
            "loose phrasing to the closest id (e.g. 'deck slot 1' -> a "
            "'deck_slot1_*' node, 'go home' -> 'robot_home', 'grab the tray' "
            "-> a grasp gripper state like 'grip_120').\n"
            "- Prefer a node that is currently reachable when the phrasing is "
            "ambiguous, but always return your best-guess target even if it is "
            "not reachable — a separate validator decides feasibility.\n"
            "- If the command does not correspond to any known action or "
            "target, use action 'none' and set 'error'.\n\n"
            "Current state:\n"
            f"- current_node: {context.get('current_node')!r}\n"
            f"- gripper_state: {context.get('gripper_state')!r}\n"
            f"- directly reachable nodes: {context.get('reachable_nodes')}\n"
            f"- allowed gripper transitions now: "
            f"{context.get('allowed_gripper_targets')}\n\n"
            "Node catalog (id, tags):\n"
            + "\n".join(
                f"- {n['id']} tags={n['tags']}" for n in nodes
            )
            + "\n\nGripper-state catalog (name, intent):\n"
            + "\n".join(
                f"- {gs['name']} intent={gs['intent']}" for gs in gripper_states
            )
        )

    def interpret(self, text: str, context: dict[str, Any]) -> NLIntent:
        """Call the LLM and return a structured :class:`NLIntent`.

        Raises :class:`NLCommandError` when the LLM is unavailable or the
        call fails. Never raises on a merely-unmappable command — that comes
        back as an ``action='none'`` intent.
        """
        client = self._get_client()
        try:
            message = client.messages.create(
                model=self.model,
                max_tokens=512,
                system=self._system_prompt(context),
                tools=[_ROBOT_COMMAND_TOOL],
                tool_choice={"type": "tool", "name": "robot_command"},
                messages=[{"role": "user", "content": text}],
            )
        except Exception as exc:  # network / auth / rate-limit / SDK errors
            logger.warning("LLM interpret call failed: %r", exc)
            raise NLCommandError(f"LLM request failed: {exc}") from exc

        payload = self._extract_tool_input(message)
        if payload is None:
            raise NLCommandError("LLM returned no structured tool call.")
        return NLIntent(
            action=str(payload.get("action", "none")),
            target=payload.get("target"),
            explanation=str(payload.get("explanation", "")),
            error=payload.get("error"),
        )

    @staticmethod
    def _extract_tool_input(message: Any) -> Optional[dict[str, Any]]:
        """Pull the tool_use input dict out of an Anthropic Messages reply."""
        for block in getattr(message, "content", []) or []:
            if getattr(block, "type", None) == "tool_use":
                data = getattr(block, "input", None)
                if isinstance(data, dict):
                    return data
                # Some SDK versions surface input as a JSON string.
                if isinstance(data, str):
                    try:
                        return json.loads(data)
                    except json.JSONDecodeError:
                        return None
        return None
