"""OpenRouter natural-language -> structured intent layer.

The LLM's ONLY job is to translate an operator's free-text request into
exactly one structured tool call (``move_to`` / ``pick`` / ``place`` /
``set_gripper`` / ``go_home``) drawn from the motion-graph-derived
catalog, or to reply in plain text when it needs clarification or the
request is out of scope. It never plans motion, never talks to hardware,
and never sees a claim token — all of that stays in
``assistant_actions`` + the controller, keeping the model out of the
safety loop.

The model is reached through OpenRouter's OpenAI-compatible API using
the ``openai`` SDK pointed at ``https://openrouter.ai/api/v1``.

Graceful degradation: if the ``openai`` package is not installed or
``OPENROUTER_API_KEY`` is unset (or ``XARM_ASSISTANT_ENABLED=0``), the
assistant reports itself disabled and the API layer returns a clear
"assistant unavailable" message instead of raising.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from typing import Any

from dotenv import load_dotenv

from .assistant_actions import Place

# Load a local .env (git-ignored) so OPENROUTER_API_KEY can live in a file
# instead of a shell export. Existing environment variables take precedence.
load_dotenv()

# OpenRouter base URL for the OpenAI-compatible client.
_OPENROUTER_BASE_URL = "https://openrouter.ai/api/v1"

# Default to GLM 5.2 via OpenRouter; override with XARM_ASSISTANT_MODEL
# to use any other OpenRouter slug (e.g. openai/gpt-4o-mini).
DEFAULT_MODEL = "z-ai/glm-5.2"

_ACTION_TOOL_NAME = "control_robot"

_SYSTEM_PROMPT = """\
You are the motion-control assistant for a single UFACTORY xArm \
translocation robot in a chemistry lab. The robot has an arm on a linear \
rail and a gripper, and moves labware trays between fixed stations.

Your ONLY capability is to translate the operator's request into exactly \
one call of the `control_robot` tool. You do NOT plan robot paths, run \
lab workflows, or control any other instrument (no Opentrons liquid \
handling, no plate reader, no sealer, no shaker chemistry) — only the \
physical motion of THIS arm/gripper.

Rules:
- Pick the single action that best matches the request and call \
`control_robot` once. Do not narrate the steps; the backend computes and \
verifies the actual path.
- `move_to`: reposition the arm at a place (no grip change).
- `pick`: go to a place and grip the tray there.
- `place`: go to a place and release the tray there.
- `set_gripper`: change only the gripper (e.g. open/close).
- `go_home`: return to the safe home pose.
- Only use `place`/`gripper` values from the provided enums. If the \
request is ambiguous, names an unknown location, or is outside motion \
control, DO NOT call the tool — reply briefly in plain text asking for \
clarification or explaining you can't do it.
"""


@dataclass
class Interpretation:
    """Result of one interpret() call.

    Exactly one of ``action`` (a tool call) or ``reply`` (plain-text
    clarification/refusal) is meaningful. ``action`` is None when the
    model chose to answer in text instead of calling the tool.
    """

    action: str | None = None
    args: dict[str, Any] = field(default_factory=dict)
    reply: str | None = None
    raw_model: str | None = None


class AssistantDisabled(Exception):
    """Raised when the assistant cannot run (no lib / no key / disabled).

    Carries a human-readable ``reason`` the API layer surfaces verbatim.
    """

    def __init__(self, reason: str):
        self.reason = reason
        super().__init__(reason)


class AssistantError(Exception):
    """The LLM call failed at runtime (network, API error, bad output)."""


def _truthy(value: str | None, default: bool) -> bool:
    if value is None:
        return default
    return value.strip().lower() not in {"0", "false", "no", "off", ""}


def is_enabled() -> bool:
    """True if the assistant could run right now (lib + key + toggle)."""
    if not _truthy(os.environ.get("XARM_ASSISTANT_ENABLED"), default=True):
        return False
    if not os.environ.get("OPENROUTER_API_KEY"):
        return False
    try:
        import openai  # noqa: F401
    except ImportError:
        return False
    return True


def disabled_reason() -> str | None:
    """Explain why the assistant is disabled, or None if it's available."""
    if not _truthy(os.environ.get("XARM_ASSISTANT_ENABLED"), default=True):
        return "assistant disabled via XARM_ASSISTANT_ENABLED=0"
    if not os.environ.get("OPENROUTER_API_KEY"):
        return "assistant unavailable: OPENROUTER_API_KEY is not set"
    try:
        import openai  # noqa: F401
    except ImportError:
        return (
            "assistant unavailable: the 'openai' package is not installed "
            "(pip install openai)"
        )
    return None


def model_name() -> str:
    return os.environ.get("XARM_ASSISTANT_MODEL", DEFAULT_MODEL).strip() or DEFAULT_MODEL


def build_tool_schema(
    catalog: dict[str, Place], gripper_states: list[str],
) -> dict[str, Any]:
    """JSON-schema tool definition whose enums are the live catalog.

    Constraining ``place`` and ``gripper_state`` to actual keys makes the
    model's output directly resolvable and keeps it from inventing
    locations. The enum descriptions include human labels so the model
    can map phrasing like "slot 1" to ``deck_slot1``.
    """
    place_keys = sorted(catalog)
    place_lines = "; ".join(
        f"{p.key} = {p.label}" for p in sorted(catalog.values(), key=lambda x: x.key)
    )
    return {
        "type": "function",
        "function": {
            "name": _ACTION_TOOL_NAME,
            "description": (
                "Execute one motion-control action on the xArm. Places: "
                f"{place_lines}."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["move_to", "pick", "place", "set_gripper", "go_home"],
                        "description": "The single action to perform.",
                    },
                    "place": {
                        "type": "string",
                        "enum": place_keys,
                        "description": (
                            "Target location key. Required for move_to/pick/place."
                        ),
                    },
                    "gripper_state": {
                        "type": "string",
                        "enum": sorted(gripper_states),
                        "description": (
                            "Target gripper catalog state. Required for set_gripper."
                        ),
                    },
                },
                "required": ["action"],
                "additionalProperties": False,
            },
        },
    }


def _state_preamble(
    current_node: str | None,
    current_gripper_state: str | None,
    catalog: dict[str, Place],
) -> str:
    node_txt = current_node or "unknown (off-grid — not pinned to a node)"
    grip_txt = current_gripper_state or "unknown"
    place_list = ", ".join(sorted(catalog)) or "(none)"
    return (
        f"Current robot state -> node: {node_txt}; gripper: {grip_txt}.\n"
        f"Known places: {place_list}."
    )


def interpret(
    message: str,
    *,
    catalog: dict[str, Place],
    gripper_states: list[str],
    current_node: str | None,
    current_gripper_state: str | None,
    history: list[dict[str, str]] | None = None,
    max_tokens: int = 512,
) -> Interpretation:
    """Send ``message`` to the model via OpenRouter and return an intent.

    ``history`` is an optional list of prior ``{"role", "content"}`` turns
    (user/assistant text only) for light multi-turn context. Raises
    ``AssistantDisabled`` when unavailable and ``AssistantError`` on a
    runtime failure.
    """
    reason = disabled_reason()
    if reason is not None:
        raise AssistantDisabled(reason)

    import openai

    client = openai.OpenAI(
        api_key=os.environ["OPENROUTER_API_KEY"],
        base_url=_OPENROUTER_BASE_URL,
    )
    tool = build_tool_schema(catalog, gripper_states)

    messages: list[dict[str, Any]] = [
        {"role": "system", "content": _SYSTEM_PROMPT},
    ]
    for turn in history or []:
        role = turn.get("role")
        content = turn.get("content")
        if role in ("user", "assistant") and content:
            messages.append({"role": role, "content": str(content)})
    messages.append({
        "role": "user",
        "content": f"{_state_preamble(current_node, current_gripper_state, catalog)}\n\n"
                   f"Request: {message.strip()}",
    })

    model = model_name()
    try:
        response = client.chat.completions.create(
            model=model,
            max_tokens=max_tokens,
            tools=[tool],
            tool_choice="auto",
            messages=messages,
        )
    except Exception as exc:  # openai.APIError and friends
        raise AssistantError(f"LLM request failed: {exc}") from exc

    choice_message = response.choices[0].message
    tool_calls = getattr(choice_message, "tool_calls", None) or []
    tool_call = next(
        (
            tc for tc in tool_calls
            if getattr(getattr(tc, "function", None), "name", None) == _ACTION_TOOL_NAME
        ),
        None,
    )
    text = (choice_message.content or "").strip()

    if tool_call is not None:
        try:
            args = json.loads(tool_call.function.arguments or "{}")
        except (json.JSONDecodeError, TypeError) as exc:
            raise AssistantError(f"LLM returned invalid tool arguments: {exc}") from exc
        if not isinstance(args, dict):
            args = {}
        return Interpretation(
            action=args.get("action"),
            args={k: v for k, v in args.items() if k != "action"},
            reply=text or None,
            raw_model=model,
        )

    return Interpretation(
        action=None,
        reply=text or "Sorry, I couldn't interpret that as a robot motion command.",
        raw_model=model,
    )
