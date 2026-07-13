"""Unit tests for the natural-language command layer (src/core/nl_command.py).

Covers:
- the deterministic planner/validator (plan_from_intent) against a graph
  snapshot: reachable move accepted, non-adjacent move rejected, off-grid,
  already-there, unknown target, gripper transitions accepted/rejected,
  and 'none' intent handling
- build_context shape from a stub controller
- NLCommandInterpreter availability gating (no key / no target)
- the interpreter's tool-use parsing with a fully mocked Anthropic client
  (no network, no API key required)
"""

from __future__ import annotations

import os
import sys
import types

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core.motion_graph import MotionGraph
from src.core.nl_command import (
    DEFAULT_MODEL,
    NLCommandError,
    NLCommandInterpreter,
    NLIntent,
    build_context,
    plan_from_intent,
)


REAL_YAML = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'src', 'settings', 'motion_graph.yaml')
)


def _context(**overrides):
    """A graph snapshot as build_context() would produce it.

    Default: parked at deck_high with an empty gripper; deck_home and
    deck_slot1_high are directly reachable.
    """
    ctx = {
        "current_node": "deck_high",
        "gripper_state": "empty",
        "reachable_nodes": ["deck_home", "deck_slot1_high"],
        "allowed_gripper_targets": [],
        "nodes": [
            {"id": "deck_home", "tags": ["deck"], "gripper_states": ["empty"]},
            {"id": "deck_high", "tags": ["deck"], "gripper_states": ["empty"]},
            {"id": "deck_slot1_high", "tags": ["deck", "slot1"],
             "gripper_states": ["empty", "grip_120"]},
            {"id": "deck_slot1_low", "tags": ["deck", "slot1"],
             "gripper_states": ["empty", "grip_120"]},
        ],
        "gripper_states": [
            {"name": "empty", "stroke": 150, "intent": "none"},
            {"name": "grip_120", "stroke": 120, "intent": "grasp"},
        ],
    }
    ctx.update(overrides)
    return ctx


# ── plan_from_intent: move_to ────────────────────────────────────────


def test_move_to_reachable_node_is_valid():
    intent = NLIntent(action="move_to", target="deck_home", explanation="go home")
    plan = plan_from_intent(intent, _context())
    assert plan.valid is True
    assert len(plan.steps) == 1
    assert plan.steps[0].type == "move_to"
    assert plan.steps[0].target == "deck_home"


def test_move_to_non_adjacent_node_is_rejected_with_reason():
    # deck_slot1_low is a real node but not directly reachable from deck_high.
    intent = NLIntent(action="move_to", target="deck_slot1_low")
    plan = plan_from_intent(intent, _context())
    assert plan.valid is False
    assert plan.steps == []
    assert "not directly reachable" in plan.reason


def test_move_to_unknown_node_is_rejected():
    intent = NLIntent(action="move_to", target="nowhere")
    plan = plan_from_intent(intent, _context())
    assert plan.valid is False
    assert "Unknown location" in plan.reason


def test_move_to_when_off_grid_is_rejected():
    intent = NLIntent(action="move_to", target="deck_home")
    plan = plan_from_intent(intent, _context(current_node=None, reachable_nodes=[]))
    assert plan.valid is False
    assert "off-grid" in plan.reason


def test_move_to_current_node_reports_already_there():
    intent = NLIntent(action="move_to", target="deck_high")
    plan = plan_from_intent(intent, _context())
    assert plan.valid is False
    assert "Already at" in plan.reason


# ── plan_from_intent: set_gripper ────────────────────────────────────


def test_set_gripper_allowed_transition_is_valid():
    ctx = _context(
        current_node="deck_slot1_low",
        gripper_state="empty",
        reachable_nodes=["deck_slot1_high"],
        allowed_gripper_targets=["grip_120"],
    )
    intent = NLIntent(action="set_gripper", target="grip_120", explanation="grab tray")
    plan = plan_from_intent(intent, ctx)
    assert plan.valid is True
    assert plan.steps[0].type == "set_gripper"
    assert plan.steps[0].target == "grip_120"


def test_set_gripper_not_whitelisted_is_rejected():
    # empty gripper at deck_high has no allowed transitions.
    intent = NLIntent(action="set_gripper", target="grip_120")
    plan = plan_from_intent(intent, _context())
    assert plan.valid is False
    assert "not allowed" in plan.reason


def test_set_gripper_unknown_state_is_rejected():
    intent = NLIntent(action="set_gripper", target="grip_999")
    plan = plan_from_intent(intent, _context(allowed_gripper_targets=["grip_120"]))
    assert plan.valid is False
    assert "Unknown gripper state" in plan.reason


def test_set_gripper_already_in_state_is_rejected():
    ctx = _context(
        current_node="deck_slot1_low",
        gripper_state="grip_120",
        allowed_gripper_targets=["empty"],
    )
    intent = NLIntent(action="set_gripper", target="grip_120")
    plan = plan_from_intent(intent, ctx)
    assert plan.valid is False
    assert "already at" in plan.reason


# ── plan_from_intent: none / unsupported ─────────────────────────────


def test_none_intent_is_invalid_and_carries_error():
    intent = NLIntent(action="none", explanation="", error="I did not understand.")
    plan = plan_from_intent(intent, _context())
    assert plan.valid is False
    assert plan.reason == "I did not understand."


def test_unsupported_action_is_invalid():
    intent = NLIntent(action="teleport", target="deck_home")
    plan = plan_from_intent(intent, _context())
    assert plan.valid is False
    assert "Unsupported action" in plan.reason


# ── build_context ────────────────────────────────────────────────────


class _StubController:
    """Minimal controller surface build_context() reads."""

    def __init__(self, graph):
        self.motion_graph = graph
        self.current_node = "robot_home"
        self.current_gripper_state = "empty"

    def reachable_node_ids(self):
        return ["deck_home", "opentrons_home"]

    def allowed_gripper_targets(self):
        return []


def test_build_context_from_real_graph():
    graph = MotionGraph.from_yaml(REAL_YAML)
    ctx = build_context(_StubController(graph))
    assert ctx["current_node"] == "robot_home"
    assert ctx["gripper_state"] == "empty"
    assert ctx["reachable_nodes"] == ["deck_home", "opentrons_home"]
    # Catalog is populated from the graph.
    node_ids = {n["id"] for n in ctx["nodes"]}
    assert "deck_home" in node_ids and "deck_slot1_low" in node_ids
    gs_names = {gs["name"] for gs in ctx["gripper_states"]}
    assert "empty" in gs_names and "grip_120" in gs_names


def test_build_context_without_graph_is_safe():
    ctx = build_context(types.SimpleNamespace(motion_graph=None))
    assert ctx["current_node"] is None
    assert ctx["nodes"] == []
    assert ctx["reachable_nodes"] == []


# ── NLCommandInterpreter ─────────────────────────────────────────────


def test_interpreter_default_model_and_env_override(monkeypatch):
    monkeypatch.delenv("XARM_LLM_MODEL", raising=False)
    assert NLCommandInterpreter(api_key="k").model == DEFAULT_MODEL
    monkeypatch.setenv("XARM_LLM_MODEL", "claude-custom")
    assert NLCommandInterpreter(api_key="k").model == "claude-custom"


def test_interpreter_unavailable_without_key(monkeypatch):
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    interp = NLCommandInterpreter(api_key=None)
    assert interp.available is False
    with pytest.raises(NLCommandError):
        interp.interpret("go home", _context())


class _FakeToolUseBlock:
    type = "tool_use"

    def __init__(self, data):
        self.input = data


class _FakeMessage:
    def __init__(self, blocks):
        self.content = blocks


class _FakeMessages:
    def __init__(self, payload):
        self._payload = payload
        self.last_kwargs = None

    def create(self, **kwargs):
        self.last_kwargs = kwargs
        return _FakeMessage([_FakeToolUseBlock(self._payload)])


class _FakeClient:
    def __init__(self, payload):
        self.messages = _FakeMessages(payload)


def test_interpret_parses_tool_use(monkeypatch):
    interp = NLCommandInterpreter(api_key="k")
    fake = _FakeClient({
        "action": "move_to",
        "target": "deck_home",
        "explanation": "Heading to the deck home position.",
        "error": None,
    })
    # Inject the fake client so no real SDK/network is used.
    interp._client = fake
    intent = interp.interpret("go to deck home", _context())
    assert intent.action == "move_to"
    assert intent.target == "deck_home"
    assert "deck home" in intent.explanation.lower()
    # The forced tool-use call was configured correctly.
    kwargs = fake.messages.last_kwargs
    assert kwargs["tool_choice"] == {"type": "tool", "name": "robot_command"}
    assert kwargs["messages"][0]["content"] == "go to deck home"


def test_interpret_wraps_sdk_errors(monkeypatch):
    interp = NLCommandInterpreter(api_key="k")

    class _BoomMessages:
        def create(self, **kwargs):
            raise RuntimeError("rate limited")

    interp._client = types.SimpleNamespace(messages=_BoomMessages())
    with pytest.raises(NLCommandError, match="LLM request failed"):
        interp.interpret("go home", _context())


def test_interpret_end_to_end_move_plan(monkeypatch):
    """Interpreter output feeds the validator into a runnable plan."""
    interp = NLCommandInterpreter(api_key="k")
    interp._client = _FakeClient({
        "action": "move_to",
        "target": "deck_home",
        "explanation": "go home",
        "error": None,
    })
    ctx = _context()
    intent = interp.interpret("take me to deck home", ctx)
    plan = plan_from_intent(intent, ctx)
    assert plan.valid is True
    assert plan.steps[0].target == "deck_home"
