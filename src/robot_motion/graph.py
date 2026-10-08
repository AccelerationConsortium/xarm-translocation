"""Arm-only topology preview reusing the existing xArm BFS path planner.

This validates graph structure, not physical safety. It never sends motion.
The legacy xArm's gripper/rail schema and interlocks remain unchanged.
"""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator
from core.motion_graph import MotionGraph, UnknownNodeError

from .config import MODELS

Identifier = Annotated[str, Field(pattern=r"^[a-zA-Z][a-zA-Z0-9_-]{0,63}$")]
Number = Annotated[float, Field(allow_inf_nan=False)]


class Node(BaseModel):
    model_config = ConfigDict(extra="forbid")
    id: Identifier
    joints_deg: list[Number] = Field(min_length=4, max_length=6)
    tcp_mm_rpy_deg: list[Number] | None = Field(
        default=None, min_length=6, max_length=6
    )


class Edge(BaseModel):
    model_config = ConfigDict(extra="forbid")
    source: Identifier
    target: Identifier
    mode: Literal["joint", "linear"] = "joint"


class Graph(BaseModel):
    model_config = ConfigDict(extra="forbid")
    schema_version: Literal["robot-motion/1"] = "robot-motion/1"
    robot_model: Literal["xarm5", "ur3e", "ur5e", "ur5_cb3", "mg400"]
    nodes: list[Node] = Field(default_factory=list, max_length=250)
    edges: list[Edge] = Field(default_factory=list, max_length=1000)

    @model_validator(mode="after")
    def valid_topology(self):
        nodes = {n.id: n for n in self.nodes}
        if len(nodes) != len(self.nodes):
            raise ValueError("Duplicate node id")
        expected = MODELS[self.robot_model]["joints"]
        for node in self.nodes:
            if len(node.joints_deg) != expected:
                raise ValueError(
                    f"{self.robot_model} requires exactly {expected} joint angles"
                )
        pairs = set()
        for edge in self.edges:
            if edge.source not in nodes or edge.target not in nodes:
                raise ValueError("Edge references an unknown node")
            if edge.source == edge.target:
                raise ValueError("Self edges are not permitted")
            pair = (edge.source, edge.target)
            if pair in pairs:
                raise ValueError("Duplicate directed edge")
            pairs.add(pair)
            if edge.mode == "linear" and nodes[edge.target].tcp_mm_rpy_deg is None:
                raise ValueError("Linear target requires an explicit TCP pose")
        return self

    def path(self, source: str, target: str) -> list[str]:
        view = _PathView(self)
        # Reuse only the pure traversal algorithm. Neither the existing xArm
        # graph loader nor its hardware-specific validation is changed.
        return MotionGraph.plan_path(view, source, target, None)


class _PathView:
    def __init__(self, graph):
        self.nodes = {n.id: n for n in graph.nodes}
        self.edges = graph.edges

    def node(self, name):
        if name not in self.nodes:
            raise UnknownNodeError(f"Unknown node: {name}")
        return self.nodes[name]

    def allowed_targets_for_state(self, current, _gripper_state):
        return [e.target for e in self.edges if e.source == current]


class PreviewRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    graph: Graph
    source: Identifier
    target: Identifier
