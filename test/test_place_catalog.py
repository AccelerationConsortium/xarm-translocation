"""The assistant's place catalog, against the repo's real motion graph.

Written when the `hood` tag came off the hood_* nodes (2026-09-21). That tag
existed for the fume-hood sash interlock, but `assistant_actions` was quietly
a second consumer: it was the *station* tag that grouped all seven hood nodes
into places, so removing it would have dropped the shaker and the filtration
setup out of the catalog entirely — the assistant would have answered "I don't
know that place" for two stations that are still very much there, with nothing
failing loudly to say why.

`shaker` and `filter` took its place in `_STATION_TAGS`. These tests exist so
the next tag edit finds out from a red test rather than from an operator.
"""

from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.core.assistant_actions import build_place_catalog, find_place_key  # noqa: E402
from src.core.motion_graph import DEFAULT_PRECONDITIONS, MotionGraph  # noqa: E402

_GRAPH_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "settings", "motion_graph.yaml",
)


@pytest.fixture(scope="module")
def catalog():
    graph = MotionGraph.from_yaml(_GRAPH_PATH, preconditions=DEFAULT_PRECONDITIONS)
    return build_place_catalog(graph)


def test_the_two_hood_stations_are_still_places(catalog):
    assert "shaker" in catalog, "hood shaker fell out of the catalog"
    assert "filter" in catalog, "hood filtration fell out of the catalog"


@pytest.mark.parametrize(
    "key, node_id",
    [
        ("shaker", "hood_shaker_high"),
        ("shaker", "hood_shaker_low"),
        ("filter", "hood_filter_home"),
        ("filter", "hood_filter_high"),
        ("filter", "hood_filter_low"),
    ],
)
def test_every_hood_work_node_belongs_to_a_place(catalog, key, node_id):
    assert node_id in catalog[key].nodes_by_role.values()


def test_top_plate_still_collapses_into_the_low_role(catalog):
    """Pre-existing and unchanged by the tag swap, but easy to misread as
    a regression: `_ROLE_SUFFIXES` maps `top_plate` to `low`, and
    `hood_filter_low` claims that role first, so the top-plate pose is not
    separately addressable from the catalog. It is still a graph node and
    still reachable by id — only the assistant's place view collapses it.
    """
    assert catalog["filter"].nodes_by_role["low"] == "hood_filter_low"


def test_the_hood_stations_have_somewhere_to_arrive_and_pick_from(catalog):
    """A place is only useful to the assistant if it can resolve both."""
    for key in ("shaker", "filter"):
        assert catalog[key].arrival_node is not None
        assert catalog[key].pick_node is not None


def test_no_place_is_built_from_the_hood_transit_gateway(catalog):
    """`hood_home` is a front door, not a destination.

    Its only tag is `transit_home`, which describes how a node behaves
    rather than where it is, so it is deliberately not a place. Paths still
    route *through* it — that is the graph's job, not the catalog's.
    """
    assert "hood" not in catalog
    all_nodes = {n for p in catalog.values() for n in p.nodes_by_role.values()}
    assert "hood_home" not in all_nodes


def test_plain_english_still_finds_the_hood_stations(catalog):
    assert find_place_key(catalog, "shaker") == "shaker"
    assert find_place_key(catalog, "Filter") == "filter"
