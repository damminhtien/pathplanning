"""Focused correctness tests for safe-interval graph search."""

from __future__ import annotations

from collections.abc import Mapping
import heapq
import math

import pytest

from pathplanning import TemporalProblem, plan, plan_temporal
from pathplanning.core.results import StopReason
from pathplanning.core.trace import TraceOptions
from pathplanning.native import NativeGraph


def _time_expanded_arrival(
    problem: TemporalProblem[int],
    adjacency: Mapping[int, tuple[tuple[int, int], ...]],
    horizon: int,
) -> int | None:
    node_blocks = problem.node_blocked or {}
    edge_blocks = problem.edge_blocked or {}

    def node_safe(node: int, time: int) -> bool:
        return not any(start <= time < end for start, end in node_blocks.get(node, ()))

    def edge_safe(source: int, target: int, depart: int, arrive: int) -> bool:
        return not any(
            depart < end and arrive > start for start, end in edge_blocks.get((source, target), ())
        )

    initial = (problem.start, int(problem.start_time))
    queue = [(initial[1], initial[0])]
    best = {initial: initial[1]}
    while queue:
        time, node = heapq.heappop(queue)
        if best[(node, time)] != time:
            continue
        if node == problem.goal:
            return time
        if time < horizon and node_safe(node, time + 1):
            waiting = (node, time + 1)
            if time + 1 < best.get(waiting, math.inf):
                best[waiting] = time + 1
                heapq.heappush(queue, (time + 1, node))
        for neighbor, duration in adjacency[node]:
            arrival = time + duration
            if (
                arrival <= horizon
                and node_safe(neighbor, arrival)
                and edge_safe(node, neighbor, time, arrival)
            ):
                state = (neighbor, arrival)
                if arrival < best.get(state, math.inf):
                    best[state] = arrival
                    heapq.heappush(queue, (arrival, neighbor))
    return None


def test_sipp_matches_small_time_expanded_oracle_and_traces() -> None:
    graph: NativeGraph[int] = NativeGraph.from_edges(
        nodes=(0, 1, 2, 3),
        edges=((0, 1, 1.0), (1, 3, 1.0), (0, 2, 1.0), (2, 3, 1.0), (0, 3, 6.0)),
    )
    problem = TemporalProblem(
        graph=graph,
        start=0,
        goal=3,
        node_blocked={1: [(0.0, 3.0)]},
        edge_blocked={(2, 3): [(0.0, 3.0)]},
    )
    result = plan(problem, trace=TraceOptions(max_bytes=4096))

    assert result.success
    assert result.stop_reason is StopReason.SUCCESS
    oracle_graph = {0: ((1, 1), (2, 1), (3, 6)), 1: ((3, 1),), 2: ((3, 1),), 3: ()}
    assert result.times[-1] == _time_expanded_arrival(problem, oracle_graph, horizon=8) == 4.0
    assert result.times[0] == 0.0
    assert result.trace is not None and result.trace.events.size > 0


def test_sipp_observes_half_open_boundaries_and_node_waiting() -> None:
    graph: NativeGraph[int] = NativeGraph.from_edges(nodes=(0, 1), edges=((0, 1, 1.0),))
    problem = TemporalProblem(
        graph=graph,
        start=0,
        goal=1,
        node_blocked={1: [(1.0, 3.0)]},
        edge_blocked={(0, 1): [(1.0, 2.0)]},
    )
    result = plan_temporal(problem)

    assert result.success
    assert result.states == (0, 1)
    assert result.times == (0.0, 3.0)
    assert result.stats["path_cost"] == 3.0

    with pytest.raises(ValueError, match="start < end"):
        plan_temporal(TemporalProblem(graph=graph, start=0, goal=1, node_blocked={1: [(2.0, 2.0)]}))


def test_sipp_handles_trivial_unreachable_and_bounded_searches() -> None:
    graph: NativeGraph[int] = NativeGraph.from_edges(nodes=(0, 1), edges=((0, 1, 1.0),))
    trivial = plan_temporal(TemporalProblem(graph=graph, start=0, goal=0))
    assert trivial.success and trivial.states == (0,) and trivial.times == (0.0,)

    blocked_start = plan_temporal(
        TemporalProblem(graph=graph, start=0, goal=1, node_blocked={0: [(0.0, math.inf)]})
    )
    assert not blocked_start.success
    assert blocked_start.stop_reason is StopReason.NO_PROGRESS

    expansion_limited = plan_temporal(
        TemporalProblem(graph=graph, start=0, goal=1),
        params={"max_expansions": 1},
    )
    assert expansion_limited.success
    assert expansion_limited.stop_reason is StopReason.MAX_ITERS
    assert expansion_limited.times == (0.0, 1.0)

    timed = plan_temporal(
        TemporalProblem(graph=graph, start=0, goal=1),
        params={"max_runtime_ms": 1e-20},
    )
    assert timed.stop_reason is StopReason.TIME_BUDGET
    if timed.success:
        assert timed.states == (0, 1)
        assert timed.times == (0.0, 1.0)


def test_bounded_suboptimal_sipp_uses_focal_hops_and_obeys_weight() -> None:
    graph = NativeGraph.from_edges(
        nodes=(0, 1, 2, 3, 4),
        edges=((0, 1, 1.0), (1, 2, 1.0), (2, 3, 1.0), (0, 4, 2.5), (4, 3, 1.0)),
    )
    problem = TemporalProblem(graph=graph, start=0, goal=3)

    optimal = plan_temporal(problem, planner="bounded_suboptimal_sipp", params={"w": 1.0})
    bounded = plan_temporal(
        problem,
        planner="bounded_suboptimal_sipp",
        params={"w": 1.5},
        trace=TraceOptions(max_bytes=2048),
    )

    assert optimal.success and optimal.stop_reason is StopReason.SUCCESS
    assert optimal.stats["path_cost"] == 3.0
    assert bounded.success and bounded.stop_reason is StopReason.SUCCESS
    assert bounded.stats["path_cost"] == 3.5
    assert bounded.stats["path_cost"] <= 1.5 * optimal.stats["path_cost"]
    assert bounded.trace is not None and bounded.trace.events.size > 0

    budgeted = plan_temporal(
        problem,
        planner="bounded_suboptimal_sipp",
        params={"w": 1.5, "max_expansions": 2},
    )
    assert budgeted.success and budgeted.stop_reason is StopReason.MAX_ITERS
    assert budgeted.times == (0.0, 2.5, 3.5)

    with pytest.raises(ValueError, match="w must be a finite number"):
        plan_temporal(problem, planner="bounded_suboptimal_sipp", params={"w": 0.9})
