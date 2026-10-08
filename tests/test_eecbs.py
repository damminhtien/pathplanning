"""Focused EECBS correctness checks against a tiny joint-state oracle."""

from __future__ import annotations

from heapq import heappop, heappush
from itertools import product

import numpy as np
import pytest

from pathplanning import MultiAgentProblem, plan, plan_multi_agent
from pathplanning.core.results import StopReason
from pathplanning.core.trace import TraceOptions
from pathplanning.native import NativeGraph
from pathplanning.spaces import Grid2DMultiAgentAdapter, Grid2DSearchSpace


def _joint_state_soc_oracle(
    adjacency: tuple[tuple[int, ...], ...], starts: tuple[int, ...], goals: tuple[int, ...]
) -> int | None:
    start = starts
    queue: list[tuple[int, tuple[int, ...]]] = [(0, start)]
    best = {start: 0}
    while queue:
        cost, positions = heappop(queue)
        if best[positions] != cost:
            continue
        if positions == goals:
            return cost
        actions = [
            (position,) if position == goals[agent] else (position, *adjacency[position])
            for agent, position in enumerate(positions)
        ]
        for next_positions in product(*actions):
            if len(set(next_positions)) != len(next_positions):
                continue
            if any(
                positions[first] != next_positions[first]
                and positions[first] == next_positions[second]
                and positions[second] == next_positions[first]
                for first in range(len(positions))
                for second in range(first + 1, len(positions))
            ):
                continue
            next_state = tuple(next_positions)
            next_cost = cost + sum(
                position != goals[agent] for agent, position in enumerate(positions)
            )
            if next_cost < best.get(next_state, 1 << 60):
                best[next_state] = next_cost
                heappush(queue, (next_cost, next_state))
    return None


def test_eecbs_matches_joint_oracle_bound_and_traces() -> None:
    graph: NativeGraph[int] = NativeGraph.from_edges(
        nodes=(0, 1, 2, 3),
        edges=((0, 1, 1.0), (1, 2, 1.0), (2, 3, 1.0), (3, 0, 1.0)),
        directed=False,
    )
    problem = MultiAgentProblem(graph, starts=(0, 2), goals=(2, 0))
    result = plan(
        problem,
        params={"w": 1.25},
        trace=TraceOptions(max_bytes=4096),
    )

    optimum = _joint_state_soc_oracle(((1, 3), (0, 2), (1, 3), (0, 2)), (0, 2), (2, 0))
    assert optimum == 4
    assert result.success and result.stop_reason is StopReason.SUCCESS
    assert result.sum_of_costs <= 1.25 * optimum
    assert result.makespan == max(len(path) - 1 for path in result.paths)
    assert result.trace is not None and result.trace.events.size > 0

    optimal = plan_multi_agent(problem, params={"w": 1.0})
    assert optimal.success and optimal.sum_of_costs == optimum


def test_eecbs_resolves_edge_swap_conflict() -> None:
    graph: NativeGraph[int] = NativeGraph.from_edges(
        nodes=(0, 1, 2),
        edges=((0, 1, 1.0), (1, 2, 1.0), (2, 0, 1.0)),
        directed=False,
    )
    result = plan_multi_agent(MultiAgentProblem(graph, starts=(0, 1), goals=(1, 0)))

    assert result.success
    assert result.sum_of_costs == 3
    assert result.makespan == 2


def test_eecbs_returns_feasible_budget_incumbent_and_reports_no_path() -> None:
    single: NativeGraph[int] = NativeGraph.from_csr(
        (0, 0), np.empty(0, dtype=np.uint64), np.empty(0), node_labels=(0,)
    )
    result = plan_multi_agent(
        MultiAgentProblem(single, starts=(0,), goals=(0,)),
        params={"max_expansions": 1},
    )
    assert result.success and result.paths == ((0,),)
    assert result.stop_reason is StopReason.MAX_ITERS
    assert result.sum_of_costs == result.makespan == 0

    disconnected: NativeGraph[int] = NativeGraph.from_csr(
        (0, 0, 0),
        np.empty(0, dtype=np.uint64),
        np.empty(0),
        node_labels=(0, 1),
    )
    no_path = plan_multi_agent(MultiAgentProblem(disconnected, starts=(0,), goals=(1,)))
    assert not no_path.success and no_path.stop_reason is StopReason.NO_PROGRESS


def test_eecbs_rejects_non_unit_or_non_undirected_graphs() -> None:
    weighted: NativeGraph[int] = NativeGraph.from_edges(
        nodes=(0, 1), edges=((0, 1, 2.0),), directed=False
    )
    with pytest.raises(ValueError, match="unit-weight"):
        plan_multi_agent(MultiAgentProblem(weighted, starts=(0,), goals=(1,)))

    directed: NativeGraph[int] = NativeGraph.from_edges(
        nodes=(0, 1), edges=((0, 1, 1.0),), directed=True
    )
    with pytest.raises(ValueError, match="undirected"):
        plan_multi_agent(MultiAgentProblem(directed, starts=(0,), goals=(1,)))

    graph: NativeGraph[int] = NativeGraph.from_edges(
        nodes=(0, 1), edges=((0, 1, 1.0),), directed=False
    )
    with pytest.raises(ValueError, match="starts must be unique"):
        plan_multi_agent(MultiAgentProblem(graph, starts=(0, 0), goals=(0, 1)))


def test_eecbs_grid_adapter_uses_four_connected_free_cells() -> None:
    grid = Grid2DMultiAgentAdapter(Grid2DSearchSpace(width=3, height=2, obstacles={(1, 0)}))
    result = plan_multi_agent(MultiAgentProblem(grid, starts=((0, 0),), goals=((2, 0),)))

    assert result.success
    assert result.sum_of_costs == 4
    assert (1, 0) not in result.paths[0]
    with pytest.raises(ValueError, match="valid graph nodes"):
        plan_multi_agent(MultiAgentProblem(grid, starts=((1, 0),), goals=((2, 0),)))
