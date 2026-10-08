"""Focused LaCAM* checks against a small joint-state Dijkstra oracle."""

from __future__ import annotations

from heapq import heappop, heappush
from itertools import product

import numpy as np

from pathplanning import MultiAgentProblem, plan_multi_agent
from pathplanning.core.results import StopReason
from pathplanning.core.trace import TraceOptions
from pathplanning.native import NativeGraph


def _joint_state_soc_oracle(
    adjacency: tuple[tuple[int, ...], ...], starts: tuple[int, ...], goals: tuple[int, ...]
) -> int | None:
    queue: list[tuple[int, tuple[int, ...]]] = [(0, starts)]
    best = {starts: 0}
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
            state = tuple(next_positions)
            next_cost = cost + sum(
                position != goals[agent] for agent, position in enumerate(positions)
            )
            if next_cost < best.get(state, 1 << 60):
                best[state] = next_cost
                heappush(queue, (next_cost, state))
    return None


def test_lacam_star_matches_joint_oracle_and_records_trace() -> None:
    graph: NativeGraph[int] = NativeGraph.from_edges(
        nodes=(0, 1, 2, 3),
        edges=((0, 1, 1.0), (1, 2, 1.0), (2, 3, 1.0), (3, 0, 1.0)),
        directed=False,
    )
    problem = MultiAgentProblem(graph, starts=(0, 2), goals=(2, 0))
    oracle = _joint_state_soc_oracle(((1, 3), (0, 2), (1, 3), (0, 2)), (0, 2), (2, 0))

    result = plan_multi_agent(
        problem,
        planner="lacam_star",
        seed=7,
        trace=TraceOptions(max_bytes=4096),
    )

    assert oracle == 4
    assert result.success and result.stop_reason is StopReason.SUCCESS
    assert result.sum_of_costs == oracle
    assert result.makespan == max(len(path) - 1 for path in result.paths)
    assert result.trace is not None and result.trace.events.size > 0


def test_lacam_star_returns_budget_incumbent_and_reports_no_path() -> None:
    line: NativeGraph[int] = NativeGraph.from_edges(
        nodes=(0, 1, 2), edges=((0, 1, 1.0), (1, 2, 1.0)), directed=False
    )
    budgeted = plan_multi_agent(
        MultiAgentProblem(line, starts=(0,), goals=(2,)),
        planner="lacam_star",
        params={"max_expansions": 8},
    )
    assert budgeted.success and budgeted.paths == ((0, 1, 2),)
    assert budgeted.sum_of_costs == budgeted.makespan == 2
    assert budgeted.stop_reason is StopReason.MAX_ITERS

    disconnected: NativeGraph[int] = NativeGraph.from_csr(
        (0, 0, 0), np.empty(0, dtype=np.uint64), np.empty(0), node_labels=(0, 1)
    )
    no_path = plan_multi_agent(
        MultiAgentProblem(disconnected, starts=(0,), goals=(1,)), planner="lacam_star"
    )
    assert not no_path.success and no_path.stop_reason is StopReason.NO_PROGRESS
