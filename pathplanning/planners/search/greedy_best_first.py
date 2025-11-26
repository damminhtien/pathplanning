"""Greedy best-first discrete-search planner."""

from __future__ import annotations

from collections.abc import Mapping
import heapq
import itertools
import math
import time
from typing import TypeVar

from pathplanning.core.contracts import DiscreteProblem, HeuristicDiscreteGraph
from pathplanning.core.results import PlanResult, StopReason
from pathplanning.core.types import RNG
from pathplanning.planners.search._internal.common import (
    coerce_max_expansions,
    path_to_matrix,
    reconstruct_path,
)

N = TypeVar("N")


def plan_greedy_best_first(
    problem: DiscreteProblem[N],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Plan a path for one ``DiscreteProblem`` with greedy best-first search."""
    _ = rng
    max_expansions = coerce_max_expansions(params)

    graph = problem.graph
    start = problem.start
    goal_test = problem.resolve_goal_test()

    exact_goal: N | None = None
    if not hasattr(problem.goal, "is_goal"):
        exact_goal = problem.goal

    def heuristic(node: N) -> float:
        if exact_goal is not None and isinstance(graph, HeuristicDiscreteGraph):
            return float(graph.heuristic(node, exact_goal))
        return 0.0

    open_heap: list[tuple[float, int, N]] = []
    order = itertools.count()
    heapq.heappush(open_heap, (heuristic(start), next(order), start))

    parent: dict[N, N] = {start: start}
    g_cost: dict[N, float] = {start: 0.0}
    closed: set[N] = set()
    expanded = 0
    start_time = time.perf_counter()

    while open_heap:
        _priority, _order, node = heapq.heappop(open_heap)
        if node in closed:
            continue

        closed.add(node)
        expanded += 1

        if goal_test.is_goal(node):
            path_nodes = reconstruct_path(parent, start, node)
            path = path_to_matrix(path_nodes)
            elapsed = time.perf_counter() - start_time
            stats: dict[str, float] = {
                "path_cost": float(g_cost[node]),
                "elapsed_s": elapsed,
                "expanded": float(expanded),
            }
            if path is not None:
                stats["path_length"] = float(path.shape[0])
            return PlanResult(
                success=True,
                path=path,
                best_path=path,
                stop_reason=StopReason.SUCCESS,
                iters=expanded,
                nodes=len(g_cost),
                stats=stats,
            )

        if max_expansions is not None and expanded >= max_expansions:
            break

        for neighbor in graph.neighbors(node):
            edge = float(graph.edge_cost(node, neighbor))
            if not math.isfinite(edge):
                continue
            if neighbor in g_cost:
                continue
            parent[neighbor] = node
            g_cost[neighbor] = float(g_cost[node] + edge)
            heapq.heappush(open_heap, (heuristic(neighbor), next(order), neighbor))

    elapsed = time.perf_counter() - start_time
    return PlanResult(
        success=False,
        path=None,
        best_path=None,
        stop_reason=StopReason.MAX_ITERS if max_expansions is not None else StopReason.NO_PROGRESS,
        iters=expanded,
        nodes=len(g_cost),
        stats={"elapsed_s": elapsed, "expanded": float(expanded)},
    )


__all__ = ["plan_greedy_best_first"]
