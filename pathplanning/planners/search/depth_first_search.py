"""Depth-first discrete-search planner."""

from __future__ import annotations

from collections.abc import Mapping
import math
import time
from typing import TypeVar

from pathplanning.core.contracts import DiscreteProblem
from pathplanning.core.results import PlanResult, StopReason
from pathplanning.core.types import RNG
from pathplanning.planners.search._internal.common import (
    coerce_max_expansions,
    path_to_matrix,
    reconstruct_path,
)

N = TypeVar("N")


def plan_depth_first_search(
    problem: DiscreteProblem[N],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Plan a path for one ``DiscreteProblem`` with DFS."""
    _ = rng
    max_expansions = coerce_max_expansions(params)

    graph = problem.graph
    start = problem.start
    goal_test = problem.resolve_goal_test()

    stack: list[N] = [start]
    seen: set[N] = {start}
    parent: dict[N, N] = {start: start}
    g_cost: dict[N, float] = {start: 0.0}

    expanded = 0
    start_time = time.perf_counter()

    while stack:
        node = stack.pop()
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
                nodes=len(seen),
                stats=stats,
            )

        if max_expansions is not None and expanded >= max_expansions:
            break

        neighbors = list(graph.neighbors(node))
        neighbors.reverse()
        for neighbor in neighbors:
            if neighbor in seen:
                continue
            edge = float(graph.edge_cost(node, neighbor))
            if not math.isfinite(edge):
                continue
            seen.add(neighbor)
            parent[neighbor] = node
            g_cost[neighbor] = float(g_cost[node] + edge)
            stack.append(neighbor)

    elapsed = time.perf_counter() - start_time
    return PlanResult(
        success=False,
        path=None,
        best_path=None,
        stop_reason=StopReason.MAX_ITERS if max_expansions is not None else StopReason.NO_PROGRESS,
        iters=expanded,
        nodes=len(seen),
        stats={"elapsed_s": elapsed, "expanded": float(expanded)},
    )


__all__ = ["plan_depth_first_search"]
