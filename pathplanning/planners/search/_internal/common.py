"""Shared utilities for best-first discrete-search planners."""

from __future__ import annotations

from collections.abc import Mapping
import heapq
import itertools
import math
import time
from typing import Any, TypeVar, cast

import numpy as np

from pathplanning.core.contracts import DiscreteProblem, HeuristicDiscreteGraph
from pathplanning.core.results import PlanResult, StopReason

N = TypeVar("N")


def coerce_max_expansions(params: Mapping[str, object] | None) -> int | None:
    """Parse and validate optional expansion budget from planner params."""
    if params is None:
        return None
    raw_value = params.get("max_expansions")
    if raw_value is None:
        return None
    if isinstance(raw_value, bool) or type(raw_value) is not int:
        raise TypeError("max_expansions must be an integer when provided")
    if raw_value <= 0:
        raise ValueError("max_expansions must be > 0 when provided")
    return raw_value


def coerce_weight(params: Mapping[str, object] | None) -> float:
    """Parse and validate weighted-A* heuristic weight."""
    if params is None:
        return 1.0
    raw_value = params.get("weight", 1.0)
    if isinstance(raw_value, bool) or not isinstance(raw_value, (int, float)):
        raise TypeError("weight must be a finite real number when provided")
    weight = float(raw_value)
    if not math.isfinite(weight):
        raise TypeError("weight must be a finite real number when provided")
    if weight < 1.0:
        raise ValueError("weight must be >= 1.0")
    return weight


def _path_to_matrix(path_nodes: list[N]) -> np.ndarray | None:
    if not path_nodes:
        return None
    try:
        path_array = np.asarray(path_nodes, dtype=float)
    except Exception:
        return None
    if path_array.ndim == 1:
        path_array = path_array.reshape(-1, 1)
    return path_array.astype(float, copy=False)


def path_to_matrix(path_nodes: list[N]) -> np.ndarray | None:
    """Public wrapper for converting one node path to matrix form."""
    return _path_to_matrix(path_nodes)


def reconstruct_path(parent: Mapping[N, N], start: N, reached: N) -> list[N]:
    """Reconstruct one path from ``start`` to ``reached`` using parent links."""
    path_nodes: list[N] = [reached]
    current = reached
    while current != start:
        current = parent[current]
        path_nodes.append(current)
    path_nodes.reverse()
    return path_nodes


def _resolve_heuristic(problem: DiscreteProblem[N]) -> tuple[Any, N | None]:
    goal_value = problem.goal
    if hasattr(goal_value, "is_goal"):
        return (None, None)
    if isinstance(problem.graph, HeuristicDiscreteGraph):
        return (problem.graph, cast(N, goal_value))
    return (None, None)


def run_best_first(
    problem: DiscreteProblem[N],
    *,
    max_expansions: int | None,
    use_heuristic: bool,
    heuristic_weight: float = 1.0,
) -> PlanResult:
    """Run a best-first family planner (A*/Dijkstra/weighted A*)."""
    graph = problem.graph
    start = problem.start
    goal_test = problem.resolve_goal_test()
    heuristic_graph, exact_goal = _resolve_heuristic(problem)

    if use_heuristic and exact_goal is not None:
        heuristic = lambda node: (
            heuristic_weight * float(heuristic_graph.heuristic(node, exact_goal))
        )
    else:
        heuristic = lambda node: 0.0

    g_cost: dict[N, float] = {start: 0.0}
    parent: dict[N, N] = {}
    open_heap: list[tuple[float, float, int, N]] = []
    tie_breaker = itertools.count()
    heapq.heappush(open_heap, (heuristic(start), heuristic(start), next(tie_breaker), start))

    closed: set[N] = set()
    expanded = 0
    start_time = time.perf_counter()

    while open_heap:
        _f_score, _h_score, _order, node = heapq.heappop(open_heap)
        if node in closed:
            continue

        closed.add(node)
        expanded += 1

        if goal_test.is_goal(node):
            path_nodes = reconstruct_path(parent, start, node)
            path = _path_to_matrix(path_nodes)
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
            tentative = float(g_cost[node] + edge)
            if tentative < g_cost.get(neighbor, float("inf")):
                g_cost[neighbor] = tentative
                parent[neighbor] = node
                h_value = heuristic(neighbor)
                heapq.heappush(
                    open_heap,
                    (tentative + h_value, h_value, next(tie_breaker), neighbor),
                )

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


__all__ = [
    "coerce_max_expansions",
    "coerce_weight",
    "path_to_matrix",
    "reconstruct_path",
    "run_best_first",
]
