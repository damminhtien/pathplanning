"""Problem-oriented discrete planner entrypoints."""

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
from pathplanning.core.types import RNG

N = TypeVar("N")


def _coerce_max_expansions(params: Mapping[str, object] | None) -> int | None:
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


def _resolve_heuristic(problem: DiscreteProblem[N]) -> tuple[Any, N | None]:
    goal_value = problem.goal
    if hasattr(goal_value, "is_goal"):
        return (None, None)
    if isinstance(problem.graph, HeuristicDiscreteGraph):
        return (problem.graph, cast(N, goal_value))
    return (None, None)


def _run_astar(
    problem: DiscreteProblem[N],
    max_expansions: int | None,
    *,
    use_heuristic: bool,
) -> PlanResult:
    graph = problem.graph
    start = problem.start
    goal_test = problem.resolve_goal_test()
    heuristic_graph, exact_goal = _resolve_heuristic(problem)

    if use_heuristic and exact_goal is not None:
        heuristic = lambda node: float(heuristic_graph.heuristic(node, exact_goal))
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
            path_nodes: list[N] = [node]
            current = node
            while current in parent:
                current = parent[current]
                path_nodes.append(current)
            path_nodes.reverse()
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


def plan_astar(
    problem: DiscreteProblem[N],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Plan a path for one ``DiscreteProblem`` with A*."""
    _ = rng
    max_expansions = _coerce_max_expansions(params)
    return _run_astar(problem, max_expansions=max_expansions, use_heuristic=True)


def plan_dijkstra(
    problem: DiscreteProblem[N],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Plan a path for one ``DiscreteProblem`` with Dijkstra."""
    _ = rng
    max_expansions = _coerce_max_expansions(params)
    return _run_astar(problem, max_expansions=max_expansions, use_heuristic=False)


__all__ = ["plan_astar", "plan_dijkstra"]
