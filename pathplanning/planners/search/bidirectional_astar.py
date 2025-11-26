"""Bidirectional A* discrete-search planner."""

from __future__ import annotations

from collections.abc import Mapping
import heapq
import itertools
import math
import time
from typing import TypeVar, cast

from pathplanning.core.contracts import DiscreteProblem
from pathplanning.core.results import PlanResult, StopReason
from pathplanning.core.types import RNG
from pathplanning.planners.search._internal.common import coerce_max_expansions, path_to_matrix

N = TypeVar("N")


def _coerce_exact_goal(problem: DiscreteProblem[N]) -> N | None:
    goal_value = problem.goal
    if hasattr(goal_value, "is_goal"):
        return None
    return cast(N, goal_value)


def _reconstruct_bidirectional_path(
    start: N,
    goal: N,
    meet: N,
    parent_forward: Mapping[N, N],
    parent_backward: Mapping[N, N],
) -> list[N]:
    forward: list[N] = [meet]
    current = meet
    while current != start:
        current = parent_forward[current]
        forward.append(current)
    forward.reverse()

    backward: list[N] = []
    current = meet
    while current != goal:
        current = parent_backward[current]
        backward.append(current)

    return forward + backward


def plan_bidirectional_astar(
    problem: DiscreteProblem[N],
    *,
    params: Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Plan a path for one ``DiscreteProblem`` with bidirectional A*."""
    _ = rng
    max_expansions = coerce_max_expansions(params)
    graph = problem.graph
    start = problem.start
    goal_test = problem.resolve_goal_test()
    exact_goal = _coerce_exact_goal(problem)

    if exact_goal is None:
        from pathplanning.planners.search.astar import plan_astar

        return plan_astar(problem, params=params, rng=rng)

    goal = exact_goal
    if start == goal or goal_test.is_goal(start):
        path = path_to_matrix([start])
        return PlanResult(
            success=True,
            path=path,
            best_path=path,
            stop_reason=StopReason.SUCCESS,
            iters=0,
            nodes=1,
            stats={"path_cost": 0.0, "elapsed_s": 0.0, "expanded": 0.0, "path_length": 1.0},
        )

    forward_heap: list[tuple[float, int, N]] = []
    backward_heap: list[tuple[float, int, N]] = []
    tie_breaker = itertools.count()

    heapq.heappush(forward_heap, (0.0, next(tie_breaker), start))
    heapq.heappush(backward_heap, (0.0, next(tie_breaker), goal))

    g_forward: dict[N, float] = {start: 0.0}
    g_backward: dict[N, float] = {goal: 0.0}
    parent_forward: dict[N, N] = {start: start}
    parent_backward: dict[N, N] = {goal: goal}
    closed_forward: set[N] = set()
    closed_backward: set[N] = set()

    best_cost = float("inf")
    meet: N | None = None
    expanded = 0
    start_time = time.perf_counter()

    def _expand(
        heap: list[tuple[float, int, N]],
        g_this: dict[N, float],
        g_other: Mapping[N, float],
        parent_this: dict[N, N],
        closed_this: set[N],
    ) -> tuple[float, N | None]:
        nonlocal expanded

        while heap:
            _priority, _order, node = heapq.heappop(heap)
            if node in closed_this:
                continue

            closed_this.add(node)
            expanded += 1

            local_best = float("inf")
            local_meet: N | None = None
            if node in g_other:
                local_best = float(g_this[node] + g_other[node])
                local_meet = node

            for neighbor in graph.neighbors(node):
                edge = float(graph.edge_cost(node, neighbor))
                if not math.isfinite(edge):
                    continue
                tentative = float(g_this[node] + edge)
                if tentative < g_this.get(neighbor, float("inf")):
                    g_this[neighbor] = tentative
                    parent_this[neighbor] = node
                    heapq.heappush(heap, (tentative, next(tie_breaker), neighbor))
                    if neighbor in g_other:
                        candidate = float(tentative + g_other[neighbor])
                        if candidate < local_best:
                            local_best = candidate
                            local_meet = neighbor
            return local_best, local_meet

        return float("inf"), None

    while forward_heap and backward_heap:
        if max_expansions is not None and expanded >= max_expansions:
            break

        if forward_heap[0][0] <= backward_heap[0][0]:
            local_best, local_meet = _expand(
                forward_heap,
                g_forward,
                g_backward,
                parent_forward,
                closed_forward,
            )
        else:
            local_best, local_meet = _expand(
                backward_heap,
                g_backward,
                g_forward,
                parent_backward,
                closed_backward,
            )

        if local_meet is not None and local_best < best_cost:
            best_cost = local_best
            meet = local_meet

        forward_bound = forward_heap[0][0] if forward_heap else float("inf")
        backward_bound = backward_heap[0][0] if backward_heap else float("inf")
        if meet is not None and (forward_bound + backward_bound) >= best_cost:
            break

    elapsed = time.perf_counter() - start_time
    if meet is None:
        return PlanResult(
            success=False,
            path=None,
            best_path=None,
            stop_reason=StopReason.MAX_ITERS if max_expansions is not None else StopReason.NO_PROGRESS,
            iters=expanded,
            nodes=len(g_forward) + len(g_backward),
            stats={"elapsed_s": elapsed, "expanded": float(expanded)},
        )

    path_nodes = _reconstruct_bidirectional_path(start, goal, meet, parent_forward, parent_backward)
    path = path_to_matrix(path_nodes)
    stats: dict[str, float] = {
        "path_cost": float(best_cost),
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
        nodes=len(g_forward) + len(g_backward),
        stats=stats,
    )


__all__ = ["plan_bidirectional_astar"]
