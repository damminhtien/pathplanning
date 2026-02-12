"""Contract-first 2D search module.

This module provides a lightweight, deterministic API for graph-search planners
operating on ``DiscreteGraph`` contracts.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Iterable
from dataclasses import dataclass, field
from enum import Enum
import heapq
import itertools
import math
import time
from typing import TypeAlias, cast

from pathplanning.core.contracts import DiscreteGraph, DiscreteProblem, GoalTest, HeuristicDiscreteGraph

Coord: TypeAlias = tuple[int, int]
Path: TypeAlias = list[Coord]


class Heuristic(str, Enum):
    """Heuristic policies used by informed planners."""

    EUCLIDEAN = "euclidean"
    MANHATTAN = "manhattan"


class Planner(str, Enum):
    """Planner set for 2D graph search."""

    BFS = "bfs"
    DFS = "dfs"
    DIJKSTRA = "dijkstra"
    GREEDY_BEST_FIRST = "greedy_best_first"
    ASTAR = "astar"
    BIDIRECTIONAL_ASTAR = "bidirectional_astar"
    WEIGHTED_ASTAR = "weighted_astar"
    ANYTIME_ASTAR = "anytime_astar"


@dataclass(slots=True)
class PlanConfig:
    """Input configuration for one 2D search run."""

    graph: DiscreteGraph[Coord]
    start: Coord
    goal: Coord | GoalTest[Coord]
    heuristic: Heuristic = Heuristic.EUCLIDEAN
    max_expansions: int | None = None
    timeout_s: float | None = None
    weight: float = 1.0
    anytime_weights: tuple[float, ...] = (2.5, 2.0, 1.5, 1.0)

    def __post_init__(self) -> None:
        if self.max_expansions is not None:
            if isinstance(self.max_expansions, bool) or type(self.max_expansions) is not int:
                raise TypeError("max_expansions must be int when provided")
            if self.max_expansions <= 0:
                raise ValueError("max_expansions must be > 0")

        if self.timeout_s is not None:
            if isinstance(self.timeout_s, bool) or not isinstance(self.timeout_s, (int, float)):
                raise TypeError("timeout_s must be numeric when provided")
            if float(self.timeout_s) <= 0.0:
                raise ValueError("timeout_s must be > 0")

        if not isinstance(self.weight, (int, float)):
            raise TypeError("weight must be numeric")
        if float(self.weight) < 1.0:
            raise ValueError("weight must be >= 1.0")

        if not self.anytime_weights:
            raise ValueError("anytime_weights must be non-empty")
        for value in self.anytime_weights:
            if not isinstance(value, (int, float)):
                raise TypeError("anytime_weights must be numeric")
            if float(value) < 1.0:
                raise ValueError("anytime_weights must be >= 1.0")


@dataclass(slots=True)
class PlanResult:
    """Result of one search run."""

    planner: Planner
    start: Coord
    goal: Coord | GoalTest[Coord]
    success: bool
    path: Path | None
    cost: float | None
    nodes_expanded: int
    runtime_s: float
    visited: list[Coord] = field(default_factory=list)
    paths: list[Path] | None = None
    visited_iters: list[list[Coord]] | None = None
    stats: dict[str, float] = field(default_factory=dict)


def _parse_planner(planner: Planner | str) -> Planner:
    if isinstance(planner, Planner):
        return planner

    key = str(planner).strip().lower()
    aliases: dict[str, Planner] = {
        "breadth_first_search": Planner.BFS,
        "depth_first_search": Planner.DFS,
        "best_first_search": Planner.GREEDY_BEST_FIRST,
        "dijkstra": Planner.DIJKSTRA,
        "astar": Planner.ASTAR,
        "bidirectional_astar": Planner.BIDIRECTIONAL_ASTAR,
        "weighted_astar": Planner.WEIGHTED_ASTAR,
        "anytime_astar": Planner.ANYTIME_ASTAR,
        # Backward-friendly aliases for old names.
        "anytime_repairing_astar": Planner.ANYTIME_ASTAR,
        "anytime_dstar": Planner.ANYTIME_ASTAR,
        "learning_realtime_astar": Planner.ASTAR,
        "realtime_adaptive_astar": Planner.ASTAR,
        "lifelong_planning_astar": Planner.ASTAR,
        "dstar_lite": Planner.ASTAR,
        "dstar": Planner.ASTAR,
    }
    if key in aliases:
        return aliases[key]
    return Planner(key)


def _resolve_goal_node(problem: DiscreteProblem[Coord]) -> Coord | None:
    goal_value = problem.goal
    if hasattr(goal_value, "is_goal"):
        return None
    return cast(Coord, goal_value)


def _ordered_neighbors(graph: DiscreteGraph[Coord], node: Coord) -> list[Coord]:
    neighbors = list(graph.neighbors(node))
    neighbors.sort()
    return neighbors


def _heuristic_cost(
    graph: DiscreteGraph[Coord],
    heuristic_kind: Heuristic,
    node: Coord,
    goal: Coord | None,
) -> float:
    if goal is None:
        return 0.0

    if isinstance(graph, HeuristicDiscreteGraph):
        return float(graph.heuristic(node, goal))

    if heuristic_kind is Heuristic.MANHATTAN:
        return float(abs(goal[0] - node[0]) + abs(goal[1] - node[1]))
    return float(math.hypot(goal[0] - node[0], goal[1] - node[1]))


def _reconstruct_path(parent: dict[Coord, Coord], start: Coord, reached: Coord) -> Path | None:
    if reached not in parent:
        return None

    out: Path = [reached]
    current = reached
    while current != start:
        current = parent[current]
        out.append(current)
    out.reverse()
    return out


def _path_cost(graph: DiscreteGraph[Coord], path: Path | None) -> float | None:
    if path is None:
        return None
    if len(path) <= 1:
        return 0.0

    total = 0.0
    for src, dst in zip(path[:-1], path[1:], strict=True):
        edge = float(graph.edge_cost(src, dst))
        if not math.isfinite(edge):
            return float("inf")
        total += edge
    return total


def _time_expired(start_time: float, timeout_s: float | None) -> bool:
    if timeout_s is None:
        return False
    return (time.perf_counter() - start_time) >= timeout_s


def _run_best_first(
    problem: DiscreteProblem[Coord],
    heuristic_kind: Heuristic,
    *,
    mode: Planner,
    weight: float,
    max_expansions: int | None,
    timeout_s: float | None,
    start_time: float,
) -> tuple[Path | None, list[Coord], bool]:
    graph = problem.graph
    start = problem.start
    goal_test = problem.resolve_goal_test()
    goal_node = _resolve_goal_node(problem)

    open_heap: list[tuple[float, float, int, Coord]] = []
    order = itertools.count()

    parent: dict[Coord, Coord] = {start: start}
    g_score: dict[Coord, float] = {start: 0.0}
    closed: set[Coord] = set()
    visited: list[Coord] = []

    h_start = _heuristic_cost(graph, heuristic_kind, start, goal_node)
    heapq.heappush(open_heap, (h_start, h_start, next(order), start))

    expansions = 0
    while open_heap:
        if _time_expired(start_time, timeout_s):
            return None, visited, True

        _priority, _h_value, _order, node = heapq.heappop(open_heap)
        if node in closed:
            continue

        closed.add(node)
        visited.append(node)
        expansions += 1

        if goal_test.is_goal(node):
            return _reconstruct_path(parent, start, node), visited, False

        if max_expansions is not None and expansions >= max_expansions:
            return None, visited, True

        node_g = g_score[node]
        for neighbor in _ordered_neighbors(graph, node):
            edge = float(graph.edge_cost(node, neighbor))
            if not math.isfinite(edge):
                continue

            tentative_g = node_g + edge
            if tentative_g >= g_score.get(neighbor, float("inf")):
                continue

            g_score[neighbor] = tentative_g
            parent[neighbor] = node
            h_neighbor = _heuristic_cost(graph, heuristic_kind, neighbor, goal_node)

            if mode is Planner.DIJKSTRA:
                priority = tentative_g
            elif mode is Planner.GREEDY_BEST_FIRST:
                priority = h_neighbor
            elif mode is Planner.WEIGHTED_ASTAR:
                priority = tentative_g + float(weight) * h_neighbor
            else:
                priority = tentative_g + h_neighbor

            heapq.heappush(open_heap, (priority, h_neighbor, next(order), neighbor))

    return None, visited, False


def _run_bfs(
    problem: DiscreteProblem[Coord],
    *,
    max_expansions: int | None,
    timeout_s: float | None,
    start_time: float,
) -> tuple[Path | None, list[Coord], bool]:
    graph = problem.graph
    start = problem.start
    goal_test = problem.resolve_goal_test()

    queue: deque[Coord] = deque([start])
    seen: set[Coord] = {start}
    parent: dict[Coord, Coord] = {start: start}
    visited: list[Coord] = []

    expansions = 0
    while queue:
        if _time_expired(start_time, timeout_s):
            return None, visited, True

        node = queue.popleft()
        visited.append(node)
        expansions += 1

        if goal_test.is_goal(node):
            return _reconstruct_path(parent, start, node), visited, False

        if max_expansions is not None and expansions >= max_expansions:
            return None, visited, True

        for neighbor in _ordered_neighbors(graph, node):
            if neighbor in seen:
                continue
            edge = float(graph.edge_cost(node, neighbor))
            if not math.isfinite(edge):
                continue
            seen.add(neighbor)
            parent[neighbor] = node
            queue.append(neighbor)

    return None, visited, False


def _run_dfs(
    problem: DiscreteProblem[Coord],
    *,
    max_expansions: int | None,
    timeout_s: float | None,
    start_time: float,
) -> tuple[Path | None, list[Coord], bool]:
    graph = problem.graph
    start = problem.start
    goal_test = problem.resolve_goal_test()

    stack: list[Coord] = [start]
    seen: set[Coord] = {start}
    parent: dict[Coord, Coord] = {start: start}
    visited: list[Coord] = []

    expansions = 0
    while stack:
        if _time_expired(start_time, timeout_s):
            return None, visited, True

        node = stack.pop()
        visited.append(node)
        expansions += 1

        if goal_test.is_goal(node):
            return _reconstruct_path(parent, start, node), visited, False

        if max_expansions is not None and expansions >= max_expansions:
            return None, visited, True

        neighbors = _ordered_neighbors(graph, node)
        neighbors.reverse()
        for neighbor in neighbors:
            if neighbor in seen:
                continue
            edge = float(graph.edge_cost(node, neighbor))
            if not math.isfinite(edge):
                continue
            seen.add(neighbor)
            parent[neighbor] = node
            stack.append(neighbor)

    return None, visited, False


def _run_bidirectional_bfs(
    problem: DiscreteProblem[Coord],
    *,
    max_expansions: int | None,
    timeout_s: float | None,
    start_time: float,
) -> tuple[Path | None, list[Coord], bool]:
    start = problem.start
    goal_test = problem.resolve_goal_test()
    goal_node = _resolve_goal_node(problem)

    if goal_node is None:
        # Without a concrete goal node, fallback to A* behavior.
        return _run_best_first(
            problem,
            Heuristic.EUCLIDEAN,
            mode=Planner.ASTAR,
            weight=1.0,
            max_expansions=max_expansions,
            timeout_s=timeout_s,
            start_time=start_time,
        )

    if start == goal_node:
        return [start], [start], False

    graph = problem.graph
    queue_f: deque[Coord] = deque([start])
    queue_b: deque[Coord] = deque([goal_node])

    seen_f: set[Coord] = {start}
    seen_b: set[Coord] = {goal_node}
    parent_f: dict[Coord, Coord] = {start: start}
    parent_b: dict[Coord, Coord] = {goal_node: goal_node}
    visited: list[Coord] = []

    expansions = 0

    def _expand_one(
        queue: deque[Coord],
        seen_this: set[Coord],
        seen_other: set[Coord],
        parent_this: dict[Coord, Coord],
    ) -> Coord | None:
        nonlocal expansions

        if not queue:
            return None

        node = queue.popleft()
        visited.append(node)
        expansions += 1

        if node in seen_other:
            return node

        for neighbor in _ordered_neighbors(graph, node):
            if neighbor in seen_this:
                continue
            edge = float(graph.edge_cost(node, neighbor))
            if not math.isfinite(edge):
                continue
            seen_this.add(neighbor)
            parent_this[neighbor] = node
            queue.append(neighbor)
            if neighbor in seen_other:
                return neighbor

        return None

    while queue_f and queue_b:
        if _time_expired(start_time, timeout_s):
            return None, visited, True
        if max_expansions is not None and expansions >= max_expansions:
            return None, visited, True

        meet = _expand_one(queue_f, seen_f, seen_b, parent_f)
        if meet is None and goal_test.is_goal(start):
            meet = start
        if meet is not None:
            break

        if _time_expired(start_time, timeout_s):
            return None, visited, True
        if max_expansions is not None and expansions >= max_expansions:
            return None, visited, True

        meet = _expand_one(queue_b, seen_b, seen_f, parent_b)
        if meet is not None:
            break
    else:
        return None, visited, False

    path_f = _reconstruct_path(parent_f, start, meet)
    if path_f is None:
        return None, visited, False

    path_b: Path = []
    node = meet
    while node != goal_node:
        node = parent_b[node]
        path_b.append(node)

    return path_f + path_b, visited, False


class Search2D:
    """High-level contract-based planner for 2D graph search."""

    def plan(self, planner: Planner | str, config: PlanConfig) -> PlanResult:
        planner_kind = _parse_planner(planner)
        problem = DiscreteProblem[Coord](
            graph=config.graph,
            start=config.start,
            goal=config.goal,
        )

        t0 = time.perf_counter()

        if planner_kind is Planner.BFS:
            path, visited, budget_hit = _run_bfs(
                problem,
                max_expansions=config.max_expansions,
                timeout_s=config.timeout_s,
                start_time=t0,
            )
            paths = None
            visited_iters = None

        elif planner_kind is Planner.DFS:
            path, visited, budget_hit = _run_dfs(
                problem,
                max_expansions=config.max_expansions,
                timeout_s=config.timeout_s,
                start_time=t0,
            )
            paths = None
            visited_iters = None

        elif planner_kind is Planner.BIDIRECTIONAL_ASTAR:
            path, visited, budget_hit = _run_bidirectional_bfs(
                problem,
                max_expansions=config.max_expansions,
                timeout_s=config.timeout_s,
                start_time=t0,
            )
            paths = None
            visited_iters = None

        elif planner_kind is Planner.ANYTIME_ASTAR:
            paths = []
            visited_iters = []
            best_path: Path | None = None
            best_cost = float("inf")
            budget_hit = False
            visited = []

            for weight in config.anytime_weights:
                run_path, run_visited, run_budget_hit = _run_best_first(
                    problem,
                    config.heuristic,
                    mode=Planner.WEIGHTED_ASTAR,
                    weight=float(weight),
                    max_expansions=config.max_expansions,
                    timeout_s=config.timeout_s,
                    start_time=t0,
                )
                paths.append([] if run_path is None else run_path)
                visited_iters.append(run_visited)
                visited.extend(run_visited)

                run_cost = _path_cost(config.graph, run_path)
                if run_path is not None and run_cost is not None and run_cost < best_cost:
                    best_path = run_path
                    best_cost = run_cost

                if run_budget_hit:
                    budget_hit = True
                    break

            path = best_path

        else:
            mode = planner_kind
            if planner_kind is Planner.ASTAR:
                mode = Planner.ASTAR
            elif planner_kind is Planner.DIJKSTRA:
                mode = Planner.DIJKSTRA
            elif planner_kind is Planner.GREEDY_BEST_FIRST:
                mode = Planner.GREEDY_BEST_FIRST
            elif planner_kind is Planner.WEIGHTED_ASTAR:
                mode = Planner.WEIGHTED_ASTAR

            path, visited, budget_hit = _run_best_first(
                problem,
                config.heuristic,
                mode=mode,
                weight=config.weight,
                max_expansions=config.max_expansions,
                timeout_s=config.timeout_s,
                start_time=t0,
            )
            paths = None
            visited_iters = None

        runtime_s = time.perf_counter() - t0
        cost = _path_cost(config.graph, path)
        success = path is not None
        stats = {
            "budget_hit": 1.0 if budget_hit else 0.0,
        }

        return PlanResult(
            planner=planner_kind,
            start=config.start,
            goal=config.goal,
            success=success,
            path=path,
            cost=cost,
            nodes_expanded=len(visited),
            runtime_s=runtime_s,
            visited=visited,
            paths=paths,
            visited_iters=visited_iters,
            stats=stats,
        )


def plan(planner: Planner | str, config: PlanConfig) -> PlanResult:
    """Convenience function to run one 2D planner."""
    return Search2D().plan(planner, config)


__all__ = [
    "Coord",
    "Path",
    "Heuristic",
    "Planner",
    "PlanConfig",
    "PlanResult",
    "Search2D",
    "plan",
]
