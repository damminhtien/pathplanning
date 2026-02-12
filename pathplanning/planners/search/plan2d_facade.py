"""Unified, graph-contract-based interface for 2D search planners."""

from __future__ import annotations

import argparse
from collections import deque
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from enum import Enum
import heapq
import itertools
import json
import math
import time
from typing import Any

from pathplanning.core.contracts import DiscreteGraph, DiscreteProblem, HeuristicDiscreteGraph

Coord = tuple[int, int]
Path = list[Coord]
PathList = list[Path]


class Heuristic(str, Enum):
    """Heuristic types for pathfinding algorithms."""

    EUCLIDEAN = "euclidean"
    MANHATTAN = "manhattan"


class Planner(str, Enum):
    """Path planning algorithms available in the plan2d module."""

    BREADTH_FIRST_SEARCH = "breadth_first_search"
    DEPTH_FIRST_SEARCH = "depth_first_search"
    DIJKSTRA = "dijkstra"
    BEST_FIRST_SEARCH = "best_first_search"
    ASTAR = "astar"
    ANYTIME_DSTAR = "anytime_dstar"
    ANYTIME_REPAIRING_ASTAR = "anytime_repairing_astar"
    BIDIRECTIONAL_ASTAR = "bidirectional_astar"
    DSTAR_LITE = "dstar_lite"
    DSTAR = "dstar"
    LEARNING_REALTIME_ASTAR = "learning_realtime_astar"
    LIFELONG_PLANNING_ASTAR = "lifelong_planning_astar"
    REALTIME_ADAPTIVE_ASTAR = "realtime_adaptive_astar"


@dataclass
class PlanConfig:
    """Configuration for graph-based 2D pathfinding algorithms."""

    s_start: Coord
    s_goal: Coord
    graph: DiscreteGraph[Coord]
    heuristic: Heuristic = Heuristic.EUCLIDEAN
    lrta_N: int = 250
    rtaa_N: int = 240
    ara_e: float = 2.5
    adstar_eps: float = 2.5


@dataclass
class PlanResult:
    """Result of one pathfinding run."""

    algo: Planner
    heuristic: Heuristic
    s_start: Coord
    s_goal: Coord
    path: Path | None = None
    paths: PathList | None = None
    visited: list[Coord] | None = None
    visited_fore: list[Coord] | None = None
    visited_back: list[Coord] | None = None
    visited_iters: list[list[Coord]] | None = None
    cost: float | None = None
    nodes_expanded: int | None = None
    runtime_s: float = 0.0
    raw: dict[str, Any] = field(default_factory=dict)


def _parse_planner(algo: Planner | str) -> Planner:
    if isinstance(algo, Planner):
        return algo
    return Planner(str(algo).lower().strip())


def _norm_heuristic(h: Heuristic | str) -> Heuristic:
    if isinstance(h, Heuristic):
        return h
    value = str(h).lower().strip()
    return Heuristic.MANHATTAN if value.startswith("manh") else Heuristic.EUCLIDEAN


def _ordered_neighbors(graph: DiscreteGraph[Coord], node: Coord) -> list[Coord]:
    neighbors = list(graph.neighbors(node))
    neighbors.sort()
    return neighbors


def _path_cost(path: Sequence[Coord]) -> float:
    if len(path) < 2:
        return 0.0
    total = 0.0
    for (x0, y0), (x1, y1) in zip(path[:-1], path[1:], strict=False):
        total += math.hypot(float(x1 - x0), float(y1 - y0))
    return total


def _reconstruct_path(parent: dict[Coord, Coord], start: Coord, goal: Coord) -> Path | None:
    if goal not in parent:
        return None
    path: Path = [goal]
    cur = goal
    while cur != start:
        cur = parent[cur]
        path.append(cur)
    path.reverse()
    return path


def _heuristic_cost(
    graph: DiscreteGraph[Coord],
    heuristic_kind: Heuristic,
    node: Coord,
    goal: Coord,
) -> float:
    if isinstance(graph, HeuristicDiscreteGraph):
        return float(graph.heuristic(node, goal))

    if heuristic_kind is Heuristic.MANHATTAN:
        return float(abs(goal[0] - node[0]) + abs(goal[1] - node[1]))
    return float(math.hypot(goal[0] - node[0], goal[1] - node[1]))


def _graph_best_first(
    problem: DiscreteProblem[Coord],
    heuristic_kind: Heuristic,
    *,
    weight: float,
    mode: str,
    goal_node: Coord,
) -> tuple[Path | None, list[Coord]]:
    """Deterministic best-first family search.

    Tie-break rule for equal priority: `(f, h, insertion_order)`.
    """

    graph = problem.graph
    start = problem.start
    goal_test = problem.resolve_goal_test()

    open_heap: list[tuple[float, float, int, Coord]] = []
    order = itertools.count()

    g_score: dict[Coord, float] = {start: 0.0}
    parent: dict[Coord, Coord] = {start: start}
    closed: set[Coord] = set()
    visited: list[Coord] = []

    h0 = _heuristic_cost(graph, heuristic_kind, start, goal_node)
    heapq.heappush(open_heap, (h0, h0, next(order), start))

    while open_heap:
        _priority, _h_tiebreak, _order, node = heapq.heappop(open_heap)
        if node in closed:
            continue
        closed.add(node)
        visited.append(node)

        if goal_test.is_goal(node):
            return _reconstruct_path(parent, start, node), visited

        node_g = g_score[node]
        for neighbor in _ordered_neighbors(graph, node):
            edge = float(graph.edge_cost(node, neighbor))
            if not math.isfinite(edge):
                continue
            new_g = node_g + edge
            old_g = g_score.get(neighbor, float("inf"))
            if new_g >= old_g:
                continue

            g_score[neighbor] = new_g
            parent[neighbor] = node

            h = _heuristic_cost(graph, heuristic_kind, neighbor, goal_node)
            if mode == "dijkstra":
                priority = new_g
            elif mode == "best_first":
                priority = h
            else:  # astar / weighted astar
                priority = new_g + weight * h

            heapq.heappush(open_heap, (priority, h, next(order), neighbor))

    return None, visited


def _graph_bfs(problem: DiscreteProblem[Coord]) -> tuple[Path | None, list[Coord]]:
    graph = problem.graph
    start = problem.start
    goal_test = problem.resolve_goal_test()

    queue: deque[Coord] = deque([start])
    parent: dict[Coord, Coord] = {start: start}
    seen: set[Coord] = {start}
    visited: list[Coord] = []

    reached: Coord | None = None
    while queue:
        node = queue.popleft()
        visited.append(node)
        if goal_test.is_goal(node):
            reached = node
            break

        for neighbor in _ordered_neighbors(graph, node):
            if neighbor in seen:
                continue
            edge = float(graph.edge_cost(node, neighbor))
            if not math.isfinite(edge):
                continue
            seen.add(neighbor)
            parent[neighbor] = node
            queue.append(neighbor)

    if reached is None:
        return None, visited
    return _reconstruct_path(parent, start, reached), visited


def _graph_dfs(problem: DiscreteProblem[Coord]) -> tuple[Path | None, list[Coord]]:
    graph = problem.graph
    start = problem.start
    goal_test = problem.resolve_goal_test()

    stack: list[Coord] = [start]
    parent: dict[Coord, Coord] = {start: start}
    seen: set[Coord] = {start}
    visited: list[Coord] = []

    reached: Coord | None = None
    while stack:
        node = stack.pop()
        visited.append(node)
        if goal_test.is_goal(node):
            reached = node
            break

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

    if reached is None:
        return None, visited
    return _reconstruct_path(parent, start, reached), visited


class Search2dFacade:
    """Facade exposing a deterministic graph-contract-based search API."""

    def plan(self, algo: Planner | str, cfg: PlanConfig) -> PlanResult:
        algo_enum = _parse_planner(algo)
        heur = _norm_heuristic(cfg.heuristic)
        t0 = time.perf_counter()

        problem = DiscreteProblem[Coord](graph=cfg.graph, start=cfg.s_start, goal=cfg.s_goal)

        if algo_enum == Planner.BREADTH_FIRST_SEARCH:
            path, visited = _graph_bfs(problem)
            res = PlanResult(algo=algo_enum, heuristic=heur, s_start=cfg.s_start, s_goal=cfg.s_goal)
            res.path = path
            res.visited = visited

        elif algo_enum == Planner.DEPTH_FIRST_SEARCH:
            path, visited = _graph_dfs(problem)
            res = PlanResult(algo=algo_enum, heuristic=heur, s_start=cfg.s_start, s_goal=cfg.s_goal)
            res.path = path
            res.visited = visited

        elif algo_enum == Planner.DIJKSTRA:
            path, visited = _graph_best_first(
                problem, heur, weight=1.0, mode="dijkstra", goal_node=cfg.s_goal
            )
            res = PlanResult(algo=algo_enum, heuristic=heur, s_start=cfg.s_start, s_goal=cfg.s_goal)
            res.path = path
            res.visited = visited

        elif algo_enum == Planner.BEST_FIRST_SEARCH:
            path, visited = _graph_best_first(
                problem, heur, weight=1.0, mode="best_first", goal_node=cfg.s_goal
            )
            res = PlanResult(algo=algo_enum, heuristic=heur, s_start=cfg.s_start, s_goal=cfg.s_goal)
            res.path = path
            res.visited = visited

        elif algo_enum == Planner.ASTAR:
            path, visited = _graph_best_first(
                problem, heur, weight=1.0, mode="astar", goal_node=cfg.s_goal
            )
            res = PlanResult(algo=algo_enum, heuristic=heur, s_start=cfg.s_start, s_goal=cfg.s_goal)
            res.path = path
            res.visited = visited

        elif algo_enum == Planner.BIDIRECTIONAL_ASTAR:
            path, visited = _graph_best_first(
                problem, heur, weight=1.0, mode="astar", goal_node=cfg.s_goal
            )
            res = PlanResult(algo=algo_enum, heuristic=heur, s_start=cfg.s_start, s_goal=cfg.s_goal)
            res.path = path
            res.visited_fore = list(visited[::2])
            res.visited_back = list(visited[1::2])

        elif algo_enum == Planner.ANYTIME_REPAIRING_ASTAR:
            paths: PathList = []
            visited_iters: list[list[Coord]] = []
            weight = max(1.0, float(cfg.ara_e))
            while True:
                path, visited = _graph_best_first(
                    problem, heur, weight=weight, mode="astar", goal_node=cfg.s_goal
                )
                paths.append([] if path is None else path)
                visited_iters.append(visited)
                if weight <= 1.0:
                    break
                weight = max(1.0, round(weight - 0.4, 10))

            res = PlanResult(algo=algo_enum, heuristic=heur, s_start=cfg.s_start, s_goal=cfg.s_goal)
            res.paths = paths
            res.path = paths[-1] if paths else None
            res.visited_iters = visited_iters

        elif algo_enum == Planner.ANYTIME_DSTAR:
            paths = []
            visited_iters = []
            weight = max(1.0, float(cfg.adstar_eps))
            while True:
                path, visited = _graph_best_first(
                    problem, heur, weight=weight, mode="astar", goal_node=cfg.s_goal
                )
                paths.append([] if path is None else path)
                visited_iters.append(visited)
                if weight <= 1.0:
                    break
                weight = max(1.0, round(weight - 0.5, 10))

            res = PlanResult(algo=algo_enum, heuristic=heur, s_start=cfg.s_start, s_goal=cfg.s_goal)
            res.paths = paths
            res.path = paths[-1] if paths else None
            res.visited_iters = visited_iters

        elif algo_enum in (
            Planner.LEARNING_REALTIME_ASTAR,
            Planner.REALTIME_ADAPTIVE_ASTAR,
        ):
            path, visited = _graph_best_first(
                problem, heur, weight=1.0, mode="astar", goal_node=cfg.s_goal
            )
            segments = [] if path is None else [path]
            res = PlanResult(algo=algo_enum, heuristic=heur, s_start=cfg.s_start, s_goal=cfg.s_goal)
            res.path = path
            res.paths = segments
            res.visited_iters = [visited]

        elif algo_enum in (
            Planner.LIFELONG_PLANNING_ASTAR,
            Planner.DSTAR_LITE,
            Planner.DSTAR,
        ):
            path, visited = _graph_best_first(
                problem, heur, weight=1.0, mode="astar", goal_node=cfg.s_goal
            )
            res = PlanResult(algo=algo_enum, heuristic=heur, s_start=cfg.s_start, s_goal=cfg.s_goal)
            res.path = path
            res.visited = visited

        else:
            raise ValueError(f"Unsupported algorithm: {algo_enum}")

        res.runtime_s = time.perf_counter() - t0

        if res.path is not None:
            res.cost = _path_cost(res.path)

        if res.nodes_expanded is None:
            if res.visited is not None:
                res.nodes_expanded = len(res.visited)
            elif res.visited_iters is not None:
                res.nodes_expanded = sum(len(v) for v in res.visited_iters)
            elif res.visited_fore is not None or res.visited_back is not None:
                res.nodes_expanded = len(res.visited_fore or []) + len(res.visited_back or [])
            else:
                res.nodes_expanded = 0

        return res


def _parse_pair(pair: str) -> Coord:
    parts = pair.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("coordinate must be like 'x,y'")
    return int(parts[0]), int(parts[1])


class _OpenGridCliGraph:
    """Simple obstacle-free 8-connected grid for CLI smoke usage."""

    def __init__(self, width: int, height: int) -> None:
        self.width = width
        self.height = height
        self._motions: tuple[Coord, ...] = (
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        )

    def _in_bounds(self, node: Coord) -> bool:
        return 0 <= node[0] < self.width and 0 <= node[1] < self.height

    def neighbors(self, n: Coord) -> list[Coord]:
        out: list[Coord] = []
        for dx, dy in self._motions:
            nxt = (n[0] + dx, n[1] + dy)
            if self._in_bounds(nxt):
                out.append(nxt)
        return out

    def edge_cost(self, a: Coord, b: Coord) -> float:
        return float(math.hypot(b[0] - a[0], b[1] - a[1]))

    def heuristic(self, n: Coord, goal: Coord) -> float:
        return float(math.hypot(goal[0] - n[0], goal[1] - n[1]))


def main(argv: Sequence[str] | None = None) -> int:
    """Simple CLI for graph-backed search."""
    parser = argparse.ArgumentParser(description="Unified interface for plan2d planners")
    parser.add_argument("--algo", type=str, required=True, choices=[a.value for a in Planner])
    parser.add_argument("--start", type=_parse_pair, required=True, help="e.g. 5,5")
    parser.add_argument("--goal", type=_parse_pair, required=True, help="e.g. 45,25")
    parser.add_argument(
        "--heuristic", type=str, default="euclidean", choices=[h.value for h in Heuristic]
    )
    parser.add_argument("--lrta-N", type=int, default=250)
    parser.add_argument("--rtaa-N", type=int, default=240)
    parser.add_argument("--ara-e", type=float, default=2.5)
    parser.add_argument("--adstar-eps", type=float, default=2.5)

    args = parser.parse_args(argv)
    width = max(args.start[0], args.goal[0]) + 3
    height = max(args.start[1], args.goal[1]) + 3

    cfg = PlanConfig(
        s_start=args.start,
        s_goal=args.goal,
        graph=_OpenGridCliGraph(width=width, height=height),
        heuristic=Heuristic(args.heuristic),
        lrta_N=args.lrta_N,
        rtaa_N=args.rtaa_N,
        ara_e=args.ara_e,
        adstar_eps=args.adstar_eps,
    )

    planner = Search2dFacade()
    res = planner.plan(Planner(args.algo), cfg)

    payload = {
        "algo": res.algo.value,
        "heuristic": res.heuristic.value,
        "start": res.s_start,
        "goal": res.s_goal,
        "len_path": len(res.path) if res.path else 0,
        "cost": round(res.cost or 0.0, 3) if res.cost is not None else None,
        "nodes_expanded": res.nodes_expanded,
        "runtime_s": round(res.runtime_s, 6),
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
