"""
Unified interface for the plan2d planners.

This module provides a single, production-grade API to run any planner in
`pathplanning/planners/search` with consistent inputs/outputs,
plus a tiny CLI.

Design goals
------------
- Consistent API across heterogeneous implementations (A*, D*, LPA*, LRTA*, RTAA*, ARA*, …)
- Clear typing, dataclasses, small but helpful metrics (cost, nodes expanded, runtime)
- No plotting by default (headless); plotting stays available in the original files
- Minimal coupling: we don’t modify original files

Usage (Python)
--------------
from plan2d_interface import Search2D, Planner, Heuristic, PlanConfig

planner = Search2D()
cfg = PlanConfig(
    s_start=(5,5), s_goal=(45,25), heuristic=Heuristic.EUCLIDEAN,
    lrta_N=250, rtaa_N=240, ara_e=2.5, adstar_eps=2.5)
result = planner.plan(Planner.ASTAR, cfg)
print(result.path)         # normalized start->goal
print(result.cost)         # total path length

CLI
---
python -m plan2d_interface --algo astar --start 5,5 --goal 45,25

author: damminhtien
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from enum import Enum
import importlib
import json
import time
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

# -- Type definitions ----------------------------------------------------------
Coord = Tuple[int, int]
Path = List[Coord]
PathList = List[Path]


class Heuristic(str, Enum):
    """Heuristic types for pathfinding algorithms."""

    EUCLIDEAN = "euclidean"
    MANHATTAN = "manhattan"


class Planner(str, Enum):
    """Path planning algorithms available in the plan2d module."""

    # Uninformed Search
    BREADTH_FIRST_SEARCH = "breadth_first_search"
    DEPTH_FIRST_SEARCH = "depth_first_search"
    # Best-First & Dijkstra
    DIJKSTRA = "dijkstra"
    BEST_FIRST_SEARCH = "best_first_search"
    # A* Variants
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
    """Configuration for pathfinding algorithms."""

    s_start: Coord
    s_goal: Coord
    heuristic: Heuristic = Heuristic.EUCLIDEAN

    # Plannerrithm-specific knobs (kept together for convenience)
    lrta_N: int = 250  # LRTA*: expansions per iteration
    rtaa_N: int = 240  # RTAA*: expansions per iteration
    ara_e: float = 2.5  # ARA*: initial inflation factor
    adstar_eps: float = 2.5  # AD*: initial inflation factor

    # Optional: future custom env or motion set (not used by stock impls)
    # obs: Optional[set[Coord]] = None


@dataclass
class PlanResult:
    """Result of a pathfinding plan execution."""

    algo: Planner
    heuristic: Heuristic
    s_start: Coord
    s_goal: Coord

    # Normalized final path: start -> goal when available
    path: Optional[Path] = None

    # Some planners produce multiple iterations (ARA*, LRTA*, RTAA*)
    paths: Optional[PathList] = None

    # Visited/expanded nodes. For RT algorithms this is often per-iteration.
    visited: Optional[List[Coord]] = None
    visited_fore: Optional[List[Coord]] = None  # Bi-A*
    visited_back: Optional[List[Coord]] = None  # Bi-A*
    visited_iters: Optional[List[List[Coord]]] = None

    # Metrics
    cost: Optional[float] = None  # sum of euclidean segment lengths
    nodes_expanded: Optional[int] = None
    runtime_s: float = 0.0

    # Raw (implementation-specific) payload for power users
    raw: Dict[str, Any] = field(default_factory=dict)


# -- Utility helpers -----------------------------------------------------------


def _norm_heuristic(h: Heuristic | str) -> str:
    if isinstance(h, Heuristic):
        return h.value
    s = str(h).lower()
    return "manhattan" if s.startswith("manh") else "euclidean"


def _path_cost(path: Sequence[Coord]) -> float:
    if not path or len(path) < 2:
        return 0.0
    total = 0.0
    for (x0, y0), (x1, y1) in zip(path[:-1], path[1:]):
        total += ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
    return total


def _ensure_start_to_goal(
    path: Optional[Sequence[Coord]], start: Coord, goal: Coord
) -> Optional[List[Coord]]:
    """Ensure the path starts at `start` and ends at `goal`, normalizing if needed."""
    if not path:
        return None
    p = list(path)
    if len(p) == 0:
        return p
    if p[0] == start and p[-1] == goal:
        return p
    if p[0] == goal and p[-1] == start:
        p.reverse()
        return p
    # Some implementations may include duplicates; normalize lightly
    if p[0] != start and start in p:
        idx = p.index(start)
        p = p[idx:]
    if p[-1] != goal and goal in p:
        idx = p.index(goal)
        p = p[: idx + 1]
    # If still not matched, leave as-is.
    return p


def _concat_unique(seq_of_paths: Iterable[Sequence[Coord]]) -> List[Coord]:
    """Concatenate segments while avoiding immediate duplicates."""
    out: List[Coord] = []
    for seg in seq_of_paths:
        for c in seg:
            if not out or out[-1] != c:
                out.append(c)
    return out


def _parse_planner(algo: Planner | str) -> Planner:
    """Parse planner enum values."""
    if isinstance(algo, Planner):
        return algo
    return Planner(str(algo).lower().strip())


def _import_local(module_name: str):
    """
    Import a sibling module under plan2d with support for:
    1) package imports (`pathplanning.planners.search...`)
    2) script execution from this folder (`python run.py`)
    """
    if __package__:
        return importlib.import_module(f"{__package__}.{module_name}")
    try:
        return importlib.import_module(f"pathplanning.planners.search.{module_name}")
    except ModuleNotFoundError:
        return importlib.import_module(module_name)


# -- Core interface ------------------------------------------------------------
class Search2dFacade:
    """
    Facade providing a unified .plan() API on top of the original planners.
    The original modules are imported lazily to avoid side effects at import time.
    """

    def plan(self, algo: Planner | str, cfg: PlanConfig) -> PlanResult:
        """Run the specified planner algorithm with the given configuration.
        :param algo: Planner algorithm to run (e.g. Planner.ASTAR)
        :param cfg: PlanConfig with start, goal, heuristic, and optional knobs
        :return: PlanResult with the path, cost, nodes expanded, runtime, etc.
        """
        algo = _parse_planner(algo)
        if not isinstance(algo, Planner):
            raise ValueError(f"Unsupported algorithm type: {type(algo)}")
        name = algo.value
        heur = _norm_heuristic(cfg.heuristic)

        t0 = time.perf_counter()

        if name in (
            Planner.ASTAR,
            Planner.BREADTH_FIRST_SEARCH,
            Planner.DEPTH_FIRST_SEARCH,
            Planner.DIJKSTRA,
            Planner.BEST_FIRST_SEARCH,
        ):
            res = self._run_astar_family(name, heur, cfg)
        elif name == Planner.BIDIRECTIONAL_ASTAR:
            res = self._run_bi_astar(heur, cfg)
        elif name == Planner.ANYTIME_REPAIRING_ASTAR:
            res = self._run_arastar(heur, cfg)
        elif name == Planner.LEARNING_REALTIME_ASTAR:
            res = self._run_lrta(heur, cfg)
        elif name == Planner.REALTIME_ADAPTIVE_ASTAR:
            res = self._run_rtaa(heur, cfg)
        elif name == Planner.LIFELONG_PLANNING_ASTAR:
            res = self._run_lpastar(heur, cfg)
        elif name == Planner.DSTAR_LITE:
            res = self._run_dstar_lite(heur, cfg)
        elif name == Planner.DSTAR:
            res = self._run_dstar(heur, cfg)
        elif name == Planner.ANYTIME_DSTAR:
            res = self._run_adstar(heur, cfg)
        else:
            raise ValueError(f"Unsupported algorithm: {algo}")

        res.runtime_s = time.perf_counter() - t0
        # Compute default cost/expanded if missing
        if res.path and res.cost is None:
            res.cost = _path_cost(res.path)
        if res.nodes_expanded is None:
            if res.visited is not None:
                res.nodes_expanded = len(res.visited)
            elif res.visited_iters is not None:
                res.nodes_expanded = sum(len(v) for v in res.visited_iters)
            elif res.visited_fore is not None or res.visited_back is not None:
                res.nodes_expanded = len(res.visited_fore or []) + len(res.visited_back or [])
        return res

    def _run_astar_family(self, name: Planner, heur: str, cfg: PlanConfig) -> PlanResult:
        """A*, BFS, DFS, Dijkstra, Best-First (all inherit Astar or only override searching())."""
        mod_map = {
            Planner.ASTAR: ("astar", "Astar"),
            Planner.BREADTH_FIRST_SEARCH: ("breadth_first_search", "BreadthFirstSearch"),
            Planner.DEPTH_FIRST_SEARCH: ("depth_first_search", "DepthFirstSearch"),
            Planner.DIJKSTRA: ("dijkstra", "Dijkstra"),
            Planner.BEST_FIRST_SEARCH: ("best_first_search", "BestFirstSearch"),
        }
        mod_name, cls_name = mod_map[name]
        Mod = _import_local(mod_name)
        Cls = getattr(Mod, cls_name)
        inst = Cls(cfg.s_start, cfg.s_goal, heur)
        path_raw, visited = inst.searching()  # type: ignore[assignment]
        path = _ensure_start_to_goal(path_raw, cfg.s_start, cfg.s_goal)
        return PlanResult(
            algo=name,
            heuristic=Heuristic(heur),
            s_start=cfg.s_start,
            s_goal=cfg.s_goal,
            path=path,
            # de-dup like dfs example
            visited=list(dict.fromkeys(visited)) if visited else None,
        )

    def _run_bi_astar(self, heur: str, cfg: PlanConfig) -> PlanResult:
        Mod = _import_local("bidirectional_astar")
        Cls = getattr(Mod, "BidirectionalAstar")
        inst = Cls(cfg.s_start, cfg.s_goal, heur)
        path, visited_fore, visited_back = inst.searching()
        path = _ensure_start_to_goal(path, cfg.s_start, cfg.s_goal)
        return PlanResult(
            algo=Planner.BIDIRECTIONAL_ASTAR,
            heuristic=Heuristic(heur),
            s_start=cfg.s_start,
            s_goal=cfg.s_goal,
            path=path,
            visited_fore=visited_fore,
            visited_back=visited_back,
        )

    def _run_arastar(self, heur: str, cfg: PlanConfig) -> PlanResult:
        Mod = _import_local("anytime_repairing_astar")
        Cls = getattr(Mod, "AnytimeRepairingAstar")
        inst = Cls(cfg.s_start, cfg.s_goal, cfg.ara_e, heur)
        paths_raw, visited_iters = inst.searching()
        # Normalize each path
        paths = [_ensure_start_to_goal(p, cfg.s_start, cfg.s_goal) or [] for p in paths_raw]
        final_path = paths[-1] if paths else None
        return PlanResult(
            algo=Planner.ANYTIME_REPAIRING_ASTAR,
            heuristic=Heuristic(heur),
            s_start=cfg.s_start,
            s_goal=cfg.s_goal,
            path=final_path,
            paths=paths,
            visited_iters=visited_iters,
        )

    def _run_lrta(self, heur: str, cfg: PlanConfig) -> PlanResult:
        Mod = _import_local("learning_realtime_astar")
        Cls = getattr(Mod, "LearningRealtimeAstar")
        inst = Cls(cfg.s_start, cfg.s_goal, cfg.lrta_N, heur)
        inst.searching()
        # Combine segments like plotting.animation_lrta
        # list of per-iteration small paths
        segments: List[List[Coord]] = list(inst.path)
        combined = _concat_unique(segments)
        # remove duplicate initial node once (mimic plotting.animation_lrta)
        if combined and combined[0] != cfg.s_start and cfg.s_start in combined:
            combined.pop(combined.index(cfg.s_start))
        final_path = _ensure_start_to_goal(combined, cfg.s_start, cfg.s_goal)
        visited_iters = [list(v) for v in inst.visited]
        return PlanResult(
            algo=Planner.LEARNING_REALTIME_ASTAR,
            heuristic=Heuristic(heur),
            s_start=cfg.s_start,
            s_goal=cfg.s_goal,
            path=final_path,
            paths=segments,
            visited_iters=visited_iters,
            raw={"h_table_size": len(inst.h_table)},
        )

    def _run_rtaa(self, heur: str, cfg: PlanConfig) -> PlanResult:
        Mod = _import_local("realtime_adaptive_astar")
        Cls = getattr(Mod, "RealtimeAdaptiveAstar")
        inst = Cls(cfg.s_start, cfg.s_goal, cfg.rtaa_N, heur)
        inst.searching()
        segments: List[List[Coord]] = list(inst.path)
        combined = _concat_unique(segments)
        final_path = _ensure_start_to_goal(combined, cfg.s_start, cfg.s_goal)
        visited_iters = [list(v) for v in inst.visited]
        return PlanResult(
            algo=Planner.REALTIME_ADAPTIVE_ASTAR,
            heuristic=Heuristic(heur),
            s_start=cfg.s_start,
            s_goal=cfg.s_goal,
            path=final_path,
            paths=segments,
            visited_iters=visited_iters,
        )

    def _run_lpastar(self, heur: str, cfg: PlanConfig) -> PlanResult:
        Mod = _import_local("lifelong_planning_astar")
        Cls = getattr(Mod, "LifelongPlanningAstar")
        inst = Cls(cfg.s_start, cfg.s_goal, heur)
        # Headless run: directly compute shortest path and extract, no plotting
        inst.ComputeShortestPath()
        path = inst.extract_path()
        path = _ensure_start_to_goal(path, cfg.s_start, cfg.s_goal)
        visited = list(inst.visited) if hasattr(inst, "visited") else None
        return PlanResult(
            algo=Planner.LIFELONG_PLANNING_ASTAR,
            heuristic=Heuristic(heur),
            s_start=cfg.s_start,
            s_goal=cfg.s_goal,
            path=path,
            visited=visited,
        )

    def _run_dstar_lite(self, heur: str, cfg: PlanConfig) -> PlanResult:
        Mod = _import_local("dstar_lite")
        Cls = getattr(Mod, "DStarLite")
        inst = Cls(cfg.s_start, cfg.s_goal, heur)
        inst.ComputePath()  # plan once
        path = inst.extract_path()
        path = _ensure_start_to_goal(path, cfg.s_start, cfg.s_goal)
        visited = list(inst.visited) if hasattr(inst, "visited") else None
        return PlanResult(
            algo=Planner.DSTAR_LITE,
            heuristic=Heuristic(heur),
            s_start=cfg.s_start,
            s_goal=cfg.s_goal,
            path=path,
            visited=visited,
        )

    def _run_dstar(self, heur: str, cfg: PlanConfig) -> PlanResult:
        # Classic D*: replicate headless portion of run()
        Mod = _import_local("dstar")
        Cls = getattr(Mod, "Dstar")
        inst = Cls(cfg.s_start, cfg.s_goal)
        inst.init()
        inst.insert(cfg.s_goal, 0)
        while True:
            inst.process_state()
            if inst.t[cfg.s_start] == "CLOSED":
                break
        path = inst.extract_path(cfg.s_start, cfg.s_goal)
        path = _ensure_start_to_goal(path, cfg.s_start, cfg.s_goal)
        visited = list(inst.visited) if hasattr(inst, "visited") else None
        return PlanResult(
            algo=Planner.DSTAR,
            heuristic=Heuristic(heur),
            s_start=cfg.s_start,
            s_goal=cfg.s_goal,
            path=path,
            visited=visited,
        )

    def _run_adstar(self, heur: str, cfg: PlanConfig) -> PlanResult:
        # Headless version of Anytime D*
        Mod = _import_local("anytime_dstar")
        Cls = getattr(Mod, "AnytimeDstar")
        inst = Cls(cfg.s_start, cfg.s_goal, cfg.adstar_eps, heur)

        paths: List[List[Coord]] = []
        visited_iters: List[List[Coord]] = []

        def _snap():
            p = inst.extract_path()
            paths.append(list(p))
            visited_iters.append(list(inst.visited))
            inst.visited = set()

        inst.ComputeOrImprovePath()
        _snap()

        while inst.eps > 1.0:
            inst.eps -= 0.5
            inst.OPEN.update(inst.INCONS)
            for s in list(inst.OPEN.keys()):
                inst.OPEN[s] = inst.Key(s)
            inst.CLOSED = set()
            inst.ComputeOrImprovePath()
            _snap()

        final_path = _ensure_start_to_goal(paths[-1] if paths else None, cfg.s_start, cfg.s_goal)
        return PlanResult(
            algo=Planner.ANYTIME_DSTAR,
            heuristic=Heuristic(heur),
            s_start=cfg.s_start,
            s_goal=cfg.s_goal,
            path=final_path,
            paths=paths,
            visited_iters=visited_iters,
        )


# -- Simple CLI ----------------------------------------------------------------


def _parse_pair(pair: str) -> Coord:
    parts = pair.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("coordinate must be like 'x,y'")
    return int(parts[0]), int(parts[1])


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Main entry point for the CLI interface."""
    # Define the CLI parser
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

    cfg = PlanConfig(
        s_start=args.start,
        s_goal=args.goal,
        heuristic=Heuristic(args.heuristic),
        lrta_N=args.lrta_N,
        rtaa_N=args.rtaa_N,
        ara_e=args.ara_e,
        adstar_eps=args.adstar_eps,
    )
    planner = Search2dFacade()
    res = planner.plan(Planner(args.algo), cfg)

    # Pretty-print a compact JSON summary
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
