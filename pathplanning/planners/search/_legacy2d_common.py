"""Compatibility helpers for legacy 2D search planners.

These helpers preserve old module/class surfaces while routing execution
through the contract-based Search2dFacade.
"""

from __future__ import annotations

from collections.abc import Iterable
import math

from pathplanning.core.contracts import DiscreteGraph
from pathplanning.planners.search.plan2d_facade import (
    Heuristic,
    PlanConfig,
    PlanResult,
    Planner,
    Search2dFacade,
)

Coord = tuple[int, int]


class OpenGridLegacyGraph(DiscreteGraph[Coord]):
    """Obstacle-free 8-connected grid used as a default compatibility graph."""

    def __init__(self, width: int = 51, height: int = 31) -> None:
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

    def neighbors(self, n: Coord) -> Iterable[Coord]:
        for dx, dy in self._motions:
            nxt = (n[0] + dx, n[1] + dy)
            if self._in_bounds(nxt):
                yield nxt

    def edge_cost(self, a: Coord, b: Coord) -> float:
        return float(math.hypot(b[0] - a[0], b[1] - a[1]))

    def heuristic(self, n: Coord, goal: Coord) -> float:
        return float(math.hypot(goal[0] - n[0], goal[1] - n[1]))



def _coerce_coord(value: tuple[int, int] | tuple[float, float]) -> Coord:
    return (int(value[0]), int(value[1]))



def _normalize_heuristic(heuristic_type: str | Heuristic) -> Heuristic:
    if isinstance(heuristic_type, Heuristic):
        return heuristic_type
    text = str(heuristic_type).lower().strip()
    if text.startswith("manh"):
        return Heuristic.MANHATTAN
    return Heuristic.EUCLIDEAN



def _default_graph_for(start: Coord, goal: Coord) -> OpenGridLegacyGraph:
    width = max(51, start[0] + 3, goal[0] + 3)
    height = max(31, start[1] + 3, goal[1] + 3)
    return OpenGridLegacyGraph(width=width, height=height)



def reverse_path(path: list[Coord] | None) -> list[Coord]:
    if path is None:
        return []
    return list(reversed(path))


class LegacySearch2DBase:
    """Base adapter for legacy planner classes."""

    _planner_kind: Planner = Planner.ASTAR

    def __init__(
        self,
        s_start: tuple[int, int] | tuple[float, float],
        s_goal: tuple[int, int] | tuple[float, float],
        heuristic_type: str | Heuristic = "euclidean",
        graph: DiscreteGraph[Coord] | None = None,
    ) -> None:
        self.s_start: Coord = _coerce_coord(s_start)
        self.s_goal: Coord = _coerce_coord(s_goal)
        self.heuristic_type = _normalize_heuristic(heuristic_type)
        self._graph = graph
        self._facade = Search2dFacade()

    def _plan(
        self,
        planner: Planner | None = None,
        *,
        lrta_n: int = 250,
        rtaa_n: int = 240,
        ara_e: float = 2.5,
        adstar_eps: float = 2.5,
    ) -> PlanResult:
        graph = self._graph if self._graph is not None else _default_graph_for(self.s_start, self.s_goal)
        cfg = PlanConfig(
            s_start=self.s_start,
            s_goal=self.s_goal,
            graph=graph,
            heuristic=self.heuristic_type,
            lrta_N=lrta_n,
            rtaa_N=rtaa_n,
            ara_e=ara_e,
            adstar_eps=adstar_eps,
        )
        return self._facade.plan(planner or self._planner_kind, cfg)


__all__ = ["Coord", "LegacySearch2DBase", "OpenGridLegacyGraph", "reverse_path"]
