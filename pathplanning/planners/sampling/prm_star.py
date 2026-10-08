"""PRM* with a reusable native roadmap and exact-goal queries."""

from __future__ import annotations

from collections.abc import Mapping

import numpy as np

from pathplanning.core.contracts import ContinuousProblem, ContinuousSpace, State
from pathplanning.core.params import RoadmapParams, RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.core.trace import TraceOptions
from pathplanning.core.types import RNG
from pathplanning.native.continuous import NativePrmStarRoadmap
from pathplanning.planners.sampling._internal.continuous import (
    euclidean_distance,
    exact_goal_state,
    validate_objective,
)


def _resolve_params(
    problem: ContinuousProblem[State],
    params: RrtParams | Mapping[str, object] | None,
) -> tuple[RoadmapParams, object]:
    values = dict(problem.params or {})
    if params is not None:
        if isinstance(params, RrtParams):
            raise TypeError("PRM* requires RoadmapParams or a parameter mapping")
        values.update(params)
    world_version = values.pop("world_version", 0)
    return RoadmapParams(**values).validate(), world_version


class PrmStarRoadmap:
    """Reusable PRM* roadmap with build, query, clear, and reset operations."""

    def __init__(
        self,
        space: ContinuousSpace[State],
        dimension: int,
        params: RoadmapParams | None = None,
        rng: RNG | None = None,
        *,
        _lazy: bool = False,
    ) -> None:
        if isinstance(dimension, bool) or type(dimension) is not int or dimension <= 0:
            raise ValueError("dimension must be a positive integer")
        self._native = NativePrmStarRoadmap(
            space,
            dimension,
            RoadmapParams() if params is None else params.validate(),
            np.random.default_rng(0) if rng is None else rng,
            lazy=_lazy,
        )

    @property
    def build_stats(self) -> Mapping[str, float]:
        """Return counters from the most recent roadmap build."""
        return dict(self._native.build_stats)

    def build(self, *, world_version: object = 0) -> None:
        """Build once for this version, rebuilding when the world version changes."""
        self._native.build(world_version=world_version)

    def query(
        self,
        start: object,
        goal: object,
        *,
        world_version: object = 0,
        trace: TraceOptions | None = None,
    ) -> PlanResult:
        """Query the roadmap while preserving its samples for later queries."""
        return self._native.query(start, goal, world_version=world_version, trace=trace)

    def clear_query(self) -> None:
        """Release query-local state without discarding roadmap samples."""
        self._native.clear_query()

    def reset(self) -> None:
        """Discard roadmap samples and start a fresh build session."""
        self._native.reset()

    def close(self) -> None:
        """Release native roadmap buffers."""
        self._native.close()

    def __enter__(self) -> PrmStarRoadmap:
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


class LazyPrmRoadmap(PrmStarRoadmap):
    """Reusable PRM roadmap that validates candidate edges on demand."""

    def __init__(
        self,
        space: ContinuousSpace[State],
        dimension: int,
        params: RoadmapParams | None = None,
        rng: RNG | None = None,
    ) -> None:
        super().__init__(space, dimension, params, rng, _lazy=True)


def _plan_roadmap(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
    roadmap_type: type[PrmStarRoadmap],
    planner_label: str,
) -> PlanResult:
    validate_objective(problem.objective, planner_label)
    resolved_params, world_version = _resolve_params(problem, params)
    start = np.asarray(problem.start, dtype=np.float64)
    if start.ndim != 1 or start.size == 0 or not np.all(np.isfinite(start)):
        raise ValueError("start must be a finite non-empty state vector")
    goal = exact_goal_state(problem.goal, dim=int(start.size))
    euclidean_distance(problem.space, start, goal)
    with roadmap_type(problem.space, int(start.size), resolved_params, rng=rng) as roadmap:
        roadmap.build(world_version=world_version)
        return roadmap.query(start, goal, world_version=world_version, trace=trace)


def plan_prm_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Build and query a native PRM* roadmap for Euclidean path length."""
    return _plan_roadmap(
        problem,
        params=params,
        rng=rng,
        trace=trace,
        roadmap_type=PrmStarRoadmap,
        planner_label="PRM*",
    )


def plan_lazy_prm(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
    trace: TraceOptions | None = None,
) -> PlanResult:
    """Build and query a native lazy PRM roadmap for Euclidean path length."""
    return _plan_roadmap(
        problem,
        params=params,
        rng=rng,
        trace=trace,
        roadmap_type=LazyPrmRoadmap,
        planner_label="Lazy PRM",
    )


__all__ = ["LazyPrmRoadmap", "PrmStarRoadmap", "plan_lazy_prm", "plan_prm_star"]
