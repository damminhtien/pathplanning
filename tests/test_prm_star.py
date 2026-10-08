"""Focused PRM* roadmap reuse checks on an open bounded space."""

from __future__ import annotations

import numpy as np

from pathplanning import RoadmapParams, plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.core.trace import TraceOptions
from pathplanning.planners.sampling.prm_star import PrmStarRoadmap
from pathplanning.spaces.grid2d import Grid2DSamplingSpace


def test_prm_star_reuses_roadmap_across_queries_and_traces() -> None:
    space = Grid2DSamplingSpace(x_range=(0.0, 4.0), y_range=(0.0, 4.0))
    with PrmStarRoadmap(
        space,
        2,
        RoadmapParams(sample_count=24, gamma=8.0),
        np.random.default_rng(5),
    ) as roadmap:
        roadmap.build(world_version="v1")
        built_stats = roadmap.build_stats
        first = roadmap.query((0.5, 0.5), (1.5, 0.5), world_version="v1", trace=TraceOptions(4096))
        second = roadmap.query((0.5, 2.5), (1.5, 2.5), world_version="v1")
        already_at_goal = roadmap.query((2.5, 2.5), (2.5, 2.5), world_version="v1")

        assert first.success and second.success and already_at_goal.success
        assert np.isclose(first.stats["path_cost"], 1.0)
        assert np.isclose(second.stats["path_cost"], 1.0)
        assert np.isclose(already_at_goal.stats["path_cost"], 0.0)
        assert first.trace is not None and first.trace.events.size > 0
        assert roadmap.build_stats["roadmap_vertices"] == built_stats["roadmap_vertices"] == 24


def test_prm_star_world_version_change_rebuilds_roadmap() -> None:
    space = Grid2DSamplingSpace(x_range=(0.0, 4.0), y_range=(0.0, 4.0))
    with PrmStarRoadmap(
        space,
        2,
        RoadmapParams(sample_count=16, gamma=8.0),
        np.random.default_rng(9),
    ) as roadmap:
        first = roadmap.query((0.5, 0.5), (1.5, 0.5), world_version=1)
        second = roadmap.query((0.5, 2.5), (1.5, 2.5), world_version=2)
        assert first.success and second.success
        assert roadmap.build_stats["roadmap_vertices"] == 16


def test_prm_star_registry_entry_uses_native_roadmap() -> None:
    space = Grid2DSamplingSpace(x_range=(0.0, 4.0), y_range=(0.0, 4.0))
    result = plan_continuous(
        ContinuousProblem(space, (0.5, 0.5), GoalState((1.5, 0.5))),
        planner="prm_star",
        params=RoadmapParams(sample_count=12, gamma=8.0),
        seed=4,
    )
    assert result.success
    assert np.isclose(result.stats["path_cost"], 1.0)
