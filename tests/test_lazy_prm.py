"""Focused lazy PRM validation checks on a small obstacle world."""

from __future__ import annotations

import numpy as np

from pathplanning import RoadmapParams, plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.core.trace import TraceOptions
from pathplanning.planners.sampling.prm_star import LazyPrmRoadmap
from pathplanning.spaces.grid2d import Grid2DSamplingSpace


def test_lazy_prm_validates_candidate_edges_and_reuses_results() -> None:
    space = Grid2DSamplingSpace(
        x_range=(0.0, 5.0),
        y_range=(0.0, 5.0),
        obs_rectangle=([2, 0, 1, 4],),
        delta=0.0,
        collision_step=0.1,
    )
    with LazyPrmRoadmap(
        space,
        2,
        RoadmapParams(sample_count=64, gamma=8.0, collision_step=0.1),
        np.random.default_rng(13),
    ) as roadmap:
        roadmap.build(world_version="wall-v1")
        assert roadmap.build_stats["roadmap_motion_checks"] == 0.0

        first = roadmap.query((0.5, 2.5), (4.5, 2.5), world_version="wall-v1")
        second = roadmap.query((0.5, 2.5), (4.5, 2.5), world_version="wall-v1")
        traced = roadmap.query(
            (0.5, 2.5),
            (4.5, 2.5),
            world_version="wall-v1",
            trace=TraceOptions(8192),
        )

        assert first.success and second.success and traced.success
        assert first.path is not None
        assert all(space.is_motion_valid(a, b) for a, b in zip(first.path, first.path[1:]))
        assert first.stats["motion_checks"] > second.stats["motion_checks"]
        assert traced.trace is not None and traced.trace.events.size > 0


def test_lazy_prm_is_registered_for_continuous_problems() -> None:
    space = Grid2DSamplingSpace(x_range=(0.0, 4.0), y_range=(0.0, 4.0))
    result = plan_continuous(
        ContinuousProblem(space, (0.5, 0.5), GoalState((1.5, 0.5))),
        planner="lazy_prm",
        params=RoadmapParams(sample_count=12, gamma=8.0),
        seed=4,
    )
    assert result.success
    assert np.isclose(result.stats["path_cost"], 1.0)
