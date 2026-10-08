"""Focused checks for reusable EIRM* roadmaps."""

from __future__ import annotations

import numpy as np

from pathplanning import RoadmapParams, TraceOptions, plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.planners.sampling.eirm_star import EirmStarRoadmap
from pathplanning.spaces.grid2d import Grid2DSamplingSpace


def test_eirm_star_reuses_roadmap_and_edge_validation() -> None:
    space = Grid2DSamplingSpace(x_range=(0.0, 10.0), y_range=(0.0, 10.0))
    with EirmStarRoadmap(
        space,
        2,
        RoadmapParams(sample_count=96, gamma=8.0, max_expansions=5_000),
        np.random.default_rng(7),
    ) as roadmap:
        first = roadmap.query((0.5, 0.5), (9.5, 9.5), world_version="v1")
        vertices = roadmap.build_stats["roadmap_vertices"]
        second = roadmap.query((0.5, 0.5), (9.5, 9.5), world_version="v1")
        traced = roadmap.query(
            (0.5, 0.5), (9.5, 9.5), world_version="v1", trace=TraceOptions(8_192)
        )

        assert first.success and second.success and traced.success
        assert first.stats["motion_checks"] > second.stats["motion_checks"]
        assert traced.trace is not None and traced.trace.events.size > 0
        assert roadmap.build_stats["roadmap_vertices"] == vertices == 96


def test_eirm_star_registry_entry_uses_roadmap_params() -> None:
    space = Grid2DSamplingSpace(x_range=(0.0, 2.0), y_range=(0.0, 2.0))
    result = plan_continuous(
        ContinuousProblem(space, (0.25, 0.25), GoalState((1.75, 1.75))),
        planner="eirm_star",
        params=RoadmapParams(sample_count=12, gamma=8.0),
        seed=4,
    )

    assert result.success
