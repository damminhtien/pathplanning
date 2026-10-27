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
        assert first.path is not None and second.path is not None
        np.testing.assert_allclose(first.path[[0, -1]], [[0.5, 0.5], [9.5, 9.5]])
        np.testing.assert_allclose(second.path, first.path)
        assert all(space.is_motion_valid(a, b) for a, b in zip(first.path, first.path[1:]))
        path_cost = np.linalg.norm(np.diff(first.path, axis=0), axis=1).sum()
        assert np.isclose(first.stats["path_cost"], path_cost)
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
    assert result.path is not None
    np.testing.assert_allclose(result.path[[0, -1]], [[0.25, 0.25], [1.75, 1.75]])
    assert np.isclose(
        result.stats["path_cost"], np.linalg.norm(np.diff(result.path, axis=0), axis=1).sum()
    )
