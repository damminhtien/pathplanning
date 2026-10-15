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


def test_prm_star_uses_spatial_index_for_native_roadmaps() -> None:
    sample_count = 320
    space = Grid2DSamplingSpace(x_range=(0.0, 20.0), y_range=(0.0, 20.0))
    with PrmStarRoadmap(
        space,
        2,
        RoadmapParams(sample_count=sample_count, gamma=8.0),
        np.random.default_rng(23),
    ) as roadmap:
        roadmap.build(world_version="v1")

        all_pairs = sample_count * (sample_count - 1) // 2
        assert roadmap.build_stats["roadmap_vertices"] == sample_count
        assert roadmap.build_stats["roadmap_connection_candidates"] < all_pairs // 4
        assert (
            roadmap.build_stats["roadmap_motion_checks"]
            <= roadmap.build_stats["roadmap_connection_candidates"]
        )


def test_prm_star_keeps_bidirectional_validation_for_python_callbacks() -> None:
    motion_calls: list[tuple[tuple[float, ...], tuple[float, ...]]] = []

    class CallbackSpace:
        def sample_free(self, rng: np.random.Generator) -> np.ndarray:
            return rng.uniform(0.0, 1.0, size=2)

        def is_state_valid(self, _state: np.ndarray) -> bool:
            return True

        def is_motion_valid(self, start: np.ndarray, end: np.ndarray) -> bool:
            motion_calls.append((tuple(start), tuple(end)))
            return True

        def distance(self, start: np.ndarray, end: np.ndarray) -> float:
            return float(np.linalg.norm(end - start))

        def steer(self, start: np.ndarray, target: np.ndarray, step_size: float) -> np.ndarray:
            delta = target - start
            length = float(np.linalg.norm(delta))
            return target if length <= step_size else start + delta * (step_size / length)

    with PrmStarRoadmap(
        CallbackSpace(),
        2,
        RoadmapParams(sample_count=8, gamma=10.0, allow_python_callbacks=True),
        np.random.default_rng(19),
    ) as roadmap:
        roadmap.build()

        candidates = int(roadmap.build_stats["roadmap_connection_candidates"])
        assert candidates > 0
        assert roadmap.build_stats["roadmap_motion_checks"] == 2 * candidates
        checked = set(motion_calls)
        assert all((end, start) in checked for start, end in checked)
