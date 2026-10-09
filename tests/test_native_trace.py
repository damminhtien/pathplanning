"""Diagnostic tracing must preserve planner results and own its buffers."""

from __future__ import annotations

import gc

import numpy as np
import pytest

from pathplanning import TraceOptions, plan_continuous, plan_discrete
from pathplanning.core.contracts import ContinuousProblem, DiscreteProblem, GoalState
from pathplanning.native import NativeLibraryLoadError
from pathplanning.planners.sampling.dynamic_rrt import DynamicRRT3D, DynamicRRT3DConfig
from pathplanning.registry import list_planners
from pathplanning.spaces.continuous_3d import AABB, ContinuousSpace3D
from pathplanning.spaces.grid2d import Grid2DSamplingSpace, Grid2DSearchSpace
from pathplanning.spaces.terrain_grid2d import TerrainCostGrid2D
from pathplanning.viz.replay import ReplayController


def _same_plan(first, second) -> None:
    assert (first.success, first.stop_reason, first.iters, first.nodes) == (
        second.success,
        second.stop_reason,
        second.iters,
        second.nodes,
    )
    assert (first.path is None) == (second.path is None)
    if first.path is not None:
        np.testing.assert_array_equal(first.path, second.path)
    assert first.stats.get("path_cost") == second.stats.get("path_cost")


@pytest.mark.parametrize("planner", list_planners("discrete"))
def test_search_trace_matches_production(planner: str) -> None:
    graph = TerrainCostGrid2D(np.ones((12, 12))) if planner == "jpsw" else Grid2DSearchSpace(12, 12)
    problem = DiscreteProblem(graph, (1, 1), (10, 10))
    params = {"max_expansions": 5_000}
    first = plan_discrete(problem, planner=planner, params=params, seed=13)
    second = plan_discrete(
        problem, planner=planner, params=params, seed=13, trace=TraceOptions(16_384)
    )

    _same_plan(first, second)
    assert first.trace is None
    assert second.trace is not None
    assert second.trace.kind == "discrete"
    assert second.trace.points is None
    assert second.trace.node_labels is not None
    assert second.trace.events.nbytes <= 16_384
    assert second.trace.graph_bytes > 0
    assert second.stats["trace_graph_bytes"] == second.trace.graph_bytes
    gc.collect()
    assert np.all(second.trace.events["kind"] > 0)


@pytest.mark.parametrize(
    "planner",
    [
        planner
        for planner in list_planners("continuous")
        if planner not in {"prm_star", "lazy_prm", "eirm_star", "hybrid_astar", "state_lattice"}
    ],
)
def test_sampling_trace_matches_production(planner: str) -> None:
    space = Grid2DSamplingSpace(x_range=(0.0, 10.0), y_range=(0.0, 10.0))
    problem = ContinuousProblem(
        space, np.array([1.0, 1.0]), GoalState(np.array([9.0, 9.0]), radius=0.0)
    )
    params = {
        "max_iters": 120,
        "sample_count": 96,
        "batch_size": 24,
        "step_size": 1.0,
        "goal_sample_rate": 0.2,
        "rrt_star_radius_gamma": 6.0,
    }
    if planner == "jit_star":
        params["manipulability_weight"] = 0.0
    first = plan_continuous(problem, planner=planner, params=params, seed=13)
    second = plan_continuous(
        problem, planner=planner, params=params, seed=13, trace=TraceOptions(32_768)
    )

    _same_plan(first, second)
    assert first.trace is None
    assert second.trace is not None
    assert second.trace.kind == "continuous"
    assert second.trace.points is not None
    assert second.trace.points.shape[1] == 2
    assert second.trace.events.nbytes + second.trace.points.nbytes <= 32_768
    gc.collect()
    assert np.all(np.isfinite(second.trace.points))


def test_small_trace_cap_truncates_without_changing_search() -> None:
    problem = DiscreteProblem(Grid2DSearchSpace(30, 30), (0, 0), (29, 29))
    first = plan_discrete(problem, planner="bfs")
    second = plan_discrete(problem, planner="bfs", trace=TraceOptions(32))
    _same_plan(first, second)
    assert second.trace is not None
    assert second.trace.truncated
    assert second.trace.events.nbytes <= 32


def test_small_trace_cap_truncates_continuous_points_and_map() -> None:
    space = Grid2DSamplingSpace(x_range=(0.0, 10.0), y_range=(0.0, 10.0))
    problem = ContinuousProblem(
        space, np.array([1.0, 1.0]), GoalState(np.array([9.0, 9.0]), radius=0.0)
    )
    params = {"max_iters": 120, "step_size": 1.0, "goal_sample_rate": 0.2}
    first = plan_continuous(problem, planner="rrt", params=params, seed=19)
    second = plan_continuous(
        problem,
        planner="rrt",
        params=params,
        seed=19,
        trace=TraceOptions(128),
    )

    _same_plan(first, second)
    assert second.trace is not None and second.trace.truncated
    point_bytes = 0 if second.trace.points is None else second.trace.points.nbytes
    payload_bytes = second.trace.events.nbytes + point_bytes
    assert payload_bytes <= 128


def test_production_planning_does_not_load_trace_libraries(monkeypatch) -> None:
    from pathplanning.native import _ffi

    monkeypatch.setattr(_ffi, "_SEARCH_TRACE_LIB", None)
    monkeypatch.setattr(_ffi, "_CONTINUOUS_TRACE_LIB", None)
    discrete = DiscreteProblem(Grid2DSearchSpace(5, 5), (0, 0), (4, 4))
    assert plan_discrete(discrete, planner="astar").success

    space = Grid2DSamplingSpace(x_range=(0.0, 10.0), y_range=(0.0, 10.0))
    continuous = ContinuousProblem(
        space, np.array([1.0, 1.0]), GoalState(np.array([9.0, 9.0]), radius=0.0)
    )
    result = plan_continuous(
        continuous,
        planner="rrt",
        params={"max_iters": 5, "step_size": 1.0},
        seed=3,
    )
    assert result.trace is None
    assert _ffi._SEARCH_TRACE_LIB is None
    assert _ffi._CONTINUOUS_TRACE_LIB is None


def test_missing_diagnostic_library_does_not_block_production(tmp_path, monkeypatch) -> None:
    from pathplanning.native import _ffi

    problem = DiscreteProblem(Grid2DSearchSpace(5, 5), (0, 0), (4, 4))
    assert plan_discrete(problem).success
    monkeypatch.setattr(_ffi, "_SEARCH_TRACE_LIB", None)
    monkeypatch.setattr(_ffi, "native_directory", lambda: tmp_path)

    with pytest.raises(NativeLibraryLoadError, match="make build-ext"):
        plan_discrete(problem, trace=TraceOptions())
    assert plan_discrete(problem).success


def test_dynamic_prune_trace_is_separate_session() -> None:
    space = ContinuousSpace3D(lower_bound=[0, 0, 0], upper_bound=[10, 10, 10])
    planner = DynamicRRT3D(
        environment=space,
        config=DynamicRRT3DConfig(max_iterations=0),
        start=[1, 1, 1],
        goal=[9, 9, 9],
    )
    planner.init_rrt()
    planner.add_node(planner.x0, (3.0, 1.0, 1.0))
    planner.add_node((3.0, 1.0, 1.0), (5.0, 1.0, 1.0))
    space.aabbs = (AABB([2.0, 0.0, 0.0], [4.0, 2.0, 2.0]),)

    trace = planner.trim_rrt(trace=TraceOptions())

    assert trace is planner.last_trace
    assert trace is not None
    assert 7 in trace.events["kind"]  # PP_TRACE_PRUNE
    assert len(planner.nodes) == 1
    replay = ReplayController(trace)
    replayed = replay.seek(replay.length)
    assert replayed.phase == 1
    assert not replayed.parents
    replay.seek(0)
    assert replay.position == 0


def test_dynamic_regrowth_trace_includes_pruning_and_regrowth_phases() -> None:
    space = ContinuousSpace3D(lower_bound=[0, 0, 0], upper_bound=[10, 10, 10])
    planner = DynamicRRT3D(
        environment=space,
        config=DynamicRRT3DConfig(max_iterations=2, step_size=0.5),
        start=[1, 1, 1],
        goal=[9, 9, 9],
        rng=np.random.default_rng(4),
    )
    planner.init_rrt()
    planner.add_node(planner.x0, (3.0, 1.0, 1.0))
    planner.add_node((3.0, 1.0, 1.0), (5.0, 1.0, 1.0))
    space.aabbs = (AABB([2.0, 0.0, 0.0], [4.0, 2.0, 2.0]),)

    trace = planner.regrow_rrt(trace=TraceOptions())

    assert trace is planner.last_trace
    assert trace is not None and not trace.truncated
    phases = trace.events[trace.events["kind"] == 5]["value"].tolist()
    assert phases == [0.0, 1.0, 2.0]
    assert 7 in trace.events["kind"]  # PP_TRACE_PRUNE
    replay = ReplayController(trace)
    assert replay.seek(replay.length).phase == 2
