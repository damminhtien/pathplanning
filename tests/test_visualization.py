"""Headless rendering and replay behavior for the optional viewer."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

matplotlib = pytest.importorskip("matplotlib")
matplotlib.use("Agg")

from pathplanning import TraceOptions, plan_continuous, plan_discrete
from pathplanning.core.contracts import ContinuousProblem, DiscreteProblem, GoalState
from pathplanning.core.results import PlanResult, StopReason
from pathplanning.spaces.grid2d import Grid2DSamplingSpace, Grid2DSearchSpace
from pathplanning.viz import Box, Scene, Sphere, render_result, scene_from_problem
from pathplanning.viz.replay import ReplayController
from pathplanning.viz.viewer import Viewer


def _traced_grid():
    graph = Grid2DSearchSpace(12, 12, obstacles={(5, 5), (5, 6)})
    problem = DiscreteProblem(graph, (1, 1), (10, 10))
    result = plan_discrete(problem, planner="bidirectional_astar", trace=TraceOptions())
    return problem, result


def test_scene_snapshot_and_independent_figure_export(tmp_path) -> None:
    problem, result = _traced_grid()
    scene = scene_from_problem(problem)
    first = render_result(scene, result)
    second = render_result(scene, result)
    assert first.figure is not second.figure
    assert len(first.ax.collections) < 25
    assert len(second.ax.collections) < 25
    first.save(tmp_path / "plan.png")
    second.save(tmp_path / "plan.svg")
    assert (tmp_path / "plan.png").stat().st_size > 0
    assert (tmp_path / "plan.svg").stat().st_size > 0

    problem.graph.update_obs({(4, 4)})
    assert len(scene.obstacles) == 2


def test_grid_occupancy_snapshot_combines_blocked_cells_once() -> None:
    occupancy = np.zeros((6, 8), dtype=bool)
    occupancy[4, 5] = True
    graph = Grid2DSearchSpace(
        width=8,
        height=6,
        obstacles={(2, 1)},
        occupancy=occupancy,
    )
    scene = scene_from_problem(DiscreteProblem(graph, (0, 0), (7, 5)))

    assert scene.occupancy is not None
    assert int(scene.occupancy.sum()) == 2
    assert scene.occupancy[1, 2]
    assert scene.occupancy[4, 5]
    assert scene.obstacles == ()
    renderer = render_result(
        scene,
        PlanResult(
            success=False,
            path=None,
            best_path=None,
            stop_reason=StopReason.NO_PROGRESS,
            iters=0,
            nodes=0,
        ),
    )
    assert len(renderer.ax.images) == 1
    graph._occupancy[4, 5] = False
    assert scene.occupancy[4, 5]


def test_replay_seek_backward_and_viewer_cleanup() -> None:
    problem, result = _traced_grid()
    assert result.trace is not None
    controller = ReplayController(result.trace, checkpoint_stride=4, max_cache_bytes=2_048)
    end = controller.seek(controller.length).copy()
    assert controller.cache_bytes <= 2_048
    controller.seek(0)
    assert controller.position == 0
    assert controller.seek(controller.length) == end

    viewer = Viewer(scene_from_problem(problem), result)
    viewer.seek(20)
    viewer.seek(5)
    assert viewer.position == 5
    viewer.play()
    viewer.pause()
    viewer._on_key(SimpleNamespace(key="escape"))
    assert viewer.closed
    assert not viewer._timer.callbacks
    assert not viewer._widget_callbacks


def test_3d_obstacles_and_camera_survive_replay(tmp_path) -> None:
    problem, result = _traced_grid()
    assert result.trace is not None
    scene = Scene(
        bounds=np.array([[0, 0, 0], [12, 12, 12]], dtype=float),
        start=np.array([1, 1, 1], dtype=float),
        goal=np.array([10, 10, 1], dtype=float),
        obstacles=(
            Box((4, 4, 4), (6, 6, 6)),
            Sphere((8, 8, 8), 1.0),
        ),
    )
    # The discrete trace is 2D, so use a 3D coordinate map for its grid labels.
    positions = {cell: (cell[0], cell[1], 1.0) for cell in result.trace.node_labels}
    viewer = Viewer(scene, result, positions=positions)
    viewer.renderer.ax.view_init(elev=37, azim=51)
    viewer.seek(25)
    viewer.seek(4)
    assert viewer.renderer.ax.elev == 37
    assert viewer.renderer.ax.azim == 51
    assert len(viewer.renderer.ax.collections) < 25
    viewer.renderer.save(tmp_path / "plan.pdf")
    assert (tmp_path / "plan.pdf").stat().st_size > 0
    viewer.close()


def test_anytime_and_rrt_connect_traces_replay() -> None:
    discrete = DiscreteProblem(Grid2DSearchSpace(10, 10), (1, 1), (8, 8))
    anytime = plan_discrete(
        discrete,
        planner="anytime_astar",
        params={"anytime_weights": (3.0, 2.0, 1.0)},
        seed=5,
        trace=TraceOptions(),
    )
    assert anytime.trace is not None
    assert np.any(anytime.trace.events["kind"] == 5)  # PP_TRACE_PHASE
    anytime_viewer = Viewer(scene_from_problem(discrete), anytime)
    anytime_viewer.seek(anytime_viewer.controller.length)
    anytime_viewer.seek(0)
    anytime_viewer.close()

    space = Grid2DSamplingSpace(x_range=(0.0, 10.0), y_range=(0.0, 10.0))
    continuous = ContinuousProblem(
        space,
        np.array([1.0, 1.0]),
        GoalState(np.array([9.0, 9.0]), radius=0.0),
    )
    connected = plan_continuous(
        continuous,
        planner="rrt_connect",
        params={"max_iters": 120, "step_size": 1.0, "goal_sample_rate": 0.2},
        seed=5,
        trace=TraceOptions(),
    )
    assert connected.trace is not None
    assert connected.trace.points is not None
    assert len(connected.trace.events) > 0
    connect_viewer = Viewer(scene_from_problem(continuous), connected)
    connect_viewer.seek(connect_viewer.controller.length)
    connect_viewer.seek(0)
    connect_viewer.close()


def test_truncated_trace_replays_prefix_and_keeps_final_path_visible() -> None:
    problem = DiscreteProblem(Grid2DSearchSpace(30, 30), (0, 0), (29, 29))
    result = plan_discrete(problem, planner="bfs", trace=TraceOptions(32))
    assert result.trace is not None and result.trace.truncated
    viewer = Viewer(scene_from_problem(problem), result)
    viewer.seek(viewer.controller.length)
    assert viewer.position == len(result.trace.events)
    assert viewer.renderer.path_drawn
    assert "truncated" in viewer._status.get_text()
    viewer.close()


def test_failed_result_renders_without_a_path() -> None:
    result = PlanResult(
        success=False,
        path=None,
        best_path=None,
        stop_reason=StopReason.NO_PROGRESS,
        iters=0,
        nodes=0,
    )
    renderer = render_result(
        Scene(
            bounds=np.array([[0.0, 0.0], [1.0, 1.0]]),
            start=np.array([0.0, 0.0]),
            goal=np.array([1.0, 1.0]),
        ),
        result,
    )
    assert not renderer.path_drawn
