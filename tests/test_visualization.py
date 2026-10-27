"""Headless rendering and replay behavior for the optional viewer."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

matplotlib = pytest.importorskip("matplotlib")
matplotlib.use("Agg")

from pathplanning import TraceOptions, plan_discrete
from pathplanning.core.contracts import DiscreteProblem
from pathplanning.spaces.grid2d import Grid2DSearchSpace
from pathplanning.viz import scene_from_problem
from pathplanning.viz.replay import ReplayController
from pathplanning.viz.viewer import Viewer


def _traced_grid():
    graph = Grid2DSearchSpace(12, 12, obstacles={(5, 5), (5, 6)})
    problem = DiscreteProblem(graph, (1, 1), (10, 10))
    result = plan_discrete(problem, planner="bidirectional_astar", trace=TraceOptions())
    return problem, result


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
