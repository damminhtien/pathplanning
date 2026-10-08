from __future__ import annotations

from typing import Any

import numpy as np
import pytest

from pathplanning import HybridAStarParams, TraceOptions, plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.core.results import StopReason
from pathplanning.spaces import AckermannGridSpace


def _space(occupancy: np.ndarray | None = None) -> AckermannGridSpace:
    return AckermannGridSpace(
        np.zeros((20, 20), dtype=bool) if occupancy is None else occupancy,
        resolution=1.0,
        footprint_length=0.7,
        footprint_width=0.4,
    )


def _problem(
    space: Any,
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
) -> ContinuousProblem[tuple[float, float, float]]:
    return ContinuousProblem(space, start, GoalState(goal))


def test_hybrid_astar_returns_forward_analytic_path_and_handles_start_at_goal() -> None:
    space = _space()
    result = plan_continuous(
        _problem(space, (2.5, 2.5, 0.0), (5.5, 2.5, 0.0)), planner="hybrid_astar"
    )

    assert result.success
    assert result.path is not None
    np.testing.assert_allclose(result.path[[0, -1]], [[2.5, 2.5, 0.0], [5.5, 2.5, 0.0]])
    assert result.directions[0] == 0
    assert set(result.directions[1:]) == {1}
    assert result.stats["path_cost"] == pytest.approx(3.0)

    already_there = plan_continuous(
        _problem(space, (2.5, 2.5, 0.0), (2.5, 2.5, 0.0)), planner="hybrid_astar"
    )
    assert already_there.success
    assert already_there.path is not None
    assert already_there.path.shape == (1, 3)
    assert already_there.directions == (0,)


def test_hybrid_astar_uses_reverse_reeds_shepp_motion() -> None:
    space = _space()
    reverse_step = space.steer((5.5, 5.5, 0.0), (3.5, 5.5, 0.0), 0.5)
    assert reverse_step[0] < 5.5
    assert space.is_motion_valid((5.5, 5.5, 0.0), reverse_step)

    result = plan_continuous(
        _problem(space, (5.5, 5.5, 0.0), (3.5, 5.5, 0.0)), planner="hybrid_astar"
    )

    assert result.success
    assert result.path is not None
    assert result.directions[0] == 0
    assert set(result.directions[1:]) == {-1}
    assert result.stats["path_cost"] == pytest.approx(4.0)
    np.testing.assert_allclose(result.path[-1], [3.5, 5.5, 0.0], atol=1e-8)

    curved = plan_continuous(
        _problem(_space(), (5.5, 5.5, 0.0), (6.5, 7.5, 1.0)), planner="hybrid_astar"
    )
    assert curved.success
    assert curved.path is not None
    assert {-1, 1} <= set(curved.directions)
    np.testing.assert_allclose(curved.path[-1], [6.5, 7.5, 1.0], atol=1e-8)


def test_hybrid_astar_checks_the_full_footprint_around_an_obstacle() -> None:
    occupancy = np.zeros((20, 20), dtype=bool)
    occupancy[10, 10] = True
    space = _space(occupancy)
    result = plan_continuous(
        _problem(space, (3.5, 10.5, 0.0), (16.5, 10.5, 0.0)),
        planner="hybrid_astar",
        params=HybridAStarParams(max_expansions=30_000, primitive_length=1.0),
    )

    assert result.success
    assert result.path is not None
    assert all(space.is_state_valid(pose) for pose in result.path)
    assert np.max(np.abs(result.path[:, 1] - 10.5)) > 0.5
    assert set(result.directions[1:]) <= {-1, 1}
    distances = np.linalg.norm(np.diff(result.path[:, :2], axis=0), axis=1)
    yaw_changes = np.arctan2(np.sin(np.diff(result.path[:, 2])), np.cos(np.diff(result.path[:, 2])))
    moving = distances > 1e-8
    curvature = np.abs(yaw_changes[moving] / distances[moving])
    assert np.max(curvature) <= np.tan(space.max_steering_angle) / space.wheelbase * 1.01


def test_hybrid_astar_python_validity_callback_requires_opt_in() -> None:
    class CallbackSpace:
        wheelbase = 1.0
        max_steering_angle = 0.5
        footprint_length = 0.5
        footprint_width = 0.3
        collision_step = None

        def is_state_valid(self, state: np.ndarray) -> bool:
            return bool(np.all(np.isfinite(state)))

    problem = _problem(CallbackSpace(), (0.0, 0.0, 0.0), (2.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="allow_python_callbacks=True"):
        plan_continuous(problem, planner="hybrid_astar")

    result = plan_continuous(
        problem,
        planner="hybrid_astar",
        params=HybridAStarParams(allow_python_callbacks=True),
    )
    assert result.success
    assert result.stats["python_callbacks"] == 1.0

    class BrokenCallbackSpace(CallbackSpace):
        calls = 0

        def is_state_valid(self, state: np.ndarray) -> bool:
            self.calls += 1
            raise LookupError("custom validity failed")

    broken_space = BrokenCallbackSpace()
    with pytest.raises(RuntimeError, match="state-validity callback failed") as error:
        plan_continuous(
            _problem(broken_space, (0.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
            planner="hybrid_astar",
            params=HybridAStarParams(allow_python_callbacks=True),
        )
    assert isinstance(error.value.__cause__, LookupError)
    assert broken_space.calls == 1


def test_hybrid_astar_trace_is_bounded_and_keeps_solution() -> None:
    problem = _problem(_space(), (1.5, 1.5, 0.0), (3.5, 1.5, 0.0))
    result = plan_continuous(
        problem,
        planner="hybrid_astar",
        trace=TraceOptions(max_bytes=32),
    )

    assert result.success
    assert result.trace is not None
    assert result.path is not None
    assert result.trace.kind == "continuous"
    assert result.trace.truncated
    np.testing.assert_allclose(result.path[-1], [3.5, 1.5, 0.0], atol=1e-8)


def test_hybrid_astar_rejects_a_colliding_start_pose() -> None:
    occupancy = np.zeros((20, 20), dtype=bool)
    occupancy[2, 2] = True
    with pytest.raises(RuntimeError, match="start pose is invalid"):
        plan_continuous(
            _problem(_space(occupancy), (2.5, 2.5, 0.0), (5.5, 2.5, 0.0)),
            planner="hybrid_astar",
        )


def test_hybrid_astar_reports_an_exhausted_expansion_budget() -> None:
    result = plan_continuous(
        _problem(_space(), (2.5, 2.5, 0.0), (12.5, 2.5, 0.0)),
        planner="hybrid_astar",
        params=HybridAStarParams(
            max_expansions=1,
            analytic_expansion_distance=0.0,
        ),
    )

    assert not result.success
    assert result.path is None
    assert result.stop_reason is StopReason.MAX_ITERS
