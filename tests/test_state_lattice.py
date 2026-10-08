from __future__ import annotations

import numpy as np
import pytest

from pathplanning import StateLatticeParams, TraceOptions, plan_continuous
from pathplanning.core.contracts import (
    ContinuousProblem,
    GoalState,
    StateLatticePrimitive,
)
from pathplanning.core.results import StopReason
from pathplanning.spaces import (
    StateLatticeGridSpace,
    ackermann_motion_primitives,
    differential_drive_motion_primitives,
)


def _problem(space, start, goal):
    return ContinuousProblem(space, start, GoalState(goal))


def test_ackermann_lattice_finds_a_forward_path_and_handles_start_at_goal() -> None:
    space = StateLatticeGridSpace(
        np.zeros((12, 12), dtype=bool),
        resolution=1.0,
        primitives=ackermann_motion_primitives(
            1.0, 0.5, 1.0, collision_step=0.2, allow_reverse=True
        ),
        footprint_length=0.6,
        footprint_width=0.4,
        rotation_radius=1.0 / np.tan(0.5),
        collision_step=0.2,
    )
    result = plan_continuous(
        _problem(space, (2.5, 2.5, 0.0), (5.5, 2.5, 0.0)),
        planner="state_lattice",
        params=StateLatticeParams(goal_xy_tolerance=0.1, goal_yaw_tolerance=0.1),
    )

    assert result.success
    assert result.path is not None
    np.testing.assert_allclose(result.path[[0, -1]], [[2.5, 2.5, 0.0], [5.5, 2.5, 0.0]])
    assert set(result.directions[1:]) == {1}
    assert result.stats["path_cost"] == pytest.approx(3.0)

    already_there = plan_continuous(
        _problem(space, (2.5, 2.5, 0.0), (2.5, 2.5, 0.0)), planner="state_lattice"
    )
    assert already_there.success
    assert already_there.path is not None
    assert already_there.path.shape == (1, 3)


def test_differential_drive_lattice_supports_rotation_in_place() -> None:
    track_width = 0.6
    space = StateLatticeGridSpace(
        np.zeros((12, 12), dtype=bool),
        resolution=1.0,
        primitives=differential_drive_motion_primitives(track_width, 1.0, collision_step=0.1),
        footprint_length=0.6,
        footprint_width=0.4,
        rotation_radius=track_width / 2.0,
        collision_step=0.1,
    )
    result = plan_continuous(
        _problem(space, (5.5, 5.5, 0.0), (5.5, 5.5, np.pi / 6.0)),
        planner="state_lattice",
        params=StateLatticeParams(goal_xy_tolerance=0.05, goal_yaw_tolerance=0.02),
    )

    assert result.success
    assert result.path is not None
    np.testing.assert_allclose(result.path[-1], [5.5, 5.5, np.pi / 6.0], atol=1e-8)
    np.testing.assert_allclose(result.path[:, :2], np.full((len(result.path), 2), 5.5))
    assert result.stats["path_cost"] == pytest.approx(track_width * np.pi / 12.0)


def test_state_lattice_checks_each_primitive_through_occupied_cells() -> None:
    occupancy = np.zeros((3, 7), dtype=bool)
    occupancy[1, 3] = True
    primitives = (
        StateLatticePrimitive(((0.5, 0.0, 0.0), (1.0, 0.0, 0.0)), 1, 1.0),
        StateLatticePrimitive(((-0.5, 0.0, 0.0), (-1.0, 0.0, 0.0)), -1, 1.0),
    )
    space = StateLatticeGridSpace(
        occupancy,
        resolution=1.0,
        primitives=primitives,
        footprint_length=0.2,
        footprint_width=0.2,
        rotation_radius=0.1,
        collision_step=0.1,
    )
    result = plan_continuous(
        _problem(space, (1.5, 1.5, 0.0), (5.5, 1.5, 0.0)), planner="state_lattice"
    )

    assert not result.success
    assert result.path is None
    assert result.stop_reason is StopReason.NO_PROGRESS
    assert result.stats["motion_checks"] > 0


def test_state_lattice_callback_requires_opt_in_and_propagates_errors() -> None:
    primitive = StateLatticePrimitive(((0.5, 0.0, 0.0), (1.0, 0.0, 0.0)), 1, 1.0)

    class CallbackSpace:
        footprint_length = 0.4
        footprint_width = 0.3
        rotation_radius = 0.2
        motion_primitives = (primitive,)
        collision_step = 0.1

        def is_state_valid(self, state):
            return bool(np.all(np.isfinite(state)))

    problem = _problem(CallbackSpace(), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="allow_python_callbacks=True"):
        plan_continuous(problem, planner="state_lattice")

    result = plan_continuous(
        problem,
        planner="state_lattice",
        params=StateLatticeParams(allow_python_callbacks=True),
    )
    assert result.success
    assert result.stats["python_callbacks"] == 1.0

    class BrokenSpace(CallbackSpace):
        def is_state_valid(self, state):
            raise LookupError("custom validity failed")

    with pytest.raises(RuntimeError, match="state-validity callback failed") as error:
        plan_continuous(
            _problem(BrokenSpace(), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            planner="state_lattice",
            params=StateLatticeParams(allow_python_callbacks=True),
        )
    assert isinstance(error.value.__cause__, LookupError)


def test_state_lattice_trace_is_bounded_and_budget_is_reported() -> None:
    space = StateLatticeGridSpace(
        np.zeros((12, 12), dtype=bool),
        resolution=1.0,
        primitives=ackermann_motion_primitives(1.0, 0.5, 1.0, collision_step=0.2),
        footprint_length=0.6,
        footprint_width=0.4,
        rotation_radius=1.0 / np.tan(0.5),
        collision_step=0.2,
    )
    problem = _problem(space, (2.5, 2.5, 0.0), (4.5, 2.5, 0.0))
    traced = plan_continuous(problem, planner="state_lattice", trace=TraceOptions(max_bytes=32))
    assert traced.success
    assert traced.trace is not None and traced.trace.truncated
    assert traced.path is not None
    np.testing.assert_allclose(traced.path[-1], [4.5, 2.5, 0.0])

    bounded = plan_continuous(
        _problem(space, (2.5, 2.5, 0.0), (9.5, 2.5, 0.0)),
        planner="state_lattice",
        params=StateLatticeParams(max_expansions=1),
    )
    assert not bounded.success
    assert bounded.stop_reason is StopReason.MAX_ITERS
