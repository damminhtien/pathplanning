"""Native-model and callback-policy coverage for continuous planners."""

from __future__ import annotations

import numpy as np
import pytest

from pathplanning.api import plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.core.params import RrtParams
from pathplanning.native import NativeContinuousSpaceModel


class _DeclarativeLineSpace:
    distance_metric = "euclidean"

    def to_native_model(self) -> NativeContinuousSpaceModel:
        return NativeContinuousSpaceModel(lower_bounds=[0.0], upper_bounds=[1.0])

    def _callback_was_called(self) -> None:
        raise AssertionError("native model planner entered a Python space callback")

    sample_free = _callback_was_called
    is_state_valid = _callback_was_called
    is_motion_valid = _callback_was_called
    distance = _callback_was_called
    steer = _callback_was_called


class _CallbackLineSpace:
    def sample_free(self, rng: np.random.Generator) -> np.ndarray:
        return np.asarray([rng.random()], dtype=np.float64)

    def is_state_valid(self, _state: np.ndarray) -> bool:
        return True

    def is_motion_valid(self, _start: np.ndarray, _end: np.ndarray) -> bool:
        return True

    def distance(self, start: np.ndarray, end: np.ndarray) -> float:
        return float(np.linalg.norm(end - start))

    def steer(self, start: np.ndarray, target: np.ndarray, step_size: float) -> np.ndarray:
        delta = target - start
        length = float(np.linalg.norm(delta))
        return target if length <= step_size else start + delta * (step_size / length)


def test_declared_native_space_runs_without_python_callbacks() -> None:
    space = _DeclarativeLineSpace()
    problem = ContinuousProblem(
        space=space,
        start=np.asarray([0.0]),
        goal=GoalState(np.asarray([1.0]), radius=0.25, distance_fn=space.distance),
    )

    result = plan_continuous(
        problem,
        planner="rrt",
        params=RrtParams(max_iters=8, step_size=0.25, goal_sample_rate=1.0),
        seed=3,
    )

    assert result.success
    assert result.path is not None
    assert np.allclose(result.path[0], [0.0])
    assert np.linalg.norm(result.path[-1] - [1.0]) <= 0.25
    assert result.stats["python_callbacks"] == 0.0
    assert result.stats["native_space_model"] == 1.0


def test_custom_python_space_requires_explicit_callback_opt_in() -> None:
    problem = ContinuousProblem(
        space=_CallbackLineSpace(),
        start=np.asarray([0.0]),
        goal=GoalState(np.asarray([1.0])),
    )

    with pytest.raises(ValueError, match="Python callbacks are disabled"):
        plan_continuous(problem, planner="rrt", seed=3)

    result = plan_continuous(
        problem,
        planner="rrt",
        params=RrtParams(
            max_iters=8,
            step_size=0.25,
            goal_sample_rate=1.0,
            allow_python_callbacks=True,
        ),
        seed=3,
    )

    assert result.success
    assert result.stats["python_callbacks"] == 1.0
    assert result.stats["native_space_model"] == 0.0


def test_native_space_model_rejects_mismatched_obstacle_arrays() -> None:
    class InvalidNativeSpace(_DeclarativeLineSpace):
        def to_native_model(self) -> NativeContinuousSpaceModel:
            return NativeContinuousSpaceModel(
                lower_bounds=[0.0],
                upper_bounds=[1.0],
                sphere_centers=[[0.5]],
                sphere_radii=[],
            )

    problem = ContinuousProblem(
        space=InvalidNativeSpace(),
        start=np.asarray([0.0]),
        goal=GoalState(np.asarray([1.0])),
    )

    with pytest.raises(ValueError, match="sphere_radii must match centers"):
        plan_continuous(problem, planner="rrt", seed=3)


@pytest.mark.parametrize(
    ("start", "goal", "obstacle_kwargs"),
    [
        (
            [0.0, 0.0],
            [2.0, 0.0],
            {"sphere_centers": [[1.0, 0.0]], "sphere_radii": [0.2]},
        ),
        (
            [0.0, 0.0],
            [2.0, 0.0],
            {"box_minima": [[0.9, -0.2]], "box_maxima": [[1.1, 0.2]]},
        ),
        (
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            {
                "obb_centers": [[1.0, 0.0, 0.0]],
                "obb_extents": [[0.1, 0.2, 0.2]],
                "obb_orientations": [np.eye(3).ravel().tolist()],
            },
        ),
    ],
)
def test_native_motion_check_catches_obstacles_between_sparse_samples(
    start: list[float], goal: list[float], obstacle_kwargs: dict[str, object]
) -> None:
    class ObstacleSpace:
        distance_metric = "euclidean"

        def to_native_model(self) -> NativeContinuousSpaceModel:
            dimension = len(start)
            return NativeContinuousSpaceModel(
                lower_bounds=[-1.0] * dimension,
                upper_bounds=[3.0] * dimension,
                **obstacle_kwargs,
            )

    space = ObstacleSpace()
    result = plan_continuous(
        ContinuousProblem(space, np.asarray(start), GoalState(np.asarray(goal))),
        planner="rrt_connect",
        params=RrtParams(
            max_iters=1,
            step_size=3.0,
            collision_step=3.0,
            goal_sample_rate=1.0,
        ),
        seed=3,
    )

    assert not result.success
