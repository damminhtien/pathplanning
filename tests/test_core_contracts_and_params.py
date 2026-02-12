"""Core contract and parameter validation tests."""

from __future__ import annotations

from collections.abc import Iterable

import numpy as np
import pytest

from pathplanning.core.contracts import (
    BatchConfigurationSpace,
    ConfigurationSpace,
    ContinuousProblem,
    ContinuousSpace,
    ContinuousSpaceMetadata,
    DistanceAwareGoalRegion,
    DiscreteGraph,
    DiscreteProblem,
    ExactGoalTest,
    GoalRegion,
    GoalState,
    GoalTest,
    HeuristicDiscreteGraph,
    InterpolatingContinuousSpace,
    Objective,
    SupportsBatchMotionCheck,
    ValidatingDiscreteGraph,
)
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult, StopReason
from pathplanning.spaces.continuous_3d import ContinuousSpace3D
from pathplanning.spaces.grid2d import Grid2DSearchSpace


def test_core_contracts_importable() -> None:
    """Core contracts should be importable and expose typed result models."""
    assert DiscreteGraph is not None
    assert HeuristicDiscreteGraph is not None
    assert ValidatingDiscreteGraph is not None
    assert GoalTest is not None
    assert ExactGoalTest is not None
    assert DiscreteProblem is not None
    assert ContinuousSpace is not None
    assert GoalRegion is not None
    assert DistanceAwareGoalRegion is not None
    assert GoalState is not None
    assert Objective is not None
    assert SupportsBatchMotionCheck is not None
    assert ContinuousProblem is not None
    assert ConfigurationSpace is not None
    assert BatchConfigurationSpace is not None

    result = PlanResult(
        success=True,
        path=None,
        best_path=None,
        stop_reason=StopReason.SUCCESS,
        iters=0,
        nodes=0,
    )
    assert result.success is True
    assert result.path is None
    assert result.best_path is None
    assert result.stop_reason is StopReason.SUCCESS
    assert result.iters == 0
    assert result.nodes == 0
    assert result.stats == {}


def test_problem_wrappers_accept_minimal_implementations() -> None:
    """Discrete/continuous problem dataclasses should accept structural implementations."""

    class TinyGraph:
        def neighbors(self, n: int) -> Iterable[int]:
            return (n + 1,) if n < 3 else ()

        def edge_cost(self, a: int, b: int) -> float:
            _ = (a, b)
            return 1.0

    class RadiusGoal:
        def __init__(self, center: tuple[float, float], radius: float) -> None:
            self.center = center
            self.radius = radius

        def contains(self, x: tuple[float, float]) -> bool:
            return np.hypot(x[0] - self.center[0], x[1] - self.center[1]) <= self.radius

    class GoalEqualsThree:
        def is_goal(self, n: int) -> bool:
            return n == 3

    class TinyObjective:
        def path_cost(
            self,
            path: Iterable[tuple[float, float]],
            _space: ContinuousSpace[tuple[float, float]],
        ) -> float:
            points = list(path)
            if len(points) <= 1:
                return 0.0
            return float(
                sum(
                    np.hypot(x1 - x0, y1 - y0)
                    for (x0, y0), (x1, y1) in zip(points[:-1], points[1:], strict=True)
                )
            )

    class TinySpace:
        def sample_free(self, rng: np.random.Generator) -> tuple[float, float]:
            x = float(rng.uniform(0.0, 1.0))
            y = float(rng.uniform(0.0, 1.0))
            return (x, y)

        def is_state_valid(self, x: tuple[float, float]) -> bool:
            return 0.0 <= x[0] <= 1.0 and 0.0 <= x[1] <= 1.0

        def is_motion_valid(
            self, a: tuple[float, float], b: tuple[float, float]
        ) -> bool:
            return self.is_state_valid(a) and self.is_state_valid(b)

        def distance(self, a: tuple[float, float], b: tuple[float, float]) -> float:
            return float(np.hypot(a[0] - b[0], a[1] - b[1]))

        def steer(
            self, a: tuple[float, float], b: tuple[float, float], step_size: float
        ) -> tuple[float, float]:
            _ = step_size
            return b if self.is_motion_valid(a, b) else a

    discrete = DiscreteProblem[int](graph=TinyGraph(), start=0, goal=3, params={"eps": 1.0})
    assert discrete.start == 0
    assert discrete.goal == 3
    assert discrete.params is not None
    assert discrete.resolve_goal_test().is_goal(3)
    assert not discrete.resolve_goal_test().is_goal(2)

    predicate_problem = DiscreteProblem[int](graph=TinyGraph(), start=0, goal=GoalEqualsThree())
    assert predicate_problem.resolve_goal_test().is_goal(3)
    assert not predicate_problem.resolve_goal_test().is_goal(4)

    continuous_goal = GoalState(
        state=(1.0, 1.0),
        radius=0.25,
        distance_fn=lambda a, b: float(np.hypot(a[0] - b[0], a[1] - b[1])),
    )
    continuous = ContinuousProblem[tuple[float, float]](
        space=TinySpace(),
        start=(0.0, 0.0),
        goal=continuous_goal,
        objective=TinyObjective(),
        params={"step_size": 0.1},
    )
    assert continuous.start == (0.0, 0.0)
    assert continuous.goal == continuous_goal
    assert continuous.objective is not None
    assert continuous.goal.contains((1.1, 1.1))
    assert continuous.goal.distance_to_goal((1.0, 1.0)) == 0.0

    region_problem = ContinuousProblem[tuple[float, float]](
        space=TinySpace(),
        start=(0.0, 0.0),
        goal=RadiusGoal(center=(1.0, 1.0), radius=0.2),
        objective=None,
        params=None,
    )
    assert region_problem.params is None


def test_reference_spaces_conform_to_protocols() -> None:
    graph = Grid2DSearchSpace(width=20, height=20)
    assert isinstance(graph, DiscreteGraph)
    assert isinstance(graph, HeuristicDiscreteGraph)
    assert isinstance(graph, ValidatingDiscreteGraph)
    assert graph.is_valid_node((1, 1))
    assert graph.edge_cost((1, 1), (2, 2)) > 0.0

    space = ContinuousSpace3D(lower_bound=[0.0, 0.0, 0.0], upper_bound=[1.0, 1.0, 1.0])
    assert isinstance(space, ContinuousSpace)
    assert isinstance(space, ContinuousSpaceMetadata)
    assert isinstance(space, InterpolatingContinuousSpace)
    assert isinstance(space, SupportsBatchMotionCheck)
    assert space.is_state_valid(np.array([0.2, 0.2, 0.2], dtype=float))
    assert space.is_motion_valid(
        np.array([0.2, 0.2, 0.2], dtype=float), np.array([0.3, 0.3, 0.3], dtype=float)
    )
    assert space.is_motion_valid_batch(
        [
            (
                np.array([0.2, 0.2, 0.2], dtype=float),
                np.array([0.3, 0.3, 0.3], dtype=float),
            )
        ]
    ) == [True]


def test_rrt_params_validate_accepts_valid_values() -> None:
    """A valid parameter set should pass strict validation."""
    params = RrtParams(
        max_iters=500,
        step_size=0.25,
        goal_sample_rate=0.2,
        time_budget_s=2.0,
        max_sample_tries=20,
        collision_step=0.05,
        goal_reach_tolerance=1e-6,
        rrt_star_radius_gamma=1.5,
        rrt_star_radius_bias=0.5,
        rrt_star_radius_max_factor=3.0,
    )
    assert params.validate() is params


@pytest.mark.parametrize(
    ("kwargs", "error_type"),
    [
        ({"max_iters": 0}, ValueError),
        ({"max_iters": 1.5}, TypeError),
        ({"step_size": 0.0}, ValueError),
        ({"step_size": "bad"}, TypeError),
        ({"goal_sample_rate": -0.01}, ValueError),
        ({"goal_sample_rate": 1.01}, ValueError),
        ({"goal_sample_rate": "bad"}, TypeError),
        ({"time_budget_s": 0.0}, ValueError),
        ({"time_budget_s": "bad"}, TypeError),
        ({"max_sample_tries": 0}, ValueError),
        ({"max_sample_tries": 1.2}, TypeError),
        ({"collision_step": 0.0}, ValueError),
        ({"collision_step": "bad"}, TypeError),
        ({"goal_reach_tolerance": -0.1}, ValueError),
        ({"goal_reach_tolerance": "bad"}, TypeError),
        ({"rrt_star_radius_gamma": 0.0}, ValueError),
        ({"rrt_star_radius_gamma": "bad"}, TypeError),
        ({"rrt_star_radius_bias": -0.1}, ValueError),
        ({"rrt_star_radius_bias": "bad"}, TypeError),
        ({"rrt_star_radius_max_factor": 0.0}, ValueError),
        ({"rrt_star_radius_max_factor": "bad"}, TypeError),
    ],
)
def test_rrt_params_validate_rejects_invalid_values(
    kwargs: dict[str, object], error_type: type[Exception]
) -> None:
    """Invalid parameter values should fail fast with strict checks."""
    with pytest.raises(error_type):
        RrtParams(**kwargs)
