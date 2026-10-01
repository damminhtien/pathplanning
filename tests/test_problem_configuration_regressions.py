"""Regression coverage for problem-level planner configuration."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
import pytest

from pathplanning.api import plan_continuous, plan_discrete
from pathplanning.core.contracts import ContinuousProblem, ContinuousSpace, DiscreteProblem
from pathplanning.core.params import RrtParams
from pathplanning.planners.sampling.rrt import RrtPlanner
from pathplanning.planners.search._internal.native import NativeSearchError


class _SequenceSpace:
    def __init__(self, samples: Sequence[Sequence[float]]) -> None:
        self._samples = iter(samples)
        self.steps: list[float] = []

    @property
    def collision_step(self) -> float:
        return 0.4

    def sample_free(self, _rng: np.random.Generator) -> np.ndarray:
        return np.asarray(next(self._samples), dtype=float)

    def is_state_valid(self, _state: Sequence[float]) -> bool:
        return True

    def is_motion_valid(self, _start: Sequence[float], _end: Sequence[float]) -> bool:
        raise AssertionError("planner should use the step-aware motion checker")

    def is_motion_valid_with_step(
        self,
        _start: Sequence[float],
        _end: Sequence[float],
        collision_step: float,
    ) -> bool:
        self.steps.append(collision_step)
        return True

    def is_motion_valid_batch(self, edges: list[tuple[np.ndarray, np.ndarray]]) -> list[bool]:
        raise AssertionError("planner should use the step-aware batch checker")

    def is_motion_valid_batch_with_step(
        self,
        edges: Sequence[tuple[np.ndarray, np.ndarray]],
        collision_step: float,
    ) -> list[bool]:
        self.steps.extend([collision_step] * len(edges))
        return [True] * len(edges)

    def distance(self, a: Sequence[float], b: Sequence[float]) -> float:
        return float(np.linalg.norm(np.asarray(b) - np.asarray(a)))

    def steer(
        self,
        a: Sequence[float],
        b: Sequence[float],
        step_size: float,
    ) -> np.ndarray:
        start = np.asarray(a, dtype=float)
        target = np.asarray(b, dtype=float)
        direction = target - start
        distance = float(np.linalg.norm(direction))
        if distance <= step_size:
            return target
        return start + direction * (step_size / distance)


class _RightHalfPlaneGoal:
    def contains(self, state: Sequence[float]) -> bool:
        return float(state[0]) >= 1.5


class _WaypointRewardObjective:
    def path_cost(
        self,
        path: Sequence[np.ndarray],
        space: ContinuousSpace[np.ndarray],
    ) -> float:
        path_length = sum(space.distance(a, b) for a, b in zip(path[:-1], path[1:], strict=True))
        uses_waypoint = len(path) > 2 and np.allclose(path[-2], [2.0, 0.0])
        return float(path_length - (10.0 if uses_waypoint else 0.0))


@pytest.mark.parametrize(
    "planner_name",
    ["rrt_star", "informed_rrt_star", "bit_star", "abit_star"],
)
def test_rrt_star_family_uses_problem_objective_and_problem_params(planner_name: str) -> None:
    space = _SequenceSpace([[2.0, 0.0], [2.0, 2.0]])
    problem = ContinuousProblem(
        space=space,
        start=np.asarray([0.0, 0.0]),
        goal=_RightHalfPlaneGoal(),
        objective=_WaypointRewardObjective(),
        params={
            "max_iters": 1,
            "step_size": 10.0,
            "goal_sample_rate": 0.0,
            "collision_step": 0.05,
        },
    )

    result = plan_continuous(problem, planner=planner_name, params={"max_iters": 2}, seed=5)

    assert result.success
    assert result.path is not None
    assert np.allclose(result.path[-1], [2.0, 2.0])
    assert any(np.allclose(state, [2.0, 0.0]) for state in result.path[1:-1])
    assert result.iters == 2
    assert result.stats["path_cost"] == pytest.approx(-6.0)
    assert result.stats["objective_cost"] == result.stats["path_cost"]
    assert space.collision_step == 0.4
    assert space.steps
    assert set(space.steps) == {0.05}


def test_rrt_does_not_mutate_read_only_space_collision_step() -> None:
    space = _SequenceSpace([[2.0, 0.0]])
    planner = RrtPlanner(
        space,
        RrtParams(max_iters=1, step_size=10.0, collision_step=0.05),
        np.random.default_rng(2),
    )

    planner.plan([0.0, 0.0], _RightHalfPlaneGoal())

    assert space.collision_step == 0.4
    assert space.steps
    assert set(space.steps) == {0.05}


class _TwoNodeGraph:
    def neighbors(self, _node: int) -> tuple[int, ...]:
        return (1,) if _node == 0 else ()

    def edge_cost(self, _start: int, _end: int) -> float:
        return 1.0


def test_discrete_problem_params_apply_and_call_params_override() -> None:
    problem = DiscreteProblem(
        graph=_TwoNodeGraph(),
        start=0,
        goal=1,
        params={"max_materialized_nodes": 1},
    )

    with pytest.raises(NativeSearchError, match="max_materialized_nodes"):
        plan_discrete(problem)

    result = plan_discrete(problem, params={"max_materialized_nodes": 2})

    assert result.success
    assert problem.params == {"max_materialized_nodes": 1}
