"""Informed RRT*: RRT* with direct prolate-hyperspheroid sampling."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import math

import numpy as np

from pathplanning.core.contracts import ContinuousProblem, ContinuousSpace, GoalRegion, State
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.core.types import RNG
from pathplanning.data_structures.tree_array import ArrayTree
from pathplanning.nn.index import NearestNeighborIndex
from pathplanning.planners.sampling._internal.continuous import (
    as_state,
    euclidean_distance,
    exact_goal_state,
    sample_informed,
    validate_objective,
)
from pathplanning.planners.sampling._internal.problem_adapter import (
    coerce_rrt_params,
    resolve_rng,
)
from pathplanning.planners.sampling.rrt_star import IndexFactory, RrtStarPlanner


class InformedRrtStar(RrtStarPlanner):
    """RRT* with direct sampling from the current path-length informed set."""

    def __init__(
        self,
        space: ContinuousSpace[State],
        params: RrtParams,
        rng: np.random.Generator,
        nn_index_factory: IndexFactory | None = None,
    ) -> None:
        super().__init__(space, params, rng, nn_index_factory, objective=None)
        self._informed_start: State | None = None
        self._informed_goal: State | None = None
        self._informed_best_cost = math.inf

    def _sample_free(self, *, dim: int) -> State:
        if self._informed_best_cost == math.inf:
            return super()._sample_free(dim=dim)
        if self._informed_start is None or self._informed_goal is None:
            return super()._sample_free(dim=dim)
        return sample_informed(
            self.space,
            self.rng,
            self.params,
            self._informed_start,
            self._informed_goal,
            self._informed_best_cost,
        )

    def _after_iteration(self, tree: ArrayTree, goal_indices: list[int]) -> None:
        if goal_indices:
            self._informed_best_cost = min(float(tree.cost[index]) for index in goal_indices)

    def _cost_from_parent(self, tree: ArrayTree, parent_index: int, node: State) -> float:
        parent = tree.node(parent_index)
        return float(tree.cost[parent_index] + euclidean_distance(self.space, parent, node))

    def plan(self, start: Sequence[float] | State, goal_region: GoalRegion[State]) -> PlanResult:
        start_array = np.asarray(start, dtype=float)
        if start_array.ndim != 1:
            raise ValueError(f"start must be a 1D state vector, got {start_array.shape}")
        self._informed_start = as_state(start_array, "start", dim=int(start_array.size))
        self._informed_goal = exact_goal_state(goal_region, dim=int(start_array.size))
        euclidean_distance(self.space, self._informed_start, self._informed_goal)
        self._informed_best_cost = math.inf
        return super().plan(self._informed_start, goal_region)


def plan_informed_rrt_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Plan with Informed RRT* for additive Euclidean path length."""
    validate_objective(problem.objective, "Informed RRT*")
    resolved_params = coerce_rrt_params(problem, params)
    planner = InformedRrtStar(problem.space, resolved_params, resolve_rng(rng))
    return planner.plan(problem.start, problem.goal)


__all__ = ["InformedRrtStar", "NearestNeighborIndex", "plan_informed_rrt_star"]
