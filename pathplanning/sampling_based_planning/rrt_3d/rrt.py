"""Headless, environment-agnostic 3D RRT planner."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import time
from typing import TypeAlias

import numpy as np

from pathplanning.core.contracts import ConfigurationSpace, PlanResult, State, StopReason
from pathplanning.core.nn_index import NaiveIndex, NearestNeighborIndex
from pathplanning.core.tree import Tree
from pathplanning.core.params import RrtParams

GoalPredicate: TypeAlias = Callable[[State], bool]
GoalRegion: TypeAlias = GoalPredicate | tuple[Sequence[float], float] | Sequence[float] | State
IndexFactory: TypeAlias = Callable[[int], NearestNeighborIndex]


def _as_state(value: Sequence[float] | State, name: str, *, dim: int) -> State:
    """Normalize a coordinate-like value to a 1D float state vector.

    Args:
        value: Input coordinate sequence.
        name: Human-readable argument name for error messages.
        dim: Expected state dimension.

    Returns:
        Normalized NumPy state vector with shape ``(dim,)``.

    Raises:
        ValueError: If the input does not match the expected dimension.
    """
    state = np.asarray(value, dtype=float)
    if state.shape != (dim,):
        raise ValueError(f"{name} must be shape ({dim},), got {state.shape}")
    return state


@dataclass(slots=True)
class GoalSpec:
    """Parsed goal definition for sampling and termination checks."""

    predicate: GoalPredicate
    target_state: State | None


class RrtPlanner:
    """Single-query RRT planner operating on ``ConfigurationSpace`` contracts."""

    def __init__(
        self,
        space: ConfigurationSpace,
        params: RrtParams,
        rng: np.random.Generator,
        nn_index_factory: IndexFactory | None = None,
    ) -> None:
        """Initialize a contract-based RRT planner.

        Args:
            space: Configuration space implementation.
            params: Planner parameters.
            rng: Random number generator used for sampling.
        """
        self.space = space
        self.params = params.validate()
        self.rng = rng
        self._nn_index_factory = nn_index_factory or (lambda dim: NaiveIndex(dim=dim))

    def _goal_spec(self, goal_region: GoalRegion) -> GoalSpec:
        """Build an internal goal specification from supported goal inputs."""
        if callable(goal_region):
            target = getattr(self.space, "goal", None)
            target_state: State | None = None
            if target is not None:
                target_state = _as_state(target, "space.goal", dim=self.space.dim)
            return GoalSpec(predicate=goal_region, target_state=target_state)

        if (
            isinstance(goal_region, tuple)
            and len(goal_region) == 2
            and not isinstance(goal_region[1], (Sequence, np.ndarray))
        ):
            center = _as_state(goal_region[0], "goal_center", dim=self.space.dim)
            tolerance = float(goal_region[1])
            if tolerance < 0.0:
                raise ValueError("goal tolerance must be >= 0")
            return GoalSpec(
                predicate=lambda state: self.space.distance(state, center) <= tolerance,
                target_state=center,
            )

        goal_state = _as_state(goal_region, "goal_state", dim=self.space.dim)
        return GoalSpec(
            predicate=lambda state: self.space.distance(state, goal_state)
            <= self.params.goal_reach_tolerance,
            target_state=goal_state,
        )

    def _sample_free(self) -> State:
        """Sample one collision-free state within configured bounds.

        Raises:
            RuntimeError: If no free sample is found within max retries.
        """
        lower, upper = self.space.bounds
        lower_state = _as_state(lower, "space.bounds[0]", dim=self.space.dim)
        upper_state = _as_state(upper, "space.bounds[1]", dim=self.space.dim)
        for _ in range(self.params.max_sample_tries):
            sample = self.rng.uniform(lower_state, upper_state)
            if self.space.is_free(sample):
                return sample
        raise RuntimeError(
            "Failed to sample a free state within max_sample_tries. "
            "Adjust bounds/obstacles or increase max_sample_tries."
        )

    @staticmethod
    def _nearest_index(index: NearestNeighborIndex, target: State) -> int:
        """Return the index of the nearest node to ``target``."""
        return index.nearest(target)

    @staticmethod
    def _path_from_tree(tree: Tree, node_id: int) -> list[State]:
        """Return root-to-node path as a list of states."""
        path = tree.extract_path(node_id)
        return [np.asarray(state, dtype=float) for state in path]

    def plan(self, start: Sequence[float] | State, goal_region: GoalRegion) -> PlanResult:
        """Compute a collision-free path from ``start`` into ``goal_region``."""
        self.params.validate()
        start_state = _as_state(start, "start", dim=self.space.dim)
        if not self.space.is_free(start_state):
            raise ValueError("start must be collision free")

        goal = self._goal_spec(goal_region)
        if goal.predicate(start_state):
            return PlanResult(
                success=True,
                path=[np.asarray(start_state, dtype=float)],
                iters=0,
                nodes=1,
                stats={"reason": StopReason.START_IN_GOAL.value, "goal_checks": 1},
            )

        tree = Tree(self.space.dim)
        root_id = tree.append_node(start_state, -1, 0.0)
        goal_checks = 0
        index = self._nn_index_factory(self.space.dim)
        index.add(tree.nodes[root_id])

        start_time = time.perf_counter()
        iters_run = 0
        stop_reason: StopReason | None = None
        for iters_run in range(1, self.params.max_iters + 1):
            if self.params.time_budget_s is not None:
                elapsed = time.perf_counter() - start_time
                if elapsed >= self.params.time_budget_s:
                    stop_reason = StopReason.TIME_BUDGET
                    break

            use_goal_sample = (
                goal.target_state is not None and self.rng.random() < self.params.goal_sample_rate
            )
            target = goal.target_state if use_goal_sample else self._sample_free()

            nearest_index = self._nearest_index(index, target)
            nearest = tree.nodes[nearest_index]
            candidate = self.space.steer(nearest, target, self.params.step_size)

            if not self.space.is_free(candidate):
                continue
            if not self.space.segment_free(nearest, candidate, self.params.collision_step):
                continue

            new_cost = float(tree.cost[nearest_index] + self.space.distance(nearest, candidate))
            new_index = tree.append_node(candidate, nearest_index, new_cost)
            index.add(tree.nodes[new_index])

            goal_checks += 1
            if goal.predicate(tree.nodes[new_index]):
                path = self._path_from_tree(tree, new_index)
                elapsed = time.perf_counter() - start_time
                return PlanResult(
                    success=True,
                    path=path,
                    iters=iters_run,
                    nodes=tree.size,
                    stats={
                        "goal_checks": goal_checks,
                        "path_cost": float(tree.cost[new_index]),
                        "stopped_reason": StopReason.GOAL_REACHED.value,
                        "elapsed_s": elapsed,
                        "time_budget_s": self.params.time_budget_s,
                    },
                )

        if stop_reason is None:
            stop_reason = StopReason.MAX_ITERS
        elapsed = time.perf_counter() - start_time
        return PlanResult(
            success=False,
            path=[],
            iters=iters_run,
            nodes=tree.size,
            stats={
                "goal_checks": goal_checks,
                "stopped_reason": stop_reason.value,
                "elapsed_s": elapsed,
                "time_budget_s": self.params.time_budget_s,
            },
        )
