"""Headless, environment-agnostic 3D RRT planner."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from numbers import Real
import time
from typing import TypeAlias, cast

import numpy as np

from pathplanning.core.contracts import (
    BatchConfigurationSpace,
    ConfigurationSpace,
    PlanResult,
    State,
)
from pathplanning.core.nn_index import NaiveNnIndex, NearestNeighborIndex
from pathplanning.core.params import RrtParams
from pathplanning.core.tree import ArrayTree

GoalPredicate: TypeAlias = Callable[[State], bool]
GoalRegion: TypeAlias = GoalPredicate | tuple[Sequence[float], float] | Sequence[float] | State
IndexFactory: TypeAlias = Callable[[int], NearestNeighborIndex]


def _default_index_factory(dim: int) -> NearestNeighborIndex:
    return NaiveNnIndex(dim=dim)


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
        self._nn_index_factory = nn_index_factory or _default_index_factory
        self._batch_space: BatchConfigurationSpace | None = self._resolve_batch_space(space)
        if hasattr(self.space, "collision_step"):
            self.space.collision_step = self.params.collision_step  # type: ignore[attr-defined]

    @staticmethod
    def _resolve_batch_space(space: ConfigurationSpace) -> BatchConfigurationSpace | None:
        if (
            hasattr(space, "sample_free_batch")
            and hasattr(space, "is_free_batch")
            and hasattr(space, "segment_free_batch")
        ):
            return cast(BatchConfigurationSpace, space)
        return None

    def _goal_spec(self, goal_region: GoalRegion) -> GoalSpec:
        """Build an internal goal specification from supported goal inputs."""
        if callable(goal_region):
            target = getattr(self.space, "goal", None)
            target_state: State | None = None
            if target is not None:
                target_array = np.asarray(target, dtype=float)
                if target_array.shape == (self.space.dim,):
                    target_state = target_array
            return GoalSpec(predicate=goal_region, target_state=target_state)

        if isinstance(goal_region, tuple):
            if len(goal_region) != 2:
                raise ValueError("goal tuple must be (center, tolerance)")
            center_raw, tolerance_raw = goal_region
            if not isinstance(tolerance_raw, Real):
                raise ValueError("goal tuple must be (center, tolerance)")
            center = _as_state(
                cast(Sequence[float] | State, center_raw),
                "goal_center",
                dim=self.space.dim,
            )
            tolerance = float(tolerance_raw)
            if tolerance < 0.0:
                raise ValueError("goal tolerance must be >= 0")
            return GoalSpec(
                predicate=lambda state: self.space.distance(state, center) <= tolerance,
                target_state=center,
            )

        goal_state = _as_state(goal_region, "goal_state", dim=self.space.dim)
        return GoalSpec(
            predicate=lambda state: (
                self.space.distance(state, goal_state) <= self.params.goal_reach_tolerance
            ),
            target_state=goal_state,
        )

    def _sample_free(self) -> State:
        """Sample one collision-free state within configured bounds.

        Raises:
            RuntimeError: If no free sample is found within max retries.
        """
        if self._batch_space is not None:
            sampled = self._batch_space.sample_free_batch(self.rng, 1)[0]
            return _as_state(sampled, "sampled_state", dim=self.space.dim)
        sampled = self.space.sample_free(self.rng)
        return _as_state(sampled, "sampled_state", dim=self.space.dim)

    @staticmethod
    def _nearest_index(index: NearestNeighborIndex, target: State) -> int:
        """Return the index of the nearest node to ``target``."""
        return index.nearest(target)

    @staticmethod
    def _path_from_tree(tree: ArrayTree, node_id: int) -> list[State]:
        """Return root-to-node path as a list of states."""
        path = tree.extract_path(node_id)
        return [np.asarray(state, dtype=float) for state in path]

    def _is_free(self, state: State) -> bool:
        if self._batch_space is not None:
            points = np.asarray([state], dtype=float)
            return bool(self._batch_space.is_free_batch(points)[0])
        return self.space.is_free(state)

    def _segment_free(self, start: State, end: State) -> bool:
        if self._batch_space is not None:
            starts = np.asarray([start], dtype=float)
            ends = np.asarray([end], dtype=float)
            return bool(self._batch_space.segment_free_batch(starts, ends)[0])
        return self.space.segment_free(start, end)

    def plan(self, start: Sequence[float] | State, goal_region: GoalRegion) -> PlanResult:
        """Compute a collision-free path from ``start`` into ``goal_region``."""
        self.params.validate()
        start_state = _as_state(start, "start", dim=self.space.dim)
        if not self._is_free(start_state):
            raise ValueError("start must be collision free")

        goal = self._goal_spec(goal_region)
        if goal.predicate(start_state):
            return PlanResult(
                success=True,
                path=[np.asarray(start_state, dtype=float)],
                iters=0,
                nodes=1,
                stats={"reason": "start_in_goal", "goal_checks": 1},
            )

        tree = ArrayTree(self.space.dim)
        tree.add_node(start_state, -1, 0.0)
        goal_checks = 0
        index = self._nn_index_factory(self.space.dim)
        index.build(tree.nodes)

        start_time = time.perf_counter()
        iters_run = 0
        stop_reason: str | None = None
        for iters_run in range(1, self.params.max_iters + 1):
            if self.params.time_budget_s is not None:
                elapsed = time.perf_counter() - start_time
                if elapsed >= self.params.time_budget_s:
                    stop_reason = "time_budget"
                    break

            use_goal_sample = (
                goal.target_state is not None and self.rng.random() < self.params.goal_sample_rate
            )
            if use_goal_sample:
                target = goal.target_state
                if target is None:
                    continue
            else:
                target = self._sample_free()

            nearest_index = self._nearest_index(index, target)
            nearest = tree.node(nearest_index)
            candidate = self.space.steer(nearest, target, self.params.step_size)

            if not self._is_free(candidate):
                continue
            if not self._segment_free(nearest, candidate):
                continue

            new_cost = float(tree.cost[nearest_index] + self.space.distance(nearest, candidate))
            new_index = tree.add_node(candidate, nearest_index, new_cost)
            index.build(tree.nodes)

            goal_checks += 1
            if goal.predicate(tree.node(new_index)):
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
                        "stopped_reason": "goal_reached",
                        "elapsed_s": elapsed,
                        "time_budget_s": self.params.time_budget_s,
                    },
                )

        if stop_reason is None:
            stop_reason = "max_iters"
        elapsed = time.perf_counter() - start_time
        return PlanResult(
            success=False,
            path=[],
            iters=iters_run,
            nodes=tree.size,
            stats={
                "goal_checks": goal_checks,
                "stopped_reason": stop_reason,
                "elapsed_s": elapsed,
                "time_budget_s": self.params.time_budget_s,
            },
        )
