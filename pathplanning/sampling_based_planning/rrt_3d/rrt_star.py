"""Headless, environment-agnostic 3D RRT* planner."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from numbers import Real
import time
from typing import TypeAlias, cast

import numpy as np
from numpy.typing import NDArray

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
    """Parsed goal definition used internally by the planner."""

    predicate: GoalPredicate
    target_state: State | None


class RrtStarPlanner:
    """Single-query RRT* planner on top of ``ConfigurationSpace`` contracts."""

    def __init__(
        self,
        space: ConfigurationSpace,
        params: RrtParams,
        rng: np.random.Generator,
        nn_index_factory: IndexFactory | None = None,
    ) -> None:
        """Initialize a contract-based RRT* planner.

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
    def _near_indices(
        index: NearestNeighborIndex,
        target: State,
        radius: float,
    ) -> NDArray[np.int64]:
        """Return neighbor indices around ``target`` using a dynamic RRT* radius."""
        return index.radius(target, radius)

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

    @staticmethod
    def _propagate_cost_delta(
        root_index: int,
        delta: float,
        costs: NDArray[np.float64],
        children: list[set[int]],
    ) -> None:
        """Apply a cost delta to one subtree after rewiring."""
        stack = [root_index]
        while stack:
            index = stack.pop()
            costs[index] += delta
            stack.extend(children[index])

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
        children: list[set[int]] = [set()]
        index = self._nn_index_factory(self.space.dim)
        index.build(tree.nodes)

        goal_indices: list[int] = []
        goal_checks = 0
        start_time = time.perf_counter()
        iters_run = 0
        stop_reason: str | None = None

        for iter_index in range(1, self.params.max_iters + 1):
            iters_run = iter_index
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

            card_v = tree.size
            if card_v <= 1:
                near_indices = np.asarray([0], dtype=np.int64)
            else:
                dim = float(self.space.dim)
                dynamic_radius = self.params.step_size * (
                    self.params.rrt_star_radius_gamma * (np.log(card_v) / card_v) ** (1.0 / dim)
                    + self.params.rrt_star_radius_bias
                )
                near_radius = min(
                    self.params.step_size * self.params.rrt_star_radius_max_factor, dynamic_radius
                )
                near_indices = self._near_indices(index, candidate, near_radius)

            parent_index = nearest_index
            parent_cost = float(tree.cost[parent_index] + self.space.distance(nearest, candidate))
            for near_index_raw in near_indices:
                near_index = int(near_index_raw)
                near_node = tree.nodes[near_index]
                if not self._segment_free(near_node, candidate):
                    continue
                candidate_cost = float(
                    tree.cost[near_index] + self.space.distance(near_node, candidate)
                )
                if candidate_cost < parent_cost:
                    parent_index = near_index
                    parent_cost = candidate_cost

            new_index = tree.add_node(candidate, parent_index, parent_cost)
            children.append(set())
            children[parent_index].add(new_index)
            index.build(tree.nodes)

            for near_index_raw in near_indices:
                near_index = int(near_index_raw)
                if near_index in (parent_index, new_index):
                    continue
                near_node = tree.nodes[near_index]
                if not self._segment_free(candidate, near_node):
                    continue
                rewired_cost = float(
                    tree.cost[new_index] + self.space.distance(candidate, near_node)
                )
                if rewired_cost < tree.cost[near_index]:
                    old_parent = int(tree.parent[near_index])
                    if old_parent != -1:
                        children[old_parent].remove(near_index)
                    tree.parent[near_index] = new_index
                    children[new_index].add(near_index)
                    delta = float(rewired_cost - tree.cost[near_index])
                    self._propagate_cost_delta(near_index, delta, tree.cost, children)

            goal_checks += 1
            if goal.predicate(tree.node(new_index)):
                goal_indices.append(new_index)

        if goal_indices:
            best_goal_index = min(goal_indices, key=lambda idx: tree.cost[idx])
            path = self._path_from_tree(tree, best_goal_index)
            elapsed = time.perf_counter() - start_time
            if stop_reason is None:
                stop_reason = "goal_reached"
            return PlanResult(
                success=True,
                path=path,
                iters=iters_run,
                nodes=tree.size,
                stats={
                    "goal_checks": goal_checks,
                    "path_cost": float(tree.cost[best_goal_index]),
                    "goal_nodes": len(goal_indices),
                    "stopped_reason": stop_reason,
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
