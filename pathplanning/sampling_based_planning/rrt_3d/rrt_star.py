"""Headless, environment-agnostic 3D RRT* planner."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import time
from typing import TypeAlias

import numpy as np

from pathplanning.core.contracts import ConfigurationSpace, PlanResult, State
from pathplanning.core.nn_index import NaiveIndex, NearestNeighborIndex
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
            predicate=lambda state: self.space.distance(state, goal_state) <= 1e-9,
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
    def _near_indices(index: NearestNeighborIndex, target: State, radius: float) -> list[int]:
        """Return neighbor indices around ``target`` using a dynamic RRT* radius."""
        return index.radius(target, radius)

    @staticmethod
    def _reconstruct_path(nodes: list[State], parents: list[int], last_index: int) -> list[State]:
        """Backtrack parent pointers to reconstruct a root-to-leaf path."""
        path_reversed: list[State] = []
        index = last_index
        while index != -1:
            path_reversed.append(np.asarray(nodes[index], dtype=float))
            index = parents[index]
        path_reversed.reverse()
        return path_reversed

    @staticmethod
    def _propagate_cost_delta(
        root_index: int,
        delta: float,
        costs: list[float],
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
                stats={"reason": "start_in_goal", "goal_checks": 1},
            )

        nodes: list[State] = [np.asarray(start_state, dtype=float)]
        parents: list[int] = [-1]
        children: list[set[int]] = [set()]
        costs: list[float] = [0.0]
        index = self._nn_index_factory(self.space.dim)
        index.add(nodes[0])

        goal_indices: list[int] = []
        goal_checks = 0
        start_time = time.perf_counter()
        iters_run = 0

        for iters_run in range(1, self.params.max_iters + 1):
            if self.params.time_budget_s is not None:
                elapsed = time.perf_counter() - start_time
                if elapsed >= self.params.time_budget_s:
                    break

            use_goal_sample = goal.target_state is not None and self.rng.random() < self.params.goal_sample_rate
            target = goal.target_state if use_goal_sample else self._sample_free()

            nearest_index = self._nearest_index(index, target)
            nearest = nodes[nearest_index]
            candidate = self.space.steer(nearest, target, self.params.step_size)

            if not self.space.is_free(candidate):
                continue
            if not self.space.segment_free(nearest, candidate, self.params.collision_step):
                continue

            card_v = len(nodes)
            if card_v <= 1:
                near_indices = [0]
            else:
                dim = float(self.space.dim)
                dynamic_radius = self.params.step_size * (2.0 * (np.log(card_v) / card_v) ** (1.0 / dim) + 1.0)
                near_radius = min(self.params.step_size * 6.0, dynamic_radius)
                near_indices = self._near_indices(index, candidate, near_radius)

            parent_index = nearest_index
            parent_cost = costs[parent_index] + self.space.distance(nearest, candidate)
            for near_index in near_indices:
                near_node = nodes[near_index]
                if not self.space.segment_free(near_node, candidate, self.params.collision_step):
                    continue
                candidate_cost = costs[near_index] + self.space.distance(near_node, candidate)
                if candidate_cost < parent_cost:
                    parent_index = near_index
                    parent_cost = candidate_cost

            nodes.append(np.asarray(candidate, dtype=float))
            parents.append(parent_index)
            children.append(set())
            children[parent_index].add(len(nodes) - 1)
            costs.append(parent_cost)
            index.add(nodes[-1])
            new_index = len(nodes) - 1

            for near_index in near_indices:
                if near_index == parent_index or near_index == new_index:
                    continue
                near_node = nodes[near_index]
                if not self.space.segment_free(candidate, near_node, self.params.collision_step):
                    continue
                rewired_cost = costs[new_index] + self.space.distance(candidate, near_node)
                if rewired_cost < costs[near_index]:
                    old_parent = parents[near_index]
                    if old_parent != -1:
                        children[old_parent].remove(near_index)
                    parents[near_index] = new_index
                    children[new_index].add(near_index)
                    delta = rewired_cost - costs[near_index]
                    self._propagate_cost_delta(near_index, delta, costs, children)

            goal_checks += 1
            if goal.predicate(nodes[new_index]):
                goal_indices.append(new_index)

        if goal_indices:
            best_goal_index = min(goal_indices, key=lambda idx: costs[idx])
            path = self._reconstruct_path(nodes, parents, best_goal_index)
            return PlanResult(
                success=True,
                path=path,
                iters=iters_run,
                nodes=len(nodes),
                stats={
                    "goal_checks": goal_checks,
                    "path_cost": float(costs[best_goal_index]),
                    "goal_nodes": len(goal_indices),
                },
            )

        return PlanResult(
            success=False,
            path=[],
            iters=iters_run,
            nodes=len(nodes),
            stats={"goal_checks": goal_checks},
        )
