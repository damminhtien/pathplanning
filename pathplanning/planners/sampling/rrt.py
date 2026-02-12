"""Headless, contract-based RRT planner."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import time
from typing import TypeAlias, cast

import numpy as np

from pathplanning.core.contracts import (
    ContinuousSpace,
    GoalRegion,
    GoalState,
    State,
    SupportsBatchMotionCheck,
)
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult, StopReason
from pathplanning.core.types import Mat
from pathplanning.data_structures.tree_array import ArrayTree
from pathplanning.nn.index import NaiveNnIndex, NearestNeighborIndex

GoalPredicate: TypeAlias = Callable[[State], bool]
IndexFactory: TypeAlias = Callable[[int], NearestNeighborIndex]


def _default_index_factory(dim: int) -> NearestNeighborIndex:
    return NaiveNnIndex(dim=dim)


def _as_state(value: Sequence[float] | State, name: str, *, dim: int) -> State:
    """Normalize a coordinate-like value to a 1D float state vector."""
    state = np.asarray(value, dtype=float)
    if state.shape != (dim,):
        raise ValueError(f"{name} must be shape ({dim},), got {state.shape}")
    return state


@dataclass(slots=True)
class GoalSpec:
    """Parsed goal definition used by sampling and termination checks."""

    predicate: GoalPredicate
    target_state: State | None


class RrtPlanner:
    """Single-query RRT planner on top of ``ContinuousSpace`` contracts."""

    def __init__(
        self,
        space: ContinuousSpace[State],
        params: RrtParams,
        rng: np.random.Generator,
        nn_index_factory: IndexFactory | None = None,
    ) -> None:
        """Initialize an interface-driven RRT planner."""
        self.space = space
        self.params = params.validate()
        self.rng = rng
        self._nn_index_factory = nn_index_factory or _default_index_factory
        self._batch_motion_checker: SupportsBatchMotionCheck[State] | None = None
        if hasattr(space, "is_motion_valid_batch"):
            self._batch_motion_checker = cast(SupportsBatchMotionCheck[State], space)
        if hasattr(self.space, "collision_step"):
            self.space.collision_step = self.params.collision_step  # type: ignore[attr-defined]

    def _goal_spec(self, goal_region: GoalRegion[State], *, dim: int) -> GoalSpec:
        """Build planner goal helpers from a ``GoalRegion`` implementation."""
        target_state: State | None = None
        if isinstance(goal_region, GoalState):
            target_state = _as_state(goal_region.state, "goal_state", dim=dim)
        return GoalSpec(predicate=goal_region.contains, target_state=target_state)

    def _sample_free(self, *, dim: int) -> State:
        """Sample one collision-free state and normalize dimensionality."""
        sampled = self.space.sample_free(self.rng)
        return _as_state(sampled, "sampled_state", dim=dim)

    @staticmethod
    def _nearest_index(index: NearestNeighborIndex, target: State) -> int:
        """Return the index of the nearest node to ``target``."""
        return index.nearest(target)

    @staticmethod
    def _path_from_tree(tree: ArrayTree, node_id: int) -> Mat:
        """Return root-to-node path as a matrix of states."""
        return tree.extract_path(node_id)

    def _is_state_valid(self, state: State) -> bool:
        return self.space.is_state_valid(state)

    def _is_motion_valid(self, start: State, end: State) -> bool:
        return self.space.is_motion_valid(start, end)

    def _is_motion_valid_batch(self, edges: list[tuple[State, State]]) -> list[bool]:
        if self._batch_motion_checker is None:
            return [self._is_motion_valid(start, end) for start, end in edges]
        checks = self._batch_motion_checker.is_motion_valid_batch(edges)
        if len(checks) != len(edges):
            raise ValueError("is_motion_valid_batch must return one flag per edge")
        return [bool(value) for value in checks]

    def _stop_reason_for_budget(self, start_time: float) -> StopReason | None:
        """Return budget stop reason when the configured time budget is exhausted."""
        if self.params.time_budget_s is None:
            return None
        elapsed = time.perf_counter() - start_time
        if elapsed >= self.params.time_budget_s:
            return StopReason.TIME_BUDGET
        return None

    def _choose_target(self, goal: GoalSpec, *, dim: int) -> State:
        """Choose the next expansion target using configured goal bias."""
        use_goal_sample = (
            goal.target_state is not None and self.rng.random() < self.params.goal_sample_rate
        )
        if use_goal_sample and goal.target_state is not None:
            return goal.target_state
        return self._sample_free(dim=dim)

    def plan(self, start: Sequence[float] | State, goal_region: GoalRegion[State]) -> PlanResult:
        """Compute a collision-free path from ``start`` into ``goal_region``."""
        self.params.validate()
        start_array = np.asarray(start, dtype=float)
        if start_array.ndim != 1:
            raise ValueError(f"start must be a 1D state vector, got {start_array.shape}")

        dim = int(start_array.shape[0])
        start_state = _as_state(start_array, "start", dim=dim)
        if not self._is_state_valid(start_state):
            raise ValueError("start must be collision free")

        goal = self._goal_spec(goal_region, dim=dim)
        if goal.predicate(start_state):
            return PlanResult(
                success=True,
                path=np.asarray([start_state], dtype=float),
                best_path=np.asarray([start_state], dtype=float),
                stop_reason=StopReason.SUCCESS,
                iters=0,
                nodes=1,
                stats={"goal_checks": 1.0},
            )

        tree = ArrayTree(dim)
        tree.add_node(start_state, -1, 0.0)
        goal_checks = 0
        index = self._nn_index_factory(dim)
        index.build(tree.nodes)

        start_time = time.perf_counter()
        iters_run = 0
        stop_reason: StopReason | None = None
        for iters_run in range(1, self.params.max_iters + 1):
            stop_reason = self._stop_reason_for_budget(start_time)
            if stop_reason is not None:
                break

            target = self._choose_target(goal, dim=dim)

            nearest_index = self._nearest_index(index, target)
            nearest = tree.node(nearest_index)
            candidate = _as_state(
                self.space.steer(nearest, target, self.params.step_size),
                "candidate",
                dim=dim,
            )

            if not self._is_state_valid(candidate):
                continue
            if not self._is_motion_valid_batch([(nearest, candidate)])[0]:
                continue

            new_cost = float(tree.cost[nearest_index] + self.space.distance(nearest, candidate))
            new_index = tree.add_node(candidate, nearest_index, new_cost)
            index.build(tree.nodes)

            goal_checks += 1
            if goal.predicate(tree.node(new_index)):
                path = self._path_from_tree(tree, new_index)
                elapsed = time.perf_counter() - start_time
                stats: dict[str, float] = {
                    "goal_checks": float(goal_checks),
                    "path_cost": float(tree.cost[new_index]),
                    "elapsed_s": elapsed,
                }
                if self.params.time_budget_s is not None:
                    stats["time_budget_s"] = self.params.time_budget_s
                return PlanResult(
                    success=True,
                    path=path,
                    best_path=path,
                    stop_reason=StopReason.SUCCESS,
                    iters=iters_run,
                    nodes=tree.size,
                    stats=stats,
                )

        if stop_reason is None:
            stop_reason = StopReason.NO_PROGRESS if tree.size <= 1 else StopReason.MAX_ITERS
        elapsed = time.perf_counter() - start_time
        stats = {
            "goal_checks": float(goal_checks),
            "elapsed_s": elapsed,
        }
        if self.params.time_budget_s is not None:
            stats["time_budget_s"] = self.params.time_budget_s
        return PlanResult(
            success=False,
            path=None,
            best_path=None,
            stop_reason=stop_reason,
            iters=iters_run,
            nodes=tree.size,
            stats=stats,
        )
