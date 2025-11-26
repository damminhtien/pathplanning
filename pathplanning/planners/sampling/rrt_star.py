"""Headless, contract-based RRT* planner."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import time
from typing import TypeAlias, cast

import numpy as np
from numpy.typing import NDArray

from pathplanning.core.contracts import (
    ContinuousProblem,
    ContinuousSpace,
    GoalRegion,
    GoalState,
    State,
    SupportsBatchMotionCheck,
)
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult, StopReason
from pathplanning.core.types import Mat, NodeId, RNG
from pathplanning.data_structures.tree_array import ArrayTree
from pathplanning.nn.index import NaiveNnIndex, NearestNeighborIndex
from pathplanning.planners.sampling._internal.problem_adapter import (
    coerce_rrt_params,
    resolve_rng,
)

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
    """Parsed goal definition used internally by the planner."""

    predicate: GoalPredicate
    target_state: State | None


class RrtStarPlanner:
    """Single-query RRT* planner on top of ``ContinuousSpace`` contracts."""

    def __init__(
        self,
        space: ContinuousSpace[State],
        params: RrtParams,
        rng: np.random.Generator,
        nn_index_factory: IndexFactory | None = None,
    ) -> None:
        """Initialize an interface-driven RRT* planner."""
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
    def _near_indices(
        index: NearestNeighborIndex,
        target: State,
        radius: float,
    ) -> NDArray[np.int64]:
        """Return neighbor indices around ``target`` using a dynamic RRT* radius."""
        return index.radius(target, radius)

    @staticmethod
    def _path_from_tree(tree: ArrayTree, node_id: int) -> Mat:
        """Return root-to-node path as a state matrix."""
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

    def _choose_parent(
        self,
        tree: ArrayTree,
        candidate: State,
        near_indices: NDArray[np.int64],
        nearest_index: NodeId,
    ) -> NodeId:
        """Select the best parent for ``candidate`` from nearby nodes."""
        parent_index = nearest_index
        nearest = tree.node(nearest_index)
        parent_cost = float(tree.cost[parent_index] + self.space.distance(nearest, candidate))

        candidate_edges: list[tuple[State, State]] = []
        near_index_list: list[NodeId] = []
        for near_index_raw in near_indices:
            near_index = int(near_index_raw)
            near_node = tree.node(near_index)
            near_index_list.append(near_index)
            candidate_edges.append((near_node, candidate))

        validity = self._is_motion_valid_batch(candidate_edges)
        for near_index, is_valid in zip(near_index_list, validity, strict=True):
            if not is_valid:
                continue
            near_node = tree.node(near_index)
            candidate_cost = float(
                tree.cost[near_index] + self.space.distance(near_node, candidate)
            )
            if candidate_cost < parent_cost:
                parent_index = near_index
                parent_cost = candidate_cost
        return parent_index

    def _propagate_costs_from(
        self,
        root_index: NodeId,
        delta: float,
        tree: ArrayTree,
        children: list[set[NodeId]],
    ) -> None:
        """Apply a cost delta to one subtree after rewiring."""
        stack = [root_index]
        while stack:
            index = stack.pop()
            tree.cost[index] += delta
            stack.extend(children[index])

    def _rewire(
        self,
        tree: ArrayTree,
        new_index: NodeId,
        candidate: State,
        near_indices: NDArray[np.int64],
        parent_index: NodeId,
        children: list[set[NodeId]],
    ) -> None:
        """Attempt rewiring nearby nodes through ``new_index``."""
        rewire_index_list: list[NodeId] = []
        rewire_edges: list[tuple[State, State]] = []
        for near_index_raw in near_indices:
            near_index = int(near_index_raw)
            if near_index in (parent_index, new_index):
                continue
            near_node = tree.node(near_index)
            rewire_index_list.append(near_index)
            rewire_edges.append((candidate, near_node))

        validity = self._is_motion_valid_batch(rewire_edges)
        for near_index, is_valid in zip(rewire_index_list, validity, strict=True):
            if not is_valid:
                continue
            near_node = tree.node(near_index)
            rewired_cost = float(tree.cost[new_index] + self.space.distance(candidate, near_node))
            if rewired_cost < tree.cost[near_index]:
                old_parent = int(tree.parent[near_index])
                if old_parent != -1:
                    children[old_parent].remove(near_index)
                tree.parent[near_index] = new_index
                children[new_index].add(near_index)
                delta = float(rewired_cost - tree.cost[near_index])
                self._propagate_costs_from(near_index, delta, tree, children)

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
        children: list[set[NodeId]] = [set()]
        index = self._nn_index_factory(dim)
        index.build(tree.nodes)

        goal_indices: list[int] = []
        goal_checks = 0
        start_time = time.perf_counter()
        iters_run = 0
        stop_reason: StopReason | None = None

        for iter_index in range(1, self.params.max_iters + 1):
            iters_run = iter_index
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

            card_v = tree.size
            if card_v <= 1:
                near_indices = np.asarray([0], dtype=np.int64)
            else:
                dim_float = float(dim)
                dynamic_radius = self.params.step_size * (
                    self.params.rrt_star_radius_gamma * (np.log(card_v) / card_v) ** (1.0 / dim_float)
                    + self.params.rrt_star_radius_bias
                )
                near_radius = min(
                    self.params.step_size * self.params.rrt_star_radius_max_factor, dynamic_radius
                )
                near_indices = self._near_indices(index, candidate, near_radius)

            parent_index = self._choose_parent(tree, candidate, near_indices, nearest_index)
            parent_node = tree.node(parent_index)
            parent_cost = float(
                tree.cost[parent_index] + self.space.distance(parent_node, candidate)
            )

            new_index = tree.add_node(candidate, parent_index, parent_cost)
            children.append(set())
            children[parent_index].add(new_index)
            index.build(tree.nodes)

            self._rewire(tree, new_index, candidate, near_indices, parent_index, children)

            goal_checks += 1
            if goal.predicate(tree.node(new_index)):
                goal_indices.append(new_index)

        if goal_indices:
            best_goal_index = min(goal_indices, key=lambda idx: tree.cost[idx])
            path = self._path_from_tree(tree, best_goal_index)
            elapsed = time.perf_counter() - start_time
            stats: dict[str, float] = {
                "goal_checks": float(goal_checks),
                "path_cost": float(tree.cost[best_goal_index]),
                "goal_nodes": float(len(goal_indices)),
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
        stats: dict[str, float] = {
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


def plan_rrt_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Plan one ``ContinuousProblem`` with RRT*."""
    resolved_params = coerce_rrt_params(problem, params)
    planner = RrtStarPlanner(problem.space, resolved_params, resolve_rng(rng))
    return planner.plan(problem.start, problem.goal)


__all__ = ["GoalPredicate", "IndexFactory", "RrtStarPlanner", "plan_rrt_star"]
