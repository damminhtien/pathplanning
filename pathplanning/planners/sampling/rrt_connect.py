"""Bidirectional RRT-Connect for exact point-to-point motion planning."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import time

import numpy as np

from pathplanning.core.contracts import ContinuousProblem, ContinuousSpace, GoalRegion, State
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult, StopReason
from pathplanning.core.types import RNG
from pathplanning.planners.sampling._internal.continuous import (
    as_state,
    exact_goal_state,
    motion_is_valid,
)
from pathplanning.planners.sampling._internal.problem_adapter import (
    coerce_rrt_params,
    resolve_rng,
)


class _Tree:
    """Growing contiguous state/parent storage for one RRT-Connect tree."""

    def __init__(self, root: State) -> None:
        self.dimension = int(root.size)
        self.states = np.empty((16, self.dimension), dtype=float)
        self.parents = np.full(16, -1, dtype=np.int64)
        self.states[0] = root
        self.size = 1

    def add(self, state: State, parent: int) -> int:
        if self.size == self.states.shape[0]:
            capacity = self.states.shape[0] * 2
            states = np.empty((capacity, self.dimension), dtype=float)
            parents = np.empty(capacity, dtype=np.int64)
            states[: self.size] = self.states[: self.size]
            parents[: self.size] = self.parents[: self.size]
            self.states = states
            self.parents = parents
        index = self.size
        self.states[index] = state
        self.parents[index] = parent
        self.size += 1
        return index

    def nearest(self, target: State, space: ContinuousSpace[State]) -> int:
        points = self.states[: self.size]
        distances = np.fromiter(
            (float(space.distance(point, target)) for point in points),
            dtype=float,
            count=self.size,
        )
        if not np.all(np.isfinite(distances)) or np.any(distances < 0.0):
            raise ValueError("space.distance must return finite non-negative values")
        return int(np.argmin(distances))

    def path_to_root(self, node: int) -> list[State]:
        path: list[State] = []
        while node >= 0:
            path.append(self.states[node].copy())
            node = int(self.parents[node])
        path.reverse()
        return path


class RrtConnect:
    """Two-tree RRT planner that greedily connects the opposite tree."""

    def __init__(
        self,
        space: ContinuousSpace[State],
        params: RrtParams,
        rng: np.random.Generator,
    ) -> None:
        self.space = space
        self.params = params.validate()
        self.rng = rng

    def _extend(self, tree: _Tree, target: State) -> tuple[str, int]:
        parent = tree.nearest(target, self.space)
        origin = tree.states[parent]
        candidate = as_state(
            self.space.steer(origin, target, self.params.step_size),
            "candidate",
            dim=tree.dimension,
        )
        if float(self.space.distance(origin, candidate)) <= 1e-15:
            return "trapped", parent
        if not self.space.is_state_valid(candidate):
            return "trapped", parent
        if not motion_is_valid(self.space, self.params, origin, candidate):
            return "trapped", parent
        node = tree.add(candidate, parent)
        reached = float(self.space.distance(candidate, target)) <= self.params.goal_reach_tolerance
        return ("reached" if reached else "advanced"), node

    @staticmethod
    def _join_paths(
        start_tree: _Tree,
        start_node: int,
        goal_tree: _Tree,
        goal_node: int,
    ) -> np.ndarray:
        from_start = start_tree.path_to_root(start_node)
        to_goal = list(reversed(goal_tree.path_to_root(goal_node)))
        if from_start and to_goal and np.array_equal(from_start[-1], to_goal[0]):
            to_goal = to_goal[1:]
        return np.asarray([*from_start, *to_goal], dtype=float)

    def plan(self, start: Sequence[float] | State, goal_region: GoalRegion[State]) -> PlanResult:
        start_array = np.asarray(start, dtype=float)
        if start_array.ndim != 1:
            raise ValueError(f"start must be a 1D state vector, got {start_array.shape}")
        dim = int(start_array.size)
        start_state = as_state(start_array, "start", dim=dim)
        goal_state = exact_goal_state(goal_region, dim=dim)
        if not self.space.is_state_valid(start_state):
            raise ValueError("start must be collision free")
        if not self.space.is_state_valid(goal_state):
            raise ValueError("goal must be collision free")

        start_tree = _Tree(start_state)
        goal_tree = _Tree(goal_state)
        if float(self.space.distance(start_state, goal_state)) <= self.params.goal_reach_tolerance:
            path = np.asarray([start_state], dtype=float)
            return PlanResult(True, path, path, StopReason.SUCCESS, 0, 1, {"path_cost": 0.0})

        started = time.perf_counter()
        iters = 0
        extensions = 0
        connected = False
        path: np.ndarray | None = None
        stop_reason = StopReason.MAX_ITERS
        active_is_start = True

        while extensions < self.params.max_iters:
            if (
                self.params.time_budget_s is not None
                and time.perf_counter() - started >= self.params.time_budget_s
            ):
                stop_reason = StopReason.TIME_BUDGET
                break
            iters += 1
            active = start_tree if active_is_start else goal_tree
            other = goal_tree if active_is_start else start_tree
            target = (
                other.states[0].copy()
                if self.rng.random() < self.params.goal_sample_rate
                else as_state(self.space.sample_free(self.rng), "sampled_state", dim=dim)
            )

            status, active_node = self._extend(active, target)
            extensions += 1
            if status != "trapped":
                active_target = active.states[active_node].copy()
                while extensions < self.params.max_iters:
                    if (
                        self.params.time_budget_s is not None
                        and time.perf_counter() - started >= self.params.time_budget_s
                    ):
                        stop_reason = StopReason.TIME_BUDGET
                        break
                    connect_status, other_node = self._extend(other, active_target)
                    extensions += 1
                    if connect_status == "trapped":
                        break
                    if connect_status == "reached":
                        active_state = active.states[active_node]
                        other_state = other.states[other_node]
                        if active_is_start:
                            junction_valid = motion_is_valid(
                                self.space, self.params, active_state, other_state
                            )
                            if junction_valid:
                                path = self._join_paths(
                                    start_tree, active_node, goal_tree, other_node
                                )
                        else:
                            junction_valid = motion_is_valid(
                                self.space, self.params, other_state, active_state
                            )
                            if junction_valid:
                                path = self._join_paths(
                                    start_tree, other_node, goal_tree, active_node
                                )
                        if junction_valid:
                            connected = True
                            break
                if connected or stop_reason == StopReason.TIME_BUDGET:
                    break
            active_is_start = not active_is_start

        elapsed = time.perf_counter() - started
        nodes = start_tree.size + goal_tree.size
        stats: dict[str, float] = {
            "elapsed_s": elapsed,
            "extensions": float(extensions),
        }
        if self.params.time_budget_s is not None:
            stats["time_budget_s"] = self.params.time_budget_s
        if connected and path is not None:
            path_cost = sum(
                float(self.space.distance(a, b)) for a, b in zip(path[:-1], path[1:], strict=True)
            )
            stats["path_cost"] = path_cost
            return PlanResult(True, path, path, StopReason.SUCCESS, iters, nodes, stats)
        if extensions == 0 and stop_reason != StopReason.TIME_BUDGET:
            stop_reason = StopReason.NO_PROGRESS
        return PlanResult(False, None, None, stop_reason, iters, nodes, stats)


def plan_rrt_connect(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Plan a feasible path using bidirectional RRT-Connect."""
    resolved_params = coerce_rrt_params(problem, params)
    return RrtConnect(problem.space, resolved_params, resolve_rng(rng)).plan(
        problem.start,
        problem.goal,
    )


__all__ = ["RrtConnect", "plan_rrt_connect"]
