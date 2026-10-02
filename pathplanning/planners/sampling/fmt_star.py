"""FMT*: lazy dynamic programming over a fixed random sample set."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
import heapq
import time

import numpy as np

from pathplanning.core.contracts import ContinuousProblem, ContinuousSpace, GoalRegion, State
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult, StopReason
from pathplanning.core.types import RNG
from pathplanning.nn.index import NearestNeighborIndex
from pathplanning.planners.sampling._internal.continuous import (
    as_state,
    build_nn_index,
    collect_free_samples,
    connection_radius,
    euclidean_distance,
    exact_goal_state,
    motion_is_valid,
    path_from_parents,
    validate_objective,
)
from pathplanning.planners.sampling._internal.problem_adapter import (
    coerce_rrt_params,
    resolve_rng,
)

IndexFactory = Callable[[int], NearestNeighborIndex]


class FmtStar:
    """Fixed-sample FMT* planner with lazy parent collision checking."""

    def __init__(
        self,
        space: ContinuousSpace[State],
        params: RrtParams,
        rng: np.random.Generator,
        nn_index_factory: IndexFactory | None = None,
    ) -> None:
        self.space = space
        self.params = params.validate()
        self.rng = rng
        self.nn_index_factory = nn_index_factory

    def plan(self, start: Sequence[float] | State, goal_region: GoalRegion[State]) -> PlanResult:
        started = time.perf_counter()
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
        if (
            euclidean_distance(self.space, start_state, goal_state)
            <= self.params.goal_reach_tolerance
        ):
            path = np.asarray([start_state], dtype=float)
            return PlanResult(True, path, path, StopReason.SUCCESS, 0, 1, {"path_cost": 0.0})

        samples = collect_free_samples(
            self.space,
            self.rng,
            self.params,
            count=self.params.sample_count,
            dim=dim,
        )
        states = np.empty((self.params.sample_count + 2, dim), dtype=float)
        states[0] = start_state
        states[1] = goal_state
        states[2:] = samples
        node_count = int(states.shape[0])
        radius = connection_radius(self.params, count=node_count, dim=dim)
        index = build_nn_index(states, factory=self.nn_index_factory)
        parents = np.full(node_count, -1, dtype=np.int64)
        costs = np.full(node_count, np.inf, dtype=float)
        costs[0] = 0.0
        # 0=unvisited, 1=open, 2=closed. Newly opened samples become parents
        # only in the next FMT expansion, as in the standard recursion.
        status = np.zeros(node_count, dtype=np.uint8)
        status[0] = 1
        open_heap: list[tuple[float, int, int]] = [(0.0, 0, 0)]
        sequence = 1
        iters = 0
        motion_checks = 0
        stop_reason = StopReason.NO_PROGRESS

        while open_heap and iters < self.params.max_iters:
            if (
                self.params.time_budget_s is not None
                and time.perf_counter() - started >= self.params.time_budget_s
            ):
                stop_reason = StopReason.TIME_BUDGET
                break
            cost_to_come, _, node = heapq.heappop(open_heap)
            if status[node] != 1 or cost_to_come > costs[node]:
                continue
            status[node] = 2
            iters += 1
            if node == 1:
                path = path_from_parents(states, parents, node)
                elapsed = time.perf_counter() - started
                return PlanResult(
                    True,
                    path,
                    path,
                    StopReason.SUCCESS,
                    iters,
                    node_count,
                    {
                        "path_cost": float(costs[node]),
                        "sample_count": float(samples.shape[0]),
                        "connection_radius": radius,
                        "motion_checks": float(motion_checks),
                        "elapsed_s": elapsed,
                    },
                )

            unvisited = [
                int(candidate)
                for candidate in index.radius(states[node], radius)
                if status[int(candidate)] == 0
            ]
            proposals: list[tuple[int, int, float]] = []
            for candidate in unvisited:
                open_parents = [
                    int(parent)
                    for parent in index.radius(states[candidate], radius)
                    if status[int(parent)] == 1 or int(parent) == node
                ]
                open_parents.sort(
                    key=lambda parent: (
                        costs[parent]
                        + euclidean_distance(self.space, states[parent], states[candidate])
                    )
                )
                for parent in open_parents:
                    edge_cost = euclidean_distance(self.space, states[parent], states[candidate])
                    motion_checks += 1
                    if motion_is_valid(self.space, self.params, states[parent], states[candidate]):
                        proposals.append((candidate, parent, float(costs[parent] + edge_cost)))
                        break

            for candidate, parent, candidate_cost in proposals:
                if status[candidate] != 0:
                    continue
                status[candidate] = 1
                parents[candidate] = parent
                costs[candidate] = candidate_cost
                heapq.heappush(open_heap, (candidate_cost, sequence, candidate))
                sequence += 1

        if iters >= self.params.max_iters:
            stop_reason = StopReason.MAX_ITERS
        elif stop_reason != StopReason.TIME_BUDGET and not open_heap:
            stop_reason = StopReason.NO_PROGRESS
        elapsed = time.perf_counter() - started
        stats = {
            "sample_count": float(samples.shape[0]),
            "connection_radius": radius,
            "motion_checks": float(motion_checks),
            "elapsed_s": elapsed,
        }
        if self.params.time_budget_s is not None:
            stats["time_budget_s"] = self.params.time_budget_s
        return PlanResult(False, None, None, stop_reason, iters, node_count, stats)


def plan_fmt_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Plan with FMT* for exact goals and additive path length."""
    validate_objective(problem.objective, "FMT*")
    resolved_params = coerce_rrt_params(problem, params)
    return FmtStar(problem.space, resolved_params, resolve_rng(rng)).plan(
        problem.start,
        problem.goal,
    )


__all__ = ["FmtStar", "IndexFactory", "plan_fmt_star"]
