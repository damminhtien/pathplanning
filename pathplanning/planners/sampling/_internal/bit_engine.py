"""Shared implicit random-geometric-graph engine for BIT* and ABIT*."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
import heapq
import math
import time

import numpy as np

from pathplanning.core.contracts import ContinuousProblem, ContinuousSpace, GoalRegion, State
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult, StopReason
from pathplanning.core.types import RNG
from pathplanning.nn.index import IncrementalNnIndex, NearestNeighborIndex
from pathplanning.planners.sampling._internal.continuous import (
    as_state,
    build_nn_index,
    connection_radius,
    euclidean_distance,
    exact_goal_state,
    motion_is_valid,
    path_from_parents,
    sample_informed,
    validate_objective,
)
from pathplanning.planners.sampling._internal.problem_adapter import (
    coerce_rrt_params,
    resolve_rng,
)

IndexFactory = Callable[[int], NearestNeighborIndex]


class BatchInformedTreePlanner:
    """Batch sampling and ordered edge search over an implicit geometric graph."""

    planner_name = "BIT*"

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

    def _search_factors(self, batch_number: int) -> tuple[float, float]:
        _ = batch_number
        return 1.0, 1.0

    def _heuristic(self, state: State, goal: State) -> float:
        return euclidean_distance(self.space, state, goal)

    def _search_batch(
        self,
        states: np.ndarray,
        *,
        index: NearestNeighborIndex,
        costs: np.ndarray,
        parents: np.ndarray,
        heuristics: np.ndarray,
        closed: np.ndarray,
        expanded_costs: np.ndarray,
        open_costs: np.ndarray,
        best_cost: float,
        radius: float,
        inflation: float,
        truncation: float,
        max_expansions: int,
        started: float,
    ) -> tuple[int, int, bool]:
        node_count = int(states.shape[0])
        closed[:node_count] = False
        expanded_costs[:node_count] = np.inf
        open_costs[:node_count] = np.inf
        vertex_heap: list[tuple[float, int, int, float]] = []
        edge_heap: list[tuple[float, int, int, int, float, float]] = []
        sequence = 0
        for node, cost in enumerate(costs[:node_count]):
            if np.isfinite(cost):
                open_costs[node] = cost
                key = cost + inflation * heuristics[node]
                heapq.heappush(vertex_heap, (key, sequence, node, cost))
                sequence += 1
        expanded = 0
        motion_checks = 0
        timed_out = False

        def clean_vertex_heap() -> None:
            while vertex_heap:
                _, _, node, queued_cost = vertex_heap[0]
                if queued_cost != costs[node] or queued_cost != open_costs[node]:
                    heapq.heappop(vertex_heap)
                    continue
                if closed[node] and expanded_costs[node] <= queued_cost:
                    heapq.heappop(vertex_heap)
                    open_costs[node] = np.inf
                    continue
                return

        def clean_edge_heap() -> None:
            while edge_heap:
                _, _, source, _, source_cost, _ = edge_heap[0]
                if source_cost != costs[source]:
                    heapq.heappop(edge_heap)
                    continue
                return

        while vertex_heap or edge_heap:
            if (
                self.params.time_budget_s is not None
                and time.perf_counter() - started >= self.params.time_budget_s
            ):
                timed_out = True
                break
            clean_vertex_heap()
            clean_edge_heap()
            vertex_key = vertex_heap[0][0] if vertex_heap else math.inf
            edge_key = edge_heap[0][0] if edge_heap else math.inf
            if np.isfinite(best_cost) and min(vertex_key, edge_key) >= best_cost:
                break

            if vertex_key <= edge_key:
                if expanded >= max_expansions:
                    break
                _, _, source, queued_cost = heapq.heappop(vertex_heap)
                if queued_cost != costs[source]:
                    continue
                open_costs[source] = np.inf
                if closed[source] and expanded_costs[source] <= queued_cost:
                    continue
                closed[source] = True
                expanded_costs[source] = queued_cost
                expanded += 1

                for target_raw in index.radius(states[source], radius):
                    target = int(target_raw)
                    if source == target:
                        continue
                    edge_cost = euclidean_distance(self.space, states[source], states[target])
                    tentative = queued_cost + edge_cost
                    lower_bound = tentative + heuristics[target]
                    if np.isfinite(best_cost) and lower_bound > truncation * best_cost:
                        continue
                    key = tentative + inflation * heuristics[target]
                    heapq.heappush(
                        edge_heap,
                        (key, sequence, source, target, queued_cost, tentative),
                    )
                    sequence += 1
            else:
                _, _, source, target, source_cost, tentative = heapq.heappop(edge_heap)
                if source_cost != costs[source] or tentative >= costs[target]:
                    continue
                if np.isfinite(best_cost) and (
                    tentative + heuristics[target] > truncation * best_cost
                ):
                    continue
                motion_checks += 1
                if not motion_is_valid(self.space, self.params, states[source], states[target]):
                    continue
                costs[target] = tentative
                parents[target] = source
                closed[target] = False
                open_costs[target] = tentative
                key = tentative + inflation * heuristics[target]
                heapq.heappush(vertex_heap, (key, sequence, target, tentative))
                sequence += 1
                if target == 1 and tentative < best_cost:
                    best_cost = tentative

        return expanded, motion_checks, timed_out

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

        sample_limit = self.params.sample_count
        states = np.empty((sample_limit + 2, dim), dtype=float)
        states[0] = start_state
        states[1] = goal_state
        costs = np.full(sample_limit + 2, np.inf, dtype=float)
        costs[0] = 0.0
        parents = np.full(sample_limit + 2, -1, dtype=np.int64)
        heuristics = np.empty(sample_limit + 2, dtype=float)
        heuristics[0] = self._heuristic(start_state, goal_state)
        heuristics[1] = 0.0
        closed = np.zeros(sample_limit + 2, dtype=bool)
        expanded_costs = np.full(sample_limit + 2, np.inf, dtype=float)
        open_costs = np.full(sample_limit + 2, np.inf, dtype=float)
        index: NearestNeighborIndex
        if self.nn_index_factory is None:
            incremental_index = IncrementalNnIndex(dim)
            incremental_index.append(states[:2])
            index = incremental_index
        else:
            incremental_index = None
            index = build_nn_index(states[:2], factory=self.nn_index_factory)

        best_path: np.ndarray | None = None
        best_cost = math.inf
        iterations = 0
        motion_checks = 0
        stop_reason = StopReason.NO_PROGRESS
        batch_number = 0
        sample_count = 0
        while sample_count < sample_limit and iterations < self.params.max_iters:
            if (
                self.params.time_budget_s is not None
                and time.perf_counter() - started >= self.params.time_budget_s
            ):
                stop_reason = StopReason.TIME_BUDGET
                break
            batch_count = min(self.params.batch_size, sample_limit - sample_count)
            batch_states = np.empty((batch_count, dim), dtype=float)
            for batch_offset in range(batch_count):
                for _ in range(self.params.max_sample_tries):
                    if np.isfinite(best_cost):
                        sample = sample_informed(
                            self.space,
                            self.rng,
                            self.params,
                            start_state,
                            goal_state,
                            best_cost,
                        )
                    else:
                        sample = as_state(
                            self.space.sample_free(self.rng), "sampled_state", dim=dim
                        )
                    if self.space.is_state_valid(sample):
                        batch_states[batch_offset] = sample
                        break
                else:
                    raise ValueError("could not sample the requested number of valid states")

            states[2 + sample_count : 2 + sample_count + batch_count] = batch_states
            first_new_node = 2 + sample_count
            for node in range(first_new_node, first_new_node + batch_count):
                heuristics[node] = self._heuristic(states[node], goal_state)
            sample_count += batch_count
            node_count = sample_count + 2
            state_view = states[:node_count]
            if incremental_index is not None:
                incremental_index.append(batch_states)
            else:
                index = build_nn_index(state_view, factory=self.nn_index_factory)
            radius = connection_radius(self.params, count=node_count, dim=dim)
            inflation, truncation = self._search_factors(batch_number)
            remaining = self.params.max_iters - iterations
            expanded, checked, timed_out = self._search_batch(
                state_view,
                index=index,
                costs=costs,
                parents=parents,
                heuristics=heuristics,
                closed=closed,
                expanded_costs=expanded_costs,
                open_costs=open_costs,
                best_cost=best_cost,
                radius=radius,
                inflation=inflation,
                truncation=truncation,
                max_expansions=remaining,
                started=started,
            )
            iterations += expanded
            motion_checks += checked
            if np.isfinite(costs[1]) and costs[1] < best_cost:
                best_cost = float(costs[1])
                best_path = path_from_parents(state_view, parents, 1)
            batch_number += 1
            if timed_out:
                stop_reason = StopReason.TIME_BUDGET
                break
            if iterations >= self.params.max_iters:
                stop_reason = StopReason.MAX_ITERS
                break
            if sample_count >= sample_limit:
                stop_reason = (
                    StopReason.SUCCESS if best_path is not None else StopReason.NO_PROGRESS
                )
                break
            if best_path is not None and np.isclose(
                best_cost,
                euclidean_distance(self.space, start_state, goal_state),
                rtol=1e-10,
                atol=1e-12,
            ):
                stop_reason = StopReason.SUCCESS
                break

        elapsed = time.perf_counter() - started
        stats: dict[str, float] = {
            "sample_count": float(sample_count),
            "batches": float(batch_number),
            "motion_checks": float(motion_checks),
            "elapsed_s": elapsed,
        }
        if self.params.time_budget_s is not None:
            stats["time_budget_s"] = self.params.time_budget_s
        if best_path is not None:
            path_cost = sum(
                float(self.space.distance(a, b))
                for a, b in zip(best_path[:-1], best_path[1:], strict=True)
            )
            stats["path_cost"] = path_cost
            return PlanResult(
                True,
                best_path,
                best_path,
                StopReason.SUCCESS,
                iterations,
                int(2 + sample_count),
                stats,
            )
        return PlanResult(
            False,
            None,
            None,
            stop_reason,
            iterations,
            int(2 + sample_count),
            stats,
        )


def run_bit_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Run BIT* using batched samples and ordered implicit-edge search."""
    validate_objective(problem.objective, "BIT*")
    resolved_params = coerce_rrt_params(problem, params)
    return BatchInformedTreePlanner(problem.space, resolved_params, resolve_rng(rng)).plan(
        problem.start,
        problem.goal,
    )


__all__ = ["BatchInformedTreePlanner", "IndexFactory", "run_bit_star"]
