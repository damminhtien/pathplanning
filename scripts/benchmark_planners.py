"""Benchmark representative planners with deterministic settings."""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import asdict, dataclass
import json
import random
import statistics
import time

import numpy as np

from pathplanning.api import plan_continuous, plan_discrete
from pathplanning.core.contracts import ContinuousProblem, DiscreteProblem, GoalState
from pathplanning.core.params import RrtParams
from pathplanning.spaces.grid2d import Grid2DSamplingSpace, Grid2DSearchSpace
from pathplanning.spaces.grid3d import Grid3DSearchSpace


@dataclass
class BenchmarkRow:
    """One benchmark row for a planner run."""

    planner_id: str
    runtime_s: float
    nodes_expanded: int
    path_found: bool
    graph_init_s: float = 0.0
    native_search_s: float = 0.0
    samples: int = 1


def _benchmark_search2d_astar() -> BenchmarkRow:
    problem = DiscreteProblem(
        graph=Grid2DSearchSpace(),
        start=(5, 5),
        goal=(45, 25),
    )
    start = time.perf_counter()
    result = plan_discrete(
        problem,
        planner="astar",
        params={"max_expansions": 50_000},
        seed=0,
    )
    runtime_s = time.perf_counter() - start

    return BenchmarkRow(
        planner_id="discrete2d.astar",
        runtime_s=runtime_s,
        nodes_expanded=result.iters,
        path_found=result.path is not None,
        graph_init_s=result.stats.get("graph_init_s", 0.0),
        native_search_s=result.stats.get("native_search_s", 0.0),
    )


def _benchmark_sampling2d_rrt() -> BenchmarkRow:
    space = Grid2DSamplingSpace()
    problem = ContinuousProblem(
        space=space,
        start=np.array([2.0, 2.0], dtype=float),
        goal=GoalState(
            state=np.array([49.0, 24.0], dtype=float),
            radius=0.25,
            distance_fn=space.distance,
        ),
    )

    start = time.perf_counter()
    result = plan_continuous(
        problem,
        planner="rrt",
        params=RrtParams(step_size=0.5, goal_sample_rate=0.05, max_iters=3000),
        seed=0,
    )
    runtime_s = time.perf_counter() - start

    return BenchmarkRow(
        planner_id="sampling2d.rrt",
        runtime_s=runtime_s,
        nodes_expanded=result.nodes,
        path_found=result.path is not None,
    )


def _benchmark_search3d_weighted_astar() -> BenchmarkRow:
    problem = DiscreteProblem(
        graph=Grid3DSearchSpace(width=21, height=21, depth=6),
        start=(2, 2, 1),
        goal=(18, 17, 1),
    )

    start = time.perf_counter()
    result = plan_discrete(
        problem,
        planner="weighted_astar",
        params={"weight": 1.0, "max_expansions": 50000},
        seed=0,
    )
    runtime_s = time.perf_counter() - start

    return BenchmarkRow(
        planner_id="search3d.weighted_astar",
        runtime_s=runtime_s,
        nodes_expanded=result.iters,
        path_found=result.path is not None,
        graph_init_s=result.stats.get("graph_init_s", 0.0),
        native_search_s=result.stats.get("native_search_s", 0.0),
    )


def _repeat_benchmark(
    benchmark: Callable[[], BenchmarkRow],
    repeats: int,
) -> BenchmarkRow:
    """Run a benchmark repeatedly and report medians for numeric fields."""
    samples = [benchmark() for _ in range(repeats)]
    first = samples[0]
    return BenchmarkRow(
        planner_id=first.planner_id,
        runtime_s=statistics.median(row.runtime_s for row in samples),
        nodes_expanded=int(statistics.median(row.nodes_expanded for row in samples)),
        path_found=all(row.path_found for row in samples),
        graph_init_s=statistics.median(row.graph_init_s for row in samples),
        native_search_s=statistics.median(row.native_search_s for row in samples),
        samples=repeats,
    )


def run_benchmarks(seed: int, repeats: int = 1) -> list[BenchmarkRow]:
    """Run representative benchmarks and return median rows."""
    if repeats <= 0:
        raise ValueError("repeats must be > 0")
    random.seed(seed)
    np.random.seed(seed)
    return [
        _repeat_benchmark(_benchmark_search2d_astar, repeats),
        _repeat_benchmark(_benchmark_sampling2d_rrt, repeats),
        _repeat_benchmark(_benchmark_search3d_weighted_astar, repeats),
    ]


def _print_table(rows: list[BenchmarkRow]) -> None:
    header = (
        f"{'planner_id':28} {'runtime_s':>10} {'graph_init_s':>13} "
        f"{'native_search_s':>16} {'nodes_expanded':>15} {'path_found':>11} {'samples':>8}"
    )
    print(header)
    print("-" * len(header))
    for row in rows:
        print(
            f"{row.planner_id:28} "
            f"{row.runtime_s:10.4f} "
            f"{row.graph_init_s:13.4f} "
            f"{row.native_search_s:16.4f} "
            f"{row.nodes_expanded:15d} "
            f"{str(row.path_found):>11} "
            f"{row.samples:8d}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Benchmark representative path planners.")
    parser.add_argument("--seed", type=int, default=0, help="Random seed for deterministic runs.")
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print machine-readable JSON instead of a table.",
    )
    parser.add_argument(
        "--repeats",
        type=int,
        default=5,
        help="Number of runs per planner; reported times are medians.",
    )
    args = parser.parse_args()

    rows = run_benchmarks(args.seed, repeats=args.repeats)
    if args.json:
        print(json.dumps([asdict(row) for row in rows], indent=2, sort_keys=True))
        return

    _print_table(rows)


if __name__ == "__main__":
    main()
