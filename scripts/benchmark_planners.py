"""Benchmark representative planners with deterministic settings."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import random
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
    )


def run_benchmarks(seed: int) -> list[BenchmarkRow]:
    """Run representative benchmarks and return rows."""
    random.seed(seed)
    np.random.seed(seed)
    return [
        _benchmark_search2d_astar(),
        _benchmark_sampling2d_rrt(),
        _benchmark_search3d_weighted_astar(),
    ]


def _print_table(rows: list[BenchmarkRow]) -> None:
    header = f"{'planner_id':28} {'runtime_s':>10} {'nodes_expanded':>15} {'path_found':>11}"
    print(header)
    print("-" * len(header))
    for row in rows:
        print(
            f"{row.planner_id:28} "
            f"{row.runtime_s:10.4f} "
            f"{row.nodes_expanded:15d} "
            f"{str(row.path_found):>11}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Benchmark representative path planners.")
    parser.add_argument("--seed", type=int, default=0, help="Random seed for deterministic runs.")
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print machine-readable JSON instead of a table.",
    )
    args = parser.parse_args()

    rows = run_benchmarks(args.seed)
    if args.json:
        print(json.dumps([asdict(row) for row in rows], indent=2, sort_keys=True))
        return

    _print_table(rows)


if __name__ == "__main__":
    main()
