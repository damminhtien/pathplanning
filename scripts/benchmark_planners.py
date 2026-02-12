"""Benchmark representative planners with deterministic settings."""

from __future__ import annotations

import argparse
import contextlib
from dataclasses import asdict, dataclass
import io
import json
import random
import time

import numpy as np

from pathplanning import PlanConfig, Planner, Search2D
from pathplanning.sampling_based_planning.rrt_2d.rrt import Rrt
from pathplanning.search_based_planning.search_3d.astar_3d import Weighted_A_star


@dataclass
class BenchmarkRow:
    """One benchmark row for a planner run."""

    planner_id: str
    runtime_s: float
    nodes_expanded: int
    path_found: bool


def _benchmark_search2d() -> BenchmarkRow:
    planner = Search2D()
    cfg = PlanConfig(s_start=(5, 5), s_goal=(45, 25))

    start = time.perf_counter()
    result = planner.plan(Planner.ASTAR, cfg)
    runtime_s = time.perf_counter() - start

    return BenchmarkRow(
        planner_id="search2d.astar",
        runtime_s=runtime_s,
        nodes_expanded=int(result.nodes_expanded or 0),
        path_found=bool(result.path),
    )


def _benchmark_sampling2d_rrt() -> BenchmarkRow:
    planner = Rrt((2, 2), (49, 24), step_len=0.5, goal_sample_rate=0.05, iter_max=3000)

    start = time.perf_counter()
    path = planner.planning()
    runtime_s = time.perf_counter() - start

    return BenchmarkRow(
        planner_id="sampling2d.rrt",
        runtime_s=runtime_s,
        nodes_expanded=len(planner.vertex),
        path_found=path is not None,
    )


def _benchmark_search3d_astar() -> BenchmarkRow:
    planner = Weighted_A_star(resolution=1.0)

    start = time.perf_counter()
    with contextlib.redirect_stdout(io.StringIO()):
        ok = planner.run(N=500)
    runtime_s = time.perf_counter() - start

    return BenchmarkRow(
        planner_id="search3d.astar",
        runtime_s=runtime_s,
        nodes_expanded=len(planner.CLOSED),
        path_found=ok,
    )


def run_benchmarks(seed: int) -> list[BenchmarkRow]:
    """Run representative benchmarks and return rows."""
    random.seed(seed)
    np.random.seed(seed)
    return [
        _benchmark_search2d(),
        _benchmark_sampling2d_rrt(),
        _benchmark_search3d_astar(),
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
