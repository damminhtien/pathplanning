"""Benchmark representative planners using the shared v1 report contract."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from typing import Any

from benchmark_contract import create_report, execute_run, utc_now, write_report
import numpy as np

_REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(_REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPOSITORY_ROOT))

from pathplanning.api import plan_continuous, plan_discrete
from pathplanning.core.contracts import ContinuousProblem, DiscreteProblem, GoalState
from pathplanning.core.params import RrtParams
from pathplanning.spaces.grid2d import Grid2DSamplingSpace, Grid2DSearchSpace
from pathplanning.spaces.grid3d import Grid3DSearchSpace

_MOTIONS_2D = (
    (-1, 0),
    (-1, 1),
    (0, 1),
    (1, 1),
    (1, 0),
    (1, -1),
    (0, -1),
    (-1, -1),
)
_MOTIONS_3D = tuple(
    (dx, dy, dz)
    for dx in (-1, 0, 1)
    for dy in (-1, 0, 1)
    for dz in (-1, 0, 1)
    if (dx, dy, dz) != (0, 0, 0)
)
_CASES: list[dict[str, Any]] = [
    {
        "case_id": "discrete2d.astar",
        "planner": "astar",
        "problem": {
            "space": "Grid2DSearchSpace",
            "width": 51,
            "height": 31,
            "motions": _MOTIONS_2D,
            "obstacles": [],
            "start": (5, 5),
            "goal": (45, 25),
        },
        "params": {"max_expansions": 50_000},
    },
    {
        "case_id": "sampling2d.rrt",
        "planner": "rrt",
        "problem": {
            "space": "Grid2DSamplingSpace",
            "x_range": (0.0, 50.0),
            "y_range": (0.0, 30.0),
            "obstacles": [],
            "delta": 0.5,
            "collision_step": 0.5,
            "max_sample_tries": 10_000,
            "start": (2.0, 2.0),
            "goal": (49.0, 24.0),
            "goal_radius": 0.25,
        },
        "params": {
            "max_iters": 3_000,
            "step_size": 0.5,
            "goal_sample_rate": 0.05,
            "time_budget_s": None,
            "max_sample_tries": 1_000,
            "collision_step": 0.1,
            "goal_reach_tolerance": 1e-9,
            "rrt_star_radius_gamma": 2.0,
            "rrt_star_radius_bias": 1.0,
            "rrt_star_radius_max_factor": 6.0,
            "sample_count": 512,
            "batch_size": 64,
            "abit_inflation_parameter": 10.0,
            "abit_truncation_parameter": 5.0,
            "allow_python_callbacks": False,
        },
    },
    {
        "case_id": "search3d.weighted_astar",
        "planner": "weighted_astar",
        "problem": {
            "space": "Grid3DSearchSpace",
            "width": 21,
            "height": 21,
            "depth": 6,
            "motions": _MOTIONS_3D,
            "obstacles": [],
            "start": (2, 2, 1),
            "goal": (18, 17, 1),
        },
        "params": {"weight": 1.0, "max_expansions": 50_000},
    },
]


def _benchmark_search2d_astar(seed: int) -> dict[str, Any]:
    workload = _CASES[0]
    problem = DiscreteProblem(
        graph=Grid2DSearchSpace(
            width=51,
            height=31,
            motions=_MOTIONS_2D,
            obstacles=(),
        ),
        start=(5, 5),
        goal=(45, 25),
    )
    started = time.perf_counter()
    result = plan_discrete(
        problem,
        planner=workload["planner"],
        params=workload["params"],
        seed=seed,
    )
    return {
        "measurements": {
            "runtime_s": time.perf_counter() - started,
            "nodes_expanded": result.iters,
            "graph_init_s": result.stats.get("graph_init_s", 0.0),
            "native_search_s": result.stats.get("native_search_s", 0.0),
        },
        "outcomes": {"success": result.path is not None},
    }


def _benchmark_sampling2d_rrt(seed: int) -> dict[str, Any]:
    workload = _CASES[1]
    space = Grid2DSamplingSpace(
        x_range=(0.0, 50.0),
        y_range=(0.0, 30.0),
        obs_boundary=(),
        obs_circle=(),
        obs_rectangle=(),
        delta=0.5,
        collision_step=0.5,
        max_sample_tries=10_000,
    )
    problem = ContinuousProblem(
        space=space,
        start=np.asarray([2.0, 2.0], dtype=float),
        goal=GoalState(
            state=np.asarray([49.0, 24.0], dtype=float),
            radius=0.25,
            distance_fn=space.distance,
        ),
    )
    started = time.perf_counter()
    result = plan_continuous(
        problem,
        planner=workload["planner"],
        params=RrtParams(
            max_iters=3_000,
            step_size=0.5,
            goal_sample_rate=0.05,
            time_budget_s=None,
            max_sample_tries=1_000,
            collision_step=0.1,
            goal_reach_tolerance=1e-9,
            rrt_star_radius_gamma=2.0,
            rrt_star_radius_bias=1.0,
            rrt_star_radius_max_factor=6.0,
            sample_count=512,
            batch_size=64,
            abit_inflation_parameter=10.0,
            abit_truncation_parameter=5.0,
            allow_python_callbacks=False,
        ),
        seed=seed,
    )
    return {
        "measurements": {
            "runtime_s": time.perf_counter() - started,
            "nodes_expanded": result.nodes,
        },
        "outcomes": {"success": result.success},
    }


def _benchmark_search3d_weighted_astar(seed: int) -> dict[str, Any]:
    workload = _CASES[2]
    problem = DiscreteProblem(
        graph=Grid3DSearchSpace(
            width=21,
            height=21,
            depth=6,
            motions=_MOTIONS_3D,
            obstacles=(),
        ),
        start=(2, 2, 1),
        goal=(18, 17, 1),
    )
    started = time.perf_counter()
    result = plan_discrete(
        problem,
        planner=workload["planner"],
        params=workload["params"],
        seed=seed,
    )
    return {
        "measurements": {
            "runtime_s": time.perf_counter() - started,
            "nodes_expanded": result.iters,
            "graph_init_s": result.stats.get("graph_init_s", 0.0),
            "native_search_s": result.stats.get("native_search_s", 0.0),
        },
        "outcomes": {"success": result.path is not None},
    }


_BENCHMARKS = (
    ("discrete2d.astar", _benchmark_search2d_astar),
    ("sampling2d.rrt", _benchmark_sampling2d_rrt),
    ("search3d.weighted_astar", _benchmark_search3d_weighted_astar),
)


def run_benchmarks(
    seed: int = 0,
    repeats: int = 5,
    warmups: int = 1,
    output_path: str | Path | None = None,
) -> dict[str, Any]:
    """Run every representative planner and return its raw and summarized report."""
    if repeats <= 0 or warmups < 0:
        raise ValueError("repeats must be > 0 and warmups must be >= 0")

    started_at_utc = utc_now()
    started_monotonic = time.perf_counter()
    runs = []
    for case_id, benchmark in _BENCHMARKS:
        for repeat_index in range(warmups):
            warmup_seed = seed + repeats + repeat_index
            runs.append(
                execute_run(
                    case_id=case_id,
                    variant_id="default",
                    phase="warmup",
                    repeat_index=repeat_index,
                    seed=warmup_seed,
                    run=lambda benchmark=benchmark, seed=warmup_seed: benchmark(seed),
                )
            )
        for repeat_index in range(repeats):
            run_seed = seed + repeat_index
            runs.append(
                execute_run(
                    case_id=case_id,
                    variant_id="default",
                    phase="measure",
                    repeat_index=repeat_index,
                    seed=run_seed,
                    run=lambda benchmark=benchmark, seed=run_seed: benchmark(seed),
                )
            )

    return create_report(
        benchmark_id="representative_planners",
        settings={
            "seed": seed,
            "repeats": repeats,
            "warmups": warmups,
            "measured_seed_rule": "seed + repeat_index",
            "warmup_seed_rule": "seed + repeats + warmup_index",
        },
        workloads=_CASES,
        runs=runs,
        started_at_utc=started_at_utc,
        started_monotonic=started_monotonic,
        output_path=output_path,
    )


def _print_table(report: dict[str, Any]) -> None:
    header = (
        f"{'case_id':28} {'runtime_med_ms':>15} {'graph_init_med_ms':>18} "
        f"{'native_search_med_ms':>21} {'nodes_med':>10} {'success':>9} {'runs':>9}"
    )
    print(header)
    print("-" * len(header))
    for row in report["results"]:
        metrics = row["metrics"]

        def median_ms(name: str) -> str:
            metric = metrics.get(name)
            return f"{metric['median'] * 1_000:.3f}" if metric is not None else "n/a"

        runtime = metrics.get("runtime_s")
        nodes = metrics.get("nodes_expanded")
        success = row["success_rate"]
        runtime_ms = f"{runtime['median'] * 1_000:.3f}" if runtime else "n/a"
        nodes_text = f"{nodes['median']:.0f}" if nodes else "n/a"
        print(
            f"{row['case_id']:28} "
            f"{runtime_ms:>15} "
            f"{median_ms('graph_init_s'):>18} "
            f"{median_ms('native_search_s'):>21} "
            f"{nodes_text:>10} "
            f"{(f'{success:.0%}' if success is not None else 'n/a'):>9} "
            f"{row['completed_count']:4d}/{row['attempt_count']:<4d}"
        )
    if report["status"] != "completed":
        print(f"Report status: {report['status']}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Benchmark representative path planners.")
    parser.add_argument("--seed", type=int, default=0, help="Base seed for measured runs.")
    parser.add_argument("--repeats", type=int, default=5, help="Measured runs per planner.")
    parser.add_argument("--warmups", type=int, default=1, help="Warm-up runs per planner.")
    parser.add_argument("--json", action="store_true", help="Print the full JSON report.")
    parser.add_argument("--output", help="Atomically write the full JSON report to this path.")
    args = parser.parse_args()

    report = run_benchmarks(
        seed=args.seed,
        repeats=args.repeats,
        warmups=args.warmups,
        output_path=args.output,
    )
    if args.output:
        write_report(args.output, report)
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
        return
    _print_table(report)


if __name__ == "__main__":
    main()
