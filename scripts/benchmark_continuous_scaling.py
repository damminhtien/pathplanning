"""Measure DynamicRRT session growth and continuous obstacle scaling."""

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

from pathplanning.api import plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.core.params import RrtParams
from pathplanning.native.continuous import run_native_dynamic_rrt
from pathplanning.planners.sampling.dynamic_rrt import DynamicRRT3D, DynamicRRT3DConfig
from pathplanning.spaces.continuous_3d import ContinuousSpace3D
from pathplanning.spaces.grid2d import Grid2DSamplingSpace

_DYNAMIC_START = (1.0, 1.0, 1.0)
_DYNAMIC_GOAL = (19.0, 19.0, 5.0)
_DYNAMIC_STEP_SIZE = 0.2
_DYNAMIC_GOAL_RATE = 0.005
_OBSTACLE_COUNTS = (0, 10, 100, 500)
_OBSTACLE_GENERATOR_SEED = 20_261_018
_OBSTACLE_START = (1.0, 1.0)
_OBSTACLE_GOAL = (19.0, 19.0)


def _dynamic_workload(growth_calls: int, growth_iterations: int) -> dict[str, Any]:
    return {
        "case_id": "dynamic_rrt.repeated_growth",
        "planner": "DynamicRRT3D",
        "scenario": {
            "space": "ContinuousSpace3D",
            "bounds": ((0.0, 0.0, 0.0), (20.0, 20.0, 6.0)),
            "obstacles": (),
            "start": _DYNAMIC_START,
            "goal": _DYNAMIC_GOAL,
        },
        "params": {
            "growth_calls": growth_calls,
            "max_iterations_per_call": growth_iterations,
            "step_size": _DYNAMIC_STEP_SIZE,
            "goal_sample_probability": _DYNAMIC_GOAL_RATE,
            "way_point_sample_probability": 0.0,
        },
        "variants": {
            "persistent_session": {"tree_copy_policy": "append node deltas"},
            "one_shot_round_trip": {"tree_copy_policy": "full snapshot each call"},
        },
    }


def _obstacle_layout(count: int) -> tuple[tuple[float, float, float, float], ...]:
    rng = np.random.default_rng(_OBSTACLE_GENERATOR_SEED)
    rectangles: list[tuple[float, float, float, float]] = []
    while len(rectangles) < count:
        x, y = rng.uniform(0.25, 19.5, size=2)
        if np.linalg.norm((x - _OBSTACLE_START[0], y - _OBSTACLE_START[1])) < 1.0:
            continue
        if np.linalg.norm((x - _OBSTACLE_GOAL[0], y - _OBSTACLE_GOAL[1])) < 1.0:
            continue
        rectangles.append((float(x), float(y), 0.08, 0.08))
    return tuple(rectangles)


def _obstacle_workload(count: int, iterations: int) -> dict[str, Any]:
    return {
        "case_id": f"continuous_rrt.obstacles_{count:04d}",
        "planner": "rrt",
        "scenario": {
            "space": "Grid2DSamplingSpace",
            "bounds": ((0.0, 0.0), (20.0, 20.0)),
            "obstacle_count": count,
            "obstacle_generator": {
                "algorithm": "numpy.default_rng uniform rectangles",
                "seed": _OBSTACLE_GENERATOR_SEED,
                "rectangle_size": (0.08, 0.08),
                "endpoint_clearance": 1.0,
            },
            "start": _OBSTACLE_START,
            "goal": _OBSTACLE_GOAL,
        },
        "params": {
            "max_iters": iterations,
            "step_size": 0.4,
            "goal_sample_rate": 0.05,
            "collision_step": 0.25,
            "allow_python_callbacks": False,
        },
        "variants": {"native_model": {"execution_path": "native broadphase"}},
    }


def _run_dynamic_sequence(
    variant: str,
    *,
    seed: int,
    growth_calls: int,
    growth_iterations: int,
) -> dict[str, Any]:
    space = ContinuousSpace3D(
        lower_bound=np.asarray([0.0, 0.0, 0.0]),
        upper_bound=np.asarray([20.0, 20.0, 6.0]),
    )
    config = DynamicRRT3DConfig(
        step_size=_DYNAMIC_STEP_SIZE,
        max_iterations=growth_iterations,
        goal_sample_probability=_DYNAMIC_GOAL_RATE,
        way_point_sample_probability=0.0,
    )
    planner = DynamicRRT3D.with_seed(
        seed,
        environment=space,
        config=config,
        nearest_index=None,
    )
    planner.x0 = tuple(_DYNAMIC_START)
    planner.xt = tuple(_DYNAMIC_GOAL)
    planner.init_rrt()
    growth_times: list[float] = []
    tree_sizes: list[int] = []
    success = False

    for _ in range(growth_calls):
        started = time.perf_counter()
        if variant == "persistent_session":
            planner.grow_rrt()
        else:
            states, parents, success, _iterations, _invalid = run_native_dynamic_rrt(
                planner.space,
                planner.x0,
                planner.xt,
                planner.nodes,
                planner._native_parent_ids(),
                planner.config,
                planner.rng,
                max_iterations=growth_iterations,
            )
            planner._adopt_native_tree(states, parents)
            planner.done = success
        growth_times.append(time.perf_counter() - started)
        tree_sizes.append(len(planner.nodes))
        success = success or planner.done

    tail = growth_times[max(0, len(growth_times) - 3) :]
    return {
        "measurements": {
            "total_growth_s": sum(growth_times),
            "first_growth_s": growth_times[0],
            "last_growth_s": growth_times[-1],
            "tail_mean_growth_s": float(np.mean(tail)),
            "final_nodes": len(planner.nodes),
            "mean_nodes_added_per_call": (len(planner.nodes) - 1) / growth_calls,
        },
        "outcomes": {"success": success},
        "details": {
            "growth_times_s": growth_times,
            "tree_sizes": tree_sizes,
            "execution_variant": variant,
        },
    }


def _run_obstacle_case(count: int, *, seed: int, iterations: int) -> dict[str, Any]:
    rectangles = _obstacle_layout(count)
    space = Grid2DSamplingSpace(
        x_range=(0.0, 20.0),
        y_range=(0.0, 20.0),
        obs_boundary=(),
        obs_circle=(),
        obs_rectangle=rectangles,
        delta=0.0,
        collision_step=0.25,
    )
    problem = ContinuousProblem(
        space=space,
        start=np.asarray(_OBSTACLE_START, dtype=np.float64),
        goal=GoalState(
            state=np.asarray(_OBSTACLE_GOAL, dtype=np.float64),
            radius=0.4,
            distance_fn=space.distance,
        ),
    )
    started = time.perf_counter()
    result = plan_continuous(
        problem,
        planner="rrt",
        params=RrtParams(
            max_iters=iterations,
            step_size=0.4,
            goal_sample_rate=0.05,
            max_sample_tries=2_000,
            collision_step=0.25,
            allow_python_callbacks=False,
        ),
        seed=seed,
    )
    full_api_s = time.perf_counter() - started
    native_s = float(result.stats["elapsed_s"])
    checks = int(result.stats["motion_checks"])
    return {
        "measurements": {
            "full_api_s": full_api_s,
            "native_kernel_s": native_s,
            "setup_and_ffi_s": max(0.0, full_api_s - native_s),
            "nodes": result.nodes,
            "motion_checks": checks,
            "motion_checks_per_s": checks / native_s if native_s > 0 else 0.0,
            "path_cost": result.stats.get("path_cost"),
        },
        "outcomes": {"success": result.success},
        "details": {"obstacle_count": count},
    }


def run_benchmarks(
    *,
    seed: int = 11,
    repeats: int = 5,
    warmups: int = 1,
    growth_calls: int = 8,
    growth_iterations: int = 100,
    obstacle_counts: tuple[int, ...] = _OBSTACLE_COUNTS,
    obstacle_iterations: int = 1_500,
    output_path: str | Path | None = None,
) -> dict[str, Any]:
    """Run repeated DynamicRRT growth and obstacle-count scaling workloads."""
    if repeats <= 0 or warmups < 0 or growth_calls <= 0 or growth_iterations <= 0:
        raise ValueError("repeats/growth_calls/growth_iterations must be positive")
    if obstacle_iterations <= 0 or any(count < 0 for count in obstacle_counts):
        raise ValueError("obstacle iterations must be positive and counts non-negative")

    started_at_utc = utc_now()
    started_monotonic = time.perf_counter()
    dynamic = _dynamic_workload(growth_calls, growth_iterations)
    workloads = [dynamic] + [
        _obstacle_workload(count, obstacle_iterations) for count in obstacle_counts
    ]
    runs = []
    for warmup_index in range(warmups):
        warmup_seed = seed + repeats + warmup_index
        for variant in dynamic["variants"]:
            runs.append(
                execute_run(
                    case_id=dynamic["case_id"],
                    variant_id=variant,
                    phase="warmup",
                    repeat_index=warmup_index,
                    seed=warmup_seed,
                    run=lambda variant=variant, warmup_seed=warmup_seed: _run_dynamic_sequence(
                        variant,
                        seed=warmup_seed,
                        growth_calls=growth_calls,
                        growth_iterations=growth_iterations,
                    ),
                )
            )
        for count in obstacle_counts:
            case_id = f"continuous_rrt.obstacles_{count:04d}"
            runs.append(
                execute_run(
                    case_id=case_id,
                    variant_id="native_model",
                    phase="warmup",
                    repeat_index=warmup_index,
                    seed=warmup_seed,
                    run=lambda count=count, warmup_seed=warmup_seed: _run_obstacle_case(
                        count, seed=warmup_seed, iterations=obstacle_iterations
                    ),
                )
            )

    for repeat_index in range(repeats):
        run_seed = seed + repeat_index
        ordered_variants = tuple(dynamic["variants"])
        if repeat_index % 2:
            ordered_variants = tuple(reversed(ordered_variants))
        for variant in ordered_variants:
            runs.append(
                execute_run(
                    case_id=dynamic["case_id"],
                    variant_id=variant,
                    phase="measure",
                    repeat_index=repeat_index,
                    seed=run_seed,
                    run=lambda variant=variant, run_seed=run_seed: _run_dynamic_sequence(
                        variant,
                        seed=run_seed,
                        growth_calls=growth_calls,
                        growth_iterations=growth_iterations,
                    ),
                )
            )
        for count in obstacle_counts:
            case_id = f"continuous_rrt.obstacles_{count:04d}"
            runs.append(
                execute_run(
                    case_id=case_id,
                    variant_id="native_model",
                    phase="measure",
                    repeat_index=repeat_index,
                    seed=run_seed,
                    run=lambda count=count, run_seed=run_seed: _run_obstacle_case(
                        count, seed=run_seed, iterations=obstacle_iterations
                    ),
                )
            )

    return create_report(
        benchmark_id="continuous_scaling",
        settings={
            "seed": seed,
            "repeats": repeats,
            "warmups": warmups,
            "measured_seed_rule": "seed + repeat_index",
            "warmup_seed_rule": "seed + repeats + warmup_index",
            "growth_calls": growth_calls,
            "growth_iterations_per_call": growth_iterations,
            "obstacle_counts": obstacle_counts,
            "obstacle_iterations": obstacle_iterations,
            "obstacle_generator_seed": _OBSTACLE_GENERATOR_SEED,
            "dynamic_variant_order": "alternate on each measured repeat",
        },
        workloads=workloads,
        runs=runs,
        started_at_utc=started_at_utc,
        started_monotonic=started_monotonic,
        output_path=output_path,
    )


def _print_table(report: dict[str, Any]) -> None:
    print(f"Host: {report['identity']['environment']['platform']}")
    print(
        f"{'case_id':40} {'variant':24} {'wall_med_ms':>12} {'nodes_med':>10} {'checks_med':>11} {'runs':>9}"
    )
    print("-" * 105)
    for row in report["results"]:
        metrics = row["metrics"]
        wall_name = "tail_mean_growth_s" if row["case_id"].startswith("dynamic_") else "full_api_s"
        wall = metrics.get(wall_name)
        nodes = metrics.get("final_nodes") or metrics.get("nodes")
        checks = metrics.get("motion_checks")
        print(
            f"{row['case_id']:40} {row['variant_id']:24} "
            f"{(wall['median'] * 1_000 if wall else 0.0):12.3f} "
            f"{(nodes['median'] if nodes else 0.0):10.0f} "
            f"{(checks['median'] if checks else 0.0):11.0f} "
            f"{row['completed_count']:4d}/{row['attempt_count']:<4d}"
        )
    if report["status"] != "completed":
        print(f"Report status: {report['status']}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Benchmark persistent DynamicRRT growth and continuous obstacle scaling."
    )
    parser.add_argument("--seed", type=int, default=11)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--warmups", type=int, default=1)
    parser.add_argument("--growth-calls", type=int, default=8)
    parser.add_argument("--growth-iterations", type=int, default=100)
    parser.add_argument("--obstacle-counts", type=int, nargs="+", default=list(_OBSTACLE_COUNTS))
    parser.add_argument("--obstacle-iterations", type=int, default=1_500)
    parser.add_argument("--json", action="store_true", help="Print the full JSON report.")
    parser.add_argument("--output", help="Atomically write the full JSON report to this path.")
    args = parser.parse_args()
    report = run_benchmarks(
        seed=args.seed,
        repeats=args.repeats,
        warmups=args.warmups,
        growth_calls=args.growth_calls,
        growth_iterations=args.growth_iterations,
        obstacle_counts=tuple(args.obstacle_counts),
        obstacle_iterations=args.obstacle_iterations,
        output_path=args.output,
    )
    if args.output:
        write_report(args.output, report)
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    else:
        _print_table(report)


if __name__ == "__main__":
    main()
