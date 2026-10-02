"""Compare native-model and callback space paths with the shared v1 contract."""

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
from pathplanning.spaces.grid2d import Grid2DSamplingSpace

_PLANNERS = (
    "rrt",
    "rrt_star",
    "informed_rrt_star",
    "fmt_star",
    "bit_star",
    "abit_star",
    "rrt_connect",
)
_EXACT_GOAL_PLANNERS = frozenset(
    {"informed_rrt_star", "fmt_star", "bit_star", "abit_star", "rrt_connect"}
)
_BOUNDS = {"x_range": (0.0, 10.0), "y_range": (0.0, 10.0)}
_OBSTACLES = {"obs_rectangle": ((4.0, 4.0, 2.0, 2.0),)}
_STEP_SIZE = 0.5
_COLLISION_STEP = 0.25
_RADIUS_GAMMA = 8.0


class _CallbackGrid2DSamplingSpace(Grid2DSamplingSpace):
    """Force the public Python-space callback path with identical behavior."""


def _workloads(max_iters: int, sample_count: int) -> list[dict[str, Any]]:
    return [
        {
            "case_id": planner,
            "planner": planner,
            "scenario": {
                "space": "Grid2DSamplingSpace",
                **_BOUNDS,
                **_OBSTACLES,
                "obs_boundary": (),
                "obs_circle": (),
                "delta": 0.5,
                "collision_step": _COLLISION_STEP,
                "max_sample_tries": 10_000,
                "start": (1.0, 1.0),
                "goal": (9.0, 9.0),
                "goal_policy": (
                    "exact_euclidean" if planner in _EXACT_GOAL_PLANNERS else "radius_0.25"
                ),
            },
            "params": {
                "max_iters": max_iters,
                "step_size": _STEP_SIZE,
                "goal_sample_rate": 0.08,
                "time_budget_s": None,
                "max_sample_tries": 2_000,
                "collision_step": _COLLISION_STEP,
                "goal_reach_tolerance": 1e-9,
                "rrt_star_radius_gamma": _RADIUS_GAMMA,
                "rrt_star_radius_bias": 1.0,
                "rrt_star_radius_max_factor": 6.0,
                "sample_count": sample_count,
                "batch_size": max(16, sample_count // 4),
                "abit_inflation_parameter": 10.0,
                "abit_truncation_parameter": 5.0,
            },
            "variants": {
                "native_model": {"allow_python_callbacks": False},
                "python_callbacks": {"allow_python_callbacks": True},
            },
        }
        for planner in _PLANNERS
    ]


def _run_once(
    planner: str,
    *,
    callbacks: bool,
    seed: int,
    max_iters: int,
    sample_count: int,
) -> dict[str, Any]:
    space_type = _CallbackGrid2DSamplingSpace if callbacks else Grid2DSamplingSpace
    space = space_type(
        **_BOUNDS,
        **_OBSTACLES,
        obs_boundary=(),
        obs_circle=(),
        delta=0.5,
        collision_step=_COLLISION_STEP,
        max_sample_tries=10_000,
    )
    problem = ContinuousProblem(
        space=space,
        start=np.asarray([1.0, 1.0], dtype=np.float64),
        goal=GoalState(
            state=np.asarray([9.0, 9.0], dtype=np.float64),
            radius=0.0 if planner in _EXACT_GOAL_PLANNERS else 0.25,
            distance_fn=None if planner in _EXACT_GOAL_PLANNERS else space.distance,
        ),
    )
    params = RrtParams(
        max_iters=max_iters,
        step_size=_STEP_SIZE,
        goal_sample_rate=0.08,
        time_budget_s=None,
        max_sample_tries=2_000,
        collision_step=_COLLISION_STEP,
        goal_reach_tolerance=1e-9,
        rrt_star_radius_gamma=_RADIUS_GAMMA,
        rrt_star_radius_bias=1.0,
        rrt_star_radius_max_factor=6.0,
        sample_count=sample_count,
        batch_size=max(16, sample_count // 4),
        abit_inflation_parameter=10.0,
        abit_truncation_parameter=5.0,
        allow_python_callbacks=callbacks,
    )

    started = time.perf_counter()
    result = plan_continuous(problem, planner=planner, params=params, seed=seed)
    full_api_s = time.perf_counter() - started
    native_kernel_s = float(result.stats["elapsed_s"])
    return {
        "measurements": {
            "full_api_s": full_api_s,
            "native_kernel_s": native_kernel_s,
            "setup_and_ffi_s": max(0.0, full_api_s - native_kernel_s),
            "nodes": result.nodes,
            "path_cost": result.stats.get("path_cost"),
        },
        "outcomes": {"success": result.success},
        "details": {"python_callbacks": int(result.stats["python_callbacks"])},
    }


def _run_validated(
    planner: str,
    *,
    mode: str,
    callbacks: bool,
    seed: int,
    max_iters: int,
    sample_count: int,
) -> dict[str, Any]:
    observation = _run_once(
        planner,
        callbacks=callbacks,
        seed=seed,
        max_iters=max_iters,
        sample_count=sample_count,
    )
    expected_callbacks = int(mode == "python_callbacks")
    if observation["details"]["python_callbacks"] != expected_callbacks:
        raise RuntimeError(f"{planner}/{mode} used an unexpected execution path")
    return observation


def _benchmark_pair(
    planner: str,
    *,
    seed: int,
    repeats: int,
    warmups: int,
    max_iters: int,
    sample_count: int,
) -> list[dict[str, Any]]:
    modes = (("native_model", False), ("python_callbacks", True))
    records = []

    for warmup_index in range(warmups):
        warmup_seed = seed + repeats + warmup_index
        for mode, callbacks in modes:
            records.append(
                execute_run(
                    case_id=planner,
                    variant_id=mode,
                    phase="warmup",
                    repeat_index=warmup_index,
                    seed=warmup_seed,
                    run=lambda mode=mode, callbacks=callbacks, seed=warmup_seed: _run_validated(
                        planner,
                        mode=mode,
                        callbacks=callbacks,
                        seed=seed,
                        max_iters=max_iters,
                        sample_count=sample_count,
                    ),
                )
            )

    for repeat_index in range(repeats):
        run_seed = seed + repeat_index
        ordered_modes = modes if repeat_index % 2 == 0 else tuple(reversed(modes))
        for mode, callbacks in ordered_modes:
            records.append(
                execute_run(
                    case_id=planner,
                    variant_id=mode,
                    phase="measure",
                    repeat_index=repeat_index,
                    seed=run_seed,
                    run=lambda mode=mode, callbacks=callbacks, seed=run_seed: _run_validated(
                        planner,
                        mode=mode,
                        callbacks=callbacks,
                        seed=seed,
                        max_iters=max_iters,
                        sample_count=sample_count,
                    ),
                )
            )
    return records


def run_benchmarks(
    *,
    seed: int = 7,
    repeats: int = 5,
    warmups: int = 1,
    max_iters: int = 1_000,
    sample_count: int = 256,
    output_path: str | Path | None = None,
) -> dict[str, Any]:
    """Run registry sampling planners through both equivalent space paths."""
    if repeats <= 0 or warmups < 0 or max_iters <= 0 or sample_count <= 0:
        raise ValueError("repeats/max_iters/sample_count must be positive and warmups non-negative")
    started_at_utc = utc_now()
    started_monotonic = time.perf_counter()
    runs = [
        run
        for planner in _PLANNERS
        for run in _benchmark_pair(
            planner,
            seed=seed,
            repeats=repeats,
            warmups=warmups,
            max_iters=max_iters,
            sample_count=sample_count,
        )
    ]
    return create_report(
        benchmark_id="native_sampling_paths",
        settings={
            "seed": seed,
            "repeats": repeats,
            "warmups": warmups,
            "measured_seed_rule": "seed + repeat_index; paired by seed across variants",
            "warmup_seed_rule": "seed + repeats + warmup_index; paired by seed across variants",
            "max_iters": max_iters,
            "sample_count": sample_count,
            "batch_size": max(16, sample_count // 4),
            "step_size": _STEP_SIZE,
            "goal_sample_rate": 0.08,
            "max_sample_tries": 2_000,
            "collision_step": _COLLISION_STEP,
            "rrt_star_radius_gamma": _RADIUS_GAMMA,
            "bounds": _BOUNDS,
            "obstacles": _OBSTACLES,
            "paired_order": "alternate variant order on each measured repeat",
        },
        workloads=_workloads(max_iters, sample_count),
        runs=runs,
        started_at_utc=started_at_utc,
        started_monotonic=started_monotonic,
        output_path=output_path,
    )


def _print_table(report: dict[str, Any]) -> None:
    environment = report["identity"]["environment"]
    print(
        f"Host: {environment['platform']} | "
        f"CPU: {environment['processor'] or environment['machine']}"
    )
    print(f"Python: {environment['python'].split()[0]} | NumPy: {environment['numpy']}")
    print(f"C compiler: {environment['c_compiler']['version']}")
    print(f"C++ compiler: {environment['cxx_compiler']['version']}")
    header = (
        f"{'planner':20} {'space_path':18} {'full_api_med_ms':>15} {'C_kernel_med_ms':>15} "
        f"{'setup_ffi_med_ms':>17} {'success':>9} {'nodes_med':>10} {'runs':>9}"
    )
    print(header)
    print("-" * len(header))
    for row in report["results"]:
        metrics = row["metrics"]

        def median_ms(name: str) -> str:
            metric = metrics.get(name)
            return f"{metric['median'] * 1_000:.3f}" if metric is not None else "n/a"

        api = metrics.get("full_api_s")
        nodes = metrics.get("nodes")
        success = row["success_rate"]
        api_ms = f"{api['median'] * 1_000:.3f}" if api else "n/a"
        nodes_text = f"{nodes['median']:.0f}" if nodes else "n/a"
        print(
            f"{row['case_id']:20} {row['variant_id']:18} "
            f"{api_ms:>15} "
            f"{median_ms('native_kernel_s'):>15} "
            f"{median_ms('setup_and_ffi_s'):>17} "
            f"{(f'{success:.0%}' if success is not None else 'n/a'):>9} "
            f"{nodes_text:>10} "
            f"{row['completed_count']:4d}/{row['attempt_count']:<4d}"
        )
    if report["status"] != "completed":
        print(f"Report status: {report['status']}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare full-API sampling latency for native and callback spaces."
    )
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--warmups", type=int, default=1)
    parser.add_argument("--max-iters", type=int, default=1_000)
    parser.add_argument("--sample-count", type=int, default=256)
    parser.add_argument("--json", action="store_true", help="Print the full JSON report.")
    parser.add_argument("--output", help="Atomically write the full JSON report to this path.")
    args = parser.parse_args()

    report = run_benchmarks(
        seed=args.seed,
        repeats=args.repeats,
        warmups=args.warmups,
        max_iters=args.max_iters,
        sample_count=args.sample_count,
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
