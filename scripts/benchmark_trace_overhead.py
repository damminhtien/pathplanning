"""Measure optional native tracing through the public planning API.

Each observation runs in a fresh process so peak RSS belongs to one variant.
Production and diagnostic runs use the same seed and fixed work limits.
"""

from __future__ import annotations

import argparse
import hashlib
from importlib.machinery import EXTENSION_SUFFIXES
import json
from pathlib import Path
import platform
import resource
import statistics
import subprocess
import sys
import time
from typing import Any

from benchmark_contract import create_report, execute_run, utc_now, write_report
import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from pathplanning.api import plan_continuous, plan_discrete
from pathplanning.core.contracts import ContinuousProblem, DiscreteProblem, GoalState
from pathplanning.core.params import RrtParams
from pathplanning.core.trace import TraceOptions
from pathplanning.spaces.grid2d import Grid2DSamplingSpace, Grid2DSearchSpace

_CASES = ("discrete2d.astar", "sampling2d.rrt_star")
_VARIANTS = ("production", "diagnostic")


def _peak_rss_bytes() -> int:
    maximum = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return int(maximum if platform.system() == "Darwin" else maximum * 1024)


def _artifacts() -> list[dict[str, Any]]:
    """Include both production and diagnostic native binaries in report identity."""
    native = _ROOT / "pathplanning" / "native"
    result = []
    for stem in (
        "_search_engine",
        "_search_trace_engine",
        "_continuous_engine",
        "_continuous_trace_engine",
    ):
        for suffix in EXTENSION_SUFFIXES:
            path = native / f"{stem}{suffix}"
            if path.is_file():
                digest = hashlib.sha256(path.read_bytes()).hexdigest()
                result.append(
                    {
                        "path": str(path.relative_to(_ROOT)),
                        "sha256": digest,
                        "size_bytes": path.stat().st_size,
                    }
                )
    return sorted(result, key=lambda item: item["path"])


def _discrete_problem() -> DiscreteProblem[tuple[int, int]]:
    # A wall with a gap makes the frontier nontrivial while keeping graph
    # construction and search deterministic across variants.
    blocked = [(40, y) for y in range(81) if not 33 <= y <= 38]
    return DiscreteProblem(
        graph=Grid2DSearchSpace(width=81, height=81, obstacles=blocked),
        start=(5, 5),
        goal=(75, 75),
    )


def _continuous_problem() -> ContinuousProblem[np.ndarray]:
    space = Grid2DSamplingSpace(
        x_range=(0.0, 10.0),
        y_range=(0.0, 10.0),
        obs_boundary=(),
        obs_circle=(),
        obs_rectangle=((4.0, 4.0, 2.0, 2.0),),
        delta=0.5,
        collision_step=0.25,
        max_sample_tries=10_000,
    )
    return ContinuousProblem(
        space=space,
        start=np.asarray([1.0, 1.0], dtype=np.float64),
        goal=GoalState(
            state=np.asarray([9.0, 9.0], dtype=np.float64),
            radius=0.25,
            distance_fn=space.distance,
        ),
    )


def _run_worker(
    case: str,
    variant: str,
    seed: int,
    max_expansions: int,
    max_iters: int,
    trace_max_bytes: int,
) -> dict[str, Any]:
    trace = TraceOptions(max_bytes=trace_max_bytes) if variant == "diagnostic" else None
    if case == "discrete2d.astar":
        problem = _discrete_problem()
        started = time.perf_counter()
        result = plan_discrete(
            problem,
            planner="astar",
            params={"max_expansions": max_expansions},
            seed=seed,
            trace=trace,
        )
        full_api_s = time.perf_counter() - started
        native_stage_s = float(result.stats["native_search_s"])
    elif case == "sampling2d.rrt_star":
        problem = _continuous_problem()
        params = RrtParams(
            max_iters=max_iters,
            step_size=0.5,
            goal_sample_rate=0.08,
            time_budget_s=None,
            max_sample_tries=2_000,
            collision_step=0.25,
            goal_reach_tolerance=1e-9,
            rrt_star_radius_gamma=8.0,
            rrt_star_radius_bias=1.0,
            rrt_star_radius_max_factor=6.0,
            allow_python_callbacks=False,
        )
        started = time.perf_counter()
        result = plan_continuous(
            problem,
            planner="rrt_star",
            params=params,
            seed=seed,
            trace=trace,
        )
        full_api_s = time.perf_counter() - started
        native_stage_s = float(result.stats["elapsed_s"])
    else:
        raise ValueError(f"unknown case: {case}")

    if (result.trace is None) != (variant == "production"):
        raise RuntimeError(f"{case}/{variant} returned an unexpected trace")
    planner_trace = result.trace
    trace_bytes = 0
    event_count = 0
    truncated = False
    graph_bytes = 0
    if planner_trace is not None:
        trace_bytes = int(planner_trace.events.nbytes)
        if planner_trace.points is not None:
            trace_bytes += int(planner_trace.points.nbytes)
        event_count = len(planner_trace.events)
        truncated = planner_trace.truncated
        graph_bytes = planner_trace.graph_bytes
        if truncated:
            raise RuntimeError(f"{case} trace exceeded the configured {trace_max_bytes} byte cap")
        if trace_bytes > trace_max_bytes:
            raise RuntimeError(f"{case} trace payload exceeded the native memory cap")

    stats_to_match = ("sample_count", "batches", "motion_checks", "rewires")
    signature = {
        "success": result.success,
        "stop_reason": result.stop_reason.value,
        "iters": result.iters,
        "nodes": result.nodes,
        "path": None if result.path is None else np.asarray(result.path).tolist(),
        "path_cost": result.stats.get("path_cost"),
        "planner_counters": {
            name: result.stats[name] for name in stats_to_match if name in result.stats
        },
    }
    return {
        "measurements": {
            "full_api_s": full_api_s,
            "native_stage_s": native_stage_s,
            "peak_rss_bytes": _peak_rss_bytes(),
            "trace_payload_bytes": trace_bytes,
            "trace_graph_bytes": graph_bytes,
            "trace_event_count": event_count,
            "iters": result.iters,
            "nodes": result.nodes,
        },
        "outcomes": {"success": result.success, "trace_truncated": truncated},
        "details": {"signature": signature},
    }


def _subprocess_observation(
    case: str,
    variant: str,
    seed: int,
    max_expansions: int,
    max_iters: int,
    trace_max_bytes: int,
) -> dict[str, Any]:
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--worker-case",
        case,
        "--worker-variant",
        variant,
        "--seed",
        str(seed),
        "--max-expansions",
        str(max_expansions),
        "--max-iters",
        str(max_iters),
        "--trace-max-bytes",
        str(trace_max_bytes),
    ]
    completed = subprocess.run(
        command, cwd=_ROOT, capture_output=True, text=True, timeout=120, check=False
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"{case}/{variant} worker exited {completed.returncode}: {completed.stderr.strip()}"
        )
    return json.loads(completed.stdout)


def run_benchmarks(
    *,
    seed: int = 7,
    repeats: int = 5,
    warmups: int = 1,
    max_expansions: int = 20_000,
    max_iters: int = 1_000,
    trace_max_bytes: int = 32 * 1024 * 1024,
    output_path: str | Path | None = None,
) -> dict[str, Any]:
    if repeats <= 0 or warmups < 0 or max_expansions <= 0 or max_iters <= 0:
        raise ValueError("repeats and work limits must be positive; warmups must be non-negative")
    TraceOptions(trace_max_bytes)
    started_at = utc_now()
    started = time.perf_counter()
    runs: list[dict[str, Any]] = []
    for case in _CASES:
        for phase, count, seed_base in (
            ("warmup", warmups, seed + repeats),
            ("measure", repeats, seed),
        ):
            for repeat_index in range(count):
                run_seed = seed_base + repeat_index
                ordered = _VARIANTS if repeat_index % 2 == 0 else _VARIANTS[::-1]
                for variant in ordered:
                    runs.append(
                        execute_run(
                            case_id=case,
                            variant_id=variant,
                            phase=phase,
                            repeat_index=repeat_index,
                            seed=run_seed,
                            run=lambda case=case, variant=variant, run_seed=run_seed: (
                                _subprocess_observation(
                                    case,
                                    variant,
                                    run_seed,
                                    max_expansions,
                                    max_iters,
                                    trace_max_bytes,
                                )
                            ),
                        )
                    )

    paired_count = 0
    mismatches = []
    for case in _CASES:
        for repeat_index in range(repeats):
            pair = [
                run
                for run in runs
                if run["phase"] == "measure"
                and run["case_id"] == case
                and run["repeat_index"] == repeat_index
            ]
            if len(pair) != 2 or any(run["status"] != "completed" for run in pair):
                mismatches.append(
                    {
                        "case_id": case,
                        "repeat_index": repeat_index,
                        "reason": "missing or failed variant",
                    }
                )
                continue
            production = next(run for run in pair if run["variant_id"] == "production")
            diagnostic = next(run for run in pair if run["variant_id"] == "diagnostic")
            if production["details"]["signature"] != diagnostic["details"]["signature"]:
                mismatches.append(
                    {
                        "case_id": case,
                        "repeat_index": repeat_index,
                        "reason": "planner output differs",
                    }
                )
            else:
                paired_count += 1

    report = create_report(
        benchmark_id="visualization_trace_overhead",
        settings={
            "seed": seed,
            "repeats": repeats,
            "warmups": warmups,
            "max_expansions": max_expansions,
            "max_iters": max_iters,
            "trace_max_bytes": trace_max_bytes,
            "measurement_process": "one fresh subprocess per variant and seed",
            "native_stage_definition": {
                "discrete2d.astar": "Python timer around graph clone plus C++ search call",
                "sampling2d.rrt_star": "C planner elapsed_s timer",
            },
            "rss_unit": "bytes; process peak includes Python imports and native libraries",
            "native_artifacts_including_diagnostic": _artifacts(),
        },
        workloads=[
            {
                "case_id": "discrete2d.astar",
                "planner": "astar",
                "space": "81x81 Grid2DSearchSpace",
                "wall": "x=40, y=0..80 except y=33..38",
                "start": (5, 5),
                "goal": (75, 75),
            },
            {
                "case_id": "sampling2d.rrt_star",
                "planner": "rrt_star",
                "space": "10x10 Grid2DSamplingSpace with one 2x2 rectangle",
                "start": (1.0, 1.0),
                "goal": (9.0, 9.0),
                "step_size": 0.5,
            },
        ],
        runs=runs,
        started_at_utc=started_at,
        started_monotonic=started,
        output_path=output_path,
    )
    report["paired_validation"] = {
        "matched_count": paired_count,
        "expected_count": len(_CASES) * repeats,
        "mismatches": mismatches,
    }
    report["trace_summary"] = []
    for case in _CASES:
        for variant in _VARIANTS:
            observations = [
                run["measurements"]
                for run in runs
                if run["phase"] == "measure"
                and run["case_id"] == case
                and run["variant_id"] == variant
                and run["status"] == "completed"
            ]
            metrics = {}
            for name in (
                "full_api_s",
                "native_stage_s",
                "peak_rss_bytes",
                "trace_payload_bytes",
                "trace_graph_bytes",
                "trace_event_count",
            ):
                values = [float(observation[name]) for observation in observations]
                if values:
                    mean = statistics.fmean(values)
                    standard_deviation = statistics.stdev(values) if len(values) > 1 else 0.0
                    metrics[name] = {
                        "median": statistics.median(values),
                        "p95": float(np.percentile(values, 95)),
                        "mean": mean,
                        "stdev": standard_deviation,
                        "coefficient_of_variation_pct": (
                            standard_deviation / mean * 100.0 if mean else 0.0
                        ),
                    }
            report["trace_summary"].append(
                {
                    "case_id": case,
                    "variant_id": variant,
                    "count": len(observations),
                    "metrics": metrics,
                }
            )
    if mismatches:
        report["status"] = "completed_with_errors"
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--warmups", type=int, default=1)
    parser.add_argument("--max-expansions", type=int, default=20_000)
    parser.add_argument("--max-iters", type=int, default=1_000)
    parser.add_argument("--trace-max-bytes", type=int, default=32 * 1024 * 1024)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--worker-case", choices=_CASES, help=argparse.SUPPRESS)
    parser.add_argument("--worker-variant", choices=_VARIANTS, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker_case is not None:
        if args.worker_variant is None:
            parser.error("--worker-variant is required with --worker-case")
        print(
            json.dumps(
                _run_worker(
                    args.worker_case,
                    args.worker_variant,
                    args.seed,
                    args.max_expansions,
                    args.max_iters,
                    args.trace_max_bytes,
                ),
                allow_nan=False,
            )
        )
        return 0
    report = run_benchmarks(
        seed=args.seed,
        repeats=args.repeats,
        warmups=args.warmups,
        max_expansions=args.max_expansions,
        max_iters=args.max_iters,
        trace_max_bytes=args.trace_max_bytes,
        output_path=args.output,
    )
    if args.output is not None:
        write_report(args.output, report)
    else:
        print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
