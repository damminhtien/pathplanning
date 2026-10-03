"""Compare production planning before and after visualization refactoring.

Each observation runs in a fresh process. The baseline and current checkout
must both contain production native libraries built for the active Python.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import resource
import statistics
import subprocess
import sys
import time
from typing import Any

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
_CASES = ("discrete2d.astar", "sampling2d.rrt_star")
_VARIANTS = ("before", "after")


def _peak_rss_bytes() -> int:
    maximum = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return int(maximum if platform.system() == "Darwin" else maximum * 1024)


def _worker(
    root: Path,
    case: str,
    seed: int,
    max_expansions: int,
    max_iters: int,
) -> dict[str, Any]:
    sys.path.insert(0, str(root))

    from pathplanning.api import plan_continuous, plan_discrete
    from pathplanning.core.contracts import ContinuousProblem, DiscreteProblem, GoalState
    from pathplanning.core.params import RrtParams
    from pathplanning.spaces.grid2d import Grid2DSamplingSpace, Grid2DSearchSpace

    if case == "discrete2d.astar":
        blocked = [(40, y) for y in range(81) if not 33 <= y <= 38]
        problem = DiscreteProblem(
            graph=Grid2DSearchSpace(width=81, height=81, obstacles=blocked),
            start=(5, 5),
            goal=(75, 75),
        )
        started = time.perf_counter()
        result = plan_discrete(
            problem,
            planner="astar",
            params={"max_expansions": max_expansions},
            seed=seed,
        )
        full_api_s = time.perf_counter() - started
        native_stage_s = float(result.stats["native_search_s"])
    elif case == "sampling2d.rrt_star":
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
        problem = ContinuousProblem(
            space=space,
            start=np.asarray([1.0, 1.0], dtype=np.float64),
            goal=GoalState(
                state=np.asarray([9.0, 9.0], dtype=np.float64),
                radius=0.25,
                distance_fn=space.distance,
            ),
        )
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
        )
        full_api_s = time.perf_counter() - started
        native_stage_s = float(result.stats["elapsed_s"])
    else:
        raise ValueError(f"unknown benchmark case: {case}")

    return {
        "measurements": {
            "full_api_s": full_api_s,
            "native_stage_s": native_stage_s,
            "peak_rss_bytes": _peak_rss_bytes(),
            "iters": result.iters,
            "nodes": result.nodes,
        },
        "signature": {
            "success": result.success,
            "stop_reason": result.stop_reason.value,
            "iters": result.iters,
            "nodes": result.nodes,
            "path": None if result.path is None else np.asarray(result.path).tolist(),
            "path_cost": result.stats.get("path_cost"),
        },
    }


def _observe(
    root: Path,
    case: str,
    seed: int,
    max_expansions: int,
    max_iters: int,
) -> dict[str, Any]:
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--worker-root",
        str(root),
        "--worker-case",
        case,
        "--seed",
        str(seed),
        "--max-expansions",
        str(max_expansions),
        "--max-iters",
        str(max_iters),
    ]
    completed = subprocess.run(
        command, cwd=root, capture_output=True, text=True, timeout=120, check=False
    )
    if completed.returncode != 0:
        raise RuntimeError(f"worker failed in {root}: {completed.stderr.strip()}")
    return json.loads(completed.stdout)


def _artifacts(root: Path) -> list[dict[str, Any]]:
    native = root / "pathplanning" / "native"
    artifacts = []
    for name in ("_search_engine", "_continuous_engine"):
        for path in sorted(native.glob(f"{name}*.so")):
            artifacts.append(
                {
                    "path": str(path.relative_to(root)),
                    "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    "size_bytes": path.stat().st_size,
                }
            )
    return artifacts


def _source_provenance(root: Path) -> dict[str, Any]:
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=root,
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
        ).stdout.strip()
        status = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=root,
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return {"commit": None, "dirty": None}
    return {"commit": commit, "dirty": bool(status)}


def _summarize(values: list[float]) -> dict[str, float | int]:
    mean = statistics.fmean(values)
    deviation = statistics.stdev(values) if len(values) > 1 else 0.0
    return {
        "count": len(values),
        "median": statistics.median(values),
        "min": min(values),
        "max": max(values),
        "mean": mean,
        "stdev": deviation,
        "coefficient_of_variation_pct": (deviation / mean * 100.0) if mean else 0.0,
    }


def run(args: argparse.Namespace) -> dict[str, Any]:
    roots = {"before": Path(args.before_root).resolve(), "after": Path(args.after_root).resolve()}
    for label, root in roots.items():
        if not (root / "pathplanning" / "native").is_dir():
            raise ValueError(f"{label} root has no pathplanning/native directory: {root}")

    runs: list[dict[str, Any]] = []
    for case in _CASES:
        for phase, count, seed_base in (
            ("warmup", args.warmups, args.seed + args.repeats),
            ("measure", args.repeats, args.seed),
        ):
            for repeat_index in range(count):
                seed = seed_base + repeat_index
                order = _VARIANTS if repeat_index % 2 == 0 else _VARIANTS[::-1]
                for variant in order:
                    observation = _observe(
                        roots[variant],
                        case,
                        seed,
                        args.max_expansions,
                        args.max_iters,
                    )
                    runs.append(
                        {
                            "case_id": case,
                            "variant": variant,
                            "phase": phase,
                            "repeat_index": repeat_index,
                            "seed": seed,
                            **observation,
                        }
                    )

    mismatches = []
    paired_effects: list[dict[str, Any]] = []
    for case in _CASES:
        pairs = []
        for repeat_index in range(args.repeats):
            pair = {
                item["variant"]: item
                for item in runs
                if item["phase"] == "measure"
                and item["case_id"] == case
                and item["repeat_index"] == repeat_index
            }
            if set(pair) != set(_VARIANTS):
                mismatches.append({"case_id": case, "repeat_index": repeat_index})
                continue
            if pair["before"]["signature"] != pair["after"]["signature"]:
                mismatches.append({"case_id": case, "repeat_index": repeat_index})
            pairs.append(pair)

        for metric in ("full_api_s", "native_stage_s", "peak_rss_bytes"):
            changes = [
                (
                    pair["after"]["measurements"][metric] / pair["before"]["measurements"][metric]
                    - 1.0
                )
                * 100.0
                for pair in pairs
            ]
            paired_effects.append(
                {
                    "case_id": case,
                    "metric": metric,
                    "median_change_pct": statistics.median(changes),
                    "min_change_pct": min(changes),
                    "max_change_pct": max(changes),
                }
            )

    summaries = []
    for case in _CASES:
        for variant in _VARIANTS:
            observations = [
                item["measurements"]
                for item in runs
                if item["phase"] == "measure"
                and item["case_id"] == case
                and item["variant"] == variant
            ]
            summaries.append(
                {
                    "case_id": case,
                    "variant": variant,
                    "metrics": {
                        name: _summarize([float(item[name]) for item in observations])
                        for name in (
                            "full_api_s",
                            "native_stage_s",
                            "peak_rss_bytes",
                            "iters",
                            "nodes",
                        )
                    },
                }
            )

    return {
        "benchmark_id": "visualization_production_regression",
        "settings": {
            "seed": args.seed,
            "repeats": args.repeats,
            "warmups": args.warmups,
            "max_expansions": args.max_expansions,
            "max_iters": args.max_iters,
            "measurement_process": "one fresh subprocess per case, variant, and seed",
            "rss_unit": "bytes; process peak includes Python imports and native libraries",
        },
        "environment": {
            "platform": platform.platform(),
            "python": sys.version,
            "numpy": np.__version__,
            "native_compile_flags": {"c": ["-std=c11", "-O3"], "c++": ["-std=c++17", "-O3"]},
        },
        "workloads": [
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
                "max_iters": args.max_iters,
                "step_size": 0.5,
            },
        ],
        "sources": {
            "before": {
                "root": str(roots["before"]),
                "label": args.before_label,
                "provenance": _source_provenance(roots["before"]),
                "artifacts": _artifacts(roots["before"]),
            },
            "after": {
                "root": str(roots["after"]),
                "label": args.after_label,
                "provenance": _source_provenance(roots["after"]),
                "artifacts": _artifacts(roots["after"]),
            },
        },
        "paired_validation": {
            "expected_count": len(_CASES) * args.repeats,
            "matched_count": len(_CASES) * args.repeats - len(mismatches),
            "mismatches": mismatches,
        },
        "paired_effects": paired_effects,
        "summaries": summaries,
        "runs": runs,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before-root")
    parser.add_argument("--after-root", default=str(_ROOT))
    parser.add_argument("--before-label", default="before")
    parser.add_argument("--after-label", default="after")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--repeats", type=int, default=15)
    parser.add_argument("--warmups", type=int, default=2)
    parser.add_argument("--max-expansions", type=int, default=20_000)
    parser.add_argument("--max-iters", type=int, default=1_000)
    parser.add_argument("--output")
    parser.add_argument("--worker-root")
    parser.add_argument("--worker-case")
    args = parser.parse_args()

    if args.worker_root:
        print(
            json.dumps(
                _worker(
                    Path(args.worker_root).resolve(),
                    args.worker_case,
                    args.seed,
                    args.max_expansions,
                    args.max_iters,
                ),
                separators=(",", ":"),
            )
        )
        return

    if not args.before_root:
        parser.error("--before-root is required")
    if args.repeats <= 0 or args.warmups < 0:
        parser.error("--repeats must be positive and --warmups non-negative")
    report = run(args)
    serialized = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    if report["paired_validation"]["mismatches"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
