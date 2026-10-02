"""Measure the full Python API and C kernel for native sampling planners."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import platform
import shlex
import shutil
import statistics
import subprocess
import sys
import sysconfig
import time
from typing import Any

import numpy as np

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


@dataclass(frozen=True)
class SamplingBenchmarkRow:
    planner: str
    space_path: str
    full_api_median_s: float
    full_api_min_s: float
    full_api_max_s: float
    native_kernel_median_s: float
    api_setup_and_ffi_median_s: float
    success_rate: float
    nodes_median: int
    path_cost_median: float | None
    runs: int


def _compiler_info(env_name: str, config_key: str) -> dict[str, str | None]:
    configured = os.environ.get(env_name) or str(sysconfig.get_config_var(config_key) or "")
    pieces = shlex.split(configured)
    executable = shutil.which(pieces[0]) if pieces else None
    version = None
    if executable is not None:
        try:
            completed = subprocess.run(
                [executable, "--version"],
                check=False,
                capture_output=True,
                text=True,
                timeout=5,
            )
            output = completed.stdout or completed.stderr
            version = output.splitlines()[0].strip() if output else None
        except (OSError, subprocess.TimeoutExpired):
            version = None
    return {"configured": configured or None, "executable": executable, "version": version}


def _processor_name() -> str | None:
    if platform.system() == "Darwin":
        try:
            completed = subprocess.run(
                ["sysctl", "-n", "machdep.cpu.brand_string"],
                check=False,
                capture_output=True,
                text=True,
                timeout=2,
            )
            model = completed.stdout.strip()
            if completed.returncode == 0 and model:
                return model
        except (OSError, subprocess.TimeoutExpired):
            pass
    processor = platform.processor()
    if processor.lower() in {"", "i386", "i686"}:
        processor = platform.uname().machine
    return processor or None


def environment_metadata() -> dict[str, Any]:
    """Describe the host and configured native toolchain for benchmark results."""
    return {
        "platform": platform.platform(),
        "machine": platform.machine(),
        "processor": _processor_name(),
        "logical_cpu_count": os.cpu_count(),
        "python": sys.version,
        "numpy": np.__version__,
        "c_compiler": _compiler_info("CC", "CC"),
        "cxx_compiler": _compiler_info("CXX", "CXX"),
        "native_compile_flags": {
            "c": ["-std=c11", "-O3"],
            "c++": ["-std=c++17", "-O3"],
        },
    }


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
        collision_step=_COLLISION_STEP,
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
        max_sample_tries=2_000,
        collision_step=_COLLISION_STEP,
        rrt_star_radius_gamma=_RADIUS_GAMMA,
        sample_count=sample_count,
        batch_size=max(16, sample_count // 4),
        allow_python_callbacks=callbacks,
    )

    started = time.perf_counter()
    result = plan_continuous(problem, planner=planner, params=params, seed=seed)
    full_api_s = time.perf_counter() - started
    return {
        "full_api_s": full_api_s,
        "native_kernel_s": float(result.stats["elapsed_s"]),
        "success": result.success,
        "nodes": result.nodes,
        "path_cost": result.stats.get("path_cost"),
        "python_callbacks": int(result.stats["python_callbacks"]),
    }


def _benchmark_pair(
    planner: str,
    *,
    seed: int,
    repeats: int,
    warmups: int,
    max_iters: int,
    sample_count: int,
) -> list[SamplingBenchmarkRow]:
    observations: dict[str, list[dict[str, Any]]] = {"native_model": [], "python_callbacks": []}
    modes = (("native_model", False), ("python_callbacks", True))

    for _ in range(warmups):
        for _mode, callbacks in modes:
            _run_once(
                planner,
                callbacks=callbacks,
                seed=seed,
                max_iters=max_iters,
                sample_count=sample_count,
            )

    for run_index in range(repeats):
        ordered_modes = modes if run_index % 2 == 0 else tuple(reversed(modes))
        for mode, callbacks in ordered_modes:
            observation = _run_once(
                planner,
                callbacks=callbacks,
                seed=seed + run_index,
                max_iters=max_iters,
                sample_count=sample_count,
            )
            expected_callbacks = int(mode == "python_callbacks")
            if observation["python_callbacks"] != expected_callbacks:
                raise RuntimeError(f"{planner}/{mode} used an unexpected execution path")
            observations[mode].append(observation)

    rows: list[SamplingBenchmarkRow] = []
    for mode, _callbacks in modes:
        runs = observations[mode]
        api_times = [item["full_api_s"] for item in runs]
        kernel_times = [item["native_kernel_s"] for item in runs]
        path_costs = [item["path_cost"] for item in runs if item["path_cost"] is not None]
        rows.append(
            SamplingBenchmarkRow(
                planner=planner,
                space_path=mode,
                full_api_median_s=statistics.median(api_times),
                full_api_min_s=min(api_times),
                full_api_max_s=max(api_times),
                native_kernel_median_s=statistics.median(kernel_times),
                api_setup_and_ffi_median_s=statistics.median(
                    max(0.0, api_time - kernel_time)
                    for api_time, kernel_time in zip(api_times, kernel_times, strict=True)
                ),
                success_rate=sum(bool(item["success"]) for item in runs) / len(runs),
                nodes_median=int(statistics.median(item["nodes"] for item in runs)),
                path_cost_median=(statistics.median(path_costs) if path_costs else None),
                runs=repeats,
            )
        )
    return rows


def run_benchmarks(
    *,
    seed: int = 7,
    repeats: int = 5,
    warmups: int = 1,
    max_iters: int = 1_000,
    sample_count: int = 256,
) -> dict[str, Any]:
    """Run every registry sampling planner through both native and callback paths."""
    if repeats <= 0 or warmups < 0 or max_iters <= 0 or sample_count <= 0:
        raise ValueError("repeats/max_iters/sample_count must be positive and warmups non-negative")
    rows = [
        row
        for planner in _PLANNERS
        for row in _benchmark_pair(
            planner,
            seed=seed,
            repeats=repeats,
            warmups=warmups,
            max_iters=max_iters,
            sample_count=sample_count,
        )
    ]
    return {
        "metadata": environment_metadata(),
        "settings": {
            "seed": seed,
            "repeats": repeats,
            "warmups": warmups,
            "max_iters": max_iters,
            "sample_count": sample_count,
            "batch_size": max(16, sample_count // 4),
            "step_size": _STEP_SIZE,
            "goal_sample_rate": 0.08,
            "collision_step": _COLLISION_STEP,
            "rrt_star_radius_gamma": _RADIUS_GAMMA,
            "bounds": _BOUNDS,
            "obstacles": _OBSTACLES,
        },
        "results": [asdict(row) for row in rows],
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare full-API sampling latency for native and callback space paths."
    )
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--warmups", type=int, default=1)
    parser.add_argument("--max-iters", type=int, default=1_000)
    parser.add_argument("--sample-count", type=int, default=256)
    parser.add_argument("--json", action="store_true", help="Print the full JSON report.")
    parser.add_argument("--output", help="Also write the full JSON report to this path.")
    args = parser.parse_args()

    report = run_benchmarks(
        seed=args.seed,
        repeats=args.repeats,
        warmups=args.warmups,
        max_iters=args.max_iters,
        sample_count=args.sample_count,
    )
    serialized = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        with open(args.output, "w", encoding="utf-8") as output_file:
            output_file.write(serialized)
            output_file.write("\n")
    if args.json:
        print(serialized)
        return
    metadata = report["metadata"]
    print(f"Host: {metadata['platform']} | CPU: {metadata['processor'] or metadata['machine']}")
    print(f"Python: {sys.version.split()[0]} | NumPy: {np.__version__}")
    print(f"C compiler: {metadata['c_compiler']['version']}")
    print(f"C++ compiler: {metadata['cxx_compiler']['version']}")
    header = (
        f"{'planner':20} {'space_path':18} {'full_api_ms':>12} {'C_kernel_ms':>12} "
        f"{'setup_ffi_ms':>13} {'success':>9} {'nodes':>8}"
    )
    print(header)
    print("-" * len(header))
    for row in report["results"]:
        print(
            f"{row['planner']:20} {row['space_path']:18} "
            f"{row['full_api_median_s'] * 1_000:12.3f} "
            f"{row['native_kernel_median_s'] * 1_000:12.3f} "
            f"{row['api_setup_and_ffi_median_s'] * 1_000:13.3f} "
            f"{row['success_rate']:8.0%} "
            f"{row['nodes_median']:8d}"
        )


if __name__ == "__main__":
    main()
