"""Measure native PRM* roadmap-build scaling in an open 2D space."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import platform
import statistics
import sys

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from pathplanning import RoadmapParams
from pathplanning.planners.sampling.prm_star import PrmStarRoadmap
from pathplanning.spaces.grid2d import Grid2DSamplingSpace


def _benchmark(sample_counts: list[int], repeats: int, seed: int) -> dict[str, object]:
    measurements = []
    for sample_count in sample_counts:
        build_ms = []
        api_ms = []
        connection_candidates = []
        for repetition in range(repeats):
            space = Grid2DSamplingSpace(x_range=(0.0, 20.0), y_range=(0.0, 20.0))
            params = RoadmapParams(sample_count=sample_count, gamma=8.0)
            with PrmStarRoadmap(
                space,
                2,
                params,
                np.random.default_rng(seed + repetition),
            ) as roadmap:
                roadmap.build(world_version="benchmark")
                build_ms.append(1000.0 * roadmap.build_stats["roadmap_build_s"])
                api_ms.append(1000.0 * roadmap.build_stats["roadmap_api_s"])
                connection_candidates.append(roadmap.build_stats["roadmap_connection_candidates"])
        measurements.append(
            {
                "samples": sample_count,
                "native_build_ms": build_ms,
                "api_build_ms": api_ms,
                "median_native_build_ms": statistics.median(build_ms),
                "median_api_build_ms": statistics.median(api_ms),
                "connection_candidates": connection_candidates,
                "median_connection_candidates": statistics.median(connection_candidates),
            }
        )
    return {
        "schema_version": "pathplanning_prm_roadmap_build_v1",
        "environment": {"python": platform.python_version(), "numpy": np.__version__},
        "protocol": {
            "space": "open 2D Grid2DSamplingSpace, bounds [0,20] x [0,20]",
            "gamma": 8.0,
            "repeats": repeats,
            "warmups": 0,
            "seed_start": seed,
            "timing": "native build and full public API build",
        },
        "measurements": measurements,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sample-counts", default="128,256,512,1024,2048,4096")
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--seed", type=int, default=410)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        sample_counts = [int(value.strip()) for value in args.sample_counts.split(",")]
    except ValueError:
        parser.error("--sample-counts must be a comma-separated list of integers")
    if not sample_counts or any(value <= 0 for value in sample_counts):
        parser.error("--sample-counts values must be positive")
    if args.repeats <= 0:
        parser.error("--repeats must be positive")

    report = _benchmark(sample_counts, args.repeats, args.seed)
    serialized = json.dumps(report, indent=2) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
