"""Smoke-test a wheel installed in cibuildwheel's isolated test environment."""

from __future__ import annotations

from importlib.metadata import version
from pathlib import Path
import sys

import pathplanning
from pathplanning.api import plan_discrete
from pathplanning.core.contracts import DiscreteProblem
from pathplanning.native._ffi import (
    load_continuous_library,
    load_continuous_trace_library,
    load_native_library,
    load_search_metrics_library,
    load_search_trace_library,
)
from pathplanning.native.graph import NativeGraph


def main() -> None:
    package_path = Path(pathplanning.__file__).resolve()
    environment_path = Path(sys.prefix).resolve()
    if environment_path not in package_path.parents:
        raise RuntimeError(f"expected installed wheel under {environment_path}, got {package_path}")

    libraries = (
        load_native_library(),
        load_continuous_library(),
        load_search_trace_library(),
        load_search_metrics_library(),
        load_continuous_trace_library(),
    )
    if len(libraries) != 5:
        raise RuntimeError("expected all five native libraries to load")

    graph = NativeGraph.from_edges(
        nodes=[0, 1, 2],
        edges=[(0, 1, 1.0), (1, 2, 1.0)],
        directed=False,
    )
    result = plan_discrete(DiscreteProblem(graph=graph, start=0, goal=2), planner="astar")
    if not result.success or result.path is None:
        raise RuntimeError("installed wheel failed the public native A* smoke plan")

    print(
        f"Clean wheel install passed: pathplanning {version('pathplanning')}, "
        f"Python {sys.version_info.major}.{sys.version_info.minor}, "
        f"{sys.platform}, {len(libraries)} native libraries"
    )


if __name__ == "__main__":
    main()
