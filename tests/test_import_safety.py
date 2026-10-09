"""Import safety checks for headless core usage."""

from __future__ import annotations

import ast
from collections.abc import Iterable
import importlib
from pathlib import Path
import sys

from pathplanning.registry import planner_modules


def _import_modules(modules: Iterable[str]) -> None:
    for module in modules:
        importlib.import_module(module)


def _assert_no_new_prefix(prefix: str, before: set[str], after: set[str]) -> None:
    newly_loaded = {
        name for name in after - before if name == prefix or name.startswith(f"{prefix}.")
    }
    assert not newly_loaded, f"Unexpected import of {prefix}: {sorted(newly_loaded)}"


def test_core_modules_import_headless() -> None:
    core_modules = [
        "pathplanning.core.contracts",
        "pathplanning.core.params",
        "pathplanning.nn.index",
        "pathplanning.data_structures.tree_array",
        "pathplanning.spaces.continuous_3d",
        "pathplanning.registry",
    ]

    before = set(sys.modules)
    _import_modules(core_modules)
    after = set(sys.modules)

    if "matplotlib" not in before:
        _assert_no_new_prefix("matplotlib", before, after)
    _assert_no_new_prefix("pathplanning.viz", before, after)


def test_supported_planners_import_headless() -> None:
    modules = planner_modules()

    before = set(sys.modules)
    _import_modules(modules)
    after = set(sys.modules)

    if "matplotlib" not in before:
        _assert_no_new_prefix("matplotlib", before, after)
    _assert_no_new_prefix("pathplanning.viz", before, after)


def test_no_legacy_planner_package_paths() -> None:
    planner_files = Path("pathplanning").rglob("*.py")
    banned_parts = {"search_based_planning", "sampling_based_planning"}
    offenders = [
        path.as_posix()
        for path in planner_files
        if any(part in banned_parts for part in path.parts)
    ]
    assert not offenders, "Legacy planner package paths must be removed:\n" + "\n".join(
        sorted(offenders)
    )


def _imports_grid2d(path: Path) -> bool:
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source)

    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            if node.module == "pathplanning.spaces.grid2d":
                return True
            if node.module == "pathplanning.spaces":
                if any(alias.name == "grid2d" for alias in node.names):
                    return True
        elif isinstance(node, ast.Import):
            if any(alias.name == "pathplanning.spaces.grid2d" for alias in node.names):
                return True
    return False


def test_grid2d_imports_are_limited_to_grid_specialized_adapters() -> None:
    planner_files = sorted(Path("pathplanning/planners").rglob("*.py"))
    grid_specialized_adapters = {
        "pathplanning/planners/search/_grid_utils.py",
        "pathplanning/planners/search/dstar_lite.py",
        "pathplanning/planners/search/jump_point.py",
        "pathplanning/planners/search/theta_star.py",
    }
    offenders = [
        path.as_posix()
        for path in planner_files
        if _imports_grid2d(path) and path.as_posix() not in grid_specialized_adapters
    ]
    assert not offenders, (
        "Only the shared grid adapter and grid-specialized planners may import "
        "pathplanning.spaces.grid2d directly:\n" + "\n".join(offenders)
    )
