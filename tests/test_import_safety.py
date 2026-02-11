"""Import safety checks for headless core usage."""

from __future__ import annotations

from collections.abc import Iterable
import importlib
import sys

from pathplanning.registry import list_supported_algorithms


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
        "pathplanning.core.nn_index",
        "pathplanning.core.tree",
        "pathplanning.env.continuous_3d",
        "pathplanning.registry",
    ]

    before = set(sys.modules)
    _import_modules(core_modules)
    after = set(sys.modules)

    if "matplotlib" not in before:
        _assert_no_new_prefix("matplotlib", before, after)
    _assert_no_new_prefix("pathplanning.viz", before, after)


def test_supported_planners_import_headless() -> None:
    planner_modules = [spec.module for spec in list_supported_algorithms()]

    before = set(sys.modules)
    _import_modules(planner_modules)
    after = set(sys.modules)

    if "matplotlib" not in before:
        _assert_no_new_prefix("matplotlib", before, after)
    _assert_no_new_prefix("pathplanning.viz", before, after)
