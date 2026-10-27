"""Integrity tests for registry-declared planner mappings."""

from __future__ import annotations

import subprocess
import sys

from pathplanning.registry import (
    PLANNER_REGISTRY,
)


def _iter_registry_entries() -> list[tuple[str, str, object]]:
    return [
        (spec.problem_kind, planner_name, spec.planner)
        for planner_name, spec in PLANNER_REGISTRY.items()
    ]


def test_importing_all_registered_planner_modules_is_headless() -> None:
    """Importing all registered planner modules must not load matplotlib."""
    modules = sorted({fn.__module__ for _, _, fn in _iter_registry_entries()})
    code = (
        "import importlib\n"
        "import sys\n"
        f"modules = {modules!r}\n"
        "before = set(sys.modules)\n"
        "for module_name in modules:\n"
        "    importlib.import_module(module_name)\n"
        "loaded = set(sys.modules) - before\n"
        "bad = sorted(\n"
        "    name\n"
        "    for name in loaded\n"
        "    if name == 'matplotlib' or name.startswith('matplotlib.')\n"
        ")\n"
        "if bad:\n"
        "    raise SystemExit('\\n'.join(bad))\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, (
        f"Registered planner modules load matplotlib during import:\n{result.stdout}{result.stderr}"
    )


def test_supported_planner_document_matches_registry() -> None:
    """The checked-in support matrix must be generated from planner metadata."""
    result = subprocess.run(
        [sys.executable, "scripts/generate_supported_algorithms.py", "--check"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, f"{result.stdout}{result.stderr}"
