"""Smoke tests for core package namespace imports."""

from __future__ import annotations

import importlib


def test_core_package_imports_smoke() -> None:
    modules = [
        "pathplanning",
        "pathplanning.search_based_planning",
        "pathplanning.sampling_based_planning",
        "pathplanning.curves",
    ]

    failures: list[str] = []
    for module_name in modules:
        try:
            importlib.import_module(module_name)
        except ImportError as exc:  # pragma: no cover - exercised only on failures
            failures.append(f"{module_name}: {exc}")

    assert not failures, "Import smoke failures:\n" + "\n".join(failures)
