"""Integrity tests for registry-declared supported algorithms."""

from __future__ import annotations

import importlib
import importlib.util
from pathlib import Path

from pathplanning.registry import expected_entrypoint_for_algorithm, list_supported_algorithms


def _module_source_path(module_name: str) -> Path:
    spec = importlib.util.find_spec(module_name)
    assert spec is not None, f"Missing module spec for {module_name}"
    assert spec.origin is not None, f"Missing source origin for {module_name}"
    return Path(spec.origin)


def test_supported_registry_modules_are_non_empty_and_importable() -> None:
    """
    Registry integrity contract:
    - every SUPPORTED algorithm module resolves to source
    - source file is non-empty
    - module can be imported
    """
    for algo_spec in list_supported_algorithms():
        source_path = _module_source_path(algo_spec.module)
        source_text = source_path.read_text(encoding="utf-8")
        assert source_text.strip(), (
            f"{algo_spec.algorithm_id} points to an empty module: {source_path}"
        )
        importlib.import_module(algo_spec.module)


def test_supported_registry_modules_expose_expected_entrypoints() -> None:
    """Every SUPPORTED algorithm module must expose a callable expected entrypoint."""
    for algo_spec in list_supported_algorithms():
        expected_entrypoint = expected_entrypoint_for_algorithm(algo_spec.algorithm_id)
        module = importlib.import_module(algo_spec.module)
        assert hasattr(module, expected_entrypoint), (
            f"{algo_spec.algorithm_id} missing expected entrypoint '{expected_entrypoint}' "
            f"in module {algo_spec.module}"
        )
        entrypoint = getattr(module, expected_entrypoint)
        assert callable(entrypoint), (
            f"{algo_spec.algorithm_id} entrypoint '{expected_entrypoint}' is not callable"
        )
