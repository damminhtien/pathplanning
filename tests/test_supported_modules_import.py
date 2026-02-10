"""Import smoke test for every production-supported algorithm module."""

from __future__ import annotations

import importlib

from pathplanning.registry import list_supported_algorithms


def test_import_all_supported_modules() -> None:
    modules = sorted({spec.module for spec in list_supported_algorithms()})
    for module_name in modules:
        importlib.import_module(module_name)
