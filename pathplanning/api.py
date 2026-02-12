"""Top-level reusable API helpers for PathPlanning."""

from __future__ import annotations

import importlib
from types import ModuleType

from .registry import get_algorithm


def load_algorithm_module(algorithm_id: str) -> ModuleType:
    """Load an algorithm module by registry id."""
    return importlib.import_module(get_algorithm(algorithm_id).module)
