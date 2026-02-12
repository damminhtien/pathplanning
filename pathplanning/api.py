"""Top-level reusable API helpers for PathPlanning."""

from __future__ import annotations

import importlib
from types import ModuleType

from .registry import DROPPED_INCOMPLETE, get_algorithm


def load_algorithm_module(algorithm_id: str) -> ModuleType:
    """Load an algorithm module by registry id, enforcing production support."""
    spec = get_algorithm(algorithm_id)
    if spec.status == DROPPED_INCOMPLETE:
        raise ValueError(f"Algorithm '{algorithm_id}' is dropped from production: {spec.reason}")
    return importlib.import_module(spec.module)
