"""Typing contract tests for public package surfaces."""

from __future__ import annotations

from pathlib import Path
from types import ModuleType
from typing import get_type_hints

from pathplanning.api import load_algorithm_module


def test_public_api_has_explicit_load_annotation() -> None:
    hints = get_type_hints(load_algorithm_module)
    assert hints["algorithm_id"] is str
    assert hints["return"] is ModuleType


def test_package_includes_pep561_marker() -> None:
    marker = Path(__file__).resolve().parents[1] / "pathplanning" / "py.typed"
    assert marker.exists()
