"""Smoke tests for package-level API and registry behavior."""

from __future__ import annotations

import pytest

import pathplanning as pp
from pathplanning.registry import DROPPED_INCOMPLETE


def test_package_import_and_symbols() -> None:
    assert pp.Search2D is not None
    assert pp.Planner is not None
    assert pp.PlanConfig is not None
    assert pp.list_supported_algorithms()


def test_dropped_algorithm_is_enforced() -> None:
    dropped = [spec for spec in pp.list_dropped_algorithms() if spec.status == DROPPED_INCOMPLETE]
    assert any(spec.algorithm_id == "sampling3d.abit_star" for spec in dropped)
    with pytest.raises(ValueError):
        pp.load_algorithm_module("sampling3d.abit_star")
