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
    dropped_ids = {spec.algorithm_id for spec in dropped}
    expected_dropped_ids = {
        "sampling2d.advanced_batch_informed_trees",
        "sampling2d.adaptively_informed_trees",
        "sampling2d.rrt_sharp",
        "sampling3d.abit_star",
        "sampling3d.rrt_star_smart",
    }
    assert expected_dropped_ids.issubset(dropped_ids)
    for algorithm_id in expected_dropped_ids:
        with pytest.raises(ValueError):
            pp.load_algorithm_module(algorithm_id)
