"""Smoke tests for package-level API and registry behavior."""

from __future__ import annotations

import pytest

import pathplanning as pp


def test_package_import_and_symbols() -> None:
    assert pp.Search2D is not None
    assert pp.Planner is not None
    assert pp.PlanConfig is not None
    assert pp.list_supported_algorithms()


def test_unknown_algorithm_is_rejected() -> None:
    with pytest.raises(KeyError):
        pp.load_algorithm_module("sampling3d.abit_star")
