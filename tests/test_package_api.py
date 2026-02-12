"""Smoke tests for package-level API and registry behavior."""

from __future__ import annotations

import pytest

import pathplanning as pp


def test_package_import_and_symbols() -> None:
    assert pp.run_planner is not None
    assert pp.plan is not None
    assert pp.plan_discrete is not None
    assert pp.plan_continuous is not None
    assert pp.RrtParams is not None
    assert pp.Result is not None
    assert pp.Stats is not None
    assert pp.list_supported_algorithms()
    assert pp.list_supported_planners()


def test_unknown_algorithm_is_rejected() -> None:
    with pytest.raises(KeyError):
        pp.load_algorithm_module("sampling3d.abit_star")
