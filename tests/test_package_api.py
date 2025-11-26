"""Smoke tests for package-level API and registry behavior."""

from __future__ import annotations

import pytest

import pathplanning as pp
from pathplanning.core.contracts import DiscreteProblem
from pathplanning.spaces.grid2d import Grid2DSearchSpace


def test_package_import_and_symbols() -> None:
    assert pp.plan is not None
    assert pp.plan_discrete is not None
    assert pp.plan_continuous is not None
    assert pp.RrtParams is not None
    assert pp.Result is not None
    assert pp.Stats is not None


def test_unknown_planner_is_rejected() -> None:
    problem = DiscreteProblem(
        graph=Grid2DSearchSpace(width=5, height=5),
        start=(0, 0),
        goal=(4, 4),
    )
    with pytest.raises(KeyError):
        pp.plan_discrete(problem, planner="abit_star")
