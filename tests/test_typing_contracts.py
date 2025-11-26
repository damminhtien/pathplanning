"""Typing contract tests for public package surfaces."""

from __future__ import annotations

from pathlib import Path
from typing import get_type_hints

from pathplanning.api import plan_continuous, plan_discrete
from pathplanning.core.results import PlanResult


def test_public_api_has_explicit_plan_annotations() -> None:
    discrete_hints = get_type_hints(plan_discrete)
    assert "problem" in discrete_hints
    assert discrete_hints["planner"] is str
    assert discrete_hints["return"] is PlanResult

    continuous_hints = get_type_hints(plan_continuous)
    assert "problem" in continuous_hints
    assert continuous_hints["planner"] is str
    assert continuous_hints["return"] is PlanResult


def test_package_includes_pep561_marker() -> None:
    marker = Path(__file__).resolve().parents[1] / "pathplanning" / "py.typed"
    assert marker.exists()
