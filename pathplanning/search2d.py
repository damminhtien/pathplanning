"""Reusable Search2D package API."""

from __future__ import annotations

from Search_based_Planning.plan2d.plan2d_facade import (
    Heuristic,
    PlanConfig,
    PlanResult,
    Planner,
    Search2dFacade,
)  # pylint: disable=unused-import


__all__ = ["Heuristic", "PlanConfig", "PlanResult", "Planner", "Search2D"]


class Search2D(Search2dFacade):  # pylint: disable=too-few-public-methods
    """Stable package entrypoint for 2D search planners."""
