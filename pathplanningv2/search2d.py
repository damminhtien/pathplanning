"""Reusable Search2D package API."""

from __future__ import annotations

from Search_based_Planning.plan2d.plan2d_facade import (
    Heuristic,
    PlanConfig,
    PlanResult,
    Planner,
    Search2dFacade,
)


class Search2D(Search2dFacade):
    """Stable package entrypoint for 2D search planners."""
