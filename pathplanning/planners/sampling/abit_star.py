"""ABIT*: anytime BIT* with heuristic inflation and search truncation."""

from __future__ import annotations

from collections.abc import Mapping

from pathplanning.core.contracts import ContinuousProblem, State
from pathplanning.core.params import RrtParams
from pathplanning.core.results import PlanResult
from pathplanning.core.types import RNG
from pathplanning.planners.sampling._internal.bit_engine import (
    BatchInformedTreePlanner,
    IndexFactory,
)
from pathplanning.planners.sampling._internal.continuous import validate_objective
from pathplanning.planners.sampling._internal.problem_adapter import (
    coerce_rrt_params,
    resolve_rng,
)


class ABITStar(BatchInformedTreePlanner):
    """Anytime BIT* that reduces inflation and truncation across sample batches."""

    planner_name = "ABIT*"

    def _search_factors(self, batch_number: int) -> tuple[float, float]:
        q = float(batch_number + 1)
        inflation = 1.0 + self.params.abit_inflation_parameter / q
        truncation = 1.0 + self.params.abit_truncation_parameter / q
        return inflation, truncation


def plan_abit_star(
    problem: ContinuousProblem[State],
    *,
    params: RrtParams | Mapping[str, object] | None = None,
    rng: RNG | None = None,
) -> PlanResult:
    """Plan with anytime BIT* for exact goals and additive path length."""
    validate_objective(problem.objective, "ABIT*")
    resolved_params = coerce_rrt_params(problem, params)
    return ABITStar(problem.space, resolved_params, resolve_rng(rng)).plan(
        problem.start,
        problem.goal,
    )


__all__ = ["ABITStar", "IndexFactory", "plan_abit_star"]
