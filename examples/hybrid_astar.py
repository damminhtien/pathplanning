"""Plan a footprint-valid vehicle path around a grid obstacle."""

from __future__ import annotations

import numpy as np

from pathplanning import HybridAStarParams, plan_continuous
from pathplanning.core.contracts import ContinuousProblem, GoalState
from pathplanning.spaces import AckermannGridSpace


def main() -> None:
    occupancy = np.zeros((20, 20), dtype=bool)
    occupancy[10, 10] = True
    space = AckermannGridSpace(
        occupancy,
        resolution=1.0,
        footprint_length=0.7,
        footprint_width=0.4,
    )
    problem = ContinuousProblem(space, (3.5, 10.5, 0.0), GoalState((16.5, 10.5, 0.0)))
    result = plan_continuous(
        problem,
        planner="hybrid_astar",
        params=HybridAStarParams(max_expansions=30_000, primitive_length=1.0),
    )
    if not result.success:
        raise RuntimeError(f"Hybrid A* stopped: {result.stop_reason.value}")
    assert result.path is not None
    print(f"poses: {len(result.path)}")
    print(f"cost: {result.stats['path_cost']:.2f}")
    print(f"directions: {sorted(set(result.directions) - {0})}")


if __name__ == "__main__":
    main()
