"""Replay a 2D A* search after planning has finished.

Run ``python examples/viewer_2d.py`` after ``make build-ext`` and installing
the optional ``viz`` dependency.
"""

from __future__ import annotations

from pathplanning import TraceOptions
from pathplanning.api import plan_discrete
from pathplanning.core.contracts import DiscreteProblem
from pathplanning.spaces import Grid2DSearchSpace
from pathplanning.viz import scene_from_problem, view_result


def main() -> None:
    wall = {(20, y) for y in range(2, 28) if y != 15}
    problem = DiscreteProblem(
        graph=Grid2DSearchSpace(width=40, height=30, obstacles=wall),
        start=(3, 4),
        goal=(36, 25),
    )
    result = plan_discrete(problem, planner="astar", trace=TraceOptions())
    viewer = view_result(scene_from_problem(problem), result)
    viewer.show(block=True)


if __name__ == "__main__":
    main()
