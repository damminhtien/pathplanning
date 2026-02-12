"""Legacy LPA* compatibility wrapper backed by shared graph contracts."""

from __future__ import annotations

from pathplanning.planners.search.plan2d_facade import Planner

from ._legacy2d_common import LegacySearch2DBase


class LifelongPlanningAstar(LegacySearch2DBase):
    """Lifelong Planning A* compatibility class (headless run)."""

    def run(self):
        result = self._plan(planner=Planner.LIFELONG_PLANNING_ASTAR)
        self.path = result.path or []
        self.visited = result.visited or []
        return self.path


def main():
    planner = LifelongPlanningAstar((5, 5), (45, 25), "euclidean")
    path = planner.run()
    print(f"path_len={len(path)}")


if __name__ == "__main__":
    main()
