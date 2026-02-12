"""Legacy D* compatibility wrapper backed by shared graph contracts."""

from __future__ import annotations

from pathplanning.planners.search.plan2d_facade import Planner

from ._legacy2d_common import LegacySearch2DBase


class Dstar(LegacySearch2DBase):
    """D* compatibility class (headless run)."""

    def __init__(self, s_start, s_goal, graph=None):
        super().__init__(s_start, s_goal, heuristic_type="euclidean", graph=graph)

    def run(self, s_start=None, s_end=None):
        if s_start is not None:
            self.s_start = (int(s_start[0]), int(s_start[1]))
        if s_end is not None:
            self.s_goal = (int(s_end[0]), int(s_end[1]))

        result = self._plan(planner=Planner.DSTAR)
        self.path = result.path or []
        self.visited = result.visited or []
        return self.path


def main():
    planner = Dstar((5, 5), (45, 25))
    path = planner.run()
    print(f"path_len={len(path)}")


if __name__ == "__main__":
    main()
