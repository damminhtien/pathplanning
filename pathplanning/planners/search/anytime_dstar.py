"""Legacy AD* compatibility wrapper backed by shared graph contracts."""

from __future__ import annotations

from pathplanning.planners.search.plan2d_facade import Planner

from ._legacy2d_common import LegacySearch2DBase


class AnytimeDstar(LegacySearch2DBase):
    """Anytime D* compatibility class (headless run)."""

    def __init__(self, s_start, s_goal, eps, heuristic_type, graph=None):
        super().__init__(s_start, s_goal, heuristic_type=heuristic_type, graph=graph)
        self.eps = float(eps)

    def run(self):
        result = self._plan(planner=Planner.ANYTIME_DSTAR, adstar_eps=self.eps)
        self.path = result.path or []
        self.visited = result.visited or []
        return self.path


def main():
    planner = AnytimeDstar((5, 5), (45, 25), 2.5, "euclidean")
    path = planner.run()
    print(f"path_len={len(path)}")


if __name__ == "__main__":
    main()
