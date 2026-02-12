"""Legacy LRTA* compatibility wrapper backed by shared graph contracts."""

from __future__ import annotations

from pathplanning.planners.search.plan2d_facade import Planner

from ._legacy2d_common import LegacySearch2DBase


class LearningRealtimeAstar(LegacySearch2DBase):
    """Learning Real-Time A* compatibility class."""

    def __init__(self, s_start, s_goal, N, heuristic_type, graph=None):
        super().__init__(s_start, s_goal, heuristic_type=heuristic_type, graph=graph)
        self.N = int(N)

    def searching(self):
        result = self._plan(planner=Planner.LEARNING_REALTIME_ASTAR, lrta_n=self.N)
        return (result.path or []), (result.visited or [])


def main():
    planner = LearningRealtimeAstar((5, 5), (45, 25), 250, "euclidean")
    path, visited = planner.searching()
    print(f"path_len={len(path)}, visited={len(visited)}")


if __name__ == "__main__":
    main()
