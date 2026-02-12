"""Legacy bidirectional A* compatibility wrapper backed by shared contracts."""

from __future__ import annotations

from pathplanning.planners.search.plan2d_facade import Planner

from ._legacy2d_common import LegacySearch2DBase


class BidirectionalAstar(LegacySearch2DBase):
    """Bidirectional A* compatibility class using ``Search2dFacade`` internally."""

    _planner_kind = Planner.BIDIRECTIONAL_ASTAR

    def searching(self):
        result = self._plan()
        return (result.path or []), (result.visited_fore or []), (result.visited_back or [])


def main():
    planner = BidirectionalAstar((5, 5), (45, 25), "euclidean")
    path, visited_fore, visited_back = planner.searching()
    print(f"path_len={len(path)}, fore={len(visited_fore)}, back={len(visited_back)}")


if __name__ == "__main__":
    main()
