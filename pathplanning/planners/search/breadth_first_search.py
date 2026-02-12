"""Legacy breadth-first compatibility wrapper backed by shared graph contracts."""

from __future__ import annotations

from pathplanning.planners.search.plan2d_facade import Planner

from .astar import Astar
from ._legacy2d_common import reverse_path


class BreadthFirstSearch(Astar):
    """Breadth-first compatibility class using ``Search2dFacade`` internally."""

    def searching(self):
        result = self._plan(planner=Planner.BREADTH_FIRST_SEARCH)
        return reverse_path(result.path), (result.visited or [])


def main():
    planner = BreadthFirstSearch((5, 5), (45, 25), "euclidean")
    path, visited = planner.searching()
    print(f"path_len={len(path)}, visited={len(visited)}")


if __name__ == "__main__":
    main()
