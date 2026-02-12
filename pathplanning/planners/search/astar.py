"""Legacy A* compatibility wrapper backed by shared graph contracts."""

from __future__ import annotations

from pathplanning.planners.search.plan2d_facade import Planner

from ._legacy2d_common import LegacySearch2DBase, reverse_path


class Astar(LegacySearch2DBase):
    """A* compatibility class using ``Search2dFacade`` internally."""

    _planner_kind = Planner.ASTAR

    def searching(self):
        result = self._plan()
        return reverse_path(result.path), (result.visited or [])

    def searching_repeated_astar(self, e):
        result = self._plan(planner=Planner.ANYTIME_REPAIRING_ASTAR, ara_e=float(e))
        raw_paths = result.paths if result.paths is not None else ([result.path] if result.path else [])
        paths = [reverse_path(path) for path in raw_paths]
        return paths, (result.visited_iters or [])

    def repeated_searching(self, s_start, s_goal, e):
        planner = Astar(s_start, s_goal, self.heuristic_type, graph=self._graph)
        paths, visited = planner.searching_repeated_astar(e)
        first_path = paths[0] if paths else []
        first_visited = visited[0] if visited else []
        return first_path, first_visited


def main():
    planner = Astar((5, 5), (45, 25), "euclidean")
    path, visited = planner.searching()
    print(f"path_len={len(path)}, visited={len(visited)}")


if __name__ == "__main__":
    main()
