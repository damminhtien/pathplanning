"""Legacy ARA* compatibility wrapper backed by shared graph contracts."""

from __future__ import annotations

from pathplanning.planners.search.plan2d_facade import Planner

from ._legacy2d_common import LegacySearch2DBase, reverse_path


class AnytimeRepairingAstar(LegacySearch2DBase):
    """Anytime Repairing A* compatibility class."""

    def __init__(self, s_start, s_goal, e, heuristic_type, graph=None):
        super().__init__(s_start, s_goal, heuristic_type=heuristic_type, graph=graph)
        self.e = float(e)

    def searching(self):
        result = self._plan(planner=Planner.ANYTIME_REPAIRING_ASTAR, ara_e=self.e)
        raw_paths = result.paths if result.paths is not None else ([result.path] if result.path else [])
        paths = [reverse_path(path) for path in raw_paths]
        return paths, (result.visited_iters or [])


def main():
    planner = AnytimeRepairingAstar((5, 5), (45, 25), 2.5, "euclidean")
    paths, visited = planner.searching()
    print(f"solutions={len(paths)}, rounds={len(visited)}")


if __name__ == "__main__":
    main()
