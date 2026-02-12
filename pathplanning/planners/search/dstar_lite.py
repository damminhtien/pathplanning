"""Legacy D* Lite compatibility wrapper backed by shared graph contracts."""

from __future__ import annotations

from pathplanning.planners.search.plan2d_facade import Planner

from ._legacy2d_common import LegacySearch2DBase


class DStarLite(LegacySearch2DBase):
    """D* Lite compatibility class (headless run)."""

    _planner_kind = Planner.DSTAR_LITE

    def run(self):
        result = self._plan()
        self.path = result.path or []
        self.visited = result.visited or []
        return self.path



def main():
    planner = DStarLite((5, 5), (45, 25), "euclidean")
    path = planner.run()
    print(f"path_len={len(path)}")


if __name__ == "__main__":
    main()
