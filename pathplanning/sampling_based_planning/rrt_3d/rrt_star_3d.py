"""Legacy wrapper for 3D RRT* that delegates to the contract-based planner."""

from __future__ import annotations

import time
import warnings

import numpy as np

from examples.worlds.demo_3d_world import build_demo_3d_world
from pathplanning.core.contracts import PlanResult
from pathplanning.core.params import RrtParams
from .rrt_star import RrtStarPlanner

# Backward compatibility for tests and legacy monkeypatching.
plt = None


def visualization(*_args, **_kwargs) -> None:
    """Legacy no-op visualization hook."""


class rrtstar:  # pylint: disable=invalid-name
    """Compatibility wrapper for the legacy 3D RRT* entrypoint."""

    def __init__(self) -> None:
        warnings.warn(
            "pathplanning.sampling_based_planning.rrt_3d.rrt_star_3d.rrtstar is deprecated. "
            "Use pathplanning.sampling_based_planning.rrt_3d.rrt_star.RrtStarPlanner instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        self.space, start, goal = build_demo_3d_world()
        self.env = self.space
        self.x0 = tuple(float(v) for v in start)
        self.xt = tuple(float(v) for v in goal)

        self.maxiter = 4_000
        self.stepsize = 2.0
        self.gamma = 7.0
        self.eta = self.stepsize

        self.ind = 0
        self.i = 0
        self.done = False
        self.V: list[tuple[float, float, float]] = [self.x0]
        self.Parent: dict[tuple[float, float, float], tuple[float, float, float]] = {}
        self.COST: dict[tuple[float, float, float], float] = {self.x0: 0.0}
        self.Path: list[np.ndarray] = []
        self.D = 0.0
        self.result: PlanResult | None = None

    def wireup(self, x, y) -> None:
        """Legacy compatibility method retained for API stability."""
        self.Parent[x] = y

    def run(self) -> PlanResult:  # noqa: D401 - keep legacy public API
        """Run the planner once and populate legacy fields."""
        start_time = time.time()

        if self.maxiter <= 0:
            self.done = True
            self.ind = int(self.maxiter)
            self.result = PlanResult(success=False, path=[], iters=0, nodes=1, stats={"goal_checks": 0})
            return self.result

        params = RrtParams(
            max_iters=int(self.maxiter),
            step_size=float(self.stepsize),
            goal_sample_rate=0.05,
            max_sample_tries=1_000,
            collision_step=0.1,
        )
        planner = RrtStarPlanner(self.space, params, rng=np.random.default_rng())
        result = planner.plan(np.asarray(self.x0, dtype=float), (np.asarray(self.xt, dtype=float), self.stepsize))
        self.result = result

        self.done = True
        self.ind = int(result.iters)
        self.i = max(result.nodes - 1, 0)

        if result.path:
            tuples_path = [tuple(float(v) for v in state) for state in result.path]
            self.V = tuples_path
            self.Parent = {child: parent for parent, child in zip(tuples_path[:-1], tuples_path[1:])}
            self.COST = {tuples_path[0]: 0.0}
            self.Path = []
            cumulative = 0.0
            for parent, child in zip(tuples_path[:-1], tuples_path[1:]):
                cumulative += self.space.distance(np.asarray(parent, dtype=float), np.asarray(child, dtype=float))
                self.COST[child] = cumulative
                self.Path.append(np.array([child, parent], dtype=float))
            self.D = cumulative
            print("Total distance = " + str(self.D))
        else:
            self.V = [self.x0]
            self.Parent = {}
            self.COST = {self.x0: 0.0}
            self.Path = []
            self.D = 0.0

        print("time used = " + str(time.time() - start_time))
        return result


if __name__ == "__main__":
    planner = rrtstar()
    planner.run()
