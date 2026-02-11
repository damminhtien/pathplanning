"""Legacy wrapper for 3D RRT that delegates to the contract-based planner."""

from __future__ import annotations

import time
import warnings

import numpy as np

from examples.worlds.demo_3d_world import build_demo_3d_world
from pathplanning.core.contracts import PlanResult
from pathplanning.core.params import RrtParams
from .rrt import RrtPlanner


class rrt:  # pylint: disable=invalid-name
    """Compatibility wrapper for the legacy 3D RRT entrypoint."""

    def __init__(self) -> None:
        warnings.warn(
            "pathplanning.sampling_based_planning.rrt_3d.rrt_3d.rrt is deprecated. "
            "Use pathplanning.sampling_based_planning.rrt_3d.rrt.RrtPlanner instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        self.space, start, goal = build_demo_3d_world()
        self.env = self.space
        self.x0 = tuple(float(v) for v in start)
        self.xt = tuple(float(v) for v in goal)

        # Keep legacy mutable attributes for compatibility.
        self.maxiter = 10_000
        self.stepsize = 0.5
        self.done = False
        self.ind = 0
        self.i = 0
        self.V: list[tuple[float, float, float]] = []
        self.Parent: dict[tuple[float, float, float], tuple[float, float, float]] = {}
        self.Path: list[np.ndarray] = []
        self.result: PlanResult | None = None

    def run(self) -> PlanResult:  # noqa: D401 - keep legacy public API
        """Run the planner once and populate legacy fields."""
        params = RrtParams(
            max_iters=self.maxiter,
            step_size=self.stepsize,
            goal_sample_rate=0.05,
            max_sample_tries=1_000,
            collision_step=0.1,
        )
        planner = RrtPlanner(self.space, params, rng=np.random.default_rng())
        result = planner.plan(np.asarray(self.x0, dtype=float), (np.asarray(self.xt, dtype=float), self.stepsize))
        self.result = result

        self.done = True
        self.ind = int(result.iters)
        self.i = max(result.nodes - 1, 0)

        if result.path:
            tuples_path = [tuple(float(v) for v in state) for state in result.path]
            self.V = tuples_path
            self.Parent = {child: parent for parent, child in zip(tuples_path[:-1], tuples_path[1:])}
            self.Path = [
                np.array([child, parent], dtype=float)
                for parent, child in zip(tuples_path[:-1], tuples_path[1:])
            ]
            total_distance = 0.0
            for parent, child in zip(tuples_path[:-1], tuples_path[1:]):
                total_distance += self.space.distance(np.asarray(parent, dtype=float), np.asarray(child, dtype=float))
            print("Total distance = " + str(total_distance))
        else:
            self.V = [self.x0]
            self.Parent = {}
            self.Path = []

        return result


if __name__ == "__main__":
    planner = rrt()
    start_time = time.time()
    planner.run()
    print("time used = " + str(time.time() - start_time))
