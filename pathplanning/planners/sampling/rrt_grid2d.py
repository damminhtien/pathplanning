"""Legacy RRT 2D compatibility wrapper backed by ``RrtPlanner`` contracts."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from pathplanning.core.contracts import GoalState
from pathplanning.core.params import RrtParams
from pathplanning.planners.sampling.rrt import RrtPlanner

State2D = NDArray[np.float64]


class ContinuousSpace2D(Protocol):
    def sample_free(self, rng: np.random.Generator) -> State2D: ...

    def is_state_valid(self, x: State2D) -> bool: ...

    def is_motion_valid(self, a: State2D, b: State2D) -> bool: ...

    def distance(self, a: State2D, b: State2D) -> float: ...

    def steer(self, a: State2D, b: State2D, step_size: float) -> State2D: ...


@dataclass
class Node:
    x: float
    y: float
    parent: "Node | None" = None

    @classmethod
    def from_xy(cls, xy: tuple[float, float]) -> "Node":
        return cls(x=float(xy[0]), y=float(xy[1]))


class OpenPlane2D:
    """Simple open 2D continuous space used by the legacy wrapper."""

    def __init__(self, lower: tuple[float, float] = (0.0, 0.0), upper: tuple[float, float] = (50.0, 30.0)) -> None:
        self.lower = np.array(lower, dtype=float)
        self.upper = np.array(upper, dtype=float)

    def sample_free(self, rng: np.random.Generator) -> State2D:
        return rng.uniform(self.lower, self.upper)

    def is_state_valid(self, x: State2D) -> bool:
        point = np.asarray(x, dtype=float)
        if point.shape != (2,):
            return False
        return bool(np.all(point >= self.lower) and np.all(point <= self.upper))

    def is_motion_valid(self, a: State2D, b: State2D) -> bool:
        return self.is_state_valid(a) and self.is_state_valid(b)

    def distance(self, a: State2D, b: State2D) -> float:
        return float(np.linalg.norm(np.asarray(b, dtype=float) - np.asarray(a, dtype=float)))

    def steer(self, a: State2D, b: State2D, step_size: float) -> State2D:
        if step_size <= 0.0:
            raise ValueError("step_size must be > 0")
        start = np.asarray(a, dtype=float)
        target = np.asarray(b, dtype=float)
        direction = target - start
        norm = float(np.linalg.norm(direction))
        if norm <= step_size:
            return target
        return start + (direction / norm) * float(step_size)


class Rrt:
    """Compatibility facade preserving the old ``Rrt`` surface."""

    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, iter_max, rng=None, space=None):
        self.s_start = Node.from_xy((float(s_start[0]), float(s_start[1])))
        self.s_goal = Node.from_xy((float(s_goal[0]), float(s_goal[1])))
        self.step_len = float(step_len)
        self.goal_sample_rate = float(goal_sample_rate)
        self.iter_max = int(iter_max)
        self.rng = rng if rng is not None else np.random.default_rng()
        self.space: ContinuousSpace2D = space if space is not None else OpenPlane2D()
        self.vertex: list[Node] = [self.s_start]

    def planning(self):
        params = RrtParams(
            step_size=self.step_len,
            goal_sample_rate=self.goal_sample_rate,
            max_iters=self.iter_max,
        )
        planner = RrtPlanner(space=self.space, params=params, rng=self.rng)

        start = np.array([self.s_start.x, self.s_start.y], dtype=float)
        goal = np.array([self.s_goal.x, self.s_goal.y], dtype=float)
        goal_region = GoalState(state=goal, radius=1e-9, distance_fn=self.space.distance)
        result = planner.plan(start, goal_region)

        if result.path is None:
            return None

        path_forward = [(float(p[0]), float(p[1])) for p in result.path]
        legacy_path = list(reversed(path_forward))

        self.vertex = [Node.from_xy(point) for point in path_forward]
        for idx in range(1, len(self.vertex)):
            self.vertex[idx].parent = self.vertex[idx - 1]

        return legacy_path


def main():
    planner = Rrt((2, 2), (49, 24), 0.5, 0.05, 1000)
    path = planner.planning()
    print("No Path Found!" if path is None else f"path_len={len(path)}")


if __name__ == "__main__":
    main()
