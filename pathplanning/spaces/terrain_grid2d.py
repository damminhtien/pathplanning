"""2D occupancy grids with positive per-cell terrain costs."""

from __future__ import annotations

from collections.abc import Iterable, Sequence
import math

import numpy as np
from numpy.typing import NDArray

from pathplanning.spaces.grid2d import Grid2DSearchSpace, GridCell, Motion2D


class TerrainCostGrid2D(Grid2DSearchSpace):
    """Eight-connected grid whose moves integrate terrain across touched cells.

    Cardinal moves average the terrain cost of their two endpoint cells.
    Diagonal moves average the endpoint and two side-cell costs, then multiply
    by ``sqrt(2)``. Diagonal corner cutting is prohibited.
    """

    def __init__(
        self,
        terrain_costs: NDArray[np.float64] | Sequence[Sequence[float]],
        *,
        motions: Sequence[Motion2D] | None = None,
        obstacles: Iterable[GridCell] | None = None,
        occupancy: NDArray[np.bool_] | None = None,
    ) -> None:
        costs = np.asarray(terrain_costs, dtype=np.float64)
        if costs.ndim != 2 or costs.shape[0] == 0 or costs.shape[1] == 0:
            raise ValueError("terrain_costs must be a non-empty 2D array")
        if not np.isfinite(costs).all() or np.any(costs <= 0.0):
            raise ValueError("terrain costs must be finite and strictly positive")
        height, width = costs.shape
        super().__init__(
            width=width, height=height, motions=motions, obstacles=obstacles, occupancy=occupancy
        )
        self.terrain_costs: NDArray[np.float64] = np.ascontiguousarray(costs)

    def edge_cost(self, first: GridCell, second: GridCell) -> float:
        source = self._coerce_cell(first)
        target = self._coerce_cell(second)
        if not self.is_valid_node(source) or not self.is_valid_node(target):
            return math.inf
        dx, dy = target[0] - source[0], target[1] - source[1]
        if max(abs(dx), abs(dy)) != 1:
            return math.inf
        touched = [
            self.terrain_costs[source[1], source[0]],
            self.terrain_costs[target[1], target[0]],
        ]
        if dx != 0 and dy != 0:
            side_a = (source[0] + dx, source[1])
            side_b = (source[0], source[1] + dy)
            if not self.is_valid_node(side_a) or not self.is_valid_node(side_b):
                return math.inf
            touched.extend(
                (
                    self.terrain_costs[side_a[1], side_a[0]],
                    self.terrain_costs[side_b[1], side_b[0]],
                )
            )
            return math.sqrt(2.0) * float(np.mean(touched))
        return float(np.mean(touched))


__all__ = ["TerrainCostGrid2D"]
