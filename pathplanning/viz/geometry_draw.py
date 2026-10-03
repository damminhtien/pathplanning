"""Small geometry drawings for optional curve demos.

Pass ``ax`` to draw into a specific figure. The default remains compatible
with geometry examples that already select an axes through pyplot.
"""

from __future__ import annotations

from typing import Any

import numpy as np


def _axes(ax: Any | None) -> Any:
    if ax is not None:
        return ax
    from matplotlib import pyplot as plt

    return plt.gca()


class Arrow:
    def __init__(
        self,
        x: float,
        y: float,
        theta: float,
        length: float,
        color: str,
        *,
        ax: Any | None = None,
    ) -> None:
        target = _axes(ax)
        angle = np.deg2rad(30.0)
        tip = np.array([x, y]) + length * np.array([np.cos(theta), np.sin(theta)])
        left = tip + 0.5 * length * np.array(
            [np.cos(theta + np.pi - angle), np.sin(theta + np.pi - angle)]
        )
        right = tip + 0.5 * length * np.array(
            [np.cos(theta + np.pi + angle), np.sin(theta + np.pi + angle)]
        )
        for start, end in ((np.array([x, y]), tip), (tip, left), (tip, right)):
            target.plot((start[0], end[0]), (start[1], end[1]), color=color, linewidth=2)


class Car:
    def __init__(
        self,
        x: float,
        y: float,
        yaw: float,
        width: float,
        length: float,
        *,
        ax: Any | None = None,
    ) -> None:
        target = _axes(ax)
        center = np.array([x, y]) - 0.25 * length * np.array([np.cos(yaw), np.sin(yaw)])
        local = np.array(
            [
                [-length / 2, -width / 2],
                [-length / 2, width / 2],
                [length / 2, width / 2],
                [length / 2, -width / 2],
                [-length / 2, -width / 2],
            ],
            dtype=float,
        )
        rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]], dtype=float)
        corners = local @ rotation.T + center
        target.plot(corners[:, 0], corners[:, 1], color="black", linewidth=1)
        Arrow(x, y, yaw, length / 2, "black", ax=target)


__all__ = ["Arrow", "Car"]
