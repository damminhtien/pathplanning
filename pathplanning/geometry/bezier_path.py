"""
bezier path

author: Atsushi Sakai(@Atsushi_twi)
modified: damminhtien
"""

import numpy as np
from scipy.special import comb


def calc_4points_bezier_path(sx, sy, syaw, gx, gy, gyaw, offset):

    dist = np.hypot(sx - gx, sy - gy) / offset
    control_points = np.array(
        [
            [sx, sy],
            [sx + dist * np.cos(syaw), sy + dist * np.sin(syaw)],
            [gx - dist * np.cos(gyaw), gy - dist * np.sin(gyaw)],
            [gx, gy],
        ]
    )

    path = calc_bezier_path(control_points, n_points=100)

    return path, control_points


def calc_bezier_path(control_points, n_points=100):
    traj = []

    for t in np.linspace(0, 1, n_points):
        traj.append(bezier(t, control_points))

    return np.array(traj)


def Comb(n, i, t):
    return comb(n, i) * t**i * (1 - t) ** (n - i)


def bezier(t, control_points):
    n = len(control_points) - 1
    return np.sum([Comb(n, i, t) * control_points[i] for i in range(n + 1)], axis=0)


def bezier_derivatives_control_points(control_points, n_derivatives):
    w = {0: control_points}

    for i in range(n_derivatives):
        n = len(w[i])
        w[i + 1] = np.array([(n - 1) * (w[i][j + 1] - w[i][j]) for j in range(n - 1)])

    return w


def curvature(dx, dy, ddx, ddy):
    return (dx * ddy - dy * ddx) / (dx**2 + dy**2) ** (3 / 2)
