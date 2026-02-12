# this is the three dimensional N>1 LRTA* algo
# !/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: yue qi
"""

import numpy as np

from pathplanning.viz import lazy_import

plt = lazy_import("matplotlib.pyplot")


from pathplanning.planners.search import astar_3d

from .plot_util_3d import visualization
from .utils_3d import (
    children,
    cost,
    is_collide,
)


class LRT_A_star2:
    def __init__(self, resolution=0.5, N=7):
        self.N = N
        self.Astar = astar_3d.Weighted_A_star(resolution=resolution)
        self.path = []

    def update_heuristic(self):
        # Initialize hvalues at infinity
        for xi in self.Astar.CLOSED:
            self.Astar.h[xi] = np.inf
        Diff = True
        while Diff:  # repeat DP until converge
            hvals, lasthvals = [], []
            for xi in self.Astar.CLOSED:
                lasthvals.append(self.Astar.h[xi])
                # update h values if they are smaller
                Children = children(self.Astar, xi)
                minfval = min(
                    [cost(self.Astar, xi, xj, settings=0) + self.Astar.h[xj] for xj in Children]
                )
                # h(s) = h(s') if h(s) > cBest(s,s') + h(s')
                if self.Astar.h[xi] >= minfval:
                    self.Astar.h[xi] = minfval
                hvals.append(self.Astar.h[xi])
            if lasthvals == hvals:
                Diff = False

    def move(self):
        st = self.Astar.x0
        ind = 0
        # find the lowest path down hill
        while (
            st in self.Astar.CLOSED
        ):  # when minchild in CLOSED then continue, when minchild in OPEN, stop
            Children = children(self.Astar, st)
            minh, minchild = np.inf, None
            for child in Children:
                # check collision here, not a supper efficient
                collide, _ = is_collide(self.Astar, st, child)
                if collide:
                    continue
                h = self.Astar.h[child]
                if h <= minh:
                    minh, minchild = h, child
            self.path.append([st, minchild])
            st = minchild
            for _, p in self.Astar.OPEN.enumerate():
                if p == st:
                    break
            ind += 1
            if ind > 1000:
                break
        self.Astar.reset(st)

    def run(self):
        while True:
            if self.Astar.run(N=self.N):
                self.Astar.Path = self.Astar.Path + self.path
                self.Astar.done = True
                visualization(self.Astar)
                plt.show()
                break
            self.update_heuristic()
            self.move()


if __name__ == "__main__":
    T = LRT_A_star2(resolution=0.5, N=100)
    T.run()
