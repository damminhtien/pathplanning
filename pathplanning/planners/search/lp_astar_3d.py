import numpy as np

from pathplanning.viz import lazy_import

plt = lazy_import("matplotlib.pyplot")


import time

import pyrr

from pathplanning.spaces.environment3d import env
from pathplanning.utils import priority_queue as queue

from .plot_util_3d import visualization
from .utils_3d import (
    g_space,
    get_dist,
    get_nearest,
    get_ray,
    heuristic,
    is_collide,
    isinball,
    isinbound,
)


class Lifelong_Astar:
    def __init__(self, resolution=1):
        self.Alldirec = {
            (1, 0, 0): 1,
            (0, 1, 0): 1,
            (0, 0, 1): 1,
            (-1, 0, 0): 1,
            (0, -1, 0): 1,
            (0, 0, -1): 1,
            (1, 1, 0): np.sqrt(2),
            (1, 0, 1): np.sqrt(2),
            (0, 1, 1): np.sqrt(2),
            (-1, -1, 0): np.sqrt(2),
            (-1, 0, -1): np.sqrt(2),
            (0, -1, -1): np.sqrt(2),
            (1, -1, 0): np.sqrt(2),
            (-1, 1, 0): np.sqrt(2),
            (1, 0, -1): np.sqrt(2),
            (-1, 0, 1): np.sqrt(2),
            (0, 1, -1): np.sqrt(2),
            (0, -1, 1): np.sqrt(2),
            (1, 1, 1): np.sqrt(3),
            (-1, -1, -1): np.sqrt(3),
            (1, -1, -1): np.sqrt(3),
            (-1, 1, -1): np.sqrt(3),
            (-1, -1, 1): np.sqrt(3),
            (1, 1, -1): np.sqrt(3),
            (1, -1, 1): np.sqrt(3),
            (-1, 1, 1): np.sqrt(3),
        }
        self.env = env(resolution=resolution)
        self.g = g_space(self)
        self.start, self.goal = (
            get_nearest(self.g, self.env.start),
            get_nearest(self.g, self.env.goal),
        )
        self.x0, self.xt = self.start, self.goal
        self.rhs = g_space(self)  # rhs(.) = g(.) = inf
        self.rhs[self.start] = 0  # rhs(x0) = 0
        self.h = heuristic(self.g, self.goal)

        self.OPEN = queue.MinheapPQ()  # store [point,priority]
        self.OPEN.put(self.x0, [self.h[self.x0], 0])
        self.CLOSED = set()

        # used for A*
        self.done = False
        self.Path = []
        self.V = []
        self.ind = 0

        # initialize children list
        self.CHILDREN = {}
        self.build_children_map()

        # initialize Cost list
        self.COST = {}
        _ = self.costset()

    def costset(self):
        NodeToChange = set()
        for xi in self.CHILDREN:
            children = self.CHILDREN[xi]
            toUpdate = [self.cost(xj, xi) for xj in children]
            if xi in self.COST:
                # if the old Cost not equal to new Cost
                diff = np.not_equal(self.COST[xi], toUpdate)
                cd = np.array(children)[diff]
                for i in cd:
                    NodeToChange.add(tuple(i))
                self.COST[xi] = toUpdate
            else:
                self.COST[xi] = toUpdate
        return NodeToChange

    def get_cost_entry(self, xi, xj):
        children = self.CHILDREN[xi]
        for ind, child in enumerate(children):
            if child == xj:
                return self.COST[xi][ind]

    def children(self, x):
        allchild = []
        resolution = self.env.resolution
        for direc in self.Alldirec:
            child = tuple(map(np.add, x, np.multiply(direc, resolution)))
            if isinbound(self.env.boundary, child):
                allchild.append(child)
        return allchild

    def build_children_map(self):
        for xi in self.g:
            self.CHILDREN[xi] = self.children(xi)

    def is_collide(self, x, child):
        ray, dist = get_ray(x, child), get_dist(x, child)
        if not isinbound(self.env.boundary, child):
            return True, dist
        for i in self.env.AABB_pyrr:
            shot = pyrr.geometric_tests.ray_intersect_aabb(ray, i)
            if shot is not None:
                dist_wall = get_dist(x, shot)
                if dist_wall <= dist:  # collide
                    return True, dist
        for i in self.env.balls:
            if isinball(i, child):
                return True, dist
            shot = pyrr.geometric_tests.ray_intersect_sphere(ray, i)
            if shot != []:
                dists_ball = [get_dist(x, j) for j in shot]
                if all(dists_ball <= dist):  # collide
                    return True, dist
        return False, dist

    def cost(self, x, y):
        collide, dist = is_collide(self, x, y)
        if collide:
            return np.inf
        else:
            return dist

    def key(self, xi, epsilion=1):
        return [
            min(self.g[xi], self.rhs[xi]) + epsilion * self.h[xi],
            min(self.g[xi], self.rhs[xi]),
        ]

    def path(self):
        path = []
        x = self.xt
        start = self.x0
        ind = 0
        while x != start:
            j = x
            nei = self.CHILDREN[x]
            gset = [self.g[xi] for xi in nei]
            # collision check and make g Cost inf
            for i in range(len(nei)):
                if is_collide(self, nei[i], j)[0]:
                    gset[i] = np.inf
            parent = nei[np.argmin(gset)]
            path.append([x, parent])
            x = parent
            if ind > 100:
                break
            ind += 1
        return path

    # ------------------Lifelong Plannning A*
    def update_membership(self, xi, xparent=None):
        if xi != self.x0:
            self.rhs[xi] = min([self.g[j] + self.get_cost_entry(xi, j) for j in self.CHILDREN[xi]])
        self.OPEN.check_remove(xi)
        if self.g[xi] != self.rhs[xi]:
            self.OPEN.put(xi, self.key(xi))

    def compute_path(self):
        print("computing path ...")
        while self.key(self.xt) > self.OPEN.top_key() or self.rhs[self.xt] != self.g[self.xt]:
            xi = self.OPEN.get()
            # if g > rhs, overconsistent
            if self.g[xi] > self.rhs[xi]:
                self.g[xi] = self.rhs[xi]
                # add xi to expanded node set
                if xi not in self.CLOSED:
                    self.V.append(xi)
                self.CLOSED.add(xi)
            else:  # underconsistent and consistent
                self.g[xi] = np.inf
                self.update_membership(xi)
            for xj in self.CHILDREN[xi]:
                self.update_membership(xj)

            # visualization(self)
            self.ind += 1
        self.Path = self.path()
        self.done = True
        visualization(self)
        plt.pause(1)

    def change_env(self):
        _, _ = self.env.move_block(block_to_move=1, a=[0, 2, 0])
        self.done = False
        self.Path = []
        self.CLOSED = set()
        N = self.costset()
        for xi in N:
            self.update_membership(xi)


if __name__ == "__main__":
    sta = time.time()
    Astar = Lifelong_Astar(1)
    Astar.compute_path()
    for _i in range(5):
        Astar.change_env()
        Astar.compute_path()
        plt.pause(1)

    print(time.time() - sta)
