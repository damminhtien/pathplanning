""" 
Depth-first Search (DFS) implementation for pathfinding in a 2D grid environment.

author: damminhtien
"""
import math
import heapq

from pathplanning.planners.search.astar import Astar
from pathplanning.viz import search2d_plotting as plotting

class DepthFirstSearch(Astar):
    """DFS add the new visited node in the front of the openset
    """
    def searching(self):
        """
        Breadth-first Searching.
        :return: path, visited order
        """

        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,
                       (0, self.s_start))

        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)

            if s == self.s_goal:
                break

            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g:
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s

                    # dfs, add new node to the front of the openset
                    prior = self.OPEN[0][0]-1 if len(self.OPEN)>0 else 0
                    heapq.heappush(self.OPEN, (prior, s_n))

        return self.extract_path(self.PARENT), self.CLOSED


def main():
    """ Run a demo of the Depth-first Search planner."""
    s_start = (5, 5)
    s_goal = (45, 25)

    dfs = DepthFirstSearch(s_start, s_goal, 'None')
    plot = plotting.Plotting(s_start, s_goal)

    path, visited = dfs.searching()
    visited = list(dict.fromkeys(visited))
    plot.animate(path, visited, "Depth-first Searching (DFS)")  # animation


if __name__ == '__main__':
    main()
