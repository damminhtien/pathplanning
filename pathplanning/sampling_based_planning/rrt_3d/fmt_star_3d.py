"""
This is fast marching tree* code for 3D
@author: yue qi 
source: Janson, Lucas, et al. "Fast marching tree: A fast marching sampling-based method 
        for optimal motion planning in many dimensions." 
        The International journal of robotics research 34.7 (2015): 883-921.
"""
import numpy as np
import copy

from .env_3d import Environment3D
from .utils_3d import getDist, sampleFree, isCollide
from .queue import MinheapPQ

class FMT_star:

    def __init__(self, radius = 1, n = 1000):
        self.env = Environment3D()
        # init start and goal
            # note that the xgoal could be a region since this algorithm is a multiquery method
        self.xinit, self.xgoal = tuple(self.env.start), tuple(self.env.goal)
        self.x0, self.xt = tuple(self.env.start), tuple(self.env.goal) # used for sample free
        self.n = n # number of samples
        self.radius = radius # radius of the ball
        # self.radius = 40 * np.sqrt((np.log(self.n) / self.n))
        # sets
        self.Vopen, self.Vopen_queue, self.Vclosed, self.V, self.Vunvisited, self.c = self.initNodeSets()
        # make space for save 
        self.neighbors = {}
        # additional
        self.done = True
        self.Path = []
        self.Parent = {}

    def generateSampleSet(self, n):
        V = set()
        for i in range(n):
            V.add(tuple(sampleFree(self, bias = 0.0)))
        return V

    def initNodeSets(self):
        # open set
        Vopen = {self.xinit} # open set
        # closed set
        closed = set()
        # V, Vunvisited set 
        V = self.generateSampleSet(self.n - 2) # set of all nodes
        Vunvisited = copy.deepcopy(V) # unvisited set
        Vunvisited.add(self.xgoal)
        V.add(self.xinit)
        V.add(self.xgoal)
        # initialize all cost to come at inf
        c = {node : np.inf for node in V}
        c[self.xinit] = 0
        # use a min heap to speed up
        Vopen_queue = MinheapPQ()
        Vopen_queue.put(self.xinit, c[self.xinit]) # priority organized as the cost to come
        return Vopen, Vopen_queue, closed, V, Vunvisited, c

    def Near(self, nodeset, node, rn):
        if node in self.neighbors:
            return self.neighbors[node]
        validnodes = {i for i in nodeset if getDist(i, node) < rn}
        return validnodes

    def Save(self, V_associated, node):
        self.neighbors[node] = V_associated

    def path(self, z, initT):
        path = []
        s = self.xgoal
        i = 0
        while s != self.xinit:
            path.append((s, self.Parent[s]))
            s = self.Parent[s]
            if i > self.n:
                break
            i += 1
        return path

    def Cost(self, x, y):
        # collide, dist = isCollide(self, x, y)
        # if collide:
        #     return np.inf
        # return dist
        return getDist(x, y)

    def FMTrun(self):
        z = self.xinit
        rn = self.radius
        Nz = self.Near(self.Vunvisited, z, rn)
        E = set()
        self.Save(Nz, z)
        ind = 0
        while z != self.xgoal:
            Vopen_new = set()
            #Nz = self.Near(self.Vunvisited, z, rn)
            #self.Save(Nz, z)
            #Xnear = Nz.intersection(self.Vunvisited)
            Xnear = self.Near(self.Vunvisited, z ,rn)
            self.Save(Xnear, z)
            for x in Xnear:
                #Nx = self.Near(self.V.difference({x}), x, rn)
                #self.Save(Nx, x)
                #Ynear = list(Nx.intersection(self.Vopen))
                Ynear = list(self.Near(self.Vopen, x, rn))
                # self.Save(set(Ynear), x)
                ymin = Ynear[np.argmin([self.c[y] + self.Cost(y,x) for y in Ynear])] # DP programming equation
                collide, _ = isCollide(self, ymin, x)
                if not collide:
                    E.add((ymin, x)) # straight line joining ymin and x is collision free
                    Vopen_new.add(x)
                    self.Parent[x] = z
                    self.Vunvisited = self.Vunvisited.difference({x})
                    self.c[x] = self.c[ymin] + self.Cost(ymin, x) # estimated cost-to-arrive from xinit in tree T = (VopenUVclosed, E)
            # update open set
            self.Vopen = self.Vopen.union(Vopen_new).difference({z})
            self.Vclosed.add(z)
            if len(self.Vopen) == 0:
                print('Failure')
                return 
            ind += 1
            print(str(ind) + ' node expanded')
            # update current node
            Vopenlist = list(self.Vopen)
            z = Vopenlist[np.argmin([self.c[y] for y in self.Vopen])]
        # creating the tree
        T = (self.Vopen.union(self.Vclosed), E)
        self.done = True
        self.Path = self.path(z, T)
        # return self.path(z, T)

if __name__ == '__main__':
    A = FMT_star(radius = 1, n = 3000)
    A.FMTrun()
