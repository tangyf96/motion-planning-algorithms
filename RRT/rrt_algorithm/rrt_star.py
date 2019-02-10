"""
Path planning Sample Code with RRT*

author: AtsushiSakai(@Atsushi_twi)

"""

import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt

show_animation = True


class RRT():
    """
    Class for RRT planning
    """

    def __init__(self,
                 start,
                 goal,
                 obstacleList=[],
                 randArea=[],
                 expandDis=0.5,
                 goalSampleRate=20,
                 maxIter=400):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0], start[1])
        self.cur_goal = Node(goal[0], goal[1])
        self.min_rand = randArea[0]
        self.max_rand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacle = obstacleList
        self.path = [[self.cur_goal.x, self.cur_goal.y]]
        self.nodelist = []

    def planning(self, animation=False):
        """
        Path planning

        animation: flag for animation on or off
        """

        self.nodelist.append(self.start)
        for i in range(self.maxIter):
            rnd = self.get_random_point()
            nind = self.GetNearestListIndex(self.nodelist, rnd)

            newNode = self.steer(rnd, nind)
            #  print(newNode.cost)

            if self.__CollisionCheck(newNode, self.obstacle):
                nearinds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, nearinds)
                self.nodelist.append(newNode)
                self.rewire(newNode, nearinds)

            if animation:
                self.DrawGraph(rnd)

        # generate course
        lastIndex = self.get_best_last_index()
        if lastIndex is None:
            return None
        path = self.gen_final_course(lastIndex)
        return path

    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            dx = newNode.x - self.nodelist[i].x
            dy = newNode.y - self.nodelist[i].y
            d = math.sqrt(dx**2 + dy**2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.nodelist[i], theta, d):
                dlist.append(self.nodelist[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        newNode.cost = mincost
        newNode.parent = minind

        return newNode

    def steer(self, rnd, nind):

        # expand tree
        nearestNode = self.nodelist[nind]
        theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
        newNode = copy.deepcopy(nearestNode)
        newNode.x += self.expandDis * math.cos(theta)
        newNode.y += self.expandDis * math.sin(theta)

        newNode.cost += self.expandDis
        newNode.parent = nind
        return newNode

    def get_random_point(self):

        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand)
            ]
        else:  # goal point sampling
            rnd = [self.cur_goal.x, self.cur_goal.y]

        return rnd

    def get_best_last_index(self):

        disglist = [
            self.calc_dist_to_goal(node.x, node.y) for node in self.nodelist
        ]
        goalinds = [disglist.index(i) for i in disglist if i <= self.expandDis]
        #  print(goalinds)

        if len(goalinds) == 0:
            return None

        mincost = min([self.nodelist[i].cost for i in goalinds])
        for i in goalinds:
            if self.nodelist[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind):
        while self.nodelist[goalind].parent is not None:
            node = self.nodelist[goalind]
            self.path.append([node.x, node.y])
            goalind = node.parent
        self.path.append([self.start.x, self.start.y])
        self.path.reverse()
        return self.path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.cur_goal.x, y - self.cur_goal.y])

    def find_near_nodes(self, newNode):
        nnode = len(self.nodelist)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        #  r = self.expandDis * 5.0
        dlist = [(node.x - newNode.x)**2 + (node.y - newNode.y)**2
                 for node in self.nodelist]
        nearinds = [dlist.index(i) for i in dlist if i <= r**2 and i >= 1e-3]
        return nearinds

    def rewire(self, newNode, nearinds):
        # rewire the tree to find if new node can reduce the cost of existing node
        nnode = len(self.nodelist)
        for i in nearinds:
            nearNode = self.nodelist[i]

            dx = newNode.x - nearNode.x
            dy = newNode.y - nearNode.y
            d = math.sqrt(dx**2 + dy**2)

            scost = newNode.cost + d

            if nearNode.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(nearNode, theta, d):
                    nearNode.parent = nnode - 1
                    nearNode.cost = scost

    def check_collision_extend(self, nearNode, theta, d):

        tmpNode = copy.deepcopy(nearNode)
        # use interpolation to check if there is collision
        for i in range(int(d / self.expandDis)):
            tmpNode.x += self.expandDis * math.cos(theta)
            tmpNode.y += self.expandDis * math.sin(theta)
            if not self.__CollisionCheck(tmpNode, self.obstacle):
                return False # not safe

        return True # safe

    def DrawGraph(self, rnd=None):
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodelist:
            if node.parent is not None:
                plt.plot([node.x, self.nodelist[node.parent].x],
                         [node.y, self.nodelist[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacle:
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.cur_goal.x, self.cur_goal.y, "xr")
        plt.plot([x for (x, y) in self.path], [y for (x, y) in self.path], '-r')
        plt.axis([-2, 15, -2, 15])

        plt.grid(True)
        plt.pause(0.01)

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0])**2 + (node.y - rnd[1])**2
                 for node in nodeList]
        minind = dlist.index(min(dlist))

        return minind

    def __CollisionCheck(self, node, obstacleList):
        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = dx * dx + dy * dy
            if d <= size**2:
                return False  # collision

        return True  # safe

    def reset(self):
        self.path = [[self.cur_goal.x, self.cur_goal.y]]
        self.nodelist = []

class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


def main():
    print("Start rrt planning")

    # ====Search Path with RRT====
    #obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
    #                (9, 5, 2)]  # [x,y,size(radius)]
    obstacle = [(8, 8, 1), (6, 6, 1), (12, 12, 1)]
    # Set Initial parameters
    rrt = RRT(
        start=[4.628168491031819, 5.228134513254739],
        goal=[2, 14],
        randArea=[-1, 15],
        obstacleList=obstacle)
    path = rrt.planning()

    # calculate path distance
    dist = 0
    prev = rrt.path[0]
    for point in rrt.path:
        dist += math.sqrt((point[0] - prev[0])**2 +
                            (point[1] - prev[1])**2)
        prev = point
    print("path distance is:", dist)

    # Draw final path
    if show_animation:
        rrt.DrawGraph()
        #plt.grid(True)
        #plt.pause(0.01)  # Need for Mac
        plt.show()


if __name__ == '__main__':
    main()
