# -*- coding: utf-8 -*-
"""
This is the simple Rapidly-Exploring Random Trees algorithms 

@author: Yifan Tang
"""

import matplotlib.pyplot as plt
import random
import math
import copy


class RRT():
    """
   Class for basic RRT motion planning 
   """

    def __init__ (self, start, goal, stepsize, obstacle = [], TreeArea = [], SampleRate=2, maxIter=500):
        """
       This is the initialization for class RRT
       Parameters:
           start : the start position [x, y]
           goal : the goal position [x, y]
           stepsize : the expansion stepsize for new node
           obstacle : a list to define area of obstacle
           TreeArea : the area for generating random points
           maxIter : the maximum iteration number
       """
        self.start = Node(start[0], start[1])
        self.cur_goal = Node(goal[0], goal[1])
        self.stepsize = stepsize
        self.obstacle = obstacle
        self.min_rand = TreeArea[0]
        self.max_rand = TreeArea[1]
        self.SampleRate = SampleRate
        self.maxIter = maxIter
        self.path = [[self.cur_goal.x, self.cur_goal.y]]

    def GrowTree(self):
        """
       Grow the RRT and return the node list of tree
       
       """
        self.nodelist = [self.start]
        step = 0
        while True:
            step += 1
            if random.randint(0, 10) > self.SampleRate:
                node_rnd = [random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_rand, self.max_rand)]
            else:
                node_rnd = [self.cur_goal.x, self.cur_goal.y]

            # Find the nearest node
            node_nearest_index = self.FindNearestNode(self.nodelist, node_rnd, mode="Mahattan")
            node_nearest = self.nodelist[node_nearest_index]

            # Expand Tree
            theta = math.atan2(node_rnd[1] - node_nearest.y, node_rnd[0] - node_nearest.x)

            node_new = copy.deepcopy(node_nearest)
            node_new.x += self.stepsize * math.cos(theta)
            node_new.y += self.stepsize * math.sin(theta)
            #           node_new.x += (node_rnd[0] - node_nearest.x) * math.cos(theta)
            #           node_new.y += (node_rnd[1] - node_nearest.y) * math.sin(theta)
            node_new.parent = node_nearest_index

            # Collision check
            if not self.CollisionCheck(node_new, self.obstacle):
                continue

            # add new node to list
            self.nodelist.append(node_new)

            # Check whether reach goal
            goal_dist = math.sqrt((node_new.x - self.cur_goal.x) ** 2 + (node_new.y - self.cur_goal.y) ** 2)
            if goal_dist <= self.stepsize:
                print("The algorithm finds the goal after %d steps" % step)
                return True
            # draw the graph after finding new node
            # self.DrawTree()
            if (step >= self.maxIter):
                print('after ', self.maxIter, "still don't find the goal")
                return False

    def FindNearestNode(self, nodelist, node_rnd, mode="Euclidean"):
        """
       Use Euclidean Distance or Mahattan Distance to find the nearest node
       """
        node_index = 0
        if mode == "Euclidean":
            min_dist = (self.start.x - node_rnd[0]) ** 2 + (self.start.y - node_rnd[1]) ** 2
            for iter in range(len(nodelist)):
                distance = (nodelist[iter].x - node_rnd[0]) ** 2 + (nodelist[iter].y - node_rnd[1]) ** 2
                if distance <= min_dist:
                    min_dist = distance
                    node_index = iter
        elif mode == "Mahattan":
            min_dist = (self.start.x - node_rnd[0]) + (self.start.y - node_rnd[1])
            for item in range(len(nodelist)):
                distance = abs(nodelist[item].x - node_rnd[0]) + abs(nodelist[item].y - node_rnd[1])
                if distance <= abs(min_dist):
                    min_dist = distance
                    node_index = item
        else:
            raise NameError('distance name is wrong!')
        return node_index

    def CollisionCheck(self, node_new, obstacle):
        """
       Check whether the path from new node to its parents collide with obstacles
       """
        node_parent = self.nodelist[node_new.parent]
        dist_parent_child = math.sqrt((node_parent.x - node_new.x) ** 2 +
                                      (node_parent.y - node_new.y) ** 2)
        for (x, y, radius) in obstacle:
            dist = math.sqrt((x - node_new.x) ** 2 + (y - node_new.y) ** 2)
            if dist <= radius:
                return False
            elif math.sqrt(dist ** 2 - (dist_parent_child / 2) ** 2) <= radius:
                return False
        return True

    def FindPath(self):
        """
       Find path from the goal node to start node
       """

        index = len(self.nodelist) - 1
        while (self.nodelist[index].parent) is not None:
            node_path = self.nodelist[index]
            # print(node_path)
            self.path.append([node_path.x, node_path.y])
            # print(self.path)
            index = node_path.parent
        self.path.append([self.start.x, self.start.y])

    def DrawTree(self, find_path=False, result=False):
        """
       Draw the Rapid-explored Random Tree and also draw the obstacle area
       """
        #plt.plot(self.cur_goal.x, self.cur_goal.y, 'ob')
        plt.plot(self.start.x, self.start.y, 'ro')
        if result == False:
            for node in self.nodelist:
                if node.parent is not None:
                    plt.plot([node.x, self.nodelist[node.parent].x],
                             [node.y, self.nodelist[node.parent].y], '-k')

        else:
            if (find_path == True):
                plt.plot([x for (x, y) in self.path], [y for (x, y) in self.path], '-r')
                plt.plot(self.start.x, self.start.y, 'ro')

        # plot the obstacle
        ax = plt.gca()
        ax.set_xlim((self.min_rand, self.max_rand))
        ax.set_ylim((self.min_rand, self.max_rand))

        for (x, y, radius) in self.obstacle:
            circle = plt.Circle((x, y), radius, color='b')
            ax.add_artist(circle)
            # plt.axis([self.min_rand, self.max_rand , self.min_rand, self.max_rand])
            # plt.grid(True)
        plt.show()
        plt.pause(0.01)


class Node():
    """
    Node for flexible RRT
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def main():
    # set parameters for RRT
    start = [0, 0]
    goal = [9, 10]
    obstacle = [
        (3, 6, 1),
        (3, 8, 1),
        (10, 3, 2),
        (7, 8, 1),
        (9, 5, 2),
        (5, 5, 1)
    ]

    # start planning
    rrt = RRT(start, goal, stepsize=1, obstacle=obstacle, TreeArea=[0, 15])
    find_path = rrt.GrowTree()
    if find_path == True:
        rrt.FindPath()
    else:
        pass
    rrt.DrawTree(find_path, result=True)


if __name__ == '__main__':
    main()
