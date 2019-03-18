#-*- coding: utf-8 -*-
"""
This is the flexible Rapidly-Exploring Random Trees algorithms
considering the probability of goal change

@author: Yifan Tang
"""

import matplotlib.pyplot as plt
import random
import math
import copy
import numpy as np
import operator

class fRRT():
    """
    Class for flexible RRT planning
    """
    def __init__(self, start, cur_goal, goal_list = [], obstacle = [], TreeArea = [], trans_prob = [], stepsize = 0.5, SampleRate = 2, maxIter = 800):
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
        self.cur_goal = Node(cur_goal[0], cur_goal[1])
        self.stepsize = stepsize
        self.obstacle = obstacle
        self.min_rand = TreeArea[0]
        self.max_rand = TreeArea[1]
        self.SampleRate = SampleRate
        self.maxIter = maxIter
        self.path = [[self.cur_goal.x, self.cur_goal.y]]
        self.nodelist = []
        self.trans_prob = copy.deepcopy(trans_prob)
        self.goal_list = []
        self.tran_weight = 20
        self.path_node = []
        for (x, y) in goal_list:
            self.goal_list.append(Node(x, y))
        print("initialize frrt object")

    def planning(self):
        """
        Path planning
        """
        self.grow_tree()
        # find the closest node to the goal
        best_node_ind = self.get_best_node()
        if best_node_ind is None:
            return None

        self.FindPath(best_node_ind)
        return self.path

    def get_best_node(self):
        node_dist = [
            self.node_dist(node, self.cur_goal) for node in self.nodelist
            ]
        pos_node_ind = [node_dist.index(i) for i in node_dist if i <= self.stepsize]
        if len(pos_node_ind) == 0:
            return None
        # find the node with the minimum cost
        node_cost = [self.nodelist[i].cost for i in pos_node_ind]
        min_cost = min(node_cost)
        min_cost_ind = node_cost.index(min_cost)
        return pos_node_ind[min_cost_ind]

    def grow_tree(self):
        """
        flexible RRT Path planning
        :return: self.nodelist
        """
        temp = self.trans_cost(self.start)
        self.start.cost = self.tran_weight * temp
        self.nodelist.append(self.start)
        for step in range(self.maxIter):
                
            # generate random point
            node_rnd = self.get_random_point()

            # find nearest node
            node_nearest_index = self.FindNearestNode(Node(node_rnd[0], node_rnd[1]))

            # expand tree
            node_new = self.expand(node_rnd, node_nearest_index)

            # collision check
            if self.collision_check(node_new, self.nodelist[node_new.parent]):
                continue
            # find neighbors in an area around new node
            near_ind = self.find_near_nodes(node_new)
            # choose parent for new node with the minimum cost
            node_new = self.choose_parent(node_new, near_ind)
            # if can't choose possible parent
            if node_new.parent is None:
                continue
            # add to nodelist
            self.nodelist.append(node_new)
            # rewire neighbors
            self.rewire(node_new, near_ind)
            # self.DrawTree()

    def reset(self):
        #self.cur_goal = Node(cur_goal[0], cur_goal[1])
        self.path = [[self.cur_goal.x, self.cur_goal.y]]
        self.nodelist = []
        self.path_node = []

    def rewire(self, node_new, near_ind):
        """
        Rewire the tree, check if neighbors could reduce cost
        by choosing node_new as parent.
        """
        new_node_ind = len(self.nodelist) - 1

        for ind in near_ind:
            node = self.nodelist[ind]
            dist = self.node_dist(node_new, node)
            new_cost = node_new.cost + dist + self.tran_weight * self.trans_cost(node)
            # new_cost = node_new.cost + self.tran_weight * self.trans_cost(node)
            if new_cost < node.cost:
                if self.collision_check(node, node_new):
                    # collision
                    continue
                else:
                    # no collision
                    node.parent = new_node_ind
                    node.cost = new_cost
                    # print(self.nodelist[ind].cost)

    def FindPath(self, index):
        """
        Find path from the current goal to start node
        :return: self.path
        """
        while index is not None:
            node = self.nodelist[index]
            self.path_node.append(node)
            self.path.append([node.x, node.y])
            index = node.parent
        self.path.reverse()

    def choose_parent(self, node_new, near_ind):
        """
        Choose parent for new node from the neighbors
        :param near_ind: neighbor nodes index in nodelist
        :return: node_new
        """
        if len(near_ind) == 0:
            return node_new

        near_node_cost = []
        for ind in near_ind:
            node_new.parent = ind
            if node_new.x == self.nodelist[ind].x:
                print('error')
            if self.collision_check(node_new, self.nodelist[ind]):
                # collision, check next neighbor
                near_node_cost.append(float("inf"))
            else:
                near_node_cost.append(self.node_dist(node_new, self.nodelist[ind]) +
                                      self.nodelist[ind].cost)
        # find the neighbor with the minimum distance
        min_cost = min(near_node_cost)
        min_cost_ind = near_ind[near_node_cost.index(min_cost)]
        if min_cost == float("inf"):
            print("minimum distance is inf")
            node_new.parent = None
            node_new.cost = float("inf")
            return node_new
        else:
            node_new.parent = min_cost_ind
            # change cost, consider transition probability cost
            node_new.cost = min_cost + self.tran_weight * self.trans_cost(node_new)
            #node_new.cost = self.tran_weight * self.trans_cost(node_new)

        return node_new

    def find_near_nodes(self, node_new, alpha=5.0):
        """
        Find the nodes in the circle around new node
        :return: indices of nodes in the area
        """
        r = self.stepsize * alpha
        near_node_dist = []
        for node in self.nodelist:
            if abs(node_new.x - node.x) < 1e-4:
                # skip node_new itself
                near_node_dist.append(float("inf"))
            else:
                near_node_dist.append(self.node_dist(node_new, node))
        near_ind = [near_node_dist.index(i) for i in near_node_dist if (i <= r and i != 0)]
        return near_ind

    def expand(self, node_rnd, node_index):
        """
        Expand the tree from the parent node to the random node by a stepsize
        :param node_rnd: random node
        :param node_index: node_rnd's nearest node's index
        :return: new node that should be added to the tree nodelist
        """

        node_nearest = self.nodelist[node_index]
        theta = math.atan2(node_rnd[1] - node_nearest.y, node_rnd[0] - node_nearest.x)

        node_new = copy.deepcopy(node_nearest)
        node_new.x += self.stepsize * math.cos(theta)
        node_new.y += self.stepsize * math.sin(theta)

        node_new.parent = node_index
        # reset the cost
        node_new.cost = 0
        # don't update the cost here
        # update the cost when chosing parent
        return node_new

    def trans_cost(self, node_new):
        """
        Calculate the transition cost if the robot need to replan when current goal changes
        :return: transition probability cost
        """
        cost = 0
        for ind, node in enumerate(self.goal_list):
            if self.cur_goal == node:
                # print("check")
                to_goal_ind = ind
                break

        #cur_goal = self.goal_list[from_goal_ind]
        dist = np.array([self.node_dist(node_new, goal) for goal in self.goal_list])
        assert to_goal_ind is not None
        try:
            dist[to_goal_ind] = 0
        except:
            print('to_goal_ind assigned error')
        
        transition_prob = self.trans_prob[:, to_goal_ind]
        cost = np.sum(dist * transition_prob)

        return cost

    def node_dist(self, node_new, goal):
        """
        Calculte the distance to possible goal
        Input : node_new is a Node object
                goal is also a Node object
        """
        dist = np.linalg.norm([goal.x - node_new.x, goal.y - node_new.y])
        return dist

    def FindNearestNode(self, node_rnd):
        """
        Use Euclidean Distance to find the nearest node
        """
        dist = [self.node_dist(node_rnd, node) for node in self.nodelist]
        nearest_ind = dist.index(min(dist))
        return nearest_ind

    def get_random_point(self):
        if random.randint(0,10) > self.SampleRate:
            rnd = [random.uniform(self.min_rand, self.max_rand),
                   random.uniform(self.min_rand, self.max_rand)]
        else:
            rnd = [self.cur_goal.x, self.cur_goal.y]
        return rnd

    def collision_check(self, node_new, node_parent):
        """
        Check whether the path from new node to its parents collides with obstacles
        If there is Collision, return True;
        """
        division = 10
        delta_x = (node_new.x - node_parent.x) / division
        delta_y = (node_new.y - node_parent.y) / division
        for i in range(division+1):
            interpolate_x = delta_x * i + node_parent.x
            interpolate_y = delta_y * i + node_parent.y
            for (ox, oy, radius) in self.obstacle:
                dx = ox - interpolate_x
                dy = oy - interpolate_y
                d = dx**2 + dy**2
                if d <= radius**2:
                    return True # collision
        return False # safe

    def DrawTree(self):
        plt.clf()
        plt.plot(self.start.x, self.start.y, 'ro')
        plt.plot(self.cur_goal.x, self.cur_goal.y, 'bo')
        for node in self.nodelist:
            if node.parent is not None:
                plt.plot([node.x, self.nodelist[node.parent].x],
                         [node.y, self.nodelist[node.parent].y],'-g')
        ax = plt.gca()
        ax.set_xlim((self.min_rand, self.max_rand))
        ax.set_ylim((self.min_rand, self.max_rand))

        for (x, y, radius) in self.obstacle:
            circle = plt.Circle((x, y), radius, color='y')
            ax.add_artist(circle)
        plt.show()
        plt.pause(0.01)

    def DrawPath(self):
        print('begin to draw path!')
        fig = plt.gcf()
        #fig = plt.figure(1)
        fig.clf()
        ax = plt.gca()
        ax.set_xlim((self.min_rand, self.max_rand))
        ax.set_ylim((self.min_rand, self.max_rand))
        # plot path
        ax.plot([x for (x,y) in self.path],[y for (x,y) in self.path], "-k")
        ax.plot([x for (x,y) in self.path],[y for (x,y) in self.path], "yo")
        # plot goal list
        for node in self.goal_list:
            ax.plot(node.x, node.y, 'g*')
        # plot start and current goal
        ax.plot(self.start.x, self.start.y, 'ro')
        ax.plot(self.cur_goal.x, self.cur_goal.y, 'bo')
        # plot obstacles
        for (x, y, radius) in self.obstacle:
            circle = plt.Circle((x, y), radius, color='y')
            ax.add_artist(circle)
        plt.draw()
        plt.pause(0.001)
        #plt.close(fig)

class Node():
    """
    Node for RRT :
    :param parent : the node's parent node 's index in nodelist
    
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0
        self.parent = None

    def __eq__(self, other):
        return (self.x == other.x) and (self.y == other.y)

def main():
    print("Start flexible rrt planning")

    # ===Search Path with flexible rrt===
    # obstacle list (x,y,radius)
    '''
    obstacle = [(5,5,1),
            (3,6,2),
            (3,8,2),
            (7,5,2)]
    '''

    # initialize
    cur_goal = [14, 14]

    goal_list = [(1, 4), (4,1), (10, 5), (5, 10), (14,2), (2,14), (14, 14)]
    # trans_prob 2d array [from, to]
    trans_prob = np.array([[0.2, 0, 0.2, 0.1, 0.2, 0.1, 0.2],
                           [0.1, 0.2, 0, 0.2, 0.1, 0.2, 0.2],
                           [0.2, 0.1, 0.2, 0.1, 0.2, 0, 0.2],
                           [0.1, 0.2, 0, 0.2, 0.1, 0.2, 0.2],
                           [0.2, 0.1, 0.2, 0, 0.2, 0.1, 0.2],
                           [0.1, 0.2, 0.1, 0.2, 0, 0.2, 0.2],
                           [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.4]])
    # trans_prob = np.zeros((7,7))
    obstacle = [(8,8,1),(6,6,1),(12,12,1)]

    frrt = fRRT(start=[0,0], cur_goal=cur_goal,
                    goal_list=goal_list, obstacle=obstacle,
                    TreeArea=[-1, 15], trans_prob=trans_prob)
    #for i in range(20):
    path = frrt.planning()
    if path is not None:
        plt.figure(1)
        frrt.DrawPath()
        plt.figure(2)
        frrt.DrawTree()
    else:
        print("Can't find the path")

    # calculate path distance
    dist = 0
    prev = frrt.path[0]
    for point in frrt.path:
        dist += math.sqrt((point[0] - prev[0])**2 +
                            (point[1] - prev[1])**2)
        prev = point
    print("path distance is:", dist)
    # frrt.DrawTree()
    #frrt.reset()
    # frrt.DrawTree()

if __name__ == '__main__':
    main()