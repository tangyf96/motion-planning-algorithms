# -*- coding: utf-8 -*-
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

class fRRT():
    """
    Class for flexible RRT planning
    """
    def __init__(self, start, cur_goal, goal_list = [], obstacle = [], TreeArea = [], trans_prob = [], stepsize = 0.5, SampleRate = 2, maxIter = 500):
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
        self.trans_prob = trans_prob
        self.goal_list = []
        self.tran_weight = 2
        self.path_node = []
        for (x, y) in goal_list:
            self.goal_list.append(Node(x, y))
        print("initialize frrt object")

    def grow_tree(self):
        """
        flexible RRT Path planning
        :return: self.nodelist
        """
        self.start.cost = self.tran_weight * self.trans_cost(self.start)
        self.nodelist.append(self.start)
        for step in range(self.maxIter):
            # generate random point
            node_rnd = self.get_random_point()

            # find nearest node
            node_best_index = self.FindNearestNode(Node(node_rnd[0], node_rnd[1]))

            # expand tree
            node_new = self.expand(node_rnd, node_best_index)

            # collision check
            if self.collision_check(node_new, self.nodelist[node_new.parent]):
                continue
            # find neighbors in an area around new node
            near_ind = self.find_near_nodes(node_new)
            # choose parent for new node with the minimum cost
            node_new = self.choose_parent(node_new, near_ind)
            # add to nodelist
            self.nodelist.append(node_new)
            # rewire neighbors
            self.rewire(node_new, near_ind)
            #self.DrawTree()
            # check if reach the goal
            if self.node_dist(node_new, self.cur_goal) < self.stepsize:
                print('step is :', step)
                print("Reach the goal!")
                return True


        if step == self.maxIter - 1:
            print("Failed to find the goal!")
            return False

    def rewire(self, node_new, near_ind):
        """
        Rewire the tree, check if neighbors could reduce cost
        by choosing node_new as parent.
        """
        new_node_ind = len(self.nodelist) - 1

        for ind in range(len(near_ind)):
            node = self.nodelist[ind]
            dist = self.node_dist(node_new, node)
            new_cost = node_new.cost + dist + self.tran_weight * self.trans_cost(node)
            if new_cost < node.cost:
                if self.collision_check(node, node_new):
                    # collision
                    continue
                else:
                    # no collision
                    node.parent = new_node_ind
                    node.cost = new_cost

    def FindPath(self):
        """
        Find path from the current goal to start node
        :return: self.path
        """
        index = len(self.nodelist) - 1
        while index is not None:
            node = self.nodelist[index]
            self.path_node.append(node)
            self.path.append([node.x, node.y])
            index = node.parent

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

        return node_new

    def find_near_nodes(self, node_new, alpha=5.0):
        """
        Find the nodes in the circle around new node
        :return: indices of nodes in the circle
        """
        r = self.stepsize * alpha
        near_node_list = []
        for node in self.nodelist:
            near_node_list.append(self.node_dist(node_new, node))
        near_ind = [near_node_list.index(i) for i in near_node_list if (i <= r and i != 0)]
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

        # don't update the cost here
        # update the cost when chosing parent
        return node_new

    def trans_cost(self, node_new):
        """
        Calculate the transition cost if the robot need to replan when current goal changes
        :return: transition probability cost
        """
        cost = 0
        for ind in range(len(self.goal_list)):
            if self.goal_list[ind].x == self.cur_goal.x and self.goal_list[ind].y == self.cur_goal.y:
                from_goal_ind = ind
                break

        for ind in range(len(self.trans_prob)):
            if ind == from_goal_ind:
                continue
            goal_change_prob = self.trans_prob[from_goal_ind][ind]
            dist = self.node_dist(node_new, self.goal_list[ind])
            cost += dist * goal_change_prob

        return cost

    def node_dist(self, node_new, goal):
        """
        Calculte the distance to possible goal
        """
        dist = np.linalg.norm([goal.x - node_new.x, goal.y - node_new.y])
        return dist

    def FindNearestNode(self, node_rnd):
        """
        Use Euclidean Distance to find the nearest node
        """
        """
        dist = [self.node_dist(node_rnd, node) for node in self.nodelist]
        min_dist = min(dist)
        node_index = dist.index(min_dist)
        # print(node_index)
        return node_index
        """
        dist = [self.node_dist(node_rnd, node) for node in self.nodelist]
        dist_copy = copy.deepcopy(dist)
        dist_copy.sort()
        possible_dist = dist_copy[:2]
        ind_list = []
        for i in range(len(possible_dist)):
            ind_list.append(dist.index(possible_dist[i]))
        cost_list = [self.nodelist[ind].cost for ind in ind_list]
        ind = ind_list[cost_list.index(min(cost_list))]
        return ind

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
        # calculate gradient and intercept of the edge from parent to child node
        gradient = (node_new.y - node_parent.y) / (node_new.x - node_parent.x)
        intercept = (node_new.x * node_parent.y - node_new.y * node_parent.x) / (node_new.x - node_parent.x)

        for (x, y, radius) in self.obstacle:
            # dist1 is the distance from new node to the center of obstacle
            dist1 = math.sqrt((x - node_new.x) ** 2 + (y - node_new.y) ** 2)
            # cross flag and dist2 is used to decide whether the
            # edge between parent and node_new cross the obstacle
            cross_flag = ((node_new.x - x) * (node_parent.x - x)) <= 0 or ((node_new.y - y) * (node_parent.y - y)) <= 0
            # dist2 is the distance from center of obstacle to the edge linking parent and child
            dist2 = abs(gradient * x - y + intercept) / math.sqrt(gradient ** 2 + 1)
            if dist1 <= radius:
                # print('dist is smaller than radius')
                return True # unsafe
            elif dist2 <= radius and cross_flag:
                # print('dist2 is smaller than radius')
                return True # unsafe
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
        plt.clf()
        plt.plot([x for (x,y) in self.path],[y for (x,y) in self.path], "-k")
        plt.plot([x for (x,y) in self.path],[y for (x,y) in self.path], "yo")
        for node in self.goal_list:
            plt.plot(node.x, node.y, 'g*')
        ax = plt.gca()
        ax.set_xlim((self.min_rand, self.max_rand))
        ax.set_ylim((self.min_rand, self.max_rand))
        plt.plot(self.start.x, self.start.y, 'ro')
        plt.plot(self.cur_goal.x, self.cur_goal.y, 'bo')
        for (x, y, radius) in self.obstacle:
            circle = plt.Circle((x, y), radius, color='y')
            ax.add_artist(circle)
        plt.show()
        plt.pause(0.01)

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
    """
    goal_list = [(5,10), (1,4), (10,2)]
    """
    goal_list = [(1, 4), (4,1), (10, 5), (5, 10), (14,2), (2,14), (14, 14)]
    # trans_prob 2d array [from, to]
    #trans_prob = np.array([[1, 0.9, 0], [0.9, 1, 0.5], [0, 0.5, 1]])
    trans_prob = np.ones((len(goal_list), len(goal_list))) * 0.3
    trans_prob[6][0] += 0.5
    trans_prob[6][3] += 0.5
    trans_prob[6][5] += 0.5
    # trans_prob = np.zeros((7,7))
    obstacle = [(8,8,1),(6,6,1),(12,12,1)]

    for i in range(20):
        frrt = fRRT(start=[0, 0], cur_goal=cur_goal,
                    goal_list=goal_list, obstacle=obstacle,
                    TreeArea=[-1, 15], trans_prob=trans_prob)
        flag = frrt.grow_tree()
        if flag:
            frrt.FindPath()
            frrt.DrawPath()
            frrt.DrawTree()

    # frrt.DrawTree()

if __name__ == '__main__':
    main()