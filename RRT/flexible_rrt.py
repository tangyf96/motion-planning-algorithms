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
    def __init__(self, start, cur_goal, goal_list = [], obstacle = [], TreeArea = [], trans_prob = [], stepsize = 0.5,SampleRate = 2, maxIter = 500):
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
        self.tran_weight = 0.7
        for i in range(len(goal_list)):
            self.nodelist.append(Node(goal_list[i][0], goal_list[i][1]))


    def grow_tree(self):
        """
        RRT Path planning
        :return: path
        """
        self.nodelist.append(self.start)
        for i in range(self.maxIter):
            #generate random point
            node_rnd = self.get_random_point()

            # find nearest parent node
            node_nearest_index, _ = self.FindNearestNode(self.nodelist, node_rnd, mode="Euclidean")

            # expand tree
            node_new = self.expand(node_rnd, node_nearest_index)

            # collision check
            if self.collision_check(node_new):
                continue
            # find neighbors in an area around new node
            near_ind = self.find_near_nodes(node_new)
            self.choose_parent(node_new)
            # write until here Nov 12


    def find_near_nodes(self, node_new):
        """
        Find the nodes in the circle around new node
        :return: indices of nodes in the circle
        """
        r = self.stepsize * 10.0
        near_node_list = []
        for node in self.nodelist:
            near_node_list.append(self.dist_to_goal(node_new, node))
        near_ind = [near_node_list.index(i) for i in near_node_list if i < r]
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

        # update the cost
        # here , still some problem, the scale of two add is different
        node_new.cost = self.stepsize + self.tran_weight * self.trans_cost(node_new)
        return node_new

    def trans_cost(self, node_new):
        """
        Calculate the cost if the robot need to replan when current goal changes
        :return:
        """
        cost = 0
        for ind in len(self.goal_list):
            if self.goal_list[ind].x == self.cur_goal.x and self.goal_list[ind].y == self.cur_goal.y:
                from_goal_ind = ind
                break

        for ind in len(self.trans_prob[1]):
            if ind == from_goal_ind:
                continue
            goal_change_prob = self.trans_prob[ind][from_goal_ind]
            dist = self.dist_to_goal(node_new, self.goal_list[ind])
            cost += dist * goal_change_prob

        return cost

    def dist_to_goal(self, node_new, goal):
        """
        Calculte the distance to possible goal
        """
        dist = np.linalg.norm([goal.x - node_new.x, goal.y - node_new.y])
        return dist

    def FindNearestNode(self, nodelist, node_rnd, mode="Euclidean"):
        """
        Use Euclidean Distance or Mahattan Distance to find the nearest node
        """
        node_index = 0
        if mode == "Euclidean":
            min_dist = (self.start.x - node_rnd[0]) ** 2 + (self.start.y - node_rnd[1]) ** 2
            for iter in range(len(nodelist)):
                if (nodelist[iter].flag == True):
                    distance = (nodelist[iter].x - node_rnd[0]) ** 2 + (nodelist[iter].y - node_rnd[1]) ** 2
                    if distance <= min_dist:
                        min_dist = distance
                        node_index = iter
                else:
                    pass
        elif mode == "Mahattan":
            min_dist = (self.start.x - node_rnd[0]) + (self.start.y - node_rnd[1])
            for iter in range(len(nodelist)):
                if (nodelist[iter].flag == True):
                    distance = abs(nodelist[iter].x - node_rnd[0]) + abs(nodelist[iter].y - node_rnd[1])
                    if distance <= abs(min_dist):
                        min_dist = distance
                        node_index = iter
                else:
                    pass

        else:
            raise NameError('distance name is wrong!')
        # print(node_index)
        return node_index, min_dist

    def get_random_point(self):
        if random.randint(0,10) > self.SampleRate:
            rnd = [random.uniform(self.min_rand, self.max_rand),
                   random.uniform(self.min_rand, self.max_rand)]
        else:
            rnd = [self.cur_goal.x, self.cur_goal.y]
        return rnd

    def collision_check(self, node_new):
        """
        Check whether the path from new node to its parents collides with obstacles
        If there is Collision, return True;
        """
        node_parent = self.nodelist[node_new.parent]
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
                return True
            elif dist2 <= radius and cross_flag:
                # print('dist2 is smaller than radius')
                return True
        return False


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