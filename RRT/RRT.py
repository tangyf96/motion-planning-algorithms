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
   def __init__(self, start, goal, stepsize, obstacle = [], TreeArea, SampleRate=2, maxIter=500):
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
       self.goal = Node(goal[0], goal[1])
       self.stepsize = stepsize
       self.obstacle = obstacle
       self.min_rand = TreeArea[0]
       self.max_rand = TreeArea[1]
       self.SampleRate = SampleRate
       self.maxIter = maxIter
       

   def GrowTree(self):
       """
       Grow the RRT and return the node list of tree
       
       """
       self.nodelist = [self.start]
       
       while True:
           if random.randint(0,10) > self.SampleRate:
               node_rnd = [random.uniform(self.min_rand, self.max_rand),
                           random.uniform(self.min_rand, self.max_rand)]
           else:
               node_rnd = [self.goal.x, self.goal.y]
           
           # Find the nearest node
           node_nearest_index = self.FindNearestNode(self.nodelist, node_rnd, mode = "Euclidean")
           node_nearest = self.nodelist[node_nearest_index]
           
           # Expand Tree
           theta = math.atan2(node_rnd[0] - node_nearest.x, node_rnd[1] - node_nearest.y)
           
           node_new = copy.deepcopy(node_nearest)
           node_new.x += stepsize * math.cos(theta)
           node_new.y += stepsize * math.sin(theta)
#           node_new.x += (node_rnd[0] - node_nearest.x) * math.cos(theta)
#           node_new.y += (node_rnd[1] - node_nearest.y) * math.sin(theta)
           node_new.parent = node_nearest_index
           
           # Collision check
           if self.CollisionCheck(node_new, self.obstacle):
               break
           
           # add new node to list
           self.nodelist.append(node_new)
           
           # Check whether reach goal
           goal_dist = math_sqrt((node_new.x - self.goal.x)**2 + (node_new.y - self.goal.y)**2)
           if goal_dist <= self.stepsize:
               print("The algorithm finds the goal")
               return nodelist
               break
           
   def FindNearestNode(self, nodelist, node_rnd, mode = "Euclidean"):
       """
       Use Euclidean Distance or Mahattan Distance to find the nearest node
       """
       if mode == "Euclidean" :
           min_dist = (self.start.x - node_rnd[0]) ** 2 + (self.start.y - node_rnd[1]) ** 2
           for iter in range(len(nodelist)):
               distance = (nodelist[iter].x - node_rnd[0]) ** 2 + (nodelist[iter].y - node_rnd[1]) ** 2
               if distance <= min_dist : 
                   min_dist = distance
                   node_index = iter
       else if mode == "Mahattan": 
           min_dist = (self.start.x - node_rnd[0]) + (self.start.y - node_rnd[1])
           for iter in range(len(nodelist)):
               distance = (nodelist[iter].x - node_rnd[0]) + (nodelist[iter].y - node_rnd[1])
               if distance <= min_dist:
                   min_dist = distance
                   node_index = iter
       else:
           raise NameError('distance name is wrong!')
       
       return node_index
     
        
   def CollisionCheck(self, node_new, obstacle):
       """
       Check whether the path from new node to its parents collide with obstacles
       """
       node_parent = self.nodelist[node_new.parent]
       dist_parent_child = math.sqrt((node_parent.x - node_new.x)**2
                                     +node_parent.y - node_new.y)**2)
       for (x, y, radius) in obstacle: 
           dist = math.sqrt((x - node_new.x)**2 + (y - node_new.y)**2)
           if dist <= radius:
               return false
           else math.sqrt(dist**2 + dist_parent_child**2) <= radius:
               return false
           
   def FindPath():

   def DrawTree():

class Node():
    """
    Node for RRT
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def main():
    rrt = RRT()
    nodelist = rrt.GrowTree()
    rrt.FindPath()