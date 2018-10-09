# -*- coding: utf-8 -*-
"""
Created on Mon Oct  8 19:46:03 2018
This is the Replanning algorithms using RRT based on "Replanning with RRTs (Dave Ferguson et.al 2006)
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
   def __init__ (self, start, goal, stepsize, obstacle, TreeArea, SampleRate=2, maxIter=500):
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
       self.path = [[self.goal.x, self.goal.y]]
       self.path_index = []


   def GrowTree(self, regrow_flag = False):
       """
       Grow the RRT and return the node list of tree
       
       """
       if regrow_flag == False:
           self.nodelist = [self.start]
       else:
           pass
       
       step = 0
       while True:
           step += 1
           print('begin iteration ', step, 'to find the goal')
           if random.randint(0,10) > self.SampleRate:
               node_rnd = [random.uniform(self.min_rand, self.max_rand),
                           random.uniform(self.min_rand, self.max_rand)]
           else:
               node_rnd = [self.goal.x, self.goal.y]
           
           # Find the nearest node
           node_nearest_index = self.FindNearestNode(self.nodelist, node_rnd, mode = "Euclidean")
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
           if not self.CollisionCheck(node_new):
               continue
           
           # add new node to list
           self.nodelist.append(node_new)
           
           # Check whether reach goal
           goal_dist = math.sqrt((node_new.x - self.goal.x)**2 + (node_new.y - self.goal.y)**2)
           if goal_dist <= self.stepsize:
               print("The algorithm finds the goal")
               return True
               break
           # draw the graph after finding new node
           self.DrawTree()
           if (step >= self.maxIter):
               print('after ', self.maxIter, "still don't find the goal")
               return False
               break
           
            
   def FindNearestNode(self, nodelist, node_rnd, mode = "Euclidean"):
       """
       Use Euclidean Distance or Mahattan Distance to find the nearest node
       """
       node_index = 0
       if mode == "Euclidean" :
           min_dist = (self.start.x - node_rnd[0]) ** 2 + (self.start.y - node_rnd[1]) ** 2
           for iter in range(len(nodelist)):
               if (nodelist[iter].flag == True):
                   distance = (nodelist[iter].x - node_rnd[0]) ** 2 + (nodelist[iter].y - node_rnd[1]) ** 2
                   if distance <= min_dist : 
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
       return node_index
     
        
   def CollisionCheck(self, node_new, new_obstacle = False):
       """
       Check whether the path from new node to its parents collide with obstacles
       """
       if (new_obstacle==True):
           obstacle = self.new_obstacle
       else:
           obstacle = self.obstacle
           
       node_parent = self.nodelist[node_new.parent]
       dist_parent_child = math.sqrt((node_parent.x - node_new.x)**2 + 
                                     (node_parent.y - node_new.y)**2)
       
       for (x, y, radius) in obstacle: 
           dist = math.sqrt((x - node_new.x)**2 + (y - node_new.y)**2)
           if dist <= radius:
               return False
           elif math.sqrt(dist**2 - (dist_parent_child/2)**2) <= radius:
               return False
       return True
           
    
   def FindPath(self):
       """
       Find path from the goal node to start node
       """
       
       index = len(self.nodelist) - 1
       while (self.nodelist[index].parent) is not None:
           node_path = self.nodelist[index]
           #print(node_path)
           self.path.append([node_path.x, node_path.y])
           self.path_index.append(index)
           #print(self.path)
           index = node_path.parent
       self.path.append([self.start.x, self.start.y])
       #self.path_index.append(-1)
    
    
   def DrawTree(self, path=False, result=False):
       """
       Draw the Rapid-explored Random Tree and also draw the obstacle area
       """
       if result==False:
           for node in self.nodelist:
               if node.parent is not None and node.flag == True:
                   plt.plot([node.x, self.nodelist[node.parent].x], 
                            [node.y, self.nodelist[node.parent].y], '-k')
                   
       else:
           if (path == True):
               plt.plot([x for (x, y) in self.path], [y for (x, y) in self.path], '-r')
               plt.plot(self.start.x, self.start.y, 'ro')
       
       # plot the obstacle
       ax = plt.gca()
       ax.set_xlim((self.min_rand, self.max_rand))
       ax.set_ylim((self.min_rand, self.max_rand))
       
       for (x, y, radius) in self.obstacle:
           circle = plt.Circle((x, y), radius, color = 'b')
           ax.add_artist(circle)
       #plt.axis([self.min_rand, self.max_rand , self.min_rand, self.max_rand])
       #plt.grid(True)
       plt.show()
       plt.pause(0.01)


class Replan_RRT(RRT):
    def __init__(self, start, goal, stepsize, obstacle, TreeArea, old_path_index, new_obstacle, SampleRate=2, maxIter=250, nodelist = []):
        super(Replan_RRT, self).__init__(start, goal, stepsize, obstacle, TreeArea, SampleRate=2, maxIter=250)
        self.obstacle.extend(new_obstacle)
        self.new_obstacle = new_obstacle
        self.nodelist = nodelist
        self.old_path_index = old_path_index
        self.path = [[self.goal.x, self.goal.y]]
        
    def InvalidateNodes(self):
        for iter in range(len(self.old_path_index)):
            # there might be a problem with copy
            node = self.nodelist[self.old_path_index[iter]]
            if (self.CollisionCheck(node, new_obstacle=True)):
               self.nodelist[self.old_path_index[iter]].flag = False
            else:
                pass
    
    def regrow(self):
        new_path = self.GrowTree(regrow_flag=True)
        return(new_path)
    
class Node():
    """
    Node for RRT
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.flag = True

def main():
    # set parameters for RRT
    start = [0, 0]
    goal = [14, 14]
    obstacle = [
        (3, 6, 1),
        (3, 8, 1),
        (10, 3, 2),
        (7, 8, 1),
        (9, 5, 2),
        (5, 5, 1)
    ]
    new_obstacle = [(5, 11, 2)]
    # start initial planning
    rrt = RRT(start, goal, stepsize=1, obstacle=obstacle, TreeArea=[0,15])
    path = rrt.GrowTree()
    if (path == True):
        rrt.FindPath()
    else:
        pass
    rrt.DrawTree(path = path, result=True)
    
    # start replanning
    # nodelist = [], old_path_index
    replan_rrt = Replan_RRT(start,  goal, stepsize = 1, obstacle=obstacle, TreeArea=[0,15], 
                            nodelist=rrt.nodelist, old_path_index=rrt.path_index, 
                            new_obstacle=new_obstacle)
    replan_rrt.InvalidateNodes()
    new_path = replan_rrt.regrow()
    if (new_path == True):
        replan_rrt.FindPath()
    else:
        pass
    replan_rrt.DrawTree(path=new_path, result=True)
    
    
if __name__ == '__main__':
    main()