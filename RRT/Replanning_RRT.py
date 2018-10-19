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
       self.nodelist = []


   def GrowTree(self):
       """
       Grow the RRT , change the node list of tree and 
       return the status of path. 
       
       """
       self.nodelist.append(self.start)
       
       step = 0
       
       while True:
           step += 1
           # print('begin iteration ', step, 'to find the goal')
           if random.randint(0,10) > self.SampleRate:
               node_rnd = [random.uniform(self.min_rand, self.max_rand),
                           random.uniform(self.min_rand, self.max_rand)]
           else:
               node_rnd = [self.goal.x, self.goal.y]
           
           # Find the nearest node
           node_nearest_index, _ = self.FindNearestNode(self.nodelist, node_rnd, mode = "Euclidean")
           node_nearest = self.nodelist[node_nearest_index]
           
           # Expand Tree
           theta = math.atan2(node_rnd[1] - node_nearest.y, node_rnd[0] - node_nearest.x)
           
           node_new = copy.deepcopy(node_nearest)
           node_new.x += self.stepsize * math.cos(theta)
           node_new.y += self.stepsize * math.sin(theta)

           node_new.parent = node_nearest_index
           
           # Collision check
           if self.CollisionCheck(node_new):
               continue
           
           # add new node to list
           self.nodelist.append(node_new)
           
           # Check whether reach goal
           goal_dist = math.sqrt((node_new.x - self.goal.x)**2 + (node_new.y - self.goal.y)**2)
           if goal_dist <= self.stepsize:
               print("The algorithm finds the goal after %d steps" %step)
               return True
               #break
           # draw the graph after finding new node
           #self.DrawTree()
           if (step >= self.maxIter):
               print('after ', self.maxIter, "still don't find the goal")
               return False

            
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
       #print(node_index)
       return node_index, min_dist
     
        
   def CollisionCheck(self, node_new, new_obstacle = False, regrow = False):
       """
       Check whether the path from new node to its parents collides with obstacles
       If there is Collision, return True;
       """
       if (new_obstacle==True):
           obstacle = self.new_obstacle
       else:
           obstacle = self.obstacle
           
       if regrow:
           node_parent = self.replan_nodelist[node_new.parent]
       else:
           node_parent = self.nodelist[node_new.parent]
           
       gradient = (node_new.y - node_parent.y) / (node_new.x - node_parent.x)
       intercept = (node_new.x * node_parent.y - node_new.y * node_parent.x) / (node_new.x - node_parent.x)
       
       for (x, y, radius) in obstacle: 
           dist = math.sqrt((x - node_new.x)**2 + (y - node_new.y)**2)
           dist2 = (gradient * x - y + intercept) / math.sqrt(gradient**2 + 1)
           if dist <= radius:
               #print('dist is smaller than radius')
               return True
           elif dist2 <= radius:
               #print('dist2 is smaller than radius')
               return True
       return False
           
    
   def FindPath(self):
       """
       Find the path from the goal node to start node
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
       self.path_index.append(0)
    
    
   def DrawTree(self, path=False, result=False, replan = False):
       """
       Draw the Rapid-explored Random Tree and also draw the obstacle area
       """
       if result==False:
           for node in self.nodelist:
               if (node.parent is not None) and (node.flag == True):
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
    """
    Class for RRT replanning 
    """
    def __init__(self, start, goal, stepsize, obstacle, TreeArea, old_path_index, new_obstacle, SampleRate=2, maxIter=250, nodelist = []):
        """
        This is the initialization for class RRT
        Parameters:
            new_obstacle : a list to define new area of obstacle found in the existing path
            old_path_index : the index of nodes in nodelist of the existing path
        """
        super(Replan_RRT, self).__init__(start, goal, stepsize, obstacle, TreeArea, SampleRate=2, maxIter=300)
        self.obstacle.extend(new_obstacle)
        self.new_obstacle = new_obstacle
        self.nodelist = nodelist
        self.replan_nodelist = []
        self.old_path_index = old_path_index
        self.path = [[self.goal.x, self.goal.y]]
        
    def InvalidateNodes(self):
        """
        Invalidate edges in the old path influenced by new obstacles 
        and invalidated corresponding nodes
        """
        # invalidate nodes in original path
        for node_index in reversed(self.old_path_index):
            # there might be a problem with copy
            node = self.nodelist[node_index]
            
            # pass the start node 
            if (node.parent == None):
                continue
            # invalidate child nodes
            if (self.nodelist[node.parent].flag == False):
                self.nodelist[node_index].flag = False
                continue
            # invalidatate parent nodes
            if self.CollisionCheck(node, new_obstacle=True):
               self.nodelist[node_index].flag = False
            else:
                pass
        
        # invalidate nodes that are inside the obstacle and not part of the path
        for node_index, node in enumerate(self.nodelist):
            if node.parent == None:
                continue
            if (self.nodelist[node.parent].flag == False):
                self.nodelist[node_index].flag = False
                
            if (node.flag != False):
                for (x, y, radius) in self.new_obstacle:
                    dist = math.sqrt((x - node.x)**2 + (y - node.y)**2)
                    if dist <= radius:
                        self.nodelist[node_index].flag = False
                        #print('I find one in the obstacle')
                        continue
                    else:
                        pass
    
    def regrow(self):
        """
        regrow trees based on trimmed trees
        """

        self.replan_nodelist.append(self.goal)
        # find the nearest node of old tree 
        # and set it as the goal for the search
        index, _ = self.FindNearestNode(self.nodelist, [self.goal.x, self.goal.y], mode = "Euclidean")
        self.goal = self.nodelist[index]
               
        step = 0
        while True:
            step += 1
            # print('step %d to replan' % step)
            if random.randint(0,10) > self.SampleRate:
                node_rnd = [random.uniform(self.min_rand, self.max_rand), 
                            random.uniform(self.min_rand, self.max_rand)]
            else:
                node_rnd = [self.goal.x, self.goal.y]
                
            node_nearest_index, _ = self.FindNearestNode(self.replan_nodelist, node_rnd, mode = "Euclidean")
            
            node_nearest = self.replan_nodelist[node_nearest_index]
            # expand tree
            theta = math.atan2(node_rnd[1] - node_nearest.y, node_rnd[0] - node_nearest.x)
            
            node_new = copy.deepcopy(node_nearest)
            node_new.x += self.stepsize * math.cos(theta)
            node_new.y += self.stepsize * math.sin(theta)
            #print('new node is :', node_new.x, node_new.y)
            node_new.parent = node_nearest_index
            
            if self.CollisionCheck(node_new, regrow = True):
                continue
            #print('after collision check')
            self.replan_nodelist.append(node_new)
            
            # check if the new tree reach the existing tree
            node_nearest_index, min_dist = self.FindNearestNode(self.nodelist, [node_new.x, node_new.y])
            if min_dist <= self.stepsize:
                print("the algorithm finds the new path after %d steps" %step)
                
                self.replan_nodelist.append(self.nodelist[node_nearest_index])
                return True
                break
            
            if (step >= self.maxIter):
                print('after', self.maxIter, "still don't find the new path")
                return False
                break
        
    def FindPath(self):
       """
       Find the path from the goal node to start node
       """
       # find the path through the new tree
       # find the path from the final node added to the nodelist of new tree
       index = -2
       while (self.replan_nodelist[index].parent) is not None:
           node_path = self.replan_nodelist[index]
           self.path.append([node_path.x, node_path.y])
           index = node_path.parent

       # find the path through the old tree
       # replan_nodelist[-1] is the node of old tree that is closest to the end of new_tree
       node_path = self.replan_nodelist[-1]
       self.path.append([node_path.x, node_path.y])
       index = self.replan_nodelist[-1].parent
       while (self.nodelist[index].parent) is not None:
           node_path = self.nodelist[index]
           self.path.append([node_path.x, node_path.y])
           index = node_path.parent
       self.path.append([self.start.x, self.start.y])

       # connect two path

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
    """
    This is the main function for Replanning with RRTs
    """
    # set parameters for RRT
    start = [0, 0]
    #goal = [11, 5]
    goal = [14, 14]
    obstacle = [
        (3, 6, 1),
        (3, 8, 1),
        (10, 3, 2),
        (7, 8, 1),
        (9, 5, 1),
        (5, 5, 1)
    ]
    new_obstacle = [(11, 11, 2), (7, 6, 1)]
    # start initial planning
    rrt = RRT(start, goal, stepsize=2, obstacle=obstacle, TreeArea=[0,15])
    path = rrt.GrowTree()
    if (path == True):
        rrt.FindPath()
    else:
        pass
    rrt.DrawTree(path = path, result=True)
    #return 0
    # start replanning
    replan_rrt = Replan_RRT(start = start,  goal = goal, stepsize = 1, obstacle=obstacle, TreeArea=[0,15], 
                            nodelist=rrt.nodelist, old_path_index=rrt.path_index, 
                            new_obstacle=new_obstacle)
    # trim existing trees
    replan_rrt.InvalidateNodes()
    # regrow trees after trimming the old trees
    new_path = replan_rrt.regrow()
    # find the new path
    if (new_path == True):
        replan_rrt.FindPath()

    # draw the new path
    replan_rrt.DrawTree(path=new_path, result=True)
    
    
if __name__ == '__main__':
    main()