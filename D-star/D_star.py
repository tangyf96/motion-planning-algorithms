# -*- coding: utf-8 -*-
"""
Created on Mon Oct 13 20:58:23 2018
D* grid search algorithms
@author: Yifan Tang
email : tangyf96@outlook.com
"""
from search_class import *
from matplotlib import pyplot as plt
class D_star():
    def __init__(self, graph, start, goal, new_obstacle = []):
        self.graph = graph
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.new_obstacle = new_obstacle
        self.path = []
        
    def check_back_pointer(self, back_pointer, current, next_node):
        """
        Check whether the back_pointer points to current node
        If yes, return True;
        If no, return false. 
        """
        if next_node not in back_pointer:
            return False
        elif (back_pointer[next_node] != current.id):
            return False
        else:
            return True    
    
    def init_plan(self):
        """
        Initially search from the goal to the start 
        """
        open_list = PriorityQueue()
        open_list.put(self.start, self.start.key_func)
        
        close_list = {}
        back_pointer = {}
        back_pointer[self.start.id] = None
        
        while not open_list.empty():
            current = open_list.pop()
            #print(current)
            if self.goal.eq(current):
                print("find the goal")
                break
            close_list[current.id] = current.h_func
            # add the neighbors of current node to open list
            for node in self.graph.neighbors(current):
                edge_cost = self.graph.cost(current.id, node.id)
                current_cost = edge_cost + current.h_func
                #print(node.id)
                #
                if node.back_pointer == None and (node.id != self.start.id):
                    # update the h value of new node
                    node.h_func = current_cost
                    # update the key value of new node
                    node.key_func = node.h_func
                    # set the back pointer of new node
                    node.back_pointer = current.id
                    back_pointer[node.id] = node.back_pointer
                    # add new node to open list
                    open_list.put(node, node.key_func)
                elif (node.h_func > current_cost) and (not self.check_back_pointer(back_pointer, current, node)):
                    # update the node's priority in the priority queue
                    open_list.remove(node, node.key_func)
                    
                    node.back_pointer = current.id
                    back_pointer[node.id] = node.back_pointer
                    node.key_func = node.h_func
                    open_list.put(node, node.key_func)
                else:
                    pass
                    
        return back_pointer, close_list
    
    def find_path(self, back_pointer):
        self.path.append(self.goal.id)
        parent = back_pointer[self.goal.id]
#        test_parent = self.goal.back_pointer
        while(parent != None):
            self.path.append(parent)
            parent = back_pointer[parent]
    
    # def modify_cost(self):
        """
        Once the robot detects an error in the arc cost function
        change the arc cost function and enter affected states on the open list
        """
        
def draw_path(start, goal, path):
    """
    draw the final path
    """

    plt.plot([x for (x, y) in path], [y for (x,y) in path], '-r')    
    for (x,y) in path:
        if((x,y) == goal or (x,y) == start):
            plt.plot(x,y,'bo')
        else:
            plt.plot(x, y, 'ro')

    plt.grid(True)
    plt.show()
       
        
def main():
    # initialize graph and convert element to Node type
    graph = GridWithWeights(10, 10)
    graph.walls = [(1, 7), (5, 8), (5, 7), (9, 8), (3, 7), (3, 8)]
    graph.addNodeToGraph()
    
    start = (1,8)
    goal = (8,2)
    d_star_planner = D_star(graph, start, goal)
    back_pointer, close_list = d_star_planner.init_plan()
    # this should be updated to support diagnal pointer
    #draw_grid(graph, width = 10, point_to = back_pointer, start = start, goal = goal)
    d_star_planner.find_path(back_pointer)
    draw_path(start, goal, d_star_planner.path)

    print()
    draw_grid(graph, width = 10, number = close_list, start = start, goal = goal)
    print()

if __name__ == '__main__':
    main()
    