# -*- coding: utf-8 -*-
"""
Created on Mon Oct 13 20:58:23 2018
D* grid search algorithms
@author: Yifan Tang
email : tangyf96@outlook.com
"""
from search_class import *

class D_star():
    def __init__(self, graph, start, goal, new_obstacle = []):
        self.graph = graph
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.new_obstacle = new_obstacle
        
    def check_back_pointer(self, back_pointer, current, next_node):
        if next_node not in back_pointer:
            return False
        elif (back_pointer[next_node] != current.id):
            return False
        else:
            return True    
    
    def process_state(self):
        """
        Initially search from the goal to the start 
        """
        open_list = PriorityQueue()
        open_list.put(self.start, self.start.key_func)
        
        close_list = []
        back_pointer = {}
        back_pointer[self.start.id] = None
        
        while not open_list.empty():
            current = open_list.pop()
            print(current)
            if self.goal.__eq__(current):
                print("find the goal")
                break
            
            if (current.key_func == current.h_func):
                # 
                for next in self.graph.neighbors(current):
                    edge_cost = self.graph.cost(current.id, next.id)
                    current_cost = edge_cost + current.h_func
                    if (next not in close_list) :
                    #or (next.h_func != current_cost and self.check_back_pointer(back_pointer, current, next)) or ( not self.check_back_pointer(back_pointer, current, next) and next.h_func > current_cost):
                           
                            # update the h value of next node
                            next.h_func = current_cost
                            # update the key value of next node
                            self.graph.key_function(next)
                            
                            open_list.put(next, next.key_func)
                            back_pointer[next.id] = current.id
            
            close_list.append(current)
        
        return back_pointer, close_list

    # def modify_cost(self):
        """
        Once the robot detects an error in the arc cost function
        change the arc cost function and enter affected states on the open list
        """
        
def main():
    # initialize graph and convert element to Node type
    graph = GridWithWeights(10, 10)
    graph.walls = [(1, 7), (2, 8), (5, 7), (9, 8), (3, 7), (3, 8)]
    graph.addNodeToGraph()
    
    start = (1,8)
    goal = (8,2)
    d_star_planner = D_star(graph, start, goal)
    back_pointer, close_list = d_star_planner.process_state()
    draw_grid(graph, width = 10, point_to = back_pointer, start = start, goal = goal)
    print()
    draw_grid(graph, width = 10, number = close_list, start = start, goal = goal)
    print()

if __name__ == '__main__':
    main()
    