# -*- coding: utf-8 -*-
"""
Created on Thu Oct 11 19:45:42 2018
A* grid search algorithms
@author: Yifan Tang (tangyf96@outlook.com)
Reference: https://www.redblobgames.com/pathfinding/a-star/implementation.html
"""
import math
from search_class import *

def heuristic(current, goal):
    """
    Calculate the heuristic function, which is the Euclidean distance between current location and goal
    """
    (x_current, y_current) = current
    (x_goal, y_goal) = goal
    h = math.sqrt((x_goal - x_current) ** 2 + (y_goal - y_current) ** 2)
    return h

def A_star(graph, start, goal):
    open_list = PriorityQueue()
    open_list.put(start, 0)
    # backpoint is where the robot come from to the current location
    back_pointer = {}
    back_pointer[start] = None
    # the visited nodes are put into cost_so_far (close_list)
    visited = {}
    visited[start] = 0
    
    while not open_list.empty():
        current = open_list.pop()
        
        if current == goal:
            print("find the goal")
            break
        
        # put the neighbors into open list waiting for visited
        for next in graph.neighbors(current):
            neighbor_cost = visited[current] + graph.cost(current, next)
            if next not in visited or visited[next] > neighbor_cost:
                visited[next] = neighbor_cost
                priority = neighbor_cost + heuristic(next, goal)
                open_list.put(next, priority)
                back_pointer[next] = current
        
    return back_pointer, visited
            
            
def main():
    graph = GridWithWeights(10,10)
    graph.walls = [(1, 7), (2, 8), (5, 7), (9, 8), (3, 7), (3, 8)]
    graph.weights = {loc: 5 for loc in [(3, 4), (3, 5), (4, 1), (4, 2),
                                        (4, 3), (4, 4), (4, 5), (4, 6), 
                                       (4, 7), (4, 8), (5, 1), (5, 2),
                                       (5, 3), (5, 4), (5, 5), (5, 6), 
                                       (5, 7), (5, 8), (6, 2), (6, 3), 
                                       (6, 4), (6, 5), (6, 6), (6, 7), 
                                       (7, 3), (7, 4), (7, 5)]}
    start = (1,8)
    goal = (8,2)
    back_pointer, visited = A_star(graph, start, goal)
    draw_grid(graph, width = 10, point_to = back_pointer, start = start, goal = goal)
    print()
    draw_grid(graph, width = 10, number = visited, start = start, goal = goal)
    print()
    print(visited)
   
if __name__ == '__main__':
    main()