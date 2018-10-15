# -*- coding: utf-8 -*-
"""
Breadth First Search Algorithm
Created on Thu Oct 11 20:26:55 2018

@author: Yifan Tang (tangyf96@outlook.com)
Reference : 
Sample code from https://www.redblobgames.com/pathfinding/a-star/
Copyright 2014 Red Blob Games <redblobgames@gmail.com>

License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>
"""
from implementation import *
from queue import Queue

#def breadth_first_search(graph, start):
#       """
#       This is the basic breadth_first_search algorithm for the graph
#       """
#     frontier = Queue()
#     frontier.put(start)
#     visited = {}
#     visited[start] = True
#     
#     while not frontier.empty():
#         current = frontier.get()
#         print("visiting %r" % current)
#         for next in graph.neighbors(current):
#             if next not in visited:
#                 frontier.put(next)
#                 visited[next] = True

    
def breadth_first_search(graph, start, goal):
    """
    This function uses breadth_first_search method to 
    find a path from the start point to the goal point 
    """
    frontier = Queue()
    frontier.put(start)
    visited = {}
    visited[start] = None
    
    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            break
        for next in graph.neighbors(current):
            if next not in visited:
                frontier.put(next)
                visited[next] = current
    return visited
    
    
def main():
#    example_graph = SimpleGraph()
#    example_graph.edges = {'A' : ['B'],
#                           'B' : ['A', 'C', 'D'],
#                           'C' : ['A'],
#                           'D' : ['E', 'A'],
#                           'E' : ['B']
#                           }
#    breadth_first_search(example_graph, 'A')
#    g = SquareGrid(30, 15)
#    g.walls = DIAGRAM1_WALLS
#    
#    parents = breadth_first_search(g, (1,1), (17,2))
#    draw_grid(g, width=2, point_to=parents, start=(1,1), goal = (17,2))
    diagram4 = GridWithWeights(10, 10)
    diagram4.walls = [(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8)]
    diagram4.weights = {loc: 5 for loc in [(3, 4), (3, 5), (4, 1), (4, 2),
                                       (4, 3), (4, 4), (4, 5), (4, 6), 
                                       (4, 7), (4, 8), (5, 1), (5, 2),
                                       (5, 3), (5, 4), (5, 5), (5, 6), 
                                       (5, 7), (5, 8), (6, 2), (6, 3), 
                                       (6, 4), (6, 5), (6, 6), (6, 7), 
                                       (7, 3), (7, 4), (7, 5)]}
    draw_grid(diagram4, width = 10, point_to = (1,1), start = (1,1), goal = (17,2))
if __name__ == '__main__':
    main()