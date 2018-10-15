# -*- coding: utf-8 -*-
"""
Simple graph class for breadth_first_search algorithm
Created on Thu Oct 11 20:37:14 2018

@author: Yifan Tang
email : tangyf96@outlook.com
"""

class SimpleGraph:
    def __init__(self):
        self.edges = {}
        
    def neighbors(self, id):
        return self.edges[id]
    
class Grid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []
        
    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, id):
        return id not in self.walls
    
    def neighbors(self, id):
        (x, y) = id
        neighbors = [(x, y+1), (x+1, y), (x, y-1), (x-1, y)]
        # filter out the unreachable neighbors
        neighbors = filter(self.in_bounds, neighbors)
        neighbors = filter(self.passable, neighbors)
        return neighbors

def draw_grid(graph, width=2, **style):
    for y in range(graph.height):
        for x in range(graph.width):
            print("%%-%ds" % width % draw_tile(graph, (x, y), style, width), end="")
        print()
        
def draw_tile(graph, id, style, width):
    r = "."
    if 'number' in style and id in style['number']: r = "%d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = ">"
        if x2 == x1 - 1: r = "<"
        if y2 == y1 + 1: r = "v"
        if y2 == y1 - 1: r = "^"
    if 'start' in style and id == style['start']: r = "A"
    if 'goal' in style and id == style['goal']: r = "Z"
    if 'path' in style and id in style['path']: r = "@"
    if id in graph.walls: r = "#" * width
    return r