# -*- coding: utf-8 -*-
"""
Simple graph class for breadth_first_search algorithm
Created on Thu Oct 11 20:37:14 2018

@author: Yifan Tang
email : tangyf96@outlook.com
reference : https://www.redblobgames.com/pathfinding/a-star/
"""


#class SimpleGraph:
#    """
#    Simple graph realization using dictionary
#    """
#    def __init__(self):
#        self.edges = {}
#        
#    def neighbors(self, id):
#        return self.edges[id]
import operator

class Node:
    def __init__(self, x, y, h = 0):
        """
        id: the node's coordinate
        h_function : node's current path cost to goal
        key_function : node's priority function
        """
        self.id = (x,y)
        self.h_func = h
        self.key_func = h
        self.lower_state = True
        self.back_pointer = None
    
    def __lt__(self, other):
        return self.key_func < other.key_func
    
    def eq(self, other):
        #print(self.id)
        #print(other.id)
        return operator.eq(self.id, other.id)
        
        
class Grid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []
        self.node_list = []
        self.addNodeToGraph()
    
    def addNodeToGraph(self):
        """
        Add all Node objects into graph
        """
#        iter = 1
        for x in range(self.width):
            for y in range(self.height):
                #print(iter)
                new_node = Node(x,y)
                if (x,y) in self.walls:
                    new_node.cost = 10000
                self.node_list.append(Node(x,y))   
#                iter += 1
    
    
    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, node):
        return node not in self.walls
    
    def neighbors(self, node):
        """
        Find the neighbor of node and return the index of neighbors in the node list
        """
        (x, y) = node.id
        neighbors = []
        #neighbors_index = []
        neighbors_id = [(x, y+1), (x+1, y), (x, y-1), (x-1, y), (x+1, y+1), (x+1, y-1), (x-1, y+1), (x-1, y-1)]
        # filter out the unreachable neighbors
        neighbors_id = filter(self.in_bounds, neighbors_id)
        neighbors_id = filter(self.passable, neighbors_id)
        neighbors_id = list(neighbors_id)
        for node in self.node_list:
            if node.id in neighbors_id:
                #neighbors_index.append(index)
                neighbors.append(node)
            else:
                pass
        return neighbors


class GridWithWeights(Grid):
    def __init__(self, width, height):
        super().__init__(width, height)
        # not using weights for current algorithm
        # weight is used to change the edge cost of new obstacle
        # should think about it!!
        self.weights = {}
    
    def cost(self, from_node, to_node): 
        """
        get to_node's edge cost value 
        the default value to diagonal traversal is 1.4
        the default value to horizontal or vertical traversal is 1
        """
        if ( from_node[0] != to_node[0] and from_node[1] != to_node[1] ):
            return self.weights.get(to_node, 1.4)
        else: 
            return self.weights.get(to_node, 1)
    
    def key_function(self, node):
        """
        Calculate the key function which serves as the priority function of the state
        Not finished the part for raise state
        """
        index = self.node_list.index(node)
        if node.lower_state:
            # should use self.graph 
            self.node_list[index].key_func = node.h_func
        



import heapq
class PriorityQueue():
    def __init__(self):
        self.open_list = []
    
    def put(self, value, priority):
        heapq.heappush(self.open_list, (priority, value))
    
    def pop(self):
        """
        return the value that has the highest priority
        """
        return heapq.heappop(self.open_list)[1]
    
    def empty(self):
        return len(self.open_list) == 0
    
    def remove(self, item, new_priority):
        self.open_list.remove((item.key_func, item))
        


def draw_grid(graph, width=2, **style):
    for y in range(graph.height):
        for x in range(graph.width):
            print("%%-%ds" % width % draw_tile(graph, (x, y), style, width), end="")
        print()
        
def draw_tile(graph, id, style, width):
    r = "."
    if 'number' in style and id in style['number']: r ="%.1f" % float(style['number'][id])
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = ">"
        if x2 == x1 - 1: r = "<"
        if y2 == y1 + 1: r = "v"
        if y2 == y1 - 1: r = "^"
    if 'start' in style and id == style['start']: r = "Start"
    if 'goal' in style and id == style['goal']: r = "Goal"
    if 'path' in style and id in style['path']: r = "@"
    if id in graph.walls: r = "#" * width
    return r