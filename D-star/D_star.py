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
    def __init__(self, graph, start, goal, new_walls = []):
        self.graph = graph
        self.start = Node(start[0], start[1])
        self.cur_goal = Node(goal[0], goal[1])
        self.path = []
        self.open_list = PriorityQueue()
        self.new_walls = new_walls
        
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
    
    @property
    def init_plan(self):
        """
        Initially search from the goal to the start 
        """

        self.open_list.put(self.start, self.start.key_func)
        
        close_list = {}
        back_pointer = {}
        back_pointer[self.start] = None
        
        while not self.open_list.empty():
            current = self.open_list.pop()
            #print(current)
            if self.cur_goal.eq(current):
                print("find the goal")
                break
            close_list[current.id] = current.h_func
            # add the neighbors of current node to open list
            for node in self.graph.neighbors(current):
                edge_cost = self.graph.cost(current.id, node.id)
                current_cost = edge_cost + current.h_func
                #print(node.id)
                if node.back_pointer == None and (node.id != self.start.id):
                    # update the h value of new node
                    node.h_func = current_cost
                    # update the key value of new node
                    node.key_func = node.h_func
                    # set the back pointer of new node
                    node.back_pointer = current.id
                    back_pointer[node] = node.back_pointer
                    # add new node to open list
                    self.open_list.put(node, node.key_func)
                elif (node.h_func > current_cost) and (not self.check_back_pointer(back_pointer, current, node)):
                    # update the node's priority in the priority queue
                    self.open_list.remove(node, node.key_func)

                    node.back_pointer = current.id
                    back_pointer[node] = node.back_pointer
                    node.key_func = node.h_func
                    self.open_list.put(node, node.key_func)
                else:
                    pass
                    
        return back_pointer, close_list
 
    '''
    def replan(self):
        # move the robot along the existing path
        current_pos = self.start
        self.path.reverse()
        while(current_pos.id != self.cur_goal.id):
            for node in self.path:
                if node.id in self.new_walls:
                    # modify the edge cost of current graph
                    self.modify_cost(node)
                    # put all affected neighbors into open_list
                    # if they are not in the list
                    neighbors = self.graph.neighbors(node)
                    for neighbor in neighbors:
                        if (neighbor, neighbor.key_func) in self.open_list:
                            pass
                        else:
                            self.open_list.put(neighbor, neighbor.key_func)
                            
            # replan the path
            
            # update the path, move to the next position according to the new path
            
            # move the robot to the next position
            current_pos = item
            

    def modify_cost(self, node):
        """
        Once the robot detects an error in the arc cost function
        change the arc cost function and enter affected states on the open list
        """   
        node = self.graph.node_list[node.id[0] * self.graph.width + node.id[1]]
        node.h_func = 10000
        self.weights[node.id] = 10000
    
    '''
    def find_path(self, back_pointer):
        # extract the goal node from self.graph.node_list
        node = self.graph.node_list[self.cur_goal.id[0] * self.graph.width + self.cur_goal.id[1]]
        self.path.append(node)
        parent = node.back_pointer

        while(parent != None):
            node = self.graph.node_list[parent[0]*self.graph.width+parent[1]]
            self.path.append(node)
            parent = node.back_pointer

    #def check_lower_state(self, node):
        
def draw_path(start, goal, path, walls):
    """
    draw the final path
    """
    # draw the path
    plt.plot([node.id[0] for node in path], [node.id[1] for node in path], '-r')    
    plt.plot(start[0],start[1],'bo')
    plt.plot(goal[0], goal[1], 'bo')
    plt.grid(True)
    # draw the wall node
    plt.plot([x for (x,y) in walls], [y for (x,y) in walls], 'ko')
    plt.show()
       
        
def main():
    # initialize graph and convert element to Node type
    graph = GridWithWeights(10, 10)
    graph.walls = [(1, 7), (5, 8), (5, 7), (9, 8), (3, 7), (3, 8)]
    new_walls = [(5,5)]
    
    start = (1,8)
    goal = (8,2)
    d_star_planner = D_star(graph, start, goal, new_walls)
    back_pointer, close_list = d_star_planner.init_plan
    # this should be updated to support diagnal pointer
    #draw_grid(graph, width = 10, point_to = back_pointer, start = start, goal = goal)
    d_star_planner.find_path(back_pointer)
    draw_path(start, goal, d_star_planner.path, graph.walls)

    print()
    draw_grid(graph, width = 10, number = close_list, start = start, goal = goal)
    print()

if __name__ == '__main__':
    main()
    