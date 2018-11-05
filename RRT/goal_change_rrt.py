# -*- coding: utf-8 -*-
"""
Created on Nov 4 2018
This is the program to move the robot followed the existing path
@author: Yifan Tang
"""

import operator
from basic_rrt import *

class robot():
    def __init__(self, start, velocity = 0.05, goal_list = [], planner=None):
        self.name = "robot1"
        self.start = start
        self.cur_loc = start
        self.cur_goal = goal_list[0]
        self.new_goal = self.cur_goal
        self.velocity = velocity
        self.goal_list = goal_list
        self.planner = planner

    def move(self, start_loc, next_loc):
        """
        move the robot along the path node
        :param start_loc: [x,y]
        :param next_loc: [x,y]
        :return:
        """
        if operator.eq(self.cur_loc, start_loc):
            self.cur_loc = next_loc
        else:
            print('start node is not the same as robot\'s current location')


    def change_goal(self):
        change_prob = 0.3
        if random.random() < change_prob:
            index = self.goal_list.index(self.cur_goal)
            new_goal = self.goal_list[random.choice([x for x in range(0,len(self.goal_list)) if x!=index])]
            return new_goal
        else:
            return self.cur_goal

    def work(self):
        work_time = 40
        # make the initial plan
        path = self.find_path()

        robot_state = 1
        while work_time!=0:
            # try to change to new goal
            self.new_goal = self.change_goal()
            # if goal change
            if operator.ne(self.cur_goal, self.new_goal):
                # replan
                self.planner.start = Node(self.cur_loc[0], self.cur_loc[1])
                self.planner.goal = Node(self.new_goal[0], self.new_goal[1])
                self.planner.path = [[self.new_goal[0], self.new_goal[1]]]
                path = self.find_path()
                robot_state = 1
                # draw the new path
                for i in range(len(self.goal_list)):
                    plt.plot(self.goal_list[i][0], self.goal_list[i][1], 'y*')
                self.planner.DrawTree(find_path=True, result=True)
            else:
                # move the robot to the next point
                if robot_state < len(path):
                    self.move(self.cur_loc, path[robot_state])
                    robot_state += 1
            work_time -= 1

    def find_path(self):
        flag = self.planner.GrowTree()
        if flag:
            self.planner.FindPath()
        new_path = self.planner.path
        new_path.reverse()
        print(new_path)
        return new_path

def main():
    # define the parameters for working space
    warehouse_area = [0, 15]
    cargo_loc = [[2,2],[12,4],[9,10]]
    # initialize instance for RRT planning
    start = [0, 0]
    goal = [9, 10]
    rrt = RRT(start= start,goal = goal, stepsize=1, TreeArea=warehouse_area)
    # find_path = rrt.GrowTree()
    # if find_path == True:
    #     rrt.FindPath()
    # else:
    #     pass
    # rrt.DrawTree(find_path, result = True)

    # initialize instance for the robot working in the space
    robot1 = robot(start = start, goal_list = cargo_loc, planner= rrt)
    robot1.work()



if __name__ == '__main__':
    main()
