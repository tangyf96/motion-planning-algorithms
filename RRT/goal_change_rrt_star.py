# -*- coding: utf-8 -*-
"""
Created on Nov 4 2018
This is the program to move the robot followed the existing path
@author: Yifan Tang
"""

import time
import operator
from rrt_star import *


class Robot:
    def __init__(self, start, goal_list=[], planner=None):
        """
        This is the robot class that tries to simulate a robot working in a warehouse
        with the possibility of changing goal.
        :param start: the initial location of robot
        :param goal_list: the possible goal of the robot
        :param planner: RRT planner to generate plan
        """
        self.name = "robot1"
        self.start = start
        self.cur_loc = start
        self.cur_goal = [planner.goal.x, planner.goal.y]
        self.new_goal = self.cur_goal
        self.goal_list = goal_list
        self.planner = planner

    def move(self, start_loc, next_loc):
        """
        move the robot along the path node
        :param start_loc: [x,y]
        :param next_loc: [x,y]
        """
        if operator.eq(self.cur_loc, start_loc):
            self.cur_loc = next_loc
        else:
            print('Error: start node is not the same as robot\'s current location')

    def change_goal(self):
        """
        change the goal of the robot.
        :param: change_prob: the probability of goal change
        :return: new goal's location list [new_goal.x, new_goal.y]
        """
        change_prob = 0.15
        if random.random() < change_prob:
            index = self.goal_list.index(self.cur_goal)
            new_goal = self.goal_list[random.choice([x for x in range(0, len(self.goal_list)) if x != index])]
            return new_goal
        else:
            return self.cur_goal

    def find_path(self):
        """
        Use the RRT star planner to plan the path
        :return: the path list
        """
        new_path = self.planner.Planning()
        if new_path is None:
            print("Can't find the path")
            return None
        new_path.reverse()
        return new_path

    def draw_path(self, path):
        for i in range(len(self.goal_list)):
            plt.plot(self.goal_list[i][0], self.goal_list[i][1], 'y*')
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.plot(self.cur_loc[0], self.cur_loc[1], 'go')
        plt.plot(self.planner.start.x, self.planner.start.y, 'bo')
        plt.plot(self.planner.goal.x, self.planner.goal.y, 'bo')
        '''
        for (ox, oy, size) in self.planner.obstacleList:
        plt.plot(ox, oy, "ok", ms=30 * size)
        '''
        ax = plt.gca()
        ax.set_xlim((self.planner.minrand, self.planner.maxrand))
        ax.set_ylim((self.planner.minrand, self.planner.maxrand))
        plt.show()

    def work(self):
        """
        :param: work_time - the working time step for the robot.

        """
        work_time = 100
        # make the initial plan
        path = self.find_path()

        robot_state = 1
        while work_time != 0:
            # try to change to new goal
            self.new_goal = self.change_goal()
            # if goal change
            if operator.ne(self.cur_goal, self.new_goal):
                # replanW
                self.planner.start = Node(self.cur_loc[0], self.cur_loc[1])
                self.planner.goal = Node(self.new_goal[0], self.new_goal[1])
                self.planner.path = [[self.new_goal[0], self.new_goal[1]]]
                print("the goal change from", self.cur_goal, "to", self.new_goal)
                self.cur_goal = self.new_goal
                path = self.find_path()
                robot_state = 1
                # draw the new path
                self.draw_path(path=path)
                plt.pause(0.1)
            else:
                # move the robot to the next point
                if robot_state < len(path):
                    self.move(self.cur_loc, path[robot_state])
                    self.draw_path(path)
                    plt.pause(0.5)
                    robot_state += 1
                else:
                    print("the robot reach the goal!:", self.cur_goal)
            work_time -= 1
            time.sleep(1)

def main():
    # define the parameters for working space
    warehouse_area = [0, 50]
    cargo_loc = [[25, 5], [10, 40], [40, 40]]
    '''
        obstacleList = [
        (25, 25, 2),
        (30, 6, 2),
        (5, 10, 2),
        (7, 5, 2),
        (30, 40, 2)
    ]
    '''

    # initialize instance for RRT planning
    start = [0, 0]
    goal = [40, 40]
    # rrt_star = RRT(start=start, goal=goal, obstacleList=obstacleList, randArea = warehouse_area)
    rrt_star = RRT(start=start, goal=goal, randArea = warehouse_area)
    # initialize instance for the robot working in the space
    robot1 = Robot(start=start, goal_list=cargo_loc, planner=rrt_star)
    # start working
    robot1.work()


if __name__ == '__main__':
    main()
