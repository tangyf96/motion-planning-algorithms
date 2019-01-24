# -*- coding: utf-8 -*-
"""
This is the evaluation for flexible RRT star and traditional RRT star
@author: Yifan Tang
"""
import time
import operator
import matplotlib.pyplot as plt
import math
import copy
import numpy as np
from frrt.flexible_rrt import fRRT, Node
from rrt_algorithm.rrt_star import RRT as rrt_star


class Robot:
    def __init__(self,
                 start,
                 goal_list=[],
                 trans_prob=[],
                 planner=None,
                 human_model=None):
        """
        Robot class to simulate a robot working with different planner.
        :param start: the initial location of robot
        :param goal_list: the possible goal of the robot
        :param planner: RRT planner to generate plan
        """
        self.start = start
        self.cur_loc = start
        self.cur_goal = [planner.cur_goal.x, planner.cur_goal.y]
        self.new_goal = self.cur_goal
        self.goal_list = goal_list
        self.planner = planner
        self.trans_prob = trans_prob
        self.speed = 1.0
        self.path_distance = []
        self.human_model = human_model

    def move(self, start_loc, next_loc_ind):
        """
        move the robot along the path
        modify self.cur_loc
        :param start_loc: [x,y]
        :param next_loc: [x,y]
        """
        #print("next_loc_ind:", next_loc_ind)
        #print("len of path:", len(self.planner.path))
        next_loc = self.planner.path[next_loc_ind]
        dist = math.sqrt((next_loc[0] - start_loc[0])**2 +
                         (next_loc[1] - start_loc[1])**2)
        if dist > self.speed:
            dx = self.speed * (next_loc[0] - start_loc[0]) / dist
            dy = self.speed * (next_loc[1] - start_loc[1]) / dist
            self.cur_loc[0] += dx
            self.cur_loc[1] += dy
            return next_loc_ind
        else:
            # in one time step, robot will pass several waypoints
            move_left = self.speed
            while dist <= move_left and next_loc_ind != len(
                    self.planner.path) - 1:
                move_left -= dist
                start_loc = next_loc
                next_loc_ind += 1
                next_loc = self.planner.path[next_loc_ind]
                dist = math.sqrt((next_loc[0] - start_loc[0])**2 +
                                 (next_loc[1] - start_loc[1])**2)
            if dist > move_left:
                dx = move_left * (next_loc[0] - start_loc[0]) / dist
                dy = move_left * (next_loc[1] - start_loc[1]) / dist
                self.cur_loc[0] = start_loc[0] + dx
                self.cur_loc[1] = start_loc[1] + dy
                return next_loc_ind
            else:
                # reach the goal in one time step
                self.cur_loc = [
                    self.planner.path[-1][0], self.planner.path[-1][1]
                ]
                return next_loc_ind + 1

    def change_goal(self):
        """
        Change the goal of the robot
        :param change_prob: the probability of goal changing
        :return: new goal's location 
        """
        change_prob = 0.3
        self.goal_list.index(self.cur_goal)
        if np.random.random() < change_prob:
            # change goal
            cur_goal_ind = self.goal_list.index(self.cur_goal)
            new_goal_ind = np.random.choice(
                len(self.goal_list), 1, p=self.trans_prob[cur_goal_ind])[0]
            new_goal = self.goal_list[new_goal_ind]
            return new_goal
        else:
            return self.cur_goal

    def path_dist(self):
        """
        Calculate the length of path
        """
        dist = 0
        prev = self.planner.path[0]
        for point in self.planner.path:
            dist += math.sqrt((point[0] - prev[0])**2 +
                              (point[1] - prev[1])**2)
            prev = point
        return dist

    def find_path(self):
        """
        Use the planner to find the path
        :return : the path list
        """
        new_path = self.planner.planning()
        if new_path:
            return new_path
        else:
            print("Can't find the path")
            return None

    def work(self, work_time):
        """
        Simulation
        :param work_time: the working time step for the robot 
        """
        path = self.find_path()
        while path is None:
            path = self.find_path()
        next_loc_ind = 1
        while work_time != 0:
            # change to new goal
            self.new_goal = self.change_goal()
            cur_goal_ind = self.goal_list.index(self.cur_goal)
            new_goal_ind = self.goal_list.index(self.new_goal)
            if operator.ne(self.cur_goal, self.new_goal):
                print("the goal change from", self.cur_goal, "to",
                      self.new_goal)

                # replan
                # reset the planner
                self.planner.start = Node(self.cur_loc[0], self.cur_loc[1])
                self.planner.cur_goal = Node(self.new_goal[0],
                                             self.new_goal[1])
                self.planner.reset()
                self.cur_goal = self.new_goal
                self.start = self.cur_loc
                # find the new path
                while self.find_path() is None:
                    path = self.find_path()
                next_loc_ind = 1
                # draw the new path
                if path:
                    # self.draw_path(path)
                    # add the path distance if the path is supposed to 
                    # biased towards new goal
                    if self.human_model[cur_goal_ind][new_goal_ind] == 1:
                        self.path_distance.append(self.path_dist())
            else:
                if operator.ne(self.cur_loc, self.cur_goal):
                    # move to next location
                    next_loc_ind = self.move(self.cur_loc, next_loc_ind)
                    if next_loc_ind == len(self.planner.path):
                        print("the robot reaches the goal!", self.cur_goal)
                    # self.draw_path(path)
                else:
                    print("the robot has reached the goal!", self.cur_goal)
            work_time -= 1
            time.sleep(1)

    def draw_path(self, path):
        """
        :param path: list of waypoints location
        """
        #print("begin to draw path!")
        fig = plt.gcf()
        fig.clf()
        ax = plt.gca()
        ax.set_xlim((self.planner.min_rand, self.planner.max_rand))
        ax.set_ylim((self.planner.min_rand, self.planner.max_rand))
        # plot path
        ax.plot([x for (x, y) in self.planner.path],
                [y for (x, y) in self.planner.path], "-k")
        ax.plot([x for (x, y) in self.planner.path],
                [y for (x, y) in self.planner.path], "yo")
        # plot goal list
        ax.plot([x for (x, y) in self.goal_list],
                [y for (x, y) in self.goal_list], 'g*')
        # plot current start and goal
        ax.plot(self.planner.start.x, self.planner.start.y, 'ro')
        ax.plot(self.cur_goal[0], self.cur_goal[1], 'bo')
        ax.plot(self.cur_loc[0], self.cur_loc[1], 'r*')
        # plot obstacles
        for (x, y, radius) in self.planner.obstacle:
            circle = plt.Circle((x, y), radius, color='y')
            ax.add_artist(circle)
        plt.draw()
        plt.pause(0.001)


def main():
    print("Start evaluation")
    # ===Search Path with flexible rrt===
    # initialize
    cur_goal = [14, 14]
    goal_list = [[1, 4], [4, 1], [5, 10], [10, 5], [2, 14], [14, 2], [14, 14]]

    simu_time = 500
    # trans_prob 2d array [from, to]
    human_goal_model = np.array([[1, -1, 1, 0, 1, 0,
                                  1], [0, 1, -1, 1, 0, 1, 1],
                                 [1, 0, 1, 0, 1, -1,
                                  1], [0, 1, -1, 1, 0, 1, 1],
                                 [1, 0, 1, -1, 1, 0,
                                  1], [0, 1, 0, 1, -1, 1, 1],
                                 [1, 0, -1, 0, 1, 1, 1]])

    trans_prob = np.array([[0.2, 0, 0.2, 0.1, 0.2, 0.1, 0.2],
                           [0.1, 0.2, 0, 0.2, 0.1, 0.2, 0.2],
                           [0.2, 0.1, 0.2, 0.1, 0.2, 0, 0.2],
                           [0.1, 0.2, 0, 0.2, 0.1, 0.2, 0.2],
                           [0.2, 0.1, 0.2, 0, 0.2, 0.1, 0.2],
                           [0.1, 0.2, 0.1, 0.2, 0, 0.2, 0.2],
                           [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.4]])
    obstacle = [(8, 8, 1), (6, 6, 1), (12, 12, 1)]
    # flexible RRT
    frrt = fRRT(
        start=[0, 0],
        cur_goal=cur_goal,
        goal_list=goal_list,
        obstacle=obstacle,
        TreeArea=[-1, 15],
        trans_prob=trans_prob)

    robot1 = Robot(
        start=[0, 0],
        goal_list=goal_list,
        trans_prob=trans_prob,
        planner=frrt,
        human_model=human_goal_model)

    robot1.work(simu_time)
    ave_path_dist1 = sum(robot1.path_distance) / len(robot1.path_distance)

    # rrt star

    rrt_star_planner = rrt_star(
        start=[0, 0], 
        goal=cur_goal, 
        obstacleList=obstacle, 
        randArea=[-1, 15])
    robot2 = Robot(
        start=[0, 0],
        goal_list=goal_list,
        trans_prob=trans_prob,
        planner=rrt_star_planner,
        human_model=human_goal_model)
    robot2.work(simu_time)
    ave_path_dist2 = sum(robot2.path_distance) / len(robot2.path_distance)
    print("average path distance for fRRT:", ave_path_dist1)
    print("average path distance for rrt star:", ave_path_dist2)


if __name__ == '__main__':
    main()