#-*- coding: utf-8 -*-
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
                # doesn't reach the goal in this time step
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

class Human():
    """
    Human class to simulate a human working in the environment, following its goal change model.
    """
    def __init__(self, speed, goal_list, cur_goal, cur_loc, trans_prob):
        """
        Input: 
            speed : the movement speed of human
            goal_list: the possible goals for human
            cur_goal: current goal position
            cur_loc : human's current location            
            trans_prob : the human transition model 
        """
        self.speed = speed
        self.trans_prob = trans_prob
        self.cur_loc = copy.deepcopy(cur_loc)
        self.goal = cur_goal
        self.goal_list_ = copy.deepcopy(goal_list)

    def move(self):
        """
        move the human towards its goal and modify self.cur_loc
        """
        dist = math.sqrt((self.goal[0] - self.cur_loc[0])**2 +
                         (self.goal[1] - self.cur_loc[1])**2)
        if dist > self.speed:
            dx = self.speed * (self.goal[0] - self.cur_loc[0]) / dist
            dy = self.speed * (self.goal[1] - self.cur_loc[1]) / dist
            # problem here
            self.cur_loc[0] += dx
            self.cur_loc[1] += dy
        else:
            # in one time step, human will reach the goal
            self.cur_loc = self.goal
    
    def change_goal(self):
        """
        Change the goal of the human
        :return: new goal's location 
        """
        change_prob = 0.3
        self.goal_list_.index(self.goal)
        if np.random.random() < change_prob:
            # change goal
            cur_goal_ind = self.goal_list_.index(self.goal)
            new_goal_ind = np.random.choice(len(self.goal_list_), 1, p=self.trans_prob[cur_goal_ind])[0]
            temp = self.goal_list_[new_goal_ind]
            self.goal = temp
            return self.goal
        else:
            return self.goal

class Experiment():
    def __init__(self, robot, human, work_time):
        self.robot = robot
        self.human = human
        self.work_time = work_time
        self.path_distance = []

    def work(self):
        """
        Simulation for work_time time steps
        """
        # find the initial path
        path = self.robot.find_path()
        while path is None:
            path = self.robot.find_path()
        next_loc_ind = 1
        # simulation begins
        while self.work_time != 0:
            # change to new goal
            self.robot.new_goal = self.human.change_goal()
            cur_goal_ind = self.robot.goal_list.index(self.robot.cur_goal)
            try:
                new_goal_ind = self.robot.goal_list.index(self.robot.new_goal)
            except Exception as e:
                print('error')
            # if goal change
            if operator.ne(self.robot.cur_goal, self.robot.new_goal):
                print("the goal change from", self.robot.cur_goal, "to",
                      self.robot.new_goal)
                # replan and reset the planner
                self.robot.planner.start = Node(self.robot.cur_loc[0], self.robot.cur_loc[1])
                self.robot.planner.cur_goal = Node(self.robot.new_goal[0],
                                             self.robot.new_goal[1])
                self.robot.planner.reset()
                self.robot.cur_goal = self.robot.new_goal
                self.robot.start = self.robot.cur_loc
                # find the new path
                path = self.robot.find_path()
                while path is None:
                    path = self.robot.find_path()
                next_loc_ind = 1
                # draw the new path
                if path:
                    self.draw_path(path)
                    # Calculate the distance between robot and human
                    if self.robot.human_model[cur_goal_ind][new_goal_ind] == 1:
                        # change to be the position between human and robot
                        self.path_distance.append(np.linalg.norm([self.robot.start[0]-self.human.cur_loc[0],
                                                                self.robot.start[1]-self.human.cur_loc[1]]))
            else:
                # move the robot to next location
                if operator.ne(self.robot.cur_loc, self.robot.cur_goal):
                    if next_loc_ind == len(self.robot.planner.path):
                        print("the robot reaches the goal!", self.robot.cur_goal)
                        continue
                    try:
                        next_loc_ind = self.robot.move(self.robot.cur_loc, next_loc_ind)
                    except IndexError:
                        print("Index Error line 238!")
                else:
                    print("the robot has reached the goal!", self.robot.cur_goal)
                
                # move the human to next location
                self.human.move()
                self.draw_path(path)

            self.work_time -= 1
            #time.sleep(1)
        
    def draw_path(self, path):
        """
        :param path: list of waypoints location
        """
        #print("begin to draw path!")
        fig = plt.gcf()
        fig.clf()
        ax = plt.gca()
        ax.set_xlim((self.robot.planner.min_rand, self.robot.planner.max_rand))
        ax.set_ylim((self.robot.planner.min_rand, self.robot.planner.max_rand))
        # plot path
        ax.plot([x for (x, y) in self.robot.planner.path],
                [y for (x, y) in self.robot.planner.path], "-k")
        ax.plot([x for (x, y) in self.robot.planner.path],
                [y for (x, y) in self.robot.planner.path], "yo")
        # plot goal list
        ax.plot([x for (x, y) in self.robot.goal_list],
                [y for (x, y) in self.robot.goal_list], 'g*')
        # plot current start and goal, robot position
        ax.plot(self.robot.planner.start.x, self.robot.planner.start.y, 'ro')
        ax.plot(self.robot.cur_goal[0], self.robot.cur_goal[1], 'bo')
        ax.plot(self.robot.cur_loc[0], self.robot.cur_loc[1], 'r*')
        # plot human position
        ax.plot(self.human.cur_loc[0], self.human.cur_loc[1], 'b*')
        # plot obstacles
        for (x, y, radius) in self.robot.planner.obstacle:
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

    simu_time = 50
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

    human1 = Human(
        speed=1.5, 
        goal_list=goal_list,
        cur_goal=cur_goal, 
        cur_loc=[5,5],
        trans_prob = trans_prob)
    
    exp1 = Experiment(robot=robot1, human=human1, work_time=simu_time)
    exp2 = Experiment(robot=robot2, human = human1, work_time=simu_time)
    ave_path_dist1 = []
    ave_path_dist2 = []

    # num_simu = 1
    exp1.work()
    ave_path_dist1.append(sum(exp1.path_distance) / len(exp1.path_distance))
    exp2.work()
    ave_path_dist2.append(sum(exp2.path_distance) / len(exp2.path_distance))



    # robot1.work(simu_time)
    # ave_path_dist1 = ave_path_dist1/num_simu
    # ave_path_dist2 = ave_path_dist2/num_simu
    print("average path distance for fRRT:", ave_path_dist1)
    print("average path distance for rrt star:", ave_path_dist2)


if __name__ == '__main__':
    main()