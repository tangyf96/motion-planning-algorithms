#-*- coding: utf-8 -*-
"""
This is the evaluation for flexible RRT star and traditional RRT star
@author: Yifan Tang
"""
import time
import warnings
import operator
import matplotlib.pyplot as plt
import math
import copy
import numpy as np
import pickle
from frrt.flexible_rrt import fRRT, Node
from rrt_algorithm.rrt_star import RRT as rrt_star
import layout_generation as generation

class Robot:
    def __init__(self,
                 start,
                 location_list=[],
                 trans_prob=[],
                 planner=None,
                 ):
        """
        Robot class to simulate a robot working with different planner.
        :param start: the initial location of robot
        :param location_list: the warehouse stations in the system
        :param planner: RRT planner to generate plan
        """
        self.cur_start = copy.deepcopy(start)
        self.fix_start = copy.deepcopy(start)
        self.cur_loc = copy.deepcopy(start)
        self.cur_goal = [planner.cur_goal.x, planner.cur_goal.y]
        self.prev_goal = [planner.cur_goal.x, planner.cur_goal.y]
        self.location_list = copy.deepcopy(location_list)
        self.planner = planner
        self.trans_prob = trans_prob
        self.speed = 1.0
        # whether robot is carrying out task of human
        self.is_on_task = False
        # whether robot is willing to help
        self.is_able_help = True


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
                # self.cur_loc = [
                #     self.planner.path[-1][0], self.planner.path[-1][1]
                # ]
                # change to this method to avoid floating error
                self.cur_loc = [self.planner.cur_goal.x, self.planner.cur_goal.y]
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
    def __init__(self, location_list, cur_loc, trans_prob, help_prob=0.5, change_prob=0):
        """
        Input: 
        :param location_list: the possible locations for human
        :param cur_loc : human's current location            
        :param trans_prob : the probability matrix that represents the human preference of 
                            locations to which the robot help carry packages. 
        :param help_prob : the probability that human needs help from the robot
        :param change_prob : the probability that human will change its working position
        """
        self.trans_prob = copy.deepcopy(trans_prob)
        self.cur_loc = copy.deepcopy(cur_loc)
        self.location_list_ = copy.deepcopy(location_list)
        self.need_help_prob = help_prob
        self.change_prob = change_prob
        self.is_need_help = False

    def change_pos(self):
        """
        Change the working position of the human
        :return: new position of human
        """
        change_prob = 0.3
        if np.random.random() < change_prob:
            # change goal using uniform distribution
            new_loc_ind = np.random.choice(len(self.location_list_))
            self.cur_loc = self.location_list_[new_loc_ind]
            return self.cur_loc
        else:
            return self.cur_loc

    def help_client(self):
        """
        The human uses this function to choose whether calling the robot for help or not
        :return: the boolean variabel indicating whether human needs help
        """
        if self.is_need_help:
            return True
        else:
            if np.random.random() < self.need_help_prob:
                self.is_need_help = True
                return True
            else:
                return False
    
    def generate_goal_pos(self):
        '''
        This function uses the transition matrix to generate the next goal for the robot.
        Or, this function chooses the goal which the human wants the robot carry the package to.
        '''
        # find the index of human's current position in the location list
        for ind, loc in enumerate(self.location_list_):
            if operator.eq(loc, self.cur_loc):
                cur_loc_ind = ind
                break
        # generate the next location for the robot
        prob = self.trans_prob[cur_loc_ind]
        # generate the index of the location
        new_loc_ind = np.random.choice(len(self.location_list_), p=prob)

        return self.location_list_[new_loc_ind]

class Experiment():
    def __init__(self, robot, human, work_time):
        self.robot = robot
        self.human = human
        self.work_time = work_time
        self.dist_to_human = []
        self.path_distance = []
        self.eval_var = []
        self.human_help_cnt = 0

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
            '''
            determine if human needs help and the robot is available to help (by on task variable)
            if yes, change the goal of the robot, store the previous goal, 
            make a new path to human's current position
            '''
            # decide whether human will call help
            if (not self.human.is_need_help) and (not self.robot.is_on_task) and (self.work_time % 5 is 0):
                self.human.is_need_help = self.human.help_client()
                if self.human.is_need_help:
                    self.human_help_cnt += 1
            # main 
            if self.human.is_need_help:
                # should I limit the time of providing help on each path? 
                if self.robot.is_on_task:
                    # the robot is on its way to the human's current position
                    # move the robot along its path
                    if operator.ne(self.robot.cur_loc, self.robot.cur_goal):
                        if next_loc_ind == len(self.robot.planner.path):
                            print("the robot reaches the goal!", self.robot.cur_goal)
                            # self.draw_path()
                            continue
                        try:
                            next_loc_ind = self.robot.move(self.robot.cur_loc, next_loc_ind)
                            # self.draw_path()
                        except IndexError:
                            print("Index Error line 221!")
                    else:
                        # robot reach the goal
                        # decide whether the goal is human or the detour station
                        if operator.eq(self.robot.cur_goal, self.human.cur_loc):
                            # the goal is human
                            # self.draw_path()
                            print("the robot has reached the human position to catch packages!", self.robot.cur_goal)
                            # reset the planner to robot's original goal
                            self.reset_planner(self.robot.prev_goal)
                            # robot has helped human
                            self.human.is_need_help = False
                            # generate new path
                            path = self.robot.find_path()
                            while path is None:
                                path = self.robot.find_path()
                            next_loc_ind = 1
                            # draw the new path
                            if path:
                                # self.draw_path()
                                # Can add the measurement function here
                                pass

                        else:
                            warnings.warn('the robot reach the goal, but not the human\' current positions!', UserWarning)
                                
                else:
                    # the robot is not on the task
                    # it is possible that the robot can help if human is on its way. 
                    # determine if human is on the robot's way, whether the robot wants to help

                    # if the robot didn't agree to help in the past
                    if self.robot.is_able_help:
                        is_help = self.robot_decide_help()
                    else:
                        # the robot has helped the human on this way. 
                        is_help = False
                    # 
                    if is_help:
                        # help the human
                        # store the robot's current goal
                        self.robot.prev_goal = self.robot.cur_goal
                        self.robot.is_on_task = True
                        # replan and reset the planner towards human's current position
                        self.reset_planner(self.human.cur_loc)
                        # find the new path
                        path = self.robot.find_path()
                        while path is None:
                            path = self.robot.find_path()
                        next_loc_ind = 1
                        # draw the new path
                        if path:
                            # self.draw_path()
                            # Add the measurement function here
                            diff = np.diff(np.array(path), axis = 0)
                            temp = np.linalg.norm(diff, axis=1)
                            # find the corresponding transition probability
                            for ind, loc in enumerate(self.robot.location_list):
                                if self.human.cur_loc[0] == loc[0] and  self.human.cur_loc[1] == loc[1]:
                                    # print("check")
                                    from_loc_ind = ind
                                if self.robot.prev_goal[0] == loc[0] and self.robot.prev_goal[1] == loc[1]:
                                    to_loc_ind = ind
                                
                            prob_to_cur_goal = self.robot.trans_prob[from_loc_ind][to_loc_ind]
                            # use probability * path distance to measure the effectiveness of the algorithm
                            self.path_distance.append(np.sum(temp))
                            self.eval_var.append(np.sum(temp) / prob_to_cur_goal)

                    else:
                        # robot refuse to help
                        print("the robot refuse to help!")
                        self.robot.is_able_help = False
                        # move 
                        if operator.ne(self.robot.cur_loc, self.robot.cur_goal):
                            if next_loc_ind == len(self.robot.planner.path):
                                # self.draw_path()
                                print("the robot reaches the goal!", self.robot.cur_goal)
                                continue
                            try:
                                next_loc_ind = self.robot.move(self.robot.cur_loc, next_loc_ind)
                                # self.draw_path()
                            except IndexError:
                                print("Index Error line 260!")
                        else:
                            # robot reaches its own goal
                            # self.draw_path()
                            print("the robot has reached the goal!", self.robot.cur_goal)
                            # return the goal and start position to start a new round 
                            self.reset_planner(self.robot.fix_start)
                            self.robot.fix_start = copy.deepcopy(self.robot.cur_loc)
                            self.robot.is_able_help = True
                            # generate new path
                            path = self.robot.find_path()
                            while path is None:
                                path = self.robot.find_path()
                            next_loc_ind = 1
                            # draw the new path
                            if path:
                                # self.draw_path()
                                pass
                                # Can Add the measurement function here
            else:
                # the human don't need help currently
                # the robot should be on its original path or on the detour path to its goal. 
                if operator.ne(self.robot.cur_loc, self.robot.cur_goal):
                    if next_loc_ind == len(self.robot.planner.path):
                        print("the robot reaches the goal!", self.robot.cur_goal)
                        continue
                    try:
                        next_loc_ind = self.robot.move(self.robot.cur_loc, next_loc_ind)
                        # self.draw_path()
                    except IndexError:
                        print("Index Error line 249!")
                else:
                    # robot reaches its own goal and need to decide for the next step
                    print("the robot has reached the goal!", self.robot.cur_goal)
                    # return the goal and start position to start a new round 
                    self.reset_planner(self.robot.fix_start)
                    self.robot.fix_start = copy.deepcopy(self.robot.cur_loc)
                    self.robot.is_able_help = True
                    self.robot.is_on_task = False
                    # generate new path
                    path = self.robot.find_path()
                    while path is None:
                        path = self.robot.find_path()
                    next_loc_ind = 1
                    # draw the new path
                    # self.draw_path()
                    # Can add the measurement function here

            self.work_time -= 1
            #time.sleep(1)
    
    def reset_planner(self, next_goal):
        '''
        This function resets the planner of the robot 
        :param next_goal: The goal of the new plannner [x, y]
        '''
        self.robot.planner.start = Node(self.robot.cur_loc[0], self.robot.cur_loc[1])
        self.robot.planner.cur_goal = Node(next_goal[0], next_goal[1])
        self.robot.planner.reset()
        self.robot.cur_goal = [self.robot.planner.cur_goal.x, self.robot.planner.cur_goal.y]
        self.robot.start = self.robot.cur_loc

    def robot_decide_help(self):
        '''
        The robot decides whether to help the human or not based on relationship between human's position and robot's goal position. 
        If helping human is on its way, the robot will help. Otherwise, it refuses to help. 
        This prevent the situation that the robot needs to turn back to help human, which decreases the efficiency of the whole system. 
        This function can also be changed to other function which measures the whole reward of helping people. 
        '''
        vec_robot_to_goal = np.array([self.robot.cur_goal[1] - self.robot.cur_loc[1], self.robot.cur_goal[0] - self.robot.cur_loc[0]])
        normalizer = np.linalg.norm(vec_robot_to_goal)
        unit_vec_robot_to_goal = vec_robot_to_goal / normalizer

        vec_robot_to_human = np.array([self.human.cur_loc[1] - self.robot.cur_loc[1], self.human.cur_loc[0] - self.robot.cur_loc[0]])
        normalizer = np.linalg.norm(vec_robot_to_human)
        unit_vec_robot_to_human = vec_robot_to_human / normalizer

        cos_theta = np.dot(unit_vec_robot_to_goal, unit_vec_robot_to_human)

        if cos_theta <= 0:
            # if the robot has passed human, refused to help
            return False
        else:
            return True

    def draw_path(self):
        """
        :param path: list of waypoints location
        """
        #print("begin to draw path!")    
        plt.gcf().clf()
        ax = plt.gca()
        ax.set_xlim((self.robot.planner.min_rand, self.robot.planner.max_rand))
        ax.set_ylim((self.robot.planner.min_rand, self.robot.planner.max_rand))
        # plot path        
        ax.plot([x for (x, y) in self.robot.planner.path],
                [y for (x, y) in self.robot.planner.path], "-k")
        ax.plot([x for (x, y) in self.robot.planner.path],
                [y for (x, y) in self.robot.planner.path], "yo")

        for (x, y, radius) in self.robot.planner.obstacle:
            circle = plt.Circle((x, y), radius, color='y')
            ax.add_artist(circle)
        # plot goal list
        ax.plot([x for (x, y) in self.robot.location_list],
                [y for (x, y) in self.robot.location_list], 'g*')
        # plot current start and goal, robot position
        ax.plot(self.robot.planner.start.x, self.robot.planner.start.y, 'ro')
        ax.plot(self.robot.cur_goal[0], self.robot.cur_goal[1], 'bo')
        # plot human position
        ax.plot(self.human.cur_loc[0], self.human.cur_loc[1], 'b*')
        ax.plot(self.robot.cur_loc[0], self.robot.cur_loc[1], 'r*')
        plt.draw()
        plt.pause(0.01)


def main():
    print("Start evaluation")
    # ===Search Path with flexible rrt===
    # initialize
    '''
    cur_goal = [14, 14]
    location_list = [[1, 4], [4, 1], [5, 10], [10, 5], [2, 14], [14, 2], [14, 14]]

    # trans_prob 2d array [from, to]
    human_goal_model = np.array([[1, -1, 1, 0, 1, 0,
                                  1], [0, 1, -1, 1, 0, 1, 1],
                                 [1, 0, 1, 0, 1, -1,
                                  1], [0, 1, -1, 1, 0, 1, 1],
                                 [1, 0, 1, -1, 1, 0,
                                  1], [0, 1, 0, 1, -1, 1, 1],
                                 [1, 0, -1, 0, 1, 1, 1]])

    # transition matrix for Markov Chain model. 
    # (from_location_index, to_location_index) each row represents the probability that 
    # the human's choose another location from the current locations
    trans_prob = np.array([[0.2, 0,   0.2, 0.1, 0.2, 0.1, 0.2],
                           [0.1, 0.2, 0,   0.2, 0.1, 0.2, 0.2],
                           [0.2, 0.1, 0.2, 0.1, 0.2, 0,   0.2],
                           [0.1, 0.2, 0,   0.2, 0.1, 0.2, 0.2],
                           [0.2, 0.1, 0.2, 0,   0.2, 0.1, 0.2],
                           [0.1, 0.2, 0.1, 0.2, 0,   0.2, 0.2],
                           [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.4]])
                           
    obstacle = [(8, 8, 1), (6, 6, 1), (12, 12, 1), (2,3,1)]
    '''
    '''
    try to use random layout generation
    '''
    x_range = [0, 30]
    y_range = [0, 30]
    num_locations = 10
    num_obstacles = 10
    simu_time = 400
    num_simu = 2
    eval_variable_frrt = []
    eval_variable_rrtstar = []
    ave_path_dist1 = []
    ave_path_dist2 = []

    for i in range(num_simu):

        # generate the layouts
        obstacles = generation.generate_obstacles(x_range, y_range, num_obstacles, [1,3])

        location_list = generation.generate_location(x_range, y_range, num_locations, obstacles)
        # generation.draw_layout(x_range, y_range, obstacles, location_list)
        P = generation.markov_transition_matrix(num_locations)

        cur_start = location_list[0]
        # cur_goal = location_list[np.random.randint(1, num_locations)]
        cur_goal = location_list[-1]
        # flexible RRT
        frrt = fRRT(
            start=cur_start,
            cur_goal=cur_goal,
            goal_list=location_list,
            obstacle=obstacles,
            TreeArea=x_range,
            trans_prob=P)

        robot1 = Robot(
            start=cur_start,
            location_list=location_list,
            trans_prob=P,
            planner=frrt)

        human1 = Human(
            location_list=location_list,
            cur_loc= location_list[5],
            trans_prob = P,
            help_prob=0.2)
        
        exp1 = Experiment(robot=robot1, human=human1, work_time=simu_time)
        exp1.work()
        print('exp1 help count is:', exp1.human_help_cnt)

        # rrt star

        rrt_star_planner = rrt_star(
            start= cur_start, 
            goal=cur_goal, 
            obstacleList=obstacles, 
            randArea=x_range)

        robot2 = Robot(
            start=cur_start,
            location_list=location_list,
            trans_prob=P,
            planner=rrt_star_planner)

        exp2 = Experiment(robot=robot2, human=human1, work_time=simu_time)
        exp2.work()

        # print('exp2 help count is:', exp1.human_help_cnt)

        # the evaluation variable 
        eval_variable_frrt.append(sum(exp1.eval_var) / len(exp1.eval_var))
        eval_variable_rrtstar.append(sum(exp2.eval_var) / len(exp2.eval_var))

        # print('the evaluation for frrt is:', eval_variable_frrt)
        # print('the evaluation for rrt star is:', eval_variable_rrtstar)
        
        ave_path_dist1.append(sum(exp1.path_distance) / len(exp1.path_distance)) 
        ave_path_dist2.append(sum(exp2.path_distance) / len(exp2.path_distance))
        
    # print("average path distance for fRRT:", ave_path_dist1)
    # print("average path distance for rrt star:", ave_path_dist2)
    with open('eval_variable_frrt.pkl', 'wb') as f:
        pickle.dump(eval_variable_frrt, f)
    with open('eval_variable_rrtstart.pkl', 'wb') as f:
        pickle.dump(eval_variable_rrtstar, f)
    with open('ave_path_dist_frrt.pkl', 'wb') as f:
        pickle.dump(ave_path_dist1, f)
    with open('ave_path_dist_rrtstar.pkl', 'wb') as f:
        pickle.dump(ave_path_dist2, f)

    
if __name__ == '__main__':
    main()