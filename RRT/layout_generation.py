#-*- coding: utf-8 -*-
"""
This class generates random layouts for flexible RRT evaluation
@author: Yifan Tang
"""
import numpy as np
import matplotlib.pyplot as plt

def generate_obstacles(x_range, y_range, num_obstacles, radius_range):
    """
    This function generates obstacles in the area. Each obstacle's 
    radius is in the radius range.
    """
    x_min = x_range[0]
    x_max = x_range[1]
    y_min = y_range[0]
    y_max = y_range[1]
    radius_min = radius_range[0]
    radius_max = radius_range[1]

    obstacles = []
    while len(obstacles) <= num_obstacles:
        x = np.random.uniform(x_min, x_max) 
        y = np.random.uniform(y_min, y_max)
        radius = np.random.uniform(radius_min, radius_max)
        obstacles.append((x, y, radius))       
    return obstacles

def draw_layout(x_range, y_range, obstacles, locations_list):
    plt.figure(1)
    ax = plt.gca()
    plt.ion()
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('Layouts')
    ax.set_xlim(x_range[0], x_range[1])
    ax.set_ylim(y_range[0], y_range[1])
    for (x, y, radius) in obstacles:
        circle = plt.Circle((x, y), radius, color='y')
        ax.add_artist(circle)
    ax.plot([x for (x,_) in locations_list], [y for (_,y) in locations_list], 'g*')
    plt.show()
    # plt.draw()
    plt.pause(1)

def markov_transition_matrix(num):
    """
    This function generates the Markov transition matrix 
    """
    P = np.random.rand(num, num)
    matrix = P / P.sum(axis=1)[:,None]
    return matrix

def generate_location(x_range, y_range, num_locations, obstacles):
    """
    This function generates the locations in the warehouse. 
    """
    x_min = x_range[0]
    x_max = x_range[1]
    y_min = y_range[0]
    y_max = y_range[1]
    safe_dist = min((x_max - x_min)/60, (y_max - y_min)/60)
    locations_list = []
    while len(locations_list) < num_locations:
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        is_collision = False
        # check if location (x, y) is in obstacle
        for i in range(len(obstacles)):
            dist = np.linalg.norm([x-obstacles[i][0], y-obstacles[i][1]])
            if dist <= obstacles[i][2] + safe_dist:
                is_collision = True
                break
        if not is_collision:
            locations_list.append([x, y])

    return locations_list

if __name__ == "__main__":
    obstacles = generate_obstacles([0, 30], [0, 30], 15, [1, 3])
    locations_list = generate_location([1, 30], [1, 30], 8, obstacles)
    draw_layout([0, 30], [0, 30], obstacles, locations_list)
    P = markov_transition_matrix(len(locations_list))
 

    # You probably won't need this if you're embedding things in a tkinter plot...
    # plt.ion()
#    x = np.linspace(0, 6*np.pi, 100)
#     y = np.sin(x)   
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # line1, = ax.plot(x, y, 'r-') # Returns a tuple of line objects, thus the comma

    # for phase in np.linspace(0, 10*np.pi, 500):
    #     line1.set_ydata(np.sin(x + phase))
    #     fig.canvas.draw()
    #     fig.canvas.flush_events()