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

def draw_layout(x_range, y_range, obstacles):
    plt.figure(1)
    ax = plt.gca()
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title('Layouts')
    ax.set_xlim(x_range[0], x_range[1])
    ax.set_ylim(y_range[0], y_range[1])
    for (x, y, radius) in obstacles:
        circle = plt.Circle((x, y), radius, color='y')
        ax.add_artist(circle)
    plt.draw()
    plt.pause(0)

def markov_transition_matrix(row, col):
    P = np.random.rand(row, col)
    matrix = P / P.sum(axis=1)[:,None]
    return matrix

def generate_location():
    pass

if __name__ == "__main__":
    obstacles = generate_obstacles([0, 30], [0, 30], 15, [1, 3])
    draw_layout([0,30], [0,30], obstacles)