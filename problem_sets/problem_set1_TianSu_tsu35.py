# -*- coding: utf-8 -*-
"""
Created on Sun Jan 15 12:00:59 2017

@author: TianSu
"""
# The function localize takes the following arguments:
#
# colors:
#        2D list, each entry either 'R' (for red cell) or 'G' (for green cell)
#
# measurements:
#        list of measurements taken by the robot, each entry either 'R' or 'G'
#
# motions:
#        list of actions taken by the robot, each entry of the form [dy,dx],
#        where dx refers to the change in the x-direction (positive meaning
#        movement to the right) and dy refers to the change in the y-direction
#        (positive meaning movement downward)
#        NOTE: the *first* coordinate is change in y; the *second* coordinate is
#              change in x
#
# sensor_right:
#        float between 0 and 1, giving the probability that any given
#        measurement is correct; the probability that the measurement is
#        incorrect is 1-sensor_right
#
# p_move:
#        float between 0 and 1, giving the probability that any given movement
#        command takes place; the probability that the movement command fails
#        (and the robot remains still) is 1-p_move; the robot will NOT overshoot
#        its destination in this exercise
#
# The function should RETURN (not just show or print) a 2D list (of the same
# dimensions as colors) that gives the probabilities that the robot occupies
# each cell in the world.
#
# Compute the probabilities by assuming the robot initially has a uniform
# probability of being in any cell.
#
# Also assume that at each step, the robot:
# 1) first makes a movement,
# 2) then takes a measurement.
#
# Motion:
#  [0,0] - stay
#  [0,1] - right
#  [0,-1] - left
#  [1,0] - down
#  [-1,0] - up
 
import numpy as np
 
 
def sense(p, colors, measurement, sensor_right):
    hit = colors == measurement
    q = p * (hit * sensor_right + ~hit * (1-sensor_right))
    s = q.sum()
    q = q/s
    return q
 
 
def move(p, colors, move, p_move):
    p_shape = p.shape
    if move == [0, 0]:
        p_change = p
    elif move == [0, 1]:
        p_change = np.concatenate((p[:, -1:], p[:, :-1]), axis=1)
    elif move == [0, -1]:
        p_change = np.concatenate((p[:, 1:], p[:, :1]), axis=1)
    elif move == [1, 0]:
        p_change = np.concatenate((p[-1:, :], p[:-1, :]), axis=0)
    elif move == [-1, 0]:
        p_change = np.concatenate((p[1:, :], p[:1, :]), axis=0)
    else:
        raise Exception("can not calculate")
    q = p_change * p_move + p * (1-p_move)
    return q
 
def localize(colors, measurements, motions, sensor_right, p_move):
    # colors as np array
    colors = np.array(colors)
    # initializes p to a uniform distribution over a grid of the same dimensions as colors
    p = np.ones(colors.shape)
    p = p/p.sum()
 
    # >>> Insert your code here <<<
    for i in range(len(measurements)):
        p = move(p, colors, motions[i], p_move)
        p = sense(p, colors, measurements[i], sensor_right)
    return p
 
 
def show(p):
    rows = ['[' + ','.join(map(lambda x: '{0:.5f}'.format(x), r)) + ']' for r in p]
    print '[' + ',\n '.join(rows) + ']'
    
#############################################################
# For the following test case, your output should be 
# [[0.01105, 0.02464, 0.06799, 0.04472, 0.02465],
#  [0.00715, 0.01017, 0.08696, 0.07988, 0.00935],
#  [0.00739, 0.00894, 0.11272, 0.35350, 0.04065],
#  [0.00910, 0.00715, 0.01434, 0.04313, 0.03642]]
# (within a tolerance of +/- 0.001 for each entry)
 
colors = [['R','G','G','R','R'],
          ['R','R','G','R','R'],
          ['R','R','G','G','R'],
          ['R','R','R','R','R']]
colors = np.array(colors)
measurements = ['G','G','G','G','G']
motions = [[0,0],[0,1],[1,0],[1,0],[0,1]]
p = localize(colors,measurements,motions,sensor_right = 0.7, p_move = 0.8)
show(p) # displays your answer