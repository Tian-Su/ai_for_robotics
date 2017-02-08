# print demo_grading(hunter, target, naive_next_move)

import sys

sys.path.append('/Users/TianSu/Dropbox/Study/GT/ai_for_robotics/project/project1/part3')
import part3 as p

import numpy as np
import timeit
import random
from robot import *


def curr_time_millis():
    return 1000 * timeit.default_timer()


# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
failed_ctr = 0
steps_list = []
time_step = []
N = 100
graph = False
for trial in range(N):
    xt = random.uniform(-20, 20)
    yt = random.uniform(-20, 20)
    xh = random.uniform(-20, 20)
    yh = random.uniform(-20, 20)
    orit = random.uniform(-pi, pi)
    orih = random.uniform(-pi, pi)
    turn = random.uniform(-pi, pi)
    dist = random.random() * 5.0
    # def __init__(self, x = 0.0, y = 0.0, heading = 0.0, turning = 2*pi/10, distance = 1.0):
    # target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
    target = robot(xt, yt, orit, turn, dist)
    measurement_noise = .05 * target.distance
    target.set_noise(0.0, 0.0, measurement_noise)
    # hunter = robot(-10.0, -10.0, 0.0)
    hunter = robot(xh, yh, orih)
    current_time = curr_time_millis()
    if graph:
        res, ctr = p.demo_grading_graph(hunter, target, p.next_move)
    else:
        res, ctr = p.demo_grading(hunter, target, p.next_move)
    time_step.append(curr_time_millis() - current_time)
    if res == False:
        failed_ctr += 1
    else:
        steps_list.append(ctr)

print "Failed attempts ==> ", failed_ctr, "out of {} trials".format(N)
print "Average time taken (ms)", np.mean(time_step)
print "Minimum time taken (ms)", np.min(time_step)
print "Maximum time taken (ms)", np.max(time_step)
if len(steps_list) > 0:
    print "Average steps taken ", np.mean(steps_list)
    print "Mininum steps taken ", np.min(steps_list)
    print "Maximum steps taken ", np.max(steps_list)
