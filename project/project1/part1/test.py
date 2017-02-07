#from project import project1 as p1
import part1 as p1
#reload(p1)
import numpy as np
import random
from robot import *
# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
failed_ctr = 0
steps_list = []
for trial in range(100):
    x = random.random()*20
    y = random.random()*20
    ori = random.random()
    turn = random.random()*pi
    dist = random.random()*2.0
    test_target = robot(x, y, ori, turn, dist)
    #test_target = robot(0., 0., 0., 2*pi / 30.0, 2.5)
    measurement_noise = 0.00 * test_target.distance
    test_target.set_noise(0.0, 0.0, measurement_noise)
    res, ctr = p1.demo_grading(p1.estimate_next_pos, test_target)
    if res == False:
        failed_ctr += 1
    else:
        steps_list.append(ctr)

print "Failed attempts ==> ", failed_ctr, "out of 100 trials"
print "Average steps taken ", np.mean(steps_list)
print "Mininum steps taken ", np.min(steps_list)
print "Maximum steps taken ", np.max(steps_list)