#from project import project1 as p1
import sys
sys.path.append('/Users/TianSu/Dropbox/Study/GT/ai_for_robotics/project/project1/part2')
import part2 as p
#reload(p1)
import numpy as np
import random
from robot import *
# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
failed_ctr = 0
steps_list = []
n = 1000
for trial in range(n):
    x = random.uniform(-20, 20)
    y = random.uniform(-20, 20)
    turn = random.uniform((10 * pi) / 180, (50 * pi) / 180)
    test = random.uniform(-1, 1)
    if test < 0:
        turn *= -1
    dist = random.uniform(1., 5.)
    ori = random.random()
    test_target = robot(x, y, ori, turn, dist)
    #test_target = robot(0., 0., 0., 2*pi / 30.0, 2.5)
    measurement_noise = 0.05 * test_target.distance
    test_target.set_noise(0.0, 0.0, measurement_noise)
    res, ctr = p.demo_grading(p.estimate_next_pos, test_target)
    if res == False:
        failed_ctr += 1
    else:
        steps_list.append(ctr)

print "Failed attempts ==> ", failed_ctr, "out of {0} trials".format(n)
print "Average steps taken ", np.mean(steps_list)
print "Mininum steps taken ", np.min(steps_list)
print "Maximum steps taken ", np.max(steps_list)