# ----------
# Part Two
#
# Now we'll make the scenario a bit more realistic. Now Traxbot's
# sensor measurements are a bit noisy (though its motions are still
# completetly noise-free and it still moves in an almost-circle).
# You'll have to write a function that takes as input the next
# noisy (x, y) sensor measurement and outputs the best guess
# for the robot's next position.
#
# ----------
# YOUR JOB
#
# Complete the function estimate_next_pos. You will be considered
# correct if your estimate is within 0.01 stepsizes of Traxbot's next
# true position.
#
# ----------
# GRADING
#
# We will make repeated calls to your estimate_next_pos function. After
# each call, we will compare your estimated position to the robot's true
# position. As soon as you are within 0.01 stepsizes of the true position,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.

# These import steps give you access to libraries which you may (or may
# not) want to use.
from robot import *
from matrix import *
import numpy as np
import math


# This is the function you have to write. Note that measurement is a
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.
def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """
    Returns the angle in radians between vectors 'v1' and 'v2'::
    >>> angle_between((1,0), (0,1))
    1.5707963267948966

    Parameters
    ----------
    v1
    v2

    Returns
    -------

    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def cal_vector(point1, point2):
    """
    >>> p1.cal_vector((1,0), (1,1))
    (0, 1)
    Parameters
    ----------
    point1
    point2

    Returns
    -------

    """
    return ((point2[0] - point1[0]), (point2[1] - point1[1]))


def estimate_next_pos(measurement, OTHER=None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    if not OTHER:
        OTHER = {'msmt': [measurement], 'length': None, 'angle': None}
        xy_estimate = OTHER['msmt'][-1]
    elif len(OTHER['msmt']) == 1:
        OTHER['msmt'].append(measurement)
        OTHER['length'] = distance_between(OTHER['msmt'][-2], OTHER['msmt'][-1])
        xy_estimate = OTHER['msmt'][-1]
    else:
        OTHER['msmt'].append(measurement)
        # update length
        # learning rate 0.1
        r = 0.02
        new_length = distance_between(OTHER['msmt'][-2], OTHER['msmt'][-1])
        OTHER['length'] = (1 - r) * OTHER['length'] + r * new_length
        # calculate angle
        v1 = cal_vector(OTHER['msmt'][-3], OTHER['msmt'][-2])
        v2 = cal_vector(OTHER['msmt'][-2], OTHER['msmt'][-1])
        angle = angle_between(v1, v2)
        if OTHER['angle']:
            OTHER['angle'] = (1 - r) * OTHER['angle'] + r * angle
        else:
            OTHER['angle'] = angle
        # v3 = np.dot(np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]), np.array(v2))
        x = OTHER['msmt'][-1][0]
        y = OTHER['msmt'][-1][1]
        heading = math.atan2(v2[1], v2[0])  # math.atan2(y, x)
        test_target = robot(x, y, heading)
        test_target.set_noise(0.0, 0.0, 0.0)
        test_target.move(OTHER['angle'], OTHER['length'])
        xy_estimate = (test_target.x, test_target.y)
    return xy_estimate, OTHER


# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


# This is here to give you a sense for how we will be running and grading
# your code. Note that the OTHER variable allows you to store any
# information that you want.
def demo_grading(estimate_next_pos_fcn, target_bot, OTHER=None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 1000:
            print "Sorry, it took you too many steps to localize the target."
    return localized, ctr


# This is a demo for what a strategy could look like. This one isn't very good.
def naive_next_pos(measurement, OTHER=None):
    """This strategy records the first reported position of the target and
    assumes that eventually the target bot will eventually return to that
    position, so it always guesses that the first position will be the next."""
    if not OTHER:  # this is the first measurement
        OTHER = measurement
    xy_estimate = OTHER
    return xy_estimate, OTHER


# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.1, 4.3, 0.5, 2 * pi / 34.0, 1.5)
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)

# demo_grading(naive_next_pos, test_target)
