# ----------
# Part Four
#
# Again, you'll track down and recover the runaway Traxbot.
# But this time, your speed will be about the same as the runaway bot.
# This may require more careful planning than you used last time.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time.
#
# ----------
# GRADING
#
# Same as part 3. Again, try to catch the target in as few steps as possible.
import sys
sys.path.append('/Users/tiansu/Documents/git/ai_for_robotics/project/project1/part3')
from robot import *
from matrix import *
import numpy as np
import math


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

def determinant(v,w):
   return v[0]*w[1]-v[1]*w[0]

def estimate_next_pos(measurement, OTHER=None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    if not OTHER:
        OTHER = {'msmt': [measurement], 'length': None, 'angle': None,
                 'counter': 1}
        xy_estimate = OTHER['msmt'][-1]
    elif len(OTHER['msmt']) == 1:
        OTHER['msmt'].append(measurement)
        OTHER['length'] = distance_between(OTHER['msmt'][-2], OTHER['msmt'][-1])
        OTHER['counter'] += 1
        xy_estimate = OTHER['msmt'][-1]
    else:
        OTHER['msmt'].append(measurement)
        OTHER['counter'] += 1
        # update length
        # average based on counter
        new_length = distance_between(OTHER['msmt'][-2], OTHER['msmt'][-1])
        OTHER['length'] = ((OTHER['counter'] - 1) * OTHER[
            'length'] + new_length) / OTHER['counter']
        # calculate angle
        v1 = cal_vector(OTHER['msmt'][-3], OTHER['msmt'][-2])
        v2 = cal_vector(OTHER['msmt'][-2], OTHER['msmt'][-1])
        angle = angle_between(v1, v2)
        if determinant(v1,v2) < 0:
            angle = -angle
        if OTHER['angle']:
            OTHER['angle'] = ((OTHER['counter'] - 1) * OTHER['angle'] + angle) / OTHER['counter']
        else:
            OTHER['angle'] = angle
        # v3 = np.dot(np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]), np.array(v2))
        x = OTHER['msmt'][-1][0]
        y = OTHER['msmt'][-1][1]
        heading = math.atan2(v2[1], v2[0])  # math.atan2(y, x)
        test_target = robot(x, y, heading)
        test_target.set_noise(0.0, 0.0, 0.0)
        test_target.move(OTHER['angle'], OTHER['length'])
        # xy_estimate = (test_target.x, test_target.y)
        xy_estimate = [(test_target.x, test_target.y)]
        test_target.move(OTHER['angle'], OTHER['length'])
        xy_estimate.append((test_target.x, test_target.y))
    return xy_estimate, OTHER


def next_move(hunter_position, hunter_heading, target_measurement, max_distance,
              OTHER=None):
    # This function will be called after each time the target moves.

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    xy_estimate, OTHER = estimate_next_pos(target_measurement, OTHER)
    if OTHER['counter'] < 3:
        a = xy_estimate
    else:
        a = xy_estimate[0]
    v = cal_vector(hunter_position, a)
    angle = atan2(v[1], v[0])
    turning = angle - hunter_heading
    distance = distance_between(hunter_position, a)
    return turning, distance, OTHER



def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER=None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance  # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance  # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position,
                                                 hunter_bot.heading,
                                                 target_measurement,
                                                 max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        ctr += 1
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return caught


def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi


def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading


def naive_next_move(hunter_position, hunter_heading, target_measurement,
                    max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all
    the target measurements, hunter positions, and hunter headings over time, but it doesn't
    do anything with that information."""
    if not OTHER:  # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions,
                 hunter_headings)  # now I can keep track of history
    else:  # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER  # now I can always refer to these variables

    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning = heading_difference  # turn towards the target
    distance = max_distance  # full speed ahead!
    return turning, distance, OTHER

# target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
# measurement_noise = .05*target.distance
# target.set_noise(0.0, 0.0, measurement_noise)

# hunter = robot(-10.0, -10.0, 0.0)

# print demo_grading(hunter, target, naive_next_move)





