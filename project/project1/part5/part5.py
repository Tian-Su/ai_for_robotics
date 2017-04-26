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

sys.path.append(
    '/Users/tiansu/Documents/git/ai_for_robotics/project/project1/part4')
from robot import *
from matrix import *
import numpy as np
import math

landmarks = [[20.0, 20.0], [20.0, -20.0], [-20.0, -20.0], [-20.0, 20.0]]


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


def determinant(v, w):
    return v[0] * w[1] - v[1] * w[0]


def estimate_next_pos(measurement, OTHER=None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""
    PF_N = 1000
    if not OTHER:
        OTHER = {'msmt': [measurement], 'length': None, 'angle': None,
                 'counter': 1}
        xy_estimate = OTHER['msmt'][-1]
    elif len(OTHER['msmt']) == 1:
        OTHER['msmt'].append(measurement)
        OTHER['length'] = distance_between(OTHER['msmt'][-2], OTHER['msmt'][-1])
        OTHER['counter'] += 1
        xy_estimate = OTHER['msmt'][-1]
    elif OTHER['counter'] < 10:
        OTHER['msmt'].append(measurement)
        OTHER['counter'] += 1
        # update length
        # average based on counter
        # new_length = distance_between(OTHER['msmt'][-2], OTHER['msmt'][-1])
        # OTHER['length'] = ((OTHER['counter'] - 1) * OTHER[
        #     'length'] + new_length) / OTHER['counter']
        new_length = distance_between(OTHER['msmt'][-2], OTHER['msmt'][-1])
        OTHER['length'] = ((OTHER['counter'] - 1) * OTHER[
            'length'] + new_length) / OTHER['counter']
        # calculate angle
        v1 = cal_vector(OTHER['msmt'][-3], OTHER['msmt'][-2])
        v2 = cal_vector(OTHER['msmt'][-2], OTHER['msmt'][-1])
        angle = angle_between(v1, v2)
        if determinant(v1, v2) < 0:
            angle = -angle
        # if OTHER['angle']:
        #     OTHER['angle'] = ((OTHER['counter'] - 1) * OTHER['angle'] + angle) / \
        #                      OTHER['counter']
        # else:
        #     OTHER['angle'] = angle
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
        test_target.move(OTHER['angle'], OTHER['length'])
        xy_estimate.append((test_target.x, test_target.y))
        if OTHER['counter'] == 10:
            p = []
            for i in range(PF_N):
                x = PF_robot(measurement)
                # TODO: can change the noise
                x.set_noise(OTHER['length'] / 20, OTHER['angle'] / 20,
                            OTHER['length'])
                p.append(x)
            OTHER['PF'] = p
    else:
        OTHER['counter'] += 1
        print 'counter'
        print OTHER['counter']
        # print 'other'
        # print len(OTHER['msmt'])
        # print measurement

        w = []
        for i in range(PF_N):
            w.append(OTHER['PF'][i].measurement_prob(measurement))

        OTHER['PF'] = \
            np.random.choice(OTHER['PF'], size=(1, 1, PF_N), replace=True,
                             p=np.array(w) / np.sum(w)).tolist()[0][0]
        print 'np.sum(w)'
        print np.sum(w)

        OTHER['msmt'].append(measurement)
        print "measurement"
        print measurement
        print 'get_position'
        print get_position(OTHER['PF'])
        # update length
        # average based on counter
        r = 0.1
        new_length = distance_between(OTHER['msmt'][-2], OTHER['msmt'][-1])
        OTHER['length'] = (1 - r) * OTHER['length'] + r * new_length
        # calculate angle
        v1 = cal_vector(OTHER['msmt'][-3], OTHER['msmt'][-2])
        v2 = cal_vector(OTHER['msmt'][-2], OTHER['msmt'][-1])
        angle = angle_between(v1, v2)
        if determinant(v1, v2) < 0:
            angle = -angle
        if OTHER['angle']:
            OTHER['angle'] = (1 - r) * OTHER['angle'] + r * angle
        else:
            OTHER['angle'] = angle
        # v3 = np.dot(np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]), np.array(v2))
        # x = OTHER['msmt'][-1][0]
        # y = OTHER['msmt'][-1][1]
        x = get_position(OTHER['PF'])[0]
        y = get_position(OTHER['PF'])[1]
        heading = math.atan2(v2[1], v2[0])  # math.atan2(y, x)
        test_target = robot(x, y, heading)
        test_target.set_noise(0.0, 0.0, 0.0)
        test_target.move(OTHER['angle'], OTHER['length'])
        # xy_estimate = (test_target.x, test_target.y)
        xy_estimate = [(test_target.x, test_target.y)]

        for i in range(PF_N):
            OTHER['PF'][i].move(angle, new_length)

        test_target.move(OTHER['angle'], OTHER['length'])
        xy_estimate.append((test_target.x, test_target.y))
        test_target.move(OTHER['angle'], OTHER['length'])
        xy_estimate.append((test_target.x, test_target.y))

    return xy_estimate, OTHER


def cal_angle_distance(hunter_position, a, hunter_heading):
    v = cal_vector(hunter_position, a)
    angle = atan2(v[1], v[0])
    turning = angle - hunter_heading
    distance = distance_between(hunter_position, a)
    return turning, distance


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
    turning, distance = cal_angle_distance(hunter_position, a, hunter_heading)
    if (distance > max_distance) and (OTHER['counter'] >= 3):
        a = xy_estimate[2]
        turning, distance = cal_angle_distance(hunter_position, a,
                                               hunter_heading)

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
    return caught, ctr


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


def demo_grading_graph(hunter_bot, target_bot, next_move_fcn, OTHER=None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance  # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance  # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0
    # For Visualization
    import turtle
    window = turtle.Screen()
    window.bgcolor('white')
    chaser_robot = turtle.Turtle()
    chaser_robot.shape('arrow')
    chaser_robot.color('blue')
    chaser_robot.resizemode('user')
    chaser_robot.shapesize(0.3, 0.3, 0.3)
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.3, 0.3, 0.3)
    size_multiplier = 10.0  # change size of animation
    chaser_robot.hideturtle()
    chaser_robot.penup()
    chaser_robot.goto(hunter_bot.x * size_multiplier,
                      hunter_bot.y * size_multiplier - 100)
    chaser_robot.showturtle()
    broken_robot.hideturtle()
    broken_robot.penup()
    broken_robot.goto(target_bot.x * size_multiplier,
                      target_bot.y * size_multiplier - 100)
    broken_robot.showturtle()
    measuredbroken_robot = turtle.Turtle()
    measuredbroken_robot.shape('circle')
    measuredbroken_robot.color('red')
    measuredbroken_robot.penup()
    measuredbroken_robot.resizemode('user')
    measuredbroken_robot.shapesize(0.1, 0.1, 0.1)
    broken_robot.pendown()
    chaser_robot.pendown()
    # End of Visualization
    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 100:
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
        # Visualize it
        measuredbroken_robot.setheading(target_bot.heading * 180 / pi)
        measuredbroken_robot.goto(target_measurement[0] * size_multiplier,
                                  target_measurement[1] * size_multiplier - 100)
        measuredbroken_robot.stamp()
        broken_robot.setheading(target_bot.heading * 180 / pi)
        broken_robot.goto(target_bot.x * size_multiplier,
                          target_bot.y * size_multiplier - 100)
        chaser_robot.setheading(hunter_bot.heading * 180 / pi)
        chaser_robot.goto(hunter_bot.x * size_multiplier,
                          hunter_bot.y * size_multiplier - 100)
        # End of visualization
        ctr += 1
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return caught, ctr


class PF_robot:
    def __init__(self, loc):
        self.x = loc[0] + random.uniform(-20, 20) / 100.
        self.y = loc[1] + random.uniform(-20, 20) / 100.
        self.orientation = random.uniform((10 * pi) / 180, (50 * pi) / 180)
        self.forward_noise = 0.0;
        self.turn_noise = 0.0;
        self.sense_noise = 0.0;

    def set(self, new_x, new_y, new_orientation):
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise);
        self.turn_noise = float(new_t_noise);
        self.sense_noise = float(new_s_noise);

    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (
                self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z

    def move(self, turning, distance, tolerance=0.001, max_turning_angle=pi):
        # apply noise, this doesn't change anything if turning_noise
        # and distance_noise are zero.
        turning = random.gauss(turning, self.turn_noise)
        distance = random.gauss(distance, self.forward_noise)

        # truncate to fit physical limitations
        turning = max(-max_turning_angle, turning)
        turning = min(max_turning_angle, turning)
        distance = max(0.0, distance)

        # Execute motion
        self.orientation += turning
        self.orientation = angle_trunc(self.orientation)
        self.x += distance * cos(self.orientation)
        self.y += distance * sin(self.orientation)

    def Gaussian(self, mu, sigma, x):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(
            2.0 * pi * (sigma ** 2))

    def measurement_prob(self, measurement):  # measurement
        mse = self.mse([self.x, self.y], measurement)
        # calculates how likely a measurement should be
        #
        # prob = 1.0;
        # for i in range(len(landmarks)):
        #     dist = sqrt((self.x - landmarks[i][0]) ** 2 + (
        #         self.y - landmarks[i][1]) ** 2)
        #     prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
        prob = np.exp(-np.array(mse/10000) / (2 * self.sense_noise ** 2))
        return prob

    def mse(self, point1, point2):
        a = np.array(point1)
        b = np.array(point2)
        return np.sum((a - b) ** 2)

    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (
            str(self.x), str(self.y), str(self.orientation))


def sense_loc(measurement):
    Z = []
    for i in range(len(landmarks)):
        dist = sqrt((measurement[0] - landmarks[i][0]) ** 2 + (
            measurement[1] - landmarks[i][1]) ** 2)
        Z.append(dist)
    return Z


def get_position(p):
    x = 0.0
    y = 0.0
    orientation = 0.0
    for i in range(len(p)):
        x += p[i].x
        y += p[i].y
        # orientation is tricky because it is cyclic. By normalizing
        # around the first particle we are somewhat more robust to
        # the 0=2pi problem
        orientation += (
            ((p[i].orientation - p[0].orientation + pi) % (2.0 * pi))
            + p[0].orientation - pi)
    return [x / len(p), y / len(p)]


target = robot(0.0, 10.0, 0.0, 2 * pi / 30, 1.5)
measurement_noise = .05 * target.distance
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

print demo_grading_graph(hunter, target, next_move)
