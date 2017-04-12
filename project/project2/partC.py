#
# === Introduction ===
#
# In this problem, you will again build a planner that helps a robot
#   find the best path through a warehouse filled with boxes
#   that it has to pick up and deliver to a dropzone. Unlike Part B,
#   however, in this problem the robot's movement is subject to error.
#   Moreover, the robot will not have access to a plan of the warehouse
#   beforehand, and the robot's perception of its surrounding will also
#   be subject to error.
#
# Your file must be called `partC.py` and must have a class called
#   `OnlineDeliveryPlanner`.
# This class must have an `__init__` function that takes four arguments:
#   `self`, `todo`, `max_distance`, and `max_steering`.
# The class must also have a function called `process_measurement` that takes
#   two arguments, `self` and `data`.
# The class must also have a function called `next_move` that takes just a
#   single argument, `self`.
#
# === Input Specifications ===
#
# The specifications of the warehouse are as in Part B, but the warehouse layout
#   will not be available at the beginning of the task. Instead, the robot will
#   perceive its surroundings by being given data through the `receive_measurement`
#   function.
#
# In this part of the project, the order in which the boxes are delivered does not
#   matter. `todo` will simply be an integer >= 1 giving the total number of boxes
#   that must be delivered.
#
# === Rules for Measurement ===
#
# Each turn, the `process_measurement` method will be called before `next_move`.
#   You should use the measurement data to update your robot's belief of its
#   current position before planning your next move.
#
# The data will consist of tuples of distances and relative bearings to markers
#   within line-of-sight of the robot's center point. Markers will be present on
#   walls and boxes. An example of measurement data may be:
#   ('wall', 1, 2.348, 0.913)
#   which means that the robot's center point is distance 2.348 away from wall marker 1,
#   and the bearing from the robot to the marker is 0.913 relative to the robot's current
#   bearing (so a relative bearing of 0 means the robot would reach the marker by moving
#   in a straight line at its current bearing)
#
# The distance and relative bearing measurements will be subject to Gaussian noise
#   with mean 0 and with variance increasing as the distance between the robot and the
#   marker increases.
#
# The robot will also receive a boolean that is True if it's currently holding a box and False
#   if not. This boolean will always be correct.
#
# === Rules for Movement ===
#
# - The robot may move any distance between 0 and `max_distance` per time step.
# - The robot may set its steering angle anywhere between -`max_steering` and
#   `max_steering` per time step. A steering angle of 0 means that the robot will
#   move according to its current bearing. A positive angle means the robot will
#   turn counterclockwise by `steering_angle` radians; a negative steering_angle
#   means the robot will turn clockwise by abs(steering_angle) radians.
# - Upon a movement, the robot will change its steering angle instantaneously to the
#   amount indicated by the move, and then it will move a distance in a straight line in its
#   new bearing according to the amount indicated move.
#   Movements will be  subject to small amounts of noise in the distance (0.9 to 1.1 times the desired
#   distance, never exceeding max_distance) and steering (+/- 0.1 of the desired steering,
#   never violating the constraint that abs(steering) <= max_steering).
# - The cost per turn is 1 plus the amount of distance traversed by the robot on that turn.
#
# - The robot may pick up a box whose center point is within 0.5 units of the robot's center point.
# - If the robot picks up a box, it incurs a total cost of 2 for that turn (this already includes
#   the 1-per-turn cost incurred by the robot).
# - While holding a box, the robot may not pick up another box.
# - The robot may attempt to put a box down at a total cost of 1.5 for that turn.
#   The box must be placed so that:
#   - The box is not contacting any walls, the exterior of the warehouse, any other boxes, or the robot
#   - The box's center point is within 0.5 units of the robot's center point
# - A box is always oriented so that two of its edges are horizontal and the other two are vertical.
# - If a box is placed entirely within the '@' space, it is considered delivered and is removed from the
#   warehouse.
# - The warehouse will be arranged so that it is always possible for the robot to move to the
#   next box on the todo list without having to rearrange any other boxes.
#
# - If the robot crashes, it will stop moving and incur a cost of 100*distance, where distance
#   is the length it attempted to move that turn. (The regular movement cost will not apply.)
# - If an illegal move is attempted, the robot will not move, but the standard cost will be incurred.
#   Illegal moves include (but are not necessarily limited to):
#     - picking up a box that doesn't exist or is too far away
#     - picking up a box while already holding one
#     - putting down a box too far away or so that it's touching a wall, the warehouse exterior,
#       another box, or the robot
#     - putting down a box while not holding a box
#
# === Output Specifications ===
#
# `next_move` should return a string in one of the following formats:
#
# 'move {steering} {distance}', where '{steering}' is a floating-point number between
#   -`max_steering` and `max_steering` (inclusive) and '{distance}' is a floating-point
#   number between 0 and `max_distance`
#
# 'lift {bearing}', where '{bearing}' is replaced by the bearing of the box's center point
#   relative to the robot (with 0 meaning right in front of the robot).
#   If a box's center point is within 0.5 units of the robot's center point and its bearing
#   is within 0.2 radians of the specified bearing, then the box is lifted. Otherwise, nothing
#   happens.
#
# 'down {bearing}', where '{bearing}' is replaced by the relative bearing where the
#   robot wishes to place its box. The robot will attempt to place the box so that its center point
#   is at this bearing at a distance of 0.5 units away. If the box can be so placed without intersecting
#   anything, it does so. Otherwise, the placement fails and the robot continues to hold the box.
#
# === Grading ===
#
# - Your planner will be graded against a set of test cases, each equally weighted.
# - If your planner successfully delivers K percent of the boxes in a test case, you will receive
#   K percent of the credit for that test case.
# - Otherwise, you will receive no credit for that test case. This could happen for one of several
#   reasons including (but not necessarily limited to):
#   - plan_delivery's moves do not deliver the boxes in the correct order.
#   - plan_delivery's output is not a list of strings in the prescribed format.
#   - plan_delivery does not return an output within the prescribed time limit.
#   - Your code raises an exception.
#
# === Additional Info ===
#
# - You may add additional classes and functions as needed provided they are all in the file `partB.py`.
# - Upload partC.py to Project 2 on T-Square in the Assignments section. Do not put it into an
#   archive with other files.
# - Ask questions on Piazza if the directions or specifications are unclear. This is the first time
#   this project has ever been given, so help working out the bugs will be greatly appreciated!
#

class OnlineDeliveryPlanner:

	def __init__(self, todo, max_distance, max_steering):
		pass

	def process_measurement(self, data):
		pass

	def next_move(self):
		move = ''
		return move