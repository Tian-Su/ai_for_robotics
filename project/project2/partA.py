#
# === Introduction ===
#
# In this problem, you will build a planner that helps a robot
#   find the best path through a warehouse filled with boxes
#   that it has to pick up and deliver to a dropzone.
#
# Your file must be called `partA.py` and must have a class
#   called `DeliveryPlanner`.
# This class must have an `__init__` function that takes three
#   arguments: `self`, `warehouse`, and `todo`.
# The class must also have a function called `plan_delivery` that
#   takes a single argument, `self`.
#
# === Input Specifications ===
#
# `warehouse` will be a list of m strings, each with n characters,
#   corresponding to the layout of the warehouse. The warehouse is an
#   m x n grid. warehouse[i][j] corresponds to the spot in the ith row
#   and jth column of the warehouse, where the 0th row is the northern
#   end of the warehouse and the 0th column is the western end.
#
# The characters in each string will be one of the following:
#
# '.' (period) : traversable space. The robot may enter from any adjacent space.
# '#' (hash) : a wall. The robot cannot enter this space.
# '@' (dropzone): the starting point for the robot and the space where all boxes must be delivered.
#   The dropzone may be traversed like a '.' space.
# [0-9a-zA-Z] (any alphanumeric character) : a box. At most one of each alphanumeric character
#   will be present in the warehouse (meaning there will be at most 62 boxes). A box may not
#   be traversed, but if the robot is adjacent to the box, the robot can pick up the box.
#   Once the box has been removed, the space functions as a '.' space.
#
# For example,
#   warehouse = ['1#2',
#                '.#.',
#                '..@']
#   is a 3x3 warehouse. The dropzone is at space (2,2), box '1' is located at space (0,0),
#   box '2' is located at space (0,2), and there are walls at spaces (0,1) and (1,1). The
#   rest of the warehouse is empty space.
#
# The argument `todo` is a list of alphanumeric characters giving the order in which the
#   boxes must be delivered to the dropzone. For example, if
#   todo = ['1','2']
#   is given with the above example `warehouse`, then the robot must first deliver box '1'
#   to the dropzone, and then the robot must deliver box '2' to the dropzone.
#
# === Rules for Movement ===
#
# - Two spaces are considered adjacent if they share an edge or a corner.
# - The robot may move horizontally or vertically at a cost of 2 per move.
# - The robot may move diagonally at a cost of 3 per move.
# - The robot may not move outside the warehouse.
# - The warehouse does not "wrap" around.
# - As described earlier, the robot may pick up a box that is in an adjacent square.
# - The cost to pick up a box is 4, regardless of the direction the box is relative to the robot.
# - While holding a box, the robot may not pick up another box.
# - The robot may put a box down on an adjacent empty space ('.') or the dropzone ('@') at a cost
#   of 2 (regardless of the direction in which the robot puts down the box).
# - If a box is placed on the '@' space, it is considered delivered and is removed from the ware-
#   house.
# - The warehouse will be arranged so that it is always possible for the robot to move to the
#   next box on the todo list without having to rearrange any other boxes.
#
# An illegal move will incur a cost of 100, and the robot will not move (the standard costs for a
#   move will not be additionally incurred). Illegal moves include:
# - attempting to move to a nonadjacent, nonexistent, or occupied space
# - attempting to pick up a nonadjacent or nonexistent box
# - attempting to pick up a box while holding one already
# - attempting to put down a box on a nonadjacent, nonexistent, or occupied space
# - attempting to put down a box while not holding one
#
# === Output Specifications ===
#
# `plan_delivery` should return a LIST of moves that minimizes the total cost of completing
#   the task successfully.
# Each move should be a string formatted as follows:
#
# 'move {i} {j}', where '{i}' is replaced by the row-coordinate of the space the robot moves
#   to and '{j}' is replaced by the column-coordinate of the space the robot moves to
#
# 'lift {x}', where '{x}' is replaced by the alphanumeric character of the box being picked up
#
# 'down {i} {j}', where '{i}' is replaced by the row-coordinate of the space the robot puts
#   the box, and '{j}' is replaced by the column-coordinate of the space the robot puts the box
#
# For example, for the values of `warehouse` and `todo` given previously (reproduced below):
#   warehouse = ['1#2',
#                '.#.',
#                '..@']
#   todo = ['1','2']
# `plan_delivery` might return the following:
#   ['move 2 1',   # cost = 2
#    'move 1 0',   # cost = 3
#    'lift 1',     # cost = 4
#    'move 2 1',   # cost = 3
#    'down 2 2',   # cost = 2
#    'move 1 2',   # cost = 3
#    'lift 2',     # cost = 4
#    'down 2 2']   # cost = 2
#
# === Grading ===
#
# - Your planner will be graded against a set of test cases, each equally weighted.
# - If your planner returns a list of moves of total cost that is K times the minimum cost of
#   successfully completing the task, you will receive 1/K of the credit for that test case.
# - Otherwise, you will receive no credit for that test case. This could happen for one of several
#   reasons including (but not necessarily limited to):
#   - plan_delivery's moves do not deliver the boxes in the correct order.
#   - plan_delivery's output is not a list of strings in the prescribed format.
#   - plan_delivery does not return an output within the prescribed time limit.
#   - Your code raises an exception.
#
# === Additional Info ===
#
# - You may add additional classes and functions as needed provided they are all in the file `partA.py`.
# - Upload partA.py to Project 2 on T-Square in the Assignments section. Do not put it into an
#   archive with other files.
# - Ask questions on Piazza if the directions or specifications are unclear. This is the first time
#   this project has ever been given, so help working out the bugs will be greatly appreciated!
#

class DeliveryPlanner:
    def __init__(self, warehouse, todo):
        self.warehouse = warehouse
        self.warehouse_uptodate = warehouse
        self.todo = todo

        for ir, row in enumerate(self.warehouse_uptodate):
            for it, item in enumerate(row):
                if item == '@':
                    self.delivery_location = [ir, it]

        self.goal_box = None
        self.current_grid = None
        self.finished_box = []
        self.carried_box = None
        self.cost = [2, 2, 2, 2, 3, 3, 3, 3]
        self.delta = [[-1, 0],  # go up
                      [0, -1],  # go left
                      [1, 0],  # go down
                      [0, 1],  # go right
                      [-1, -1],  # up left
                      [-1, 1],  # up right
                      [1, -1],  # down left
                      [1, 1]  # down right
                      ]

        self.delta_name = ['^', '<', 'v', '>', '<^', '>^', '<v', '>v', '*']
        self.output_path = []

    def plan_delivery(self):
        current_location = self.delivery_location
        with_box = False
        for box in self.todo:
            print box
            # reformat the map
            self.current_grid = []
            for ir, row in enumerate(self.warehouse_uptodate):
                temp_row = []
                for it, item in enumerate(row):
                    if item == '.':
                        temp_row.append(0)
                    elif item == '#':
                        temp_row.append(1)
                    elif item == '@':
                        temp_row.append(0)
                        self.delivery_location = [ir, it]
                    elif item == box:
                        temp_row.append(0)
                        self.goal_box = [ir, it]
                    elif item in self.finished_box:
                        temp_row.append(0)
                    else:
                        temp_row.append(1)
                self.current_grid.append(temp_row)

            # find the policy from the current location to the box
            policy = optimum_policy(grid=self.current_grid,
                                    goal=self.goal_box,
                                    cost=self.cost,
                                    delta=self.delta,
                                    delta_name=self.delta_name
                                    )
            # out put the list of moves to lift the box
            path, with_box, current_location, self.current_grid = find_path(
                grid=self.current_grid,
                start=current_location,
                goal=self.goal_box,
                policy=policy,
                delta=self.delta,
                delta_name=self.delta_name,
                box_number=box,
                with_box=with_box,
                deliver_location=self.delivery_location,
                purpose='lift'
            )

            self.output_path += path

            # find the policy from the current location back to the start
            policy = optimum_policy(grid=self.current_grid,
                                    goal=self.delivery_location,
                                    cost=self.cost,
                                    delta=self.delta,
                                    delta_name=self.delta_name
                                    )

            # out put the list of moves to put down the box
            path, with_box, current_location, self.current_grid = find_path(
                grid=self.current_grid,
                start=current_location,
                goal=self.delivery_location,
                policy=policy,
                delta=self.delta,
                delta_name=self.delta_name,
                box_number=box,
                with_box=with_box,
                deliver_location=self.delivery_location,
                purpose='down'
            )

            self.output_path += path

            # add delivered box to list
            self.finished_box.append(box)

            for line in self.current_grid:
                print line

            print self.output_path, with_box, current_location

        moves = self.output_path
        return moves


def optimum_policy(grid, goal, cost, delta, delta_name):
    value = [[99 for row in range(len(grid[0]))] for col in range(len(grid))]
    policy = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    change = True

    while change:
        change = False

        for x in range(len(grid)):
            for y in range(len(grid[0])):
                if goal[0] == x and goal[1] == y:
                    if value[x][y] > 0:
                        value[x][y] = 0
                        policy[x][y] = '*'
                        change = True

                elif grid[x][y] == 0:
                    for a in range(len(delta)):
                        x2 = x + delta[a][0]
                        y2 = y + delta[a][1]

                        if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(
                                grid[0]) and grid[x2][y2] == 0:
                            v2 = value[x2][y2] + cost[a]

                            if v2 < value[x][y]:
                                change = True
                                value[x][y] = v2
                                policy[x][y] = delta_name[a]
    for line in value:
        print line
    for line in policy:
        print line
    print "*" * 10
    return policy


def find_path(grid, start, goal, policy, delta, delta_name, box_number,
              with_box, deliver_location, purpose):
    x = start[0]
    y = start[1]
    move_ls = []
    goon = True
    while goon:
        goon = False
        symbol = policy[x][y]
        # if have a box, see whether can drop it.
        if with_box:
            for index in range(len(delta)):
                x2 = x + delta[index][0]
                y2 = y + delta[index][1]
                if x2 == deliver_location[0] and y2 == deliver_location[1]:
                    move_ls.append('down {0} {1}'.format(x2, y2))
                    with_box = False

        if symbol == '*':
            current_location = [x, y]
        else:
            index = delta_name.index(symbol)
            x2 = x + delta[index][0]
            y2 = y + delta[index][1]

            if (x2 == goal[0] and y2 == goal[1]):
                if not with_box and purpose=='lift':
                    move_ls.append('lift {0}'.format(box_number))
                    with_box = True
                    grid[x2][y2] = 0
                elif with_box and purpose=='down':
                    move_ls.append('down {0} {1}'.format(x2, y2))
                    with_box = False
                # else:
                #     # TODO: what if the purpose is lift, but have a box in hand?
                #     continue

                    # current_location = [x, y]
            else:
                x = x2
                y = y2
                goon = True
                move_ls.append('move {0} {1}'.format(x, y))

            current_location = [x, y]
    return move_ls, with_box, current_location, grid


# map(lambda x: '*' if x == 2 else '$' if x == 1 else 0, [1,2])


warehouse = ['1#2',
             '.#.',
             '..@']
warehouse = ['..1.',
             '..@.',
             '....',
             '2...']
todo = ['1', '2']

plan = DeliveryPlanner(warehouse=warehouse, todo=todo)
plan.plan_delivery()
