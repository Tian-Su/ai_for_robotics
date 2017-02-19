# ----------
# User Instructions:
#
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space
# from operator import itemgetter
import numpy as np

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid) - 1, len(grid[0]) - 1]
cost = 1

delta = [[-1, 0],  # go up
         [0, -1],  # go left
         [1, 0],  # go down
         [0, 1]]  # go right

delta_name = ['^', '<', 'v', '>']


def search(grid, init, goal, cost, delta=delta):
    # ----------------------------------------
    delta = np.array(delta)
    delta = np.concatenate((np.array([1, 1, 1, 1]).reshape(4, 1), delta),
                           axis=1)
    # print delta
    open_ls = np.array([[1] + init])
    grid = np.array(grid)
    print grid[0,2]

    while True:
        open_ls[open_ls[:, 0].argsort()]
        temp = open_ls[0].reshape(1, 3)
        temp = np.repeat(temp, [4], axis=0)
        new = temp + delta
        for i in range(4):
            idx = new[i, 1:]
            print idx
            if bound(idx, grid.shape):
                print grid[idx[0], idx[1]]
        # for in

        # np.where((vals == (0, 1)).all(axis=1))
        break
    print 'open_ls: ', open_ls
    #

def bound((y, x), shape):
    if x >=0 and y >= 0 and x < shape[1] and y < shape[0]:
        return True
    else:
        return False


    # ----------------------------------------

    # return path


search(grid, init, goal, cost)
