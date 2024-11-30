#!/usr/bin/env python
from __future__ import print_function, absolute_import, division

import numpy as np
import matplotlib.pyplot as plt


def get_tricky_circular_footprint():
    """
    Gives a not-quite-circular footprint for testing, the origin is not centered in the footprint
    :return array(N, 2)[float]: points defining the footprint
    """
    return np.array(
        [[0.22674791, 0],
         [0.22129365, 0.14978179],
         [0.21026903, 0.17772615],
         [0.13216534, 0.22714073],
         [0.07048001, 0.22728987],
         [-0.06165067, 0.2245],
         [-0.12432992, 0.2102427],
         [-0.23903614, 0.16096445],
         [-0.241, 0.15896477],
         [-0.241, -0.15896477],
         [-0.23903614, -0.16096445],
         [-0.12432992, -0.2102427],
         [-0.06165067, -0.2245],
         [0.07048001, -0.22728987],
         [0.13216534, -0.22714073],
         [0.21026903, -0.17772615],
         [0.22129365, -0.14978179]]
    )


def get_tricky_oval_footprint():
    """
    gives a not-quite-oval footprint for testing, the origin is not centered in the footprint
    :return array(N, 2)[float]: points defining the footprint
    """
    return np.array([
        [1348.35, 0.],
        [1338.56, 139.75],
        [1306.71, 280.12],
        [1224.36, 338.62],
        [1093.81, 374.64],
        [-214.37, 374.64],
        [-313.62, 308.56],
        [-366.36, 117.44],
        [-374.01, -135.75],
        [-227.96, -459.13],
        [-156.72, -458.78],
        [759.8, -442.96],
        [849.69, -426.4],
        [1171.05, -353.74],
        [1303.15, -286.54],
        [1341.34, -118.37]]
    ) / 1000.

def generate_clockwise_square(x_size, y_size, points_per_side):
    """
    Generates points on a clockwise square with the given parameters
    :param x_size float: size in the x axis
    :param y_size float: size in the y axis
    :param points_per_side int: number of points to generate per side
    :return array(N, 2)[float]: points defining the square
    """
    y_range = np.linspace(-y_size / 2, y_size / 2, points_per_side, endpoint=True)
    x_range = np.linspace(-x_size / 2, x_size / 2, points_per_side, endpoint=True)
    y_range = np.vstack(([np.zeros_like(y_range)], [y_range])).T
    x_range = np.vstack(([[x_range], np.zeros_like(x_range)])).T

    back_points = y_range + x_range[0, :]

    front_points = y_range + x_range[-1, :]
    zero_ind = np.argmin(np.abs(front_points[:, 1]))
    front_first_half = front_points[zero_ind:, :]
    front_last_half = front_points[:zero_ind]

    left_points = x_range + y_range[-1, :]

    right_points = x_range + y_range[0, :]
    right_points = right_points[::-1, :]

    # points = np.vstack((back_points, front_points, left_points, right_points))
    points = np.vstack((front_first_half, right_points, back_points, left_points, front_last_half))
    return points


def get_jackal_footprint(points_per_side=11, debug=False):
    """
    Gives an estimate of the ClearPath jackal robot footprint
    :param points_per_side int: number of points per side to discretize
    :param debug bool: show plots?
    :return array(N, 2)[float]: points defining the footprint
    """
    assert points_per_side % 2 != 0 and "Must be odd, so 0 is generated"
    length = .508
    width = .430
    points = generate_clockwise_square(length, width, points_per_side)
    if debug:
        plt.scatter(points[:, 0], points[:, 1])
        plt.show()
    return points