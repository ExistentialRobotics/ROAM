#!/usr/bin/env python
"""sensor_util.py
utilities for sensors, currently contains raytracing code
"""
from __future__ import print_function, absolute_import, division

import numpy as np


def bresenham2d_with_intensities(p1, p2, img):
    """
    https://stackoverflow.com/questions/32328179/opencv-3-0-python-lineiterator
    Produces and array that consists of the coordinates and intensities
    of each pixel in a bresenham line between two points
    :param p1 array(2)[int]: the coordinate of the first point (x, y)
    :param p2 array(2)[int]: the coordinate of the second point (x, y)
    :param img array(N,M)[Union[int, uint8, float]]: the image being processed

    :return line array(N,3)[float]: a numpy array that consists of the coordinates and intensities of each pixel
                     in the radii (shape: [numPixels, 3], row = [x, y, intensity])
    """
    # define local variables for readability
    image_height = img.shape[0]
    image_width = img.shape[1]
    p1_x = p1[0]
    p1_y = p1[1]
    p2_x = p2[0]
    p2_y = p2[1]

    # difference and absolute difference between points
    # used to calculate slope and relative location between points
    dx = p2_x - p1_x
    dy = p2_y - p1_y
    dx_abs = np.abs(dx)
    dy_abs = np.abs(dy)

    # predefine numpy array for output based on distance between points
    line = np.empty(shape=(np.maximum(dy_abs, dx_abs), 3), dtype=np.float32)
    line.fill(np.nan)

    # obtain coordinates along the line using a form of bresenham's algorithm
    neg_y = p1_y > p2_y
    neg_x = p1_x > p2_x
    if p1_x == p2_x:  # vertical line segment
        line[:, 0] = p1_x
        if neg_y:
            line[:, 1] = np.arange(p1_y - 1, p1_y - dy_abs - 1, -1)
        else:
            line[:, 1] = np.arange(p1_y + 1, p1_y + dy_abs + 1)
    elif p1_y == p2_y:  # horizontal line segment
        line[:, 1] = p1_y
        if neg_x:
            line[:, 0] = np.arange(p1_x - 1, p1_x - dx_abs - 1, -1)
        else:
            line[:, 0] = np.arange(p1_x + 1, p1_x + dx_abs + 1)
    else:  # diagonal line segment
        steep_slope = dy_abs > dx_abs
        if steep_slope:
            slope = dx.astype(np.float32) / dy.astype(np.float32)
            if neg_y:
                line[:, 1] = np.arange(p1_y - 1, p1_y - dy_abs - 1, -1)
            else:
                line[:, 1] = np.arange(p1_y + 1, p1_y + dy_abs + 1)
            line[:, 0] = (slope * (line[:, 1] - p1_y)).astype(int) + p1_x
        else:
            slope = dy.astype(np.float32) / dx.astype(np.float32)
            if neg_x:
                line[:, 0] = np.arange(p1_x - 1, p1_x - dx_abs - 1, -1)
            else:
                line[:, 0] = np.arange(p1_x + 1, p1_x + dx_abs + 1)
            line[:, 1] = (slope * (line[:, 0] - p1_x)).astype(int) + p1_y

    # remove points outside of image
    col_x = line[:, 0]
    col_y = line[:, 1]
    line = line[(col_x >= 0) & (col_y >= 0) & (col_x < image_width) & (col_y < image_height)]

    # get intensities from img ndarray
    line[:, 2] = img[line[:, 1].astype(np.uint), line[:, 0].astype(np.uint)]

    return line


def bresenham2d(p0, p1):
    """
    Implementation of Bresenham's line drawing algorithm
    See en.wikipedia.org/wiki/Bresenham's_line_algorithm

    Yield integer coordinates on the line from p0: (x0, y0) to p1: (x1, y1).
    Input coordinates should be integers.
    The result will contain both the start and the end point.

    :param p0 array(2)[int]: starting point for the algorithm
    :param p1 array(2)[int]: ending point for the algorithm
    :return array(N,2)[int]: ndarray of points from start to end point
    """

    x0 = int(np.round(p0[0]))
    y0 = int(np.round(p0[1]))
    x1 = int(np.round(p1[0]))
    y1 = int(np.round(p1[1]))
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    steep = abs(dy) > abs(dx)
    if steep:
        dx, dy = dy, dx  # swap

    if dy == 0:
        q = np.zeros((dx + 1, 1))
    else:
        q = np.append(0, np.greater_equal(
            np.diff(np.mod(np.arange(np.floor(dx / 2), -dy * dx + np.floor(dx / 2) - 1, -dy), dx)), 0))
    if steep:
        if y0 <= y1:
            y = np.arange(y0, y1 + 1)
        else:
            y = np.arange(y0, y1 - 1, -1)
        if x0 <= x1:
            x = x0 + np.cumsum(q)
        else:
            x = x0 - np.cumsum(q)
    else:
        if x0 <= x1:
            x = np.arange(x0, x1 + 1)
        else:
            x = np.arange(x0, x1 - 1, -1)
        if y0 <= y1:
            y = y0 + np.cumsum(q)
        else:
            y = y0 - np.cumsum(q)
    return np.vstack((x, y)).T
