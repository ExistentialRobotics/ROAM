#!/usr/bin/env python
"""frontier_utils.py
Supporting methods for all things frontier exploration
"""
from __future__ import print_function, absolute_import, division

import cv2
import numpy as np
from matplotlib import pyplot as plt

from mapping.costmap import Costmap
from utilities.util import which_coords_in_bounds, xy_to_rc, rc_to_xy


def extract_frontiers(occupancy_map, approx=True, approx_iters=2,
                      kernel=cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3)),
                      debug=False,
                      occupied_value=Costmap.OCCUPIED,
                      unexplored_value=Costmap.UNEXPLORED,
                      free_value=Costmap.FREE):
    """
    Given a map of free/occupied/unexplored pixels, identify the frontiers.
    This method is a quicker method than the brute force approach,
    and scales pretty well with large maps. Using cv2.dilate for 1 pixel on the unexplored space
    and comparing with the free space, we can isolate the frontiers.

    :param occupancy_map Costmap: object corresponding to the map to extract frontiers from
    :param approx bool: does an approximation of the map before extracting frontiers. (dilate erode, get rid of single
                   pixel unexplored areas creating a large number fo frontiers)
    :param approx_iters int: number of iterations for the dilate erode
    :param kernel array(N, N)[float]: the kernel of which to use to extract frontiers / approx map
    :param debug bool: show debug windows?
    :param occupied_value: uint8 Value to be considered as occupied
    :param unexplored_value: uint8 Value to be considered as unexplored
    :param free_value: uint 8Value to be considered as free
    :return List[array(N, 2][float]: list of frontiers, each frontier is a set of coordinates
    """
    # todo regional frontiers
    # extract coordinates of occupied, unexplored, and free coordinates
    occupied_coords = np.argwhere(occupancy_map.data.astype(np.uint8) == occupied_value)
    unexplored_coords = np.argwhere(occupancy_map.data.astype(np.uint8) == unexplored_value)
    free_coords = np.argwhere(occupancy_map.data.astype(np.uint8) == free_value)

    if free_coords.shape[0] == 0 or unexplored_coords.shape[0] == 0:
        return []

    # create a binary mask of unexplored pixels, letting unexplored pixels = 1
    unexplored_mask = np.zeros_like(occupancy_map.data)
    unexplored_mask[unexplored_coords[:, 0], unexplored_coords[:, 1]] = 1

    # dilate using a 3x3 kernel, effectively increasing
    # the size of the unexplored space by one pixel in all directions
    dilated_unexplored_mask = cv2.dilate(unexplored_mask, kernel=kernel)
    dilated_unexplored_mask[occupied_coords[:, 0], occupied_coords[:, 1]] = 1

    # create a binary mask of the free pixels
    free_mask = np.zeros_like(occupancy_map.data)
    free_mask[free_coords[:, 0], free_coords[:, 1]] = 1

    # can isolate the frontiers using the difference between the masks,
    # and looking for contours
    frontier_mask = ((1 - dilated_unexplored_mask) - free_mask)
    if approx:
        frontier_mask = cv2.dilate(frontier_mask, kernel=kernel, iterations=approx_iters)
        frontier_mask = cv2.erode(frontier_mask, kernel=kernel, iterations=approx_iters)

    frontiers_xy_px = cv2.findContours(frontier_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)[-2:][0]
    frontiers = [rc_to_xy(np.array(frontier).squeeze(1)[:, ::-1], occupancy_map) for frontier in frontiers_xy_px]

    if debug:
        frontier_map = np.repeat([occupancy_map.data], repeats=3, axis=0).transpose((1, 2, 0))
        # frontiers = [frontiers[rank] for i, rank in enumerate(frontier_ranks)
        #              if i < self.num_frontiers_considered]
        for frontier in frontiers:
            # if frontier.astype(np.int).tolist() in self.frontier_blacklist:
            #     continue
            frontier_px = xy_to_rc(frontier, occupancy_map).astype(int)
            frontier_px = frontier_px[which_coords_in_bounds(frontier_px, occupancy_map.get_shape())]
            frontier_map[frontier_px[:, 0], frontier_px[:, 1]] = [255, 0, 0]

        plt.imshow(frontier_map, cmap='gray', interpolation='nearest')
        plt.show()

    return frontiers


def cleanup_map_for_planning(occupancy_map, kernel, filter_obstacles=False, debug=False):
    """
    We are not allowed to plan in unexplored space, (treated as collision), so what we do is dilate/erode the free
    space on the map to eat up the small little unexplored pixels, allows for quicker planning.
    :param occupancy_map Costmap: object
    :param kernel array(N, N)[float]: kernel to use to cleanup map (dilate/erode)
    :param filter_obstacles bool: whether to filter obstacles with a median filter, potentially cleaning up single
                              pixel noise in the environment.
    :param debug bool: show debug plot?
    :return Costmap: cleaned occupancy map
    """
    occupied_coords = np.argwhere(occupancy_map.data == Costmap.OCCUPIED)
    free_coords = np.argwhere(occupancy_map.data == Costmap.FREE)

    free_mask = np.zeros_like(occupancy_map.data)
    free_mask[free_coords[:, 0], free_coords[:, 1]] = 1
    free_mask = cv2.dilate(free_mask, kernel=kernel, iterations=2)
    free_mask = cv2.erode(free_mask, kernel=kernel, iterations=2)
    new_free_coords = np.argwhere(free_mask == 1)

    if filter_obstacles:
        occupied_mask = np.zeros_like(occupancy_map.data)
        occupied_mask[occupied_coords[:, 0], occupied_coords[:, 1]] = 1
        occupied_mask = cv2.medianBlur(occupied_mask, kernel.shape[0])
        occupied_coords = np.argwhere(occupied_mask == 1)

    cleaned_occupancy_map = occupancy_map.copy()
    cleaned_occupancy_map.data[new_free_coords[:, 0], new_free_coords[:, 1]] = Costmap.FREE
    cleaned_occupancy_map.data[occupied_coords[:, 0], occupied_coords[:, 1]] = Costmap.OCCUPIED

    if debug:
        plt.imshow(cleaned_occupancy_map.data, cmap='gray', interpolation='nearest')
        plt.show()

    return cleaned_occupancy_map
