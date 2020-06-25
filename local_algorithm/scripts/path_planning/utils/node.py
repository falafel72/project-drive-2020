import numpy as np
import math
import utils.easy_map as mp


class coord_node:
    def __init__(self, x, y):
        """ Node that contains coordinate information to be used in RRT

        Args:
            x (doulbe): x coordinate in occupancy grid
            y (doulbe): y coordinate in occupancy grid
        """
        self.coord = (x, y, 0.0)
        self.children = []
        self.parent = None

    def get_distance(self, other):
        """ Get the Euclidian distance between self and another coord_node

        Args:
            other (object:coord_node): distance to this node is to be found
        Returns:
            [double]: the distance between self and other
        """
        if isinstance(other, coord_node):
            return math.sqrt(
                pow((self.coord[0] - other.coord[0]), 2)
                + pow((self.coord[1] - other.coord[1]), 2)
            )
        else:
            return None

    def is_reachable(self, other, map):
        """ Check if another node is reachable from self, given the constraint of
            a map

        Args:
            other (object:coord_node): accessability to this node is to be checked
            map (object:numpy.array): the occupancy grid of the map

        Returns:
            [bool]: True if no obstacle is between self and other; False otherwise
        """

        """ Check all coordinates in occupancy grid. If any of the cordinates has
            value that is bigger than 0, there is at least one obstacle, so the 
            other node is not reachable. Both the nearest integer coordinate above
            and below the line are checked for obstacles.
        """
        start = map.coord_to_grid(self.coord)
        end = map.coord_to_grid(other.coord)
        """
        slope = (other.coord[0] - self.coord[0]) / (other.coord[1] - self.coord[1])
        # Find intercept of line. Use average of intercept found with self and other
        inter = (
            self.coord[0]
            - self.coord[1] * slope
            + other.coord[0]
            - other.coord[1] * slope
        ) / 2
        """
        if float(end[1] - start[1]) == 0:
            return False
        slope = float(end[0] - start[0]) / float(end[1] - start[1])
        inter = (start[0] - start[1] * slope + end[0] - end[1] * slope) / 2.0

        for i in range(start[1], end[1]):
            try:
                if (
                    map.map[i][int(i * slope + inter)] > 0
                    or map.map[i][int(i * slope + inter) + 1] > 0
                ):
                    return False
            except:
                return False
        return True
