import math
import numpy as np


class grid_map:
    def __init__(self):
        """ Empty constructer to be used before first ROS update 
        """
        self.map = []
        self.car_init_pose = None
        self.dest_pose = None
        self.shape = None
        self.metric_shape = None
        self.car_size = None
        self.map_origin = None
        self.map_orientation = None
        self.map_resolution = None
        self.wall = []

    def update_map(
        self, occupancy_grid, width, height, car_size, origin, orientation, resolution
    ):
        """ Update properties of map continuously. 

        Args:
            width (int): width of map
            height (int): heigh of map
            car_size (tuple): the size of the car in x and y direction. 
            origin (tuple): origin of the map, given by metadata
            orientation (tuple): quaternion orientation of map, given by metadata
            resolution (double): ratio of grid unit to global unit, given by metadata
        """
        self.shape = (height, width)
        self.car_size = car_size
        self.map_origin = origin
        self.map_orientation = orientation
        self.map_resolution = resolution
        self.map = np.array(occupancy_grid).reshape(self.shape)
        self.metric_shape = self.grid_to_coord(self.shape)

    def set_init_dest_pose(self, init_pose):
        # Use this function to avoid overwriting init_pose with current pose of car
        self.car_init_pose = init_pose
        # Destination is currently 2 car length behind the car in y direction
        if self.car_init_pose != None and self.car_size != None:
            self.dest_pose = (
                self.car_init_pose[0],
                self.car_init_pose[1] - 2 * self.car_size[0],
                0,
            )

    def build_wall(self):
        """ Build a wall behind the car so that RRT generate a realistic path 
        """
        # Wall is currently centered at 1 car length behind the car in y direction
        wall_center = (
            self.car_init_pose[0],
            self.car_init_pose[1] - self.car_size[0],
            0,
        )
        wall_coord = self.coord_to_grid(wall_center)
        reached = False
        i = 0
        while not reached:
            # NOTE: working only with the current map
            self.map[wall_coord[0] - i][wall_coord[1]] = 100
            self.wall.append((wall_coord[0] - i, wall_coord[1]))
            i += 1
            if self.map[wall_coord[0] - i][wall_coord[1]] == 100:
                reached = True
        reached = False
        i = 0
        while not reached:
            self.map[wall_coord[0] + i][wall_coord[1]] = 100
            self.wall.append((wall_coord[0] + i, wall_coord[1]))
            i += 1
            if self.map[wall_coord[0] + i][wall_coord[1]] == 100:
                reached = True

    def has_built_wall(self):
        # check if the wall has been built
        return len(self.wall) != 0

    def remove_wall(self):
        if self.wall != None:
            for wall_coords in self.wall:
                self.map[wall_coords[0]][wall_coords[1]] = 0

    def grid_to_coord(self, grid_rc):
        # grid_rc: 2- tuple, row first, then col
        coord_x = grid_rc[1] * self.map_resolution + self.map_origin[1]
        coord_x = int(coord_x * 1000) / 1000.0
        coord_y = grid_rc[0] * self.map_resolution + self.map_origin[0]
        coord_y = int(coord_y * 1000) / 1000.0
        return (coord_y, coord_x, 0.0)

    def coord_to_grid(self, coord):
        # Give an approximate grid coordinate (truncated)
        col = int((coord[0] - self.map_origin[0]) / self.map_resolution)
        row = int((coord[1] - self.map_origin[1]) / self.map_resolution)
        return [row, col]

    def print_map(self):
        print("map size:", len(self.map))
        print("init_pose:", self.car_init_pose)
        print("resolution:", self.map_resolution)
        print("map_origin:", self.map_origin)

    def is_reachable_occ(self, origin, target):
        """ Check if two points on the occupancy grid are in reach of each other
            given the constraint of the map. 

        Args:
            origin (tuple): coordinate of a point on the occupancy grid
            target (tuple): coordinate of a point on the occupancy grid

        Returns:
            [bool]: True if no obstacle is between self and other; False otherwise
        """

        # Check for vertical lines with indefinate slope
        if target[1] == origin[1]:
            for i in range(origin[0], target[0]):
                try:
                    if map.map[i][origin[1]] > 0:
                        return False
                except:
                    return False
            return True
        slope = float(target[0] - origin[0]) / float(target[1] - origin[1])
        # Take average of intersection calculated using origin and target
        inter = (origin[0] - origin[1] * slope + target[0] - target[1] * slope) / 2.0
        """ Check all coordinates in occupancy grid. If any of the cordinates has
            value that is bigger than 0, there is at least one obstacle, so the 
            other node is not reachable. Both the nearest integer coordinate above
            and below the line are checked for obstacles.
        """
        for i in range(origin[1], target[1]):
            try:
                if (
                    map.map[i][int(i * slope + inter)] > 0
                    or map.map[i][int(i * slope + inter) + 1] > 0
                ):
                    return False
            except:
                return False
        return True

    def is_reachable_coord(self, origin, target):
        """ Check if two points are reachable using the global coordinate of two 
            points. Points are converted to occupancy grid coordinates

        Args:
            origin (tuple): coordinate of the origin point
            target (tuple): coordinate of the target point

        Returns:
            [bool]: True if no obstacle is between self and other; False otherwise
        """
        start = self.coord_to_grid(origin)
        end = self.coord_to_grid(target)
        return self.is_reachable_occ(start, end)

    def get_distance(self, coord1, coord2):
        """ Get distance between two coordinates in real world. Check 

        Args:
            coord1 (tuple): first coordination
            coord2 (tuple): second coodination

        Returns:
            [float]: the distance between the two coordinates; None if distance
            cannot be calculated 
        """
        if len(coord1) == len(coord2):
            sum_sq = 0.0
            for i in range(len(coord1)):
                sum_sq += math.pow((coord1[i] - coord2[i]), 2)
            return math.sqrt(sum_sq)
        return None
