import math
import random
from time import *

import numpy as np
from sklearn.neighbors import KDTree

from .easy_map import grid_map

# Hyperparameters
SEARCH_RADIUS = 4.0


class RRT:
    def __init__(self, origin, dest, occ_grid):
        """ Constructor of RRT

        Args:
            origin (list): Real world coordinate of the car's intial position in (x,y,z)
            dest (list): Real world coordinate of the car's destination position in (x,y,z)
            occ_grid (object:numpy.array): an occupancy grid that shows where the obstacles are
        """
        self.size = 1
        self.origin = origin
        self.dest = dest
        self.occ_grid = occ_grid
        # The path_tree stores the coordinates that have been added as waypoints
        # as key of a dictionary. The coordinates' children and parent waypoints
        # are stored as the value to each key, using the format of:
        # ([children],parent)
        self.path_tree = {self.origin: ([0,0,0])}
        # A numpy array of coordinates used by KDTree to search
        self.explored_coords = np.array([list(self.origin)])
        # An empty list to hold added points
        self.added_coords = []
        self.unconnected_cords = np.copy(self.explored_coords)
        # KD tree that searches the nearest neighbors of a random coordinate
        # NOTE: This Tree will be reconstructed every time the explored_coords
        # updates
        self.kd_search_tree = KDTree(np.array([self.origin]), leaf_size=4)
        
        self.vis_frontier = set()

    def search_and_connect_nearest_node(self, random_coord):
        """ Seach the curernt RRT to find several nearest coordinate neighbors of
            random_coord. Then check if if the random coordinate is reachable by 
            the nearest coords in RRT. If so, connect the first reachable nearest 
            coord with the random coord.

        Args:
            random_coord (list): a random coordinate in (x,y,z) list
        """
        # Append the new valid coord to the explored_coords
        self.explored_coords = np.append(self.explored_coords, list(random_coord))
        # Update the KD search Tree
        self.size += 1
        self.explored_coords = np.reshape(self.explored_coords, (self.size, 3))
        self.kd_search_tree = KDTree(np.array([self.origin]))
        n_index = self.kd_search_tree.query_radius([random_coord], r=SEARCH_RADIUS)
        for i in n_index[0]:
            if self.occ_grid.is_reachable_coord(
                np.squeeze(self.explored_coords[i]), random_coord
            ):
                # Set the parent of the random_coord
                self.path_tree[random_coord].append(np.squeeze(self.explored_coords[i]))
            return True
        self.explored_coords = np.delete(
            self.explored_coords, range((self.size - 1) * 3, self.size * 3)
        )
        self.kd_search_tree = KDTree(np.array([self.origin]))
        self.size -= 1
        return False

    def construct_RRT(self, total_coord_num,quick = True):
        """ Create a RRT that has a certain number of valid nodes. Then add 
            destination _node to the RRT

        Args:
            total_node_num (int): total number of nodes in RRT
            destination_node (object:coord_node): the destination to be reached
            quick (bool, optional): True if construct then search, False if search 
            then construct

        Return:
            bool: True if the path contains the destination, False otherwise
        """
        current_node_count = 0
        last_coord = None
        if not quick:
            while current_node_count < total_coord_num:
                rand_coord = self.create_random_coord()
                # Increment node count only with successful addition of node
                if self.search_and_connect_nearest_node(rand_coord):
                    current_node_count += 1
                    last_coord = rand_coord
        else:
            self.crude_points_gen(total_coord_num)
            self.kd_search_tree = KDTree(self.explored_coords, leaf_size=4)
            frontier = [self.origin]
            while len(frontier) != 0:
                checking = frontier.pop(0)
                if (type(checking) is not tuple):
                    checking = tuple(checking.tolist())
                n_index = self.kd_search_tree.query_radius([checking], r=SEARCH_RADIUS)
                for i in n_index[0]:
                    if self.occ_grid.is_reachable_coord(
                        np.squeeze(self.explored_coords[i]), checking
                    ):
                        # Set the parent of the random_coord
                        valid_point = np.squeeze(self.explored_coords[i]).tolist()
                        self.vis_frontier.add(tuple(checking))
                        if tuple(valid_point) == self.dest:
                            self.path_tree[self.dest][0] = checking[0]
                            self.path_tree[self.dest][1] = checking[1] 
                            self.path_tree[self.dest][2] = checking[2]
                            return True
                        elif not valid_point in self.added_coords:
                            self.added_coords.append(valid_point)
                            self.path_tree[tuple(valid_point)][0] = checking[0]
                            self.path_tree[tuple(valid_point)][1] = checking[1] 
                            self.path_tree[tuple(valid_point)][2] = checking[2]                          
                            frontier.append(tuple(valid_point))

            return False 
        # Add in destination to the last coords' children
        # TODO: find a better way. Last node may not always be reachable

    def crude_points_gen(self, total_coord_num):
        """ Generate a number of waypoints on in free space

        Args:
            total_coord_num (int): the number of waypoints to be generated
        """
        coord_num = 0
        while coord_num < total_coord_num:
            rand_coord = self.create_random_coord()
            self.explored_coords = np.append(self. explored_coords,rand_coord)
            self.path_tree[rand_coord] = ([0,0,0])
            coord_num += 1
        self.explored_coords = np.append(self.explored_coords,self.dest)
        self.path_tree[self.dest] = ([0,0,0])
        self.explored_coords = np.reshape(self.explored_coords,[-1,3])

    def create_random_coord(self):
        """ Create a random coordinate with real world coordinates. This function 
            also checks whether the coordinate has been sampled or not by creating
            sparse points

        Returns:
            [list]: random coordinate
        """
        val = 100
        grid_coord = []
        node = None
        sur_flag = False
        # Keep generating until the coord is in free space
        # TODO: add more weight to underexplored region
        while val != 0 and not sur_flag:
            x = random.uniform(0, self.occ_grid.shape[1])
            y = random.uniform(0, self.occ_grid.shape[0])
            grid_coord = (int(y),int(x))
            if not self.check_surrounding((int(y),int(x)),area = 20):
                continue
            val = self.occ_grid.map[int(y)][int(x)]
            coord = self.occ_grid.grid_to_coord((int(x), int(y)))
        self.occ_grid.map[grid_coord[0]][grid_coord[1]] = 1
        return coord

    def check_surrounding(self,point,area = 5):
        """ Helper function. Check to make sure the point is not too close to walls or other explored coords

        Args:
            point (array-like): the point to be checked
            area (int, optional): Side length of a square area on occupancy grid. Defaults to 5.

        Returns:
            bool: True if the point is not too close to wall, False otherwise
        """
        for i in range(area):
            for j in range(area):
                if self.occ_grid.map[point[0]-i][point[1]-j] > 0 or self.occ_grid.map[point[0]+i][point[1]+j] > 0:
                    return False
        return True

    def find_path(self):
        """ Find a crude path from the origin to destination

        Returns:
            [list]: A path from root to destination in waypoints (x,y,z) lists
        """
        path = []
        current_coord = self.dest
        while current_coord != list(self.origin):
            path.append(current_coord)
            current_coord = self.path_tree[tuple(current_coord)]
        return path

    def search_path(self):
        """ A simple BFS search for the shortest path from origin to dest

        Returns:
            list: waypoints that composes the path
        """
        path = []
        
        return path
