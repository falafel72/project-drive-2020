import numpy as np
import math
import random
from .easy_map import grid_map
from time import *
from sklearn.neighbors import KDTree

# Hyperparameters
SEARCH_RADIUS = 0.3

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
        self.path_tree = {self.origin:([],())}
        # A numpy array of coordinates used by KDTree to search
        self.explored_coords = np.array([list(self.origin)])
        # KD tree that searches the nearest neighbors of a random coordinate
        # NOTE: This Tree will be reconstructed every time the explored_coords
        # updates
        self.kd_search_tree = KDTree(np.array([self.origin]),leaf_size = 4)

    def search_and_connect_nearest_node(self, random_coord):
        """ Seach the curernt RRT to find several nearest coordinate neighbors of
            random_coord. Then check if if the random coordinate is reachable by 
            the nearest coords in RRT. If so, connect the first reachable nearest 
            coord with the random coord.

        Args:
            random_coord (list): a random coordinate in (x,y,z) list
        """
        # Append the new valid coord to the explored_coords
        self.explored_coords = np.append(self.explored_coords,list(random_coord))
        # Update the KD search Tree
        self.size += 1
        self.explored_coords = np.reshape(self.explored_coords,(self.size,3))
        self.kd_search_tree =  KDTree(np.array([self.origin]))
        n_index = self.kd_search_tree.query_radius([random_coord],r = SEARCH_RADIUS)
        for i in n_index[0]:
            if (self.occ_grid.is_reachable_coord(np.squeeze(self.explored_coords[i]),random_coord)):
                # Set the parent of the random_coord
                self.path_tree[random_coord] = ([],np.squeeze(self.explored_coords[i]))
                # Add random_coord to the nearest neibhbor's children
                self.path_tree[tuple(np.squeeze(self.explored_coords[i]))][0].append(random_coord)
                return True
        self.explored_coords = np.delete(self.explored_coords,range((self.size-1) * 3,self.size * 3))
        self.kd_search_tree =  KDTree(np.array([self.origin]))
        self.size -= 1
        return False 


    def construct_RRT(self, total_node_num):
        """ Create a RRT that has a certain number of valid nodes. Then add 
            destination _node to the RRT

        Args:
            total_node_num (int): total number of nodes in RRT
            destination_node (object:coord_node): the destination to be reached
        """
        current_node_count = 0
        last_coord = None
        while current_node_count < total_node_num:
            rand_coord = self.create_random_coord()
            # Increment node count only with successful addition of node
            if self.search_and_connect_nearest_node(rand_coord):
                print("found")
                current_node_count += 1
                last_coord = rand_coord
        # Add in destination to the last coords' children
        # TODO: find a better way. Last node may not always be reachable
        self.path_tree[last_coord][0].append(self.dest)

    def create_random_coord(self):
        """ Create a random coordinate with real world coordinates. This function does 
            not check whether the coordinate has been sampled or not

        Returns:
            [list]: random coordinate
        """
        val = 100
        node = None
        # Keep generating until the coord is in free space
        while val != 0:
            x = random.uniform(0, self.occ_grid.shape[0])
            y = random.uniform(0, self.occ_grid.shape[1])
            val = self.occ_grid.map[int(x)][int(y)]
            coord = self.occ_grid.grid_to_coord((x, y))
        return coord

    def find_path(self):
        """ Find a crude path from the origin to destination

        Returns:
            [list]: A path from root to destination in waypoints (x,y,z) lists
        """
        path = []
        current_coord = self.dest
        print(self.path_tree)
        while current_coord != self.origin:
            path.append(current_coord)
            current_coord = self.path_tree[current_coord][1]
        return path 
