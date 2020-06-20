import numpy as np
import math
import random
from node import coord_node

class RRT:
    def __init__(self, init_x, init_y, dest_x,dest_y,map):
        """ RRT Class. Used as a crude path generator. Root of the tree is the 
            initial location of car

        Args:
            init_x (double): the x coordinate of the car on map
            init_y (double): the y coordinate of the car on map
            dest_x (double): the x coordinate of the destination on map
            dest_y (double): the y coordinate of the destination on map
            map (object:numpy.array): the occupancy grid of the map
        """
        self.root = coord_node(init_x, init_y)
        self.dest = coord_node(dest_x, dest_y)
        self.map = map

    def search_and_connect_nearest_node(self, random_node):
        """ Search the RRT for the node that is cloestest to random_node in distance. 
            There also need to be no obstacle between the random_node and a node in 
            RRT for it to be added. The function also check for repeated node and
            makes sure that there is no repeated node in RRT
        Args:
            random_node (object:coord_node): A node of coord_node type that is 
            to be checked and possibly added to RRT

        Returns:
            [bool]: True if the node is successfully added, False if not added
        """
        # A queue of node encountered but not searched
        frontier = []
        # flag for repeatedly created random node
        repeated = False
        # Initial nearest node is set to the root
        # TODO: this is probably buggy in some way, improve this.
        nearest_node = self.root
        # Start the search at the root
        frontier.append(self.root)
        while len(frontier) > 0:
            checking = frontier.pop(0)
            # Check if the coordinate has already been added to the RRT
            if (
                checking.coord[0] == random_node.coord[0]
                and checking.coord[1] == random_node.coord[1]
            ):
                repeated = True
                break

            if checking.is_reachable(random_node, self.map):
                if checking.get_distance(random_node) < nearest_node.get_distance(
                    random_node
                ):
                    nearest_node = checking
            for nodes in checking.children:
                frontier.append(nodes)
        if repeated:
            return False
        else:
            nearest_node.add_children(random_node)
            random_node.parent = nearest_node
            return True

    def construct_RRT(self, total_node_num):
        """ Create a RRT that has a certain number of valid nodes. Then add 
            destination _node to the RRT

        Args:
            total_node_num (int): total number of nodes in RRT
            destination_node (object:coord_node): the destination to be reached
        """
        current_node_count = 0
        while current_node_count < total_node_num:
            rand_node = self.create_random_node()
            # Increment node count only with successful addition of node
            if self.search_and_connect_nearest_node(rand_node):
                current_node_count += 1
        # Add in destination
        # TODO: is the destination always reachable? What if not reachable?
        self.search_and_connect_nearest_node(self.dest)

    def create_random_node(self):
        """ Create a random coord_node with int coordinates. This function does 
            not check whether the coordinate has been sampled or not

        Returns:
            [object:coord_node]: random coord_node
        """
        val = 100
        node = None
        # Keep generating until the coord is in free space
        while val != 0:
            x = random.uniform(0, self.map.shape[0])
            y = random.uniform(0, self.map.shape[1])
            val = self.map[int(x)][int(y)]
            node = coord_node(x, y)
        return node

    def find_path(self):
        """ Find a crude path from the root to destination

        Returns:
            [list]: A path from root to destination
        """
        path = []
        current_node = self.dest
        while current_node is not None:
            path.insert(0,current_node)
            current_node = current_node.parent
        return path

