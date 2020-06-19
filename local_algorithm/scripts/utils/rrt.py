import numpy as np
import math
import random


class RRT:
    def __init__(self, init_x, init_y, map):
        """ RRT Class. Used as a crude path generator. Root of the tree is the 
            initial location of car

        Args:
            init_x (int): the x coordinate of the car 
            init_y (init): the y coordinate of the car
            map (object:numpy.array): the occupancy grid of the map
        """
        self.root = coord_node(init_x, init_y)
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
            return True

    def construct_RRT(self, total_node_num, destination_node):
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
        self.search_and_connect_nearest_node(destination_node)

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
            x = random.randint(0, self.map.shape[0])
            y = random.randint(0, self.map.shape[1])
            val = self.map[x][y]
            node = coord_node(x, y)
        return node


class coord_node:
    def __init__(self, x, y):
        """ Node that contains coordinate information to be used in RRT

        Args:
            x (int): x coordinate in occupancy grid
            y (int): y coordinate in occupancy grid
        """
        self.coord = (x, y)
        self.children = np.array()

    def add_children(self, new_node):
        if isinstance(new_node, coord_node):
            self.children.append(new_node)

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
        slope = (self.coord[1] - other.coord[1]) / (self.coord[0] - other.coord[0])
        # Find intercept of line. Use average of intercept found with self and other
        inter = (
            self.coord[1]
            - self.coord[0] * slope
            + other.coord[1]
            - other.coord[0] * slope
        ) / 2
        """ Check all coordinates in occupancy grid. If any of the cordinates has
            value that is bigger than 0, there is at least one obstacle, so the 
            other node is not reachable. Both the nearest integer coordinate above
            and below the line are checked for obstacles.
        """
        for i in range(self.coord[0], other.coord[0]):
            if (
                map[i][int(i * slope + inter)] > 0
                or map[i][int(i * slope + inter) + 1] > 0
            ):
                return False
        return True
