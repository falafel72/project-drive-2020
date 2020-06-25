import rospy
from utils.easy_map import grid_map
from utils.rrt import RRT
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from time import *

from utils.rviz_visualize import *

# The map that is to be used for path planning
MASTER_MAP = grid_map()


def initial_odom_callback(data):
    """ One time listener of the initial position of the car

    Args:
        data (object:nav_msgs.msg.Odometry): the odometry data
    """
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z
    MASTER_MAP.set_init_dest_pose((x, y, z))


def initial_map_builder(data):
    """ One time listener that get the map metadata

    Args:
        data (object:nav_msgs.msg.Occupancygrid): the occupancy grid data
    """
    occ_grid = data.data
    res = data.info.resolution
    width = data.info.width
    height = data.info.height
    og = data.info.origin
    origin = (og.position.x, og.position.y, og.position.z)
    orientation = (
        og.orientation.x,
        og.orientation.y,
        og.orientation.z,
        og.orientation.w,
    )
    MASTER_MAP.update_map(
        occ_grid, width, height, (0.1, 0.1, 0.0), origin, orientation, res
    )


def intial_state_listener():

    # TODO: check if the node has initialized
    initial_map_builder(rospy.wait_for_message("/map", OccupancyGrid))
    initial_odom_callback(rospy.wait_for_message("/odom", Odometry))


def wall_publisher():
    """ Puslish markers that represent the wall
    """
    MASTER_MAP.build_wall()
    wall_coord = []
    for i in MASTER_MAP.wall:
        wall_coord.append(MASTER_MAP.grid_to_coord(i))
    to_pub = visualize_wall("map", wall_coord)
    visualize(to_pub, "walls")


def path_publisher(pth):
    to_pub = visualize_path("map", pth)
    visualize(to_pub, "rrt_path")


def initial_cycle_builder():
    """ Create a path on the map using RRT

    Returns:
        [list]: the coordinates of waypoints that consists the path
    """
    begin_time = time()
    print("Started building path")
    crude_path = RRT(
        MASTER_MAP.car_init_pose[0],
        MASTER_MAP.car_init_pose[1],
        MASTER_MAP.dest_pose[0],
        MASTER_MAP.dest_pose[1],
        MASTER_MAP,
    )
    crude_path.construct_RRT(1000)
    path = crude_path.find_path()
    print("Finished building path")
    end_time = time()
    print("runtime:", end_time - begin_time)
    return path


if __name__ == "__main__":
    rospy.init_node("path_constructor", anonymous=True)
    intial_state_listener()
    path = initial_cycle_builder()
    try:
        path_publisher(path)
        wall_publisher()
    except rospy.ROSInterruptException:
        pass
