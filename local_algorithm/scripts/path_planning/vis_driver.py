from time import *

import numpy as np
import rospy
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from std_msgs.msg import Float32MultiArray
from tf import ExtrapolationException, LookupException, TransformListener
from visualization_msgs.msg import MarkerArray

import utils.transform as u_tf
from utils.easy_map import grid_map
from utils.rviz_visualize import *
from utils.rrt import RRT

# The map that is to be used for path planning
MASTER_MAP = grid_map()
# Angle topic
ANGLE_TOPIC = "/green_path"
# Frames of TF
CAR_FRAME = "ego_racecar/base_link"
MAP_FRAME = "/map"
# Waypoint path publisher
path_pub = rospy.Publisher("/path_vis", MarkerArray, queue_size=3)
# Angle choice publisher
angle_pub = rospy.Publisher("/steer_angle_vis", MarkerArray, queue_size=3)
# wall pub
wall_pub = rospy.Publisher("/wall",MarkerArray,queue_size=1)
# generated waypoint pub
waypoint_pub = rospy.Publisher("/wp",MarkerArray,queue_size=1)
# Waypoints
PATH = [
    [193, 303],
    [214, 268],
    [267, 250],
    [306, 291],
    [316, 336],
    [302, 371],
    [262, 406],
    [211, 412],
    [156, 403],
    [127, 374],
    [134, 343],
    [172, 322],
]
# Car size (x,y,z)
CAR_SIZE = (0.4,0.2,0.1)
# pass updated angle path for marker publisher
angle_path = None
# Global transformation of the car
car_translation = [0, 0, 0]
car_rotation = [0, 0, 0, 0]


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
        occ_grid, width, height, CAR_SIZE, origin, orientation, res
    )


def intial_state_listener():
    # Get map information from /map topic
    # TODO: check if the node has initialized
    initial_map_builder(rospy.wait_for_message("/map", OccupancyGrid))
    initial_odom_callback(rospy.wait_for_message("/odom", Odometry))


def angle_vis_callback(data):
    global angle_path
    try:
        angle_path = np.squeeze(np.reshape(np.array(data.data), [-1, 2]))
    except:
        print("unable to shape input array")    

if __name__ == "__main__":

    # The transformed angle points to be published
    angle_points = []
    rospy.init_node("path_constructor", anonymous=True)
    intial_state_listener()
    MASTER_MAP.build_wall()
    print(MASTER_MAP.wall)
    test_tree = RRT(MASTER_MAP.car_init_pose,MASTER_MAP.dest_pose,MASTER_MAP)
    print("Constructed",test_tree.construct_RRT(50))
    pth = test_tree.find_path()
    print("Path",pth)
    to_pub = markerize_points("map",np.squeeze(test_tree.explored_coords).tolist(),"red",(0.1,0.1,0.1))
    to_pub_path = markerize_points("map",[MASTER_MAP.car_init_pose,MASTER_MAP.dest_pose],"green",(0.4,0.4,0.4))
    to_put_search = markerize_points("map",list(test_tree.vis_frontier),"white",(0.2,0.2,0.2))
    to_pub_wall = markerize_points("map",MASTER_MAP.wall,"black",(0.6,0.6,0.6))
    while not rospy.is_shutdown():
        path_pub.publish(to_pub)
        angle_pub.publish(to_pub_path)
        waypoint_pub.publish(to_put_search)
        wall_pub.publish(to_pub_wall)
    # path_vis = []
    # for i in PATH:
    #     path_vis.append(MASTER_MAP.grid_to_coord(i))
    # to_pub = markerize_path("map", path_vis)
    # # Listen for transformation boardcast
    # listener = TransformListener()
    # listener.waitForTransform(MAP_FRAME, CAR_FRAME, rospy.Time(), rospy.Duration(4.0))
    # # Subscribe for the angle topic
    # rospy.Subscriber(ANGLE_TOPIC, Float32MultiArray, callback=angle_vis_callback)
    # while not rospy.is_shutdown():
    #     # Publish waypoint path
    #     path_pub.publish(to_pub)
    #     try:
    #         now = rospy.Time.now()
    #         listener.waitForTransform(
    #             MAP_FRAME, CAR_FRAME, rospy.Time(), rospy.Duration(4.0)
    #         )
    #         (car_translation, car_rotation) = listener.lookupTransform(
    #             MAP_FRAME, CAR_FRAME, rospy.Time(0)
    #         )
    #     except (LookupException, ExtrapolationException):
    #         print(angle_path, now)

    #     if angle_path is not None:
    #         # initialize angle_points according to size of input points
    #         angle_points = np.zeros([angle_path.shape[0], angle_path.shape[1] + 1])
    #         for i in range(angle_path.shape[0]):
    #             angle_points[i] = u_tf.tf_point(
    #                 angle_path[i],
    #                 car_translation,
    #                 car_rotation,
    #                 ref=True,
    #                 rot_c=1,
    #                 dim3=False,
    #             )
    #         to_pub_steer = markerize_points("map", angle_points)
    #         angle_pub.publish(to_pub_steer)
