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
import json

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
# Load waypoints from config
CONFIG_FILE = "../config.json"
config_file = open(CONFIG_FILE)
configs = json.load(config_file)
config_file.close()
PATH = configs["waypoints"]
print(PATH)
# pass updated angle path for marker publisher
angle_path = None
# Global transformation of the car
car_translation = [0, 0, 0]
car_rotation = [0, 0, 0, 0]


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
    MASTER_MAP.intial_state_listener("/map", "/odom")
    path_vis = []
    for i in PATH:
        path_vis.append(MASTER_MAP.grid_to_coord(i))
    to_pub = markerize_path("map", path_vis)
    while not rospy.is_shutdown():
        path_pub.publish(to_pub)
    # Listen for transformation boardcast
    """
    listener = TransformListener()
    listener.waitForTransform(MAP_FRAME, CAR_FRAME, rospy.Time(), rospy.Duration(4.0))
    # Subscribe for the angle topic
    rospy.Subscriber(ANGLE_TOPIC, Float32MultiArray, callback=angle_vis_callback)
    while not rospy.is_shutdown():
        # Publish waypoint path
        path_pub.publish(to_pub)
        try:
            now = rospy.Time.now()
            listener.waitForTransform(
                MAP_FRAME, CAR_FRAME, rospy.Time(), rospy.Duration(4.0)
            )
            (car_translation, car_rotation) = listener.lookupTransform(
                MAP_FRAME, CAR_FRAME, rospy.Time(0)
            )
        except (LookupException, ExtrapolationException):
            print(angle_path, now)

        if angle_path is not None:
            # initialize angle_points according to size of input points
            angle_points = np.zeros([angle_path.shape[0], angle_path.shape[1] + 1])
            for i in range(angle_path.shape[0]):
                angle_points[i] = u_tf.tf_point(
                    angle_path[i],
                    car_translation,
                    car_rotation,
                    ref=True,
                    rot_c=1,
                    dim3=False,
                )
            to_pub_steer = markerize_points("map", angle_points)
            angle_pub.publish(to_pub_steer)
    """
