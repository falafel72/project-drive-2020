#!/usr/bin/env python
import rospy
import numpy as np
from local_alg import local_alg
from std_msgs.msg import Int8
from pcl_python_helper.msg import PointArray2d


def callback(data, IO):
    # IO[1].publish(IO[0].decide_direction(data.data)[0])
    # print(type(data.x))
    stacked_points = np.stack([np.asarray(data.x), np.asarray(data.y)], -1)
    IO[1].publish(IO[0].decide_direction(stacked_points)[0])


def handle():
    rospy.init_node("local_algorithm")
    decider = local_alg("./config.json")
    decider.generate_paths()
    announcer = rospy.Publisher("angle", Int8, queue_size=10)
    rospy.Subscriber("points", PointArray2d, callback, [decider, announcer])
    rospy.spin()


if __name__ == "__main__":
    handle()
