from sensor_msgs.msg import LaserScan
import numpy as np
import math


def laser_parser(scan_data):
    laser_points = []
    ranges = scan_data.ranges
    angle_min = scan_data.angle_min
    angle_max = scan_data.angle_max
    angle_increment = scan_data.angle_increment
    for i in range(len(ranges)):
        x = ranges[i] * math.sin(angle_min + i * angle_increment)
        y = ranges[i] * math.cos(angle_min + i * angle_increment)
        laser_points.append([x, y])
    return np.asarray(laser_points)
