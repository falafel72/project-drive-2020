#!/usr/bin/env python
import rospy
import numpy as np
import time
import sys
import json
import cv2
import math
from local_alg import local_alg
from parser import laser_parser
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import *

CONFIG_FILE = "./config.json"
CONTROL_TOPIC = "/drive"
LASER_TOPIC = "/scan"
ODOM_TOPIC = "/odom"
MAP_TOPIC = "/map"
PATH_TOPIC = "/green_path"
MAX_SPEED = 20
ACCEL = 15

points=[]
relative_waypoints=[]
costs=[]
indices=[]
count=0
newest_pos = np.zeros(3)

#Read the config file to obtain the radii
config_file = open(CONFIG_FILE)
configs = json.load(config_file)
config_file.close()

"""
    Same as callback_vis, but does not
    save points for visualization
"""
def callback(data, IO):
    IO[2]+=1
    if(not IO[2]%10==0):
        return
    cur_points = laser_parser(data)
    #index is the index of the best path
    #tmp is the waypoint visualization
    #this is not used here
    [angle, index, cur_costs, tmp] = IO[0].decide_direction(cur_points, IO[3])
    message = AckermannDriveStamped()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = "No visualization"
    message.drive.steering_angle = angle
    #decide on the speed
    if(angle == 0):
        message.drive.speed = MAX_SPEED
    else:
        message.drive.speed = math.sqrt(abs(configs["radius"][index])*ACCEL)
    IO[1].publish(message)
    publish_points(IO[0].paths[index,:,:], IO[4])

"""
    The callback for the /scan channel.
    data contains the laser scan sent by ROS,
    and IO contains extra arguments
    This calls local_alg and publishes
    the desired steering angle and speed to /drive
    Saves the laser scan and waypoints
"""
def callback_vis(data, IO):
    IO[2]+=1
    if(not IO[2]%10==0):
        return
    cur_points = laser_parser(data)
    #index is the index of the best path
    [angle, index, cur_costs, waypoint] = IO[0].decide_direction(cur_points, IO[3])
    #Save the laser scan points for visualization
    points.append(cur_points)
    costs.append(cur_costs)
    indices.append(index)
    relative_waypoints.append(waypoint)
    message = AckermannDriveStamped()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = "Visualized"
    message.drive.steering_angle = angle
    #decide on the speed
    if(angle == 0):
        message.drive.speed = MAX_SPEED
    else:
        message.drive.speed = math.sqrt(abs(configs["radius"][index])*ACCEL)
    IO[1].publish(message)
    publish_points(IO[0].paths[index,:,:], IO[4])

"""
    Export the points on the chosen path
    by publishing them to a chosen topic
"""
def publish_points(points, publisher):
    dims = [MultiArrayDimension(), MultiArrayDimension()]
    dims[0].label = "point_index"
    dims[0].size = points.shape[0]
    dims[0].stride = dims[0].size*2
    dims[1].label = "x_y"
    dims[1].size = 2
    dims[1].stride = 2
    layout = MultiArrayLayout()
    layout.dim = dims
    layout.data_offset = 0
    msg = Float32MultiArray()
    msg.layout = layout
    msg.data = np.reshape(points.astype('float32'), -1)
    publisher.publish(msg)

"""
    Translate point locations to pixel locations
    This is called for output_video,
    to translate the points into the visualization
"""
def prepare_points(points, hori_size, vert_size, vis_resolution):
    tmp_points = np.zeros(points.shape)
    k = 0
    for i in range(points.shape[0]):
        if ((points[i, 0] > (-hori_size)) and (points[i, 0] < hori_size)
                and (points[i, 1] > 0) and (points[i, 1] < vert_size)):
            tmp_points[k, :] = points[i, :]
            k += 1
    points = tmp_points[:k,:]/vis_resolution
    points = points.astype(int)

    w = int(hori_size*2/vis_resolution)+1
    h = int(vert_size/vis_resolution)+1

    points[:,0] = w/2-points[:,0]-1
    points[:,1] = h-points[:,1]-1
    return points

"""
    Function that is called after halting,
    when the visualize option is given.
    Saves the laser scan points and any saved
    waypoints to a video output.
"""
def output_video():
    print('Video function')
    config_file = './config.json'
    video_output = './decider_video.avi'
    config_file = open(config_file)
    configs = json.load(config_file)
    config_file.close()
    #Calculate the video shape
    w = int(configs['hori_size']*2/configs['vis_resolution'])+1
    h = int(configs['hori_size']/configs['vis_resolution'])+1
    #Preprocess the path points
    decider = local_alg('./config.json')
    decider.generate_paths()
    paths = [prepare_points(path, configs['hori_size'], configs['vert_size'],
                            configs['vis_resolution']) for path in decider.paths]
    #Declare video output
    video_out = cv2.VideoWriter(video_output, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10, (w,h))
    flag = (len(relative_waypoints)>0)
    for i in range(len(points)):
        #Note that the dtype should be uint8 for the resulting the video
        #to look as intended
        frame = np.ones((h,w,3), dtype='uint8')*255
        #Plot the points from the LIDAR as black dots
        points_pixel = prepare_points(points[i], configs['hori_size'], configs['vert_size'],
                                      configs['vis_resolution'])
        frame[points_pixel[:,1],points_pixel[:,0],:] = 0
        #Plot the points from the best path as green dots,
        #and the points from the other paths as gray dots
        for k in range(len(configs['radius'])):
            if(not k==indices[i]):
                frame[paths[k][:,1],paths[k][:,0],:] = 100
            else:
                #frame[paths[k][:,1],paths[k][:,0],[0,2]] = 0
                frame[paths[k][:,1],paths[k][:,0],0] = 0
                frame[paths[k][:,1],paths[k][:,0],2] = 0
        #If any waypoints are present, plot them
        if(flag):
            waypoint = np.zeros((1,2))
            waypoint[0,:] = relative_waypoints[i]
            waypoint = prepare_points(waypoint, configs['hori_size'], configs['vert_size'], configs['vis_resolution'])
            if(waypoint.shape[0]==1):
                frame[waypoint[0,1],waypoint[0,0],1] = 0
        #Actually output the frame
        video_out.write(frame)
    video_out.release()

"""
    Callback to save the newest position and rotation
    published in /odom to newest_pos.
    newest_pos = [x, y, rotation about z axis]
"""
def save_odom(data, newest_pos):
    newest_pos[0] = data.pose.pose.position.x
    newest_pos[1] = data.pose.pose.position.y
    #The third element is the rotation around
    #the z axis
    quaternion = data.pose.pose.orientation
    quaternion = [quaternion.x, quaternion.y,
            quaternion.z, quaternion.w]
    newest_pos[2] = euler_from_quaternion(quaternion)[2]

"""
    Called once at the beginning
    To initialize the different publishers
    and subscribers
"""
def handle(visualize):
    rospy.init_node('local_algorithm')
    decider = local_alg('./config.json')
    decider.generate_paths()
    #announcer = rospy.Publisher('/car_1/command', AckermannDriveStamped, queue_size=2)
    announcer = rospy.Publisher(CONTROL_TOPIC, AckermannDriveStamped, queue_size=2)
    point_export = rospy.Publisher(PATH_TOPIC, Float32MultiArray, queue_size=2)
    if(visualize):
        visualize = 0
        rospy.Subscriber(LASER_TOPIC, LaserScan, callback_vis, [decider, announcer, visualize, newest_pos, point_export])
        #Subscribe to the odom topic and save the newest position
        #whenever one is published
        rospy.Subscriber(ODOM_TOPIC, Odometry, save_odom, newest_pos)
        rospy.on_shutdown(output_video)
    else:
        count = 0
        rospy.Subscriber(LASER_TOPIC, LaserScan, callback, [decider, announcer, count, newest_pos, point_export])
        rospy.Subscriber(ODOM_TOPIC, Odometry, save_odom, newest_pos)
    rospy.spin()
    #if(not visualize==-1):
    #    output_video()

if __name__ == '__main__':
    time.sleep(5)
    if((len(sys.argv)>1) and (sys.argv[1]=='visualize')):
        #Call the handle function and then visualize
        #Generates a video with the scanned points and
        #predicted paths and preferences
        handle(True)
    else:
        handle(False)
