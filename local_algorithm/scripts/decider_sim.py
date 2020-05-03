#!/usr/bin/env python
import rospy
import numpy as np
import time
import sys
import json
import cv2
from local_alg import local_alg
from parser import laser_parser
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import LaserScan

points=[]
costs=[]
indices=[]
count=0

def callback(data, IO):
    angle = IO[0].decide_direction(laser_parser(data))[0]
    IO[1].publish(AckermannDrive(steering_angle=angle, speed=0.5))

def callback_vis(data, IO):
    IO[2]+=1
    if(not IO[2]%10==0):
        return
    cur_points = laser_parser(data)
    #tmp is the index of the best path
    [angle, tmp, cur_costs] = IO[0].decide_direction(cur_points)
    points.append(cur_points)
    costs.append(cur_costs)
    indices.append(tmp)
    message = AckermannDrive(steering_angle=angle, speed=1)
    IO[1].publish(message)
    #IO[1].publish(AckermannDrive(steering_angle=angle, speed=0.25))
    #for i in range(10):
    #    IO[1].publish(message)
    #    time.sleep(0.01)
    print(IO[2])
    print(IO[0].angles)

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
    h = int(hori_size/vis_resolution)+1

    points[:,0] = w/2-points[:,0]-1
    points[:,1] = h-points[:,1]-1
    return points

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
        #Actually output the frame
        video_out.write(frame)
    video_out.release()

def handle(visualize):
    rospy.init_node('local_algorithm')
    decider = local_alg('./config.json')
    decider.generate_paths()
    #announcer = rospy.Publisher('/car_1/command', AckermannDrive, queue_size=2)
    announcer = rospy.Publisher('/car_1/multiplexer/command', AckermannDrive, queue_size=2)
    if(visualize):
        rospy.Subscriber('/car_1/scan', LaserScan, callback_vis, [decider, announcer, visualize, count])
        rospy.on_shutdown(output_video)
    else:
        rospy.Subscriber('/car_1/scan', LaserScan, callback, [decider, announcer])
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
