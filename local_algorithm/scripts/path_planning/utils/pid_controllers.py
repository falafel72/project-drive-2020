from __future__ import print_function
import numpy as np
import math
import rospy
import json
import time
from .transform import tf_point

# hyperparameters:
DIST_THRESH = 0.5
I_MAX = math.pi / 8


class PIDController:
    def __init__(self, k, time, wp):
        """ PID controller

        Args:
            k (array-like): the (kp,ki,kd) coeeficients of PID
            time (float): the current ros time
            wp (array-like): the waypoints to be followed
        """
        # PID constants
        self.p = k[0]
        self.i = k[1]
        self.d = k[2]
        # last saved error, used for integral and diravitive
        self.e_last = 0.0
        # last saved integral component
        self.i_last = 0.0
        # state of the car, a tuple of (ct_error, left or right or traj)
        self.car_state = None
        # last saved time, used as delta t
        self.time = time
        # the list of waypoints to be followed
        self.wps = wp
        # indecies of the current two waypoints that forms the current trajectory
        self.current_wps = [len(self.wps) - 1, 0]

    def update_cross_track_error(self, cur_position):
        """ Calculate the distance between the current position and the trajectories
            marked by the waypoints. Also calculate cross product to determine 
            whether the point is to the left or right of the trajectory point

        Args:
            cur_position (array-like): current car position (x,y,z)
            wp1 (array-like): first waypoint, as origin of trajectory
            wp2 (array-like): second waypoint, as destination of trajectory

        Returns:
            tuple: (cross track error, to the left or not)
        """
        # check if the current direction the car is heading is to the right of
        # the desired trajectory
        to_right = False
        # turn positions of car and waypoints to vectors
        cur_vector = np.array(cur_position)
        wp1_vector = np.array(self.wps[self.current_wps[0]])
        wp2_vector = np.array(self.wps[self.current_wps[1]])
        # calculate cross product of the vectors
        # if z component is positive, current moving direction of the car is
        # to the left of the desired trajectory, negative otherwise.
        point_vector = cur_vector - wp1_vector
        traj_vector = wp2_vector - wp1_vector
        cross = np.cross(traj_vector, point_vector)
        if cross[2] > 0:
            to_left = True
        # calculate the distance between the car and the trajectory as error
        self.car_state = (
            abs(
                (wp2_vector[1] - wp1_vector[1]) * cur_position[0]
                - (wp2_vector[0] - wp1_vector[0]) * cur_position[1]
                + wp2_vector[0] * wp1_vector[1]
                - wp2_vector[1] * wp1_vector[0]
            )
            / math.sqrt(
                (wp2_vector[1] - wp1_vector[1]) ** 2
                + (wp2_vector[0] - wp1_vector[0]) ** 2
            ),
            to_right,
        )

    def steer_angle_velocity(self, curr_pose, curr_rotation, curr_time):
        """ Calculate the steering angle necessary for the car to be on the desired
            trajectory. Update the error and time value of this object
        Args:
            curr_pose (array-like): the current car position in (x,y,z)\\
            curr_rotation (array-like): the current direction of the car in
            quaternion (x,y,z,w)\\
            curr_time (float): the current_time stamp\\
        Returns:
            float: the current steering angle
        """
        # calculate the current cross track error
        self.update_cross_track_error(curr_pose)
        time_diff = curr_time - self.time
        self.time = curr_time
        # proportional term
        p_term = self.car_state[0] * self.p
        # integral term
        i_term = self.i_last + (self.car_state[0]) * time_diff * self.i
        if i_term >= I_MAX:
            i_term = I_MAX
        # save the i_term for future calculation
        self.i_last = i_term
        # differential term
        d_term = (self.car_state[0] - self.e_last) / time_diff * self.d
        self.e_last = self.car_state[0]
        velocity = p_term + i_term + d_term
        # check if the waypoint is reached
        self.update_current_wps(curr_pose)
        # negative is to the right, positive to the left
        # turn according to car state
        if self.car_state[1]:
            return -velocity
        else:
            return velocity

    def update_current_wps(self, cur_pose):
        dist = math.sqrt(
            (self.wps[self.current_wps[1]][0] - cur_pose[0]) ** 2
            + (self.wps[self.current_wps[1]][1] - cur_pose[1]) ** 2
        )
        if dist <= DIST_THRESH:
            self.current_wps[0] = self.current_wps[1]
            self.current_wps[1] = (self.current_wps[1] + 1) % len(self.wps)
