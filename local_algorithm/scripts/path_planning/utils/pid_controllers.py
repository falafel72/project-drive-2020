from __future__ import print_function
import numpy as np
import math
import rospy
import json
import time
from .transform import tf_point

# hyperparameters:
# used to move the passing line along the trajectory vector
DIST_CONSTANT = 4.0
# Maximum value for the integral term of pid
I_MAX = math.pi / 6
# the biggest angle that an be considered not steep
STEEP_ANGLE_THRESH = 5 * math.pi / 8
# constant to enlongate time for turning on turning assist
TA_DURATION_CONST = 40


class PIDController:
    def __init__(self, max_speed, max_angle, k, time, wp):
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
        self.max_speed = max_speed
        self.max_angle = max_angle
        self.turning_angle = 0.0
        # last calculated cross product between line and position vector
        self.last_cross = None
        # flag of sign change, used to indicate crossing a line
        self.sign_changed = False
        # flag for skipping one frame of cross_product
        self.skip_one_cross_flag = True
        # the slope of the current trajectory
        self.traj_slope = 0.0
        # the point on the trajectory that is used to mark if the car has reached wp
        self.dist_displaced_wp = []
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
        # angle of the next turn, default to pi, which is no turn
        self.next_turn_angle = math.pi
        # direction of next turn: 1 to left, -1 to right, 0 for no relative turn
        self.next_turn_dir = 0
        # the elapse time that turning assist has been on for
        self.ta_elapsed = 0.0
        # the approximate time that turning assist should be on for next turn
        self.approx_ta_time = 0.0
        # flag for assisting the turning over very steep angles. If true, the
        # steering angle will be forced to be the maximum or minimum angle
        self.turning_assist_on = False
        # set up initial trajectory and passing lines
        self.upon_change_wp()

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
        if cross[2] < 0:
            to_right = True
        # calculate the distance between the car and the trajectory as error
        error = abs(
            (wp2_vector[1] - wp1_vector[1]) * cur_position[0]
            - (wp2_vector[0] - wp1_vector[0]) * cur_position[1]
            + wp2_vector[0] * wp1_vector[1]
            - wp2_vector[1] * wp1_vector[0]
        ) / math.sqrt(
            (wp2_vector[1] - wp1_vector[1]) ** 2 + (wp2_vector[0] - wp1_vector[0]) ** 2
        )
        self.car_state = (
            error,
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
        # print(self.current_wps)
        time_diff = curr_time - self.time
        if self.turning_assist_on:
            self.ta_elapsed += time_diff
            if self.ta_elapsed > self.approx_ta_time:
                self.turning_assist_on = False
            self.turning_angle = self.max_angle * self.next_turn_dir
        else:
            # calculate the current cross track error
            self.update_cross_track_error(curr_pose)
            self.time = curr_time
            # proportional term, modified to sqrt rather than linear
            p_term = math.pow(self.car_state[0] * self.p, 1.0 / 3)
            # integral term
            i_term = self.i_last + (self.car_state[0]) * time_diff * self.i
            if i_term >= I_MAX:
                i_term = I_MAX
            # save the i_term for future calculation
            self.i_last = i_term
            # differential term
            d_term = (self.car_state[0] - self.e_last) / time_diff * self.d
            self.e_last = self.car_state[0]
            angle = p_term + i_term + d_term
            # check if the waypoint is reached
            self.update_current_wps(curr_pose, dist=False)
            # negative is to the right, positive to the left
            # turn according to car state
            # print("P:",p_term, "I:",i_term,"D:",d_term)
            # print("-"*20)
            if self.car_state[1]:
                self.turning_angle = angle
            else:
                self.turning_angle = -angle

    def upon_change_wp(self, dist=True):
        """ this function updates the slope of the line to be crossed by the car 
            and moved the line to be crossed by car towards the car. The distance
            the car is moved is positively proportional to the angle of the turn.

        Args:
            dist (bool, optional): [description]. Defaults to True.
        """
        self.ta_elapsed = 0.0
        if self.next_turn_angle < STEEP_ANGLE_THRESH:
            self.turning_assist_on = True
        self.get_turning_angle()
        # the slope of the trajectory line
        line_slope = (
            self.wps[self.current_wps[1]][1] - self.wps[self.current_wps[0]][1]
        ) / (self.wps[self.current_wps[1]][0] - self.wps[self.current_wps[0]][0])
        # the slope of the line to be crossed by the car
        self.traj_slope = -1 / line_slope
        moved_x = 0.0
        moved_dist = math.sqrt((moved_x * line_slope) ** 2 + moved_x ** 2)
        """
        calculate a point on the line that is a certain distance away from the 
        desitnation of the trajectory to allow early respones for steep angles.
        Currently modeled by y = sqrt(1/x)-sqrt(1/pi)
        """
        next_thresh = DIST_CONSTANT * (
            math.sqrt(1 / self.next_turn_angle) - math.sqrt(1 / math.pi)
        )
        while moved_dist < next_thresh:
            moved_x += 0.05
            moved_dist = math.sqrt((moved_x * line_slope) ** 2 + moved_x ** 2)
        # approximate the time that TA should be on with the time to travel the
        # linear distance of moved_dist with maximum speed.
        self.approx_ta_time = moved_dist / self.max_speed * TA_DURATION_CONST
        print("approx_time status", self.current_wps, moved_dist, self.approx_ta_time)
        # move the variable on x-axis according to the positioning of the origin
        # waypoint and destination waypoint of the trajectory
        if self.wps[self.current_wps[1]][0] > self.wps[self.current_wps[0]][0]:
            moved_x = -moved_x
        self.dist_displaced_wp = np.array(
            [
                self.wps[self.current_wps[1]][0] + moved_x,
                self.wps[self.current_wps[1]][1] + moved_x * line_slope,
                0.0,
            ]
        )
        # print("displaced",self.dist_displaced_wp)

    def get_turning_angle(self):
        """ this is a helper function that finds the angle between the vector 
            representing the current trajectory and the vector representing the 
            next trajectory.Uses dot product to find angles between the two vectors. 
        """
        current = [self.wps[self.current_wps[0]], self.wps[self.current_wps[1]]]
        dest = [
            self.wps[self.current_wps[1]],
            self.wps[(self.current_wps[1] + 1) % len(self.wps)],
        ]
        cur_vector = np.array(np.array(current[0]) - np.array(current[1]))
        dest_vector = np.array(np.array(dest[1]) - np.array(dest[0]))
        dot_product = np.dot(cur_vector, dest_vector)
        mag_cur = np.linalg.norm(cur_vector)
        mag_dest = np.linalg.norm(dest_vector)
        angle = math.acos(dot_product / (mag_cur * mag_dest))
        self.next_turn_angle = angle
        c_product = np.cross(cur_vector, dest_vector)
        if c_product[2] > 0:
            self.next_turn_dir = -1
        elif c_product[2] < 0:
            self.next_turn_dir = 1
        else:
            self.next_turn_dir = 0

    def update_current_wps(self, cur_pose, dist=False):
        """ Check if the destination waypoint is reached, either by distance between
            the current car position to the waypoint or whether the car position 
            has passed the line that is perpendicular to the trajectary and passes
            through the destination waypoint

        Args:
            cur_pose (array-like): the current position of car (x,y,z)\\
            dist (bool, optional): True if check by distance, False if check by 
            passing the line. Defaults to False.\\
        """
        if not dist:
            dist = math.sqrt(
                (self.wps[self.current_wps[1]][0] - cur_pose[0]) ** 2
                + (self.wps[self.current_wps[1]][1] - cur_pose[1]) ** 2
            )
            if dist <= DIST_CONSTANT:
                self.current_wps[0] = self.current_wps[1]
                self.current_wps[1] = (self.current_wps[1] + 1) % len(self.wps)
        else:
            line_vector = np.array([1, self.traj_slope, 0.0])
            pose_vector = np.array(
                [
                    cur_pose[0] - self.dist_displaced_wp[0],
                    cur_pose[1] - self.dist_displaced_wp[1],
                    0.0,
                ]
            )
            cross_product = np.cross(line_vector, pose_vector)
            if self.skip_one_cross_flag:
                self.last_cross = cross_product[2]
                self.skip_one_cross_flag = False
            else:
                self.sign_changed = not (
                    np.sign(self.last_cross) == np.sign(cross_product[2])
                )
                self.last_cross = cross_product[2]
            if self.sign_changed:
                self.skip_one_cross_flag = True
                self.sign_changed = False
                print(self.wps[self.current_wps[0]], self.wps[self.current_wps[1]])
                # print("pose",pose_vector,"line",line_vector)
                self.current_wps[0] = self.current_wps[1]
                self.current_wps[1] = (self.current_wps[1] + 1) % len(self.wps)
                self.upon_change_wp()
