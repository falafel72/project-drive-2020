#!/usr/bin/env python
from os import path
from distance_funcs import *
from car_state import *
import json
import math
import numpy as np


class local_alg:
    def __init__(self, config_file):
        # Loads configuration json file
        if not path.exists(config_file):
            print("The config file does not exist")
            raise ValueError
        config_tmp = open(config_file)
        configs = json.load(config_tmp)
        config_tmp.close()
        # Load the config values
        try:
            self.candidate_rs = configs["radius"]
            self.num_steps = configs["num_steps"]
            self.step_size = configs["step_size"]
            self.dis_exp = configs["dis_exp"]
            self.length_exp = configs["length_exp"]
            self.dis_threshold = configs["dis_threshold"]
            self.wheel_base = configs["wheel_base"]
            self.speeds = configs["speeds"]
            self.speeds = np.asarray(self.speeds)
        except KeyError as e:
            print("The config file is incomplete, missing " + e.args[0])
            raise e
        if ("hori_size" in configs) and ("vert_size" in configs):
            self.sim_flag = True
            self.hori_size = configs["hori_size"]
            self.vert_size = configs["vert_size"]
        else:
            self.sim_flag = False
        # Load the navigation waypoints if available
        if ("waypoints" in configs) and ("waypoint_weights" in configs):
            print("Waypoint navigation")
            self.navi_mode = True
            self.waypoints = np.asarray(configs["waypoints"])
            self.waypoints = self.waypoints.astype("float64")
            self.waypoint_weights = np.asarray(configs["waypoint_weights"])
            self.num_waypoints = configs["num_waypoints"]
            self.waypoint_mult = np.asarray(configs["waypoint_multipliers"])
            self.laser_on = False
            self.obstacle_thresh = configs['obstacle_thresh']
            self.obstacle_count = 0
            self.speeds_obstacle = np.asarray(configs['speeds_obstacle'])
            self.speeds_tight = np.asarray(configs['speeds_tight'])
            self.radius_tight = configs['radius_tight']
            self.predict_paths = True
            dir_name = path.abspath(__file__)
            for i in range(3):
                dir_name = path.dirname(dir_name)
            self.map_array = np.loadtxt(path.join(dir_name, configs['map_txt']))
            # The distance at which to switch to the next waypoint
            # May also include distance switching
            self.next_thresh = np.asarray(configs["thresholds"])
            self.cur_waypoint = 0
            self.map_orig = np.asarray(configs["map_orig"])
            self.resolution = configs["map_resolution"]
            for i in range(self.waypoints.shape[0]):
                # tmp = self.waypoints[i,0]
                # self.waypoints[i,0] = self.waypoints[i,1]
                # self.waypoints[i,1] = tmp
                self.waypoints[i, :] *= self.resolution
                self.waypoints[i, :] += self.map_orig
            print(self.waypoints)
        else:
            self.navi_mode = False
        # Translate the radius into steering angles
        self.generate_angles()
        # Initiate the simulator
        self.simulator = car_state()
        # Gets the importance array, which gives importance based on
        # how far each point is from the car
        self.length_weights = length_weight(self.num_steps, self.length_exp)
        print(self.length_weights)
        print(sum(self.length_weights))

    def generate_angles(self):
        self.angles = np.zeros(len(self.candidate_rs))
        for i in range(len(self.candidate_rs)):
            if not self.candidate_rs[i] == 0:
                self.angles[i] = math.asin(self.wheel_base / 2 / self.candidate_rs[i])
            else:
                self.angles[i] = 0

    def generate_paths(self):
        # Generates an array of points.
        # Each row is a path the car could take,
        # and the row number is the number of steps
        # into the future.
        num_candidates = len(self.candidate_rs)
        self.paths = np.zeros((num_candidates, self.num_steps, 2))
        # The last dimension is for the x and y coordinates
        for i in range(num_candidates):
            if self.candidate_rs[i] == 0:
                # Special case for straight-ahead
                # All x stays 0, only y changes
                for k in range(self.num_steps):
                    self.paths[i, k, 1] = k * self.step_size
                continue

            # Use the trig description of an arc to get the path
            center = self.candidate_rs[i]
            cur_radius = abs(self.candidate_rs[i])
            tot_angle = self.num_steps * self.step_size / cur_radius
            if center < 0:
                init_angle = 0
                mult = 1
            else:
                init_angle = math.pi
                mult = -1
            for k in range(self.num_steps):
                cur_angle = init_angle + tot_angle * k * mult / self.num_steps
                self.paths[i, k, 0] = center + math.cos(cur_angle) * cur_radius
                self.paths[i, k, 1] = math.sin(cur_angle) * cur_radius

    # Transforms the point to a coordinate based
    # on the current position
    # position=[x,y,rotation about z axis]
    # The rotation is in radians, counter-clockwise
    # when viewed from above.
    def transform_to_local(self, point, position):
        point = np.copy(point)
        # Translation
        point[:,0] -= position[0]
        point[:,1] -= position[1]
        # Rotate clockwise by angle of position[2]
        rotation_matrix = np.zeros((2, 2))
        rotation_matrix[0, 0] = math.cos(position[2])
        rotation_matrix[0, 1] = math.sin(position[2])
        rotation_matrix[1, 0] = -rotation_matrix[0, 1]
        rotation_matrix[1, 1] = rotation_matrix[0, 0]
        point = rotation_matrix.dot(point.T).T
        point = np.flip(point, axis=1)
        return point

    def transform_to_png(self, points, position):
        points = np.copy(points)
        points = np.flip(points, axis=1)
        # Rotate counter-clockwise by angle of position[2]
        rotation_matrix = np.zeros((2,2))
        rotation_matrix[0, 0] = math.cos(position[2])
        rotation_matrix[0, 1] = -math.sin(position[2])
        rotation_matrix[1, 0] = -rotation_matrix[0, 1]
        rotation_matrix[1, 1] = rotation_matrix[0, 0]
        points = rotation_matrix.dot(points.T).T
        # Translate by the car position
        points[:,0] += position[0]
        points[:,1] += position[1]
        # Translate by the map origin
        points[:,0] -= self.map_orig[0]
        points[:,1] -= self.map_orig[1]
        # Scale by map resolution
        points /= self.resolution
        return points.astype('int')

    def decide_direction(self, points, position):
        # Predict the paths
        num_candidates = len(self.candidate_rs)
        #time_1 = time.time()
        costs = np.zeros(num_candidates)
        if(self.predict_paths):
            self.paths, costs = self.simulator.predict_state(self.angles, self.speeds)
            for i in range(num_candidates):
                self.paths[i,:,:] = self.transform_to_local(self.paths[i,:,:], position)
        #time_2 = time.time()
        #print(time_2 - time_1)
        # Points is the list of obstacle points
        if self.sim_flag:
            tmp_points = np.zeros(points.shape)
            k = 0
            # Log points for visualization
            for i in range(points.shape[0]):
                if (
                    (points[i, 0] > (-self.hori_size))
                    and (points[i, 0] < self.hori_size)
                    and (points[i, 1] > 0)
                    and (points[i, 1] < self.vert_size)
                ):
                    tmp_points[k, :] = points[i, :]
                    k += 1
            points = tmp_points[:k, :]
        if self.navi_mode:
            # Check if the current waypoint is reached
            # switch to next waypoint if necessary
            cur_waypoint = np.zeros((1,2))
            cur_waypoint[0,:] = self.waypoints[self.cur_waypoint]
            relative_waypoint = self.transform_to_local(
                cur_waypoint, position
            )
            distance = np.linalg.norm(relative_waypoint, ord=2)
            if distance < self.next_thresh[self.cur_waypoint]:
                print("Current waypoint: " + str(self.cur_waypoint))
                self.cur_waypoint += 1
                if self.cur_waypoint == len(self.waypoints):
                    self.cur_waypoint = 0
                cur_waypoint[0,:] = self.waypoints[self.cur_waypoint]
                relative_waypoint = self.transform_to_local(
                    cur_waypoint, position
                )
                ## Conduct switching between predict and no-predict
                #if(self.cur_waypoint == 12):
                #    self.predict_paths = False
                #    self.generate_paths()
                #    print('No predict')
                #if(self.cur_waypoint == 17):
                #    self.predict_paths = True
                #    print('Predict')
                # Conduct switching between high speed
                # and tight corners
                if(self.cur_waypoint == 23):
                    self.wheel_base = 1.0
                    tmp = self.speeds
                    self.speeds = self.speeds_tight
                    self.speeds_tight = tmp
                    tmp = self.candidate_rs
                    self.candidate_rs = self.radius_tight
                    self.radius_tight = tmp
                    self.generate_angles()
                if(self.cur_waypoint == 32):
                    self.wheel_base = 0.3
                    tmp = self.speeds_tight
                    self.speeds_tight = self.speeds
                    self.speeds = tmp
                    tmp = self.radius_tight
                    self.radius_tight = self.candidate_rs
                    self.candidate_rs = tmp
                    self.generate_angles()
            # Gather multiple waypoints
            cur_waypoints = np.zeros((self.num_waypoints,2))
            waypoint_indices = []
            k = self.cur_waypoint
            for i in range(self.num_waypoints):
                cur_waypoints[i,:] = self.waypoints[k]
                waypoint_indices.append(k)
                k+=1
                k%=len(self.waypoints)
            cur_waypoints = self.transform_to_local(cur_waypoints, position)
            # Add costs based on minimum distance of
            # each candidate to the next waypoint
            for i in range(num_candidates):
                for k in range(self.num_waypoints):
                    costs[i] += (
                        ((self.paths[i] - cur_waypoints[k,:]) ** 2).sum(axis=1).min()
                        * self.waypoint_weights[waypoint_indices[k]]
                        * self.waypoint_mult[k]
                    )
        if(self.laser_on):
            # Currently, the laser scans are not considered at all
            # when running in waypoint mode. This is temporary.
            for i in range(num_candidates):
                # Each path
                for k in range(self.num_steps):
                    # Each point in the path
                    costs[i] += (
                        sum(
                            distance_score(
                                np.sqrt(
                                    np.square(points[:, 0] - self.paths[i, k, 0])
                                    + np.square(points[:, 1] - self.paths[i, k, 1])
                                ),
                                self.dis_exp,
                                self.dis_threshold,
                            )
                        )
                        * self.length_weights[k]
                        * 15
                    )
        # Add current steering angle to simulator
        self.simulator.steer_angle = self.angles[np.argmin(costs)]
        # Return the relative waypoint for visualization.
        # This is still returned when visualization is not on.
        # Not the most elegant design here
        return [
            self.angles[np.argmin(costs)],
            np.argmin(costs),
            costs,
            cur_waypoints,
            self.paths
        ]

    def check_obstacle(self, points, position):
        points = self.transform_to_png(points, position)
        np.savetxt('mapped_points.txt', points)
        total = 0
        for i in range(points.shape[0]):
            try:
                total += self.map_array[points[i,0],points[i,1]]
            except:
                continue
        if(total > self.obstacle_thresh):
            self.obstacle_count += 1
        else:
            if(self.obstacle_count > 0):
                self.obstacle_count -= 1
        if(self.obstacle_count > 2):
            print(total)
            self.laser_on = True
            self.speeds = self.speeds_obstacle
            self.next_thresh += 2.1
            print('Switched to obstacle mode')


if __name__ == "__main__":
    decider = local_alg("./config.json")
    decider.generate_paths()

    # Generate an example points array to test the decider
    # This is a straight path ahead
    num_points = 100
    interval = 0.1
    width = 0.5
    points = np.zeros((2 * num_points, 2))
    for k in range(num_points):
        points[k, 0] = -width
        points[k, 1] = k * interval
        points[k + num_points, 0] = width
        points[k + num_points, 1] = points[k, 1]
    costs = decider.decide_direction(points)
    print(costs[1])
