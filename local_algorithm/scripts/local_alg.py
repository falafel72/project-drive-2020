#!/usr/bin/env python
from os import path
from distance_funcs import *
import json
import math
import numpy as np

class local_alg:
    def __init__(self, config_file):
        #Loads configuration json file
        if(not path.exists(config_file)):
            print("The config file does not exist")
            raise ValueError
        config_tmp = open(config_file)
        configs = json.load(config_tmp)
        try:
            self.candidate_rs = configs['radius']
            self.num_steps = configs['num_steps']
            self.step_size = configs['step_size']
            self.dis_exp = configs['dis_exp']
            self.length_exp = configs['length_exp']
            self.dis_threshold = configs['dis_threshold']
            self.wheel_base = configs['wheel_base']
        except KeyError as e:
            print("The config file is incomplete, missing "+e.args[0])
            raise e
        if(('hori_size' in configs) and ('vert_size' in configs)):
            self.sim_flag = True
            self.hori_size = configs['hori_size']
            self.vert_size = configs['vert_size']
        else:
            self.sim_flag = False
        #Translate the radius into steering angles
        self.angles = []
        for i in range(len(self.candidate_rs)):
            if(not self.candidate_rs[i]==0):
                angle = math.asin(self.wheel_base/2/self.candidate_rs[i])
            else:
                angle = 0
            self.angles.append(angle)
        #Gets the importance array, which gives importance based on
        #how far each point is from the car
        self.length_weights = length_weight(self.num_steps, self.length_exp)
        print(self.length_weights)
        print(sum(self.length_weights))

    def generate_paths(self):
        #Generates an array of points.
        #Each row is a path the car could take,
        #and the row number is the number of steps
        #into the future.
        num_candidates = len(self.candidate_rs)
        self.paths = np.zeros((num_candidates,self.num_steps,2))
        #The last dimension is for the x and y coordinates
        for i in range(num_candidates):
            if(self.candidate_rs[i]==0):
                #Special case for straight-ahead
                #All x stays 0, only y changes
                for k in range(self.num_steps):
                    self.paths[i,k,1] = k*self.step_size
                continue

            #Use the trig description of an arc to get the path
            center = self.candidate_rs[i]
            cur_radius = abs(self.candidate_rs[i])
            tot_angle = self.num_steps*self.step_size/cur_radius
            if(center<0):
                init_angle = 0
                mult = 1
            else:
                init_angle = math.pi
                mult = -1
            for k in range(self.num_steps):
                cur_angle = init_angle+tot_angle*k*mult/self.num_steps
                self.paths[i,k,0] = center + math.cos(cur_angle)*cur_radius
                self.paths[i,k,1] = math.sin(cur_angle)*cur_radius

    def decide_direction(self, points):
        #Points is the list of obstacle points
        num_candidates = len(self.candidate_rs)
        costs = np.zeros(num_candidates)
        if(self.sim_flag):
            tmp_points = np.zeros(points.shape)
            k=0
            for i in range(points.shape[0]):
                if((points[i,0]>(-self.hori_size)) and (points[i,0]<self.hori_size)
                        and (points[i,1]>0) and (points[i,1]<self.vert_size)):
                    tmp_points[k,:] = points[i,:]
                    k+=1
            points = tmp_points[:k,:]
        for i in range(num_candidates):
            #Each path
            for k in range(self.num_steps):
                #Each point in the path
                costs[i] += sum(distance_score(np.sqrt(np.square(\
                    points[:,0]-self.paths[i,k,0])+np.square(\
                    points[:,1]-self.paths[i,k,1])),self.dis_exp,\
                    self.dis_threshold))\
                    *self.length_weights[k]
        return [self.angles[np.argmin(costs)], np.argmin(costs), costs]

if __name__ == "__main__":
    decider = local_alg('./config.json')
    decider.generate_paths()

    #Generate an example points array to test the decider
    #This is a straight path ahead
    num_points = 100
    interval = 0.1
    width = 0.5
    points = np.zeros((2*num_points,2))
    for k in range(num_points):
        points[k,0] = -width
        points[k,1] = k*interval
        points[k+num_points,0] = width
        points[k+num_points,1] = points[k,1]
    costs = decider.decide_direction(points)
    print(costs[1])
