import numpy as np

class car_state:
    def __init__():
        self.timesteps
        self.x
        self.y
        self.theta
        self.slip_angle
        self.velocity
        self.steer_angle
        #Config constants
        self.time_delta = 0.01 #f110_env.py
        #racecar.py
        self.max_speed = 20.0;
        self.max_accel = 9.51;
        self.max_decel = 13.26;
        self.max_steering_vel = 3.2;
        self.max_steering_angle = 0.4189;
        self.wheelbase = 0.3302
        #st_kinematics.cpp
        self.threshold = 0.5
        self.g = 9.81
        #cout extraction
        self.h_cg = 0.074
        self.l_r = 0.17145
        self.l_f = self.wheelbase - self.l_r
        self.mu = 0.523 #friction coefficient
        self.i_z = 0.04712
        self.mass = 3.74
        self.cs_f = 4.718
        self.cs_r = 5.4562

    def predict_state(angles, velocities):
        '''
            angles is the desired steering angles
            velocities is the desired velocities
        '''
        if(not angles.shape==velocities.shape):
            print(9/0)
        x = np.zeros(angles.shape)
        x += self.x
        y = np.zeros(angles.shape)
        y += self.y
        velocity = np.zeros(angles.shape)
        velocity += self.velocity
        theta = np.zeros(angles.shape)
        theta += self.theta
        steer_angle = np.zeros(angles.shape)
        steer_angle += self.steer_angle
        accel = np.zeros(angles.shape)
        angular_vel = np.zeros(angles.shape)
        slip_angle = np.zeros(angles.shape)
        slip_angle += self.slip_angle
        for i in range(self.timesteps):
            #Decide on the steering angle velocity,
            #the steering angle, the acceleration,
            #and the velocity
            diff = velocities - velocity
            velocity_positive = np.greater(velocity,0)
            diff_positive = np.greater(diff, 0)
            tmp = np.logical_and(velocity_positive, diff_positive)
            #Forward and accelerating
            accel[tmp] = 2.0*self.max_accel*diff/self.max_speed
            #Backward and accelerating
            tmp = np.logical_and(np.logical_not(velocity_positive), np.logical_not(diff_positive))
            accel[tmp] = 2.0*self.max_accel*diff/self.max_speed
            #Clip acceleration to the maximum limit
            np.clip(accel,-self.max_accel,self.max_accel)
            #Backward and decelerating
            tmp = np.logical_and(np.logical_not(velocity_positive), diff_positive)
            accel[tmp] = self.max_decel
            #Forward and decelerating
            tmp = np.logical_and(velocity_positive, np.logical_not(diff_positive))
            accel[tmp] = -self.max_decel

            #Decide on the steering angle velocity
            angle_vel = np.zeros(angles.shape)
            diff = angles - steer_angle
            positive = np.greater(diff, 0.0001)
            negative = np.less(diff, -0.0001)
            angle_vel[positive] = self.max_steering_vel
            angle_vel[negative] = -self.max_steering_vel

            #update_k
            #May want to implement the flip-flop avoidance
            #found in the actual code
            k = np.less(velocity, self.threshold)
            velocity_k = velocity[k]
            theta_k = theta[k]
            accel_k = accel[k]
            steer_angle_k = steer_angle[k]
            angle_vel_k = angle_vel[k]
            x_dot = np.multiply(velocity_k, np.cos(theta_k))
            y_dot = np.multiply(velocity_k, np.sin(theta_k))
            theta_dot = np.multiply(velocity_k/self.wheelbase, np.tan(steer_angle_k))
            theta_double_dot = np.multiply(accel_k/self.wheelbase, np.tan(steer_angle_k)) + np.divide(np.multiply(velocity_k,angle_vel_k), self.wheelbase*np.square(np.cos(steer_angle_k)))
            #update_k on the actual positional values
            x[k] += x_dot*self.time_delta
            y[k] += y_dot*self.time_delta
            theta[k] += theta_dot*self.time_delta
            velocity[k] += accek_k*self.time_delta
            steer_angle[k] += angle_vel_k*self.time_delta
            angular_vel[k] += theta_double_dot*self.time_delta

            #update with st dynamics
            nk = np.logical_not(k)
            velocity_nk = velocity[nk]
            theta_nk = theta[nk]
            accel_nk = accel[nk]
            steer_angle_nk = steer_angle[nk]
            angel_vel_nk = angle_vel[nk]
            slip_angle_nk = slip_angle[nk]
            angular_vel_nk = angular_vel[nk]

            x_dot = np.multiply(velocity_nk, np.cos(theta_nk+slip_angle_nk))
            y_dot = np.multiply(velocity_nk, np.sin(theta_nk+slip_angle_nk))

            vel_ratio = np.divide(angular_vel_nk,velocity_nk)
            first_term = self.mu/self.wheelbase/velocity_nk
            rear_val = self.g * self.l_r - accel_nk * self.h_cg
            front_val = self.g * self.l_f + accel_nk * self.h_cg

            theta_double_dot = (self.mu * self.mass / (self.i_z * self.wheelbase)) * (self.l_f * self.cs_f * steer_angle_nk * rear_val + slip_angle_nk * (self.l_r * self.cs_r * front_val - self.l_f * self.cs_f * rear_val)) - vel_ratio * ((self.l_f ** 2) * self.cs_f * rear_val + (self.l_r ** 2) * self.cs_r * front_val)
            slip_angle_dot = np.multiply(first_term, self.cs_f * rear_val * steer_angle_nk - slip_angle_nk * (self.cs_r * front_val + self.cs_f * rear_val)) + vel_ratio * (self.cs_r * self.l_r * front_val - self.cs_f * self.l_f * rear_val) - angular_velocity_nk

            #Update positional variables
            x[nk] += x_dot * self.time_delta
            y[nk] += y_dot * self.time_delta
            theta[nk] += angular_vel_nk * self.time_delta
            velocity[nk] += accel_nk * self.time_delta
            steer_angle[nk] += angel_vel_nk * self.time_delta
            angular_vel[nk] += theta_double_dot * self.time_delta
            slip_angle[nk] += slip_angle_dot * self.time_delta

            #Clip velocity and steering angle values
            np.clip(velocity, -self.max_speed, self.max_speed)
            np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
