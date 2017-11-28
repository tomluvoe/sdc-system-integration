#!/usr/bin/env python

import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
from math import cos, sin
import numpy as np
import tf

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


def local_coordinates(pose_stamped, waypoints_lane):
    pose = pose_stamped.pose
    waypoints = waypoints_lane.waypoints
    #Use Tensorflow package for Euler's Quaternion transformations
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    #Localized x and y coordinates: 'accurate_x', 'accurate_y'
    accurate_x = []
    accurate_y = []
    localX = pose.position.x
    localY = pose.position.y

    #Relative waypoints (wps) need a reference shift.
    
    rel_wps = len(waypoints)

    for i in range(rel_wps):
        #Finding relative waypoints
        
        offsetX = waypoints[i].pose.pose.position.x - localX
        offsetY = waypoints[i].pose.pose.position.y - localY
        
        #Transform new x and y coordinates using trig
        x_2 = offsetX * cos(0 - yaw) - offsetY * sin(0 - yaw)
        y_2 = offsetX * sin(0 - yaw) + offsetY * cos(0 - yaw)
        accurate_x.append(x_2)
        accurate_y.append(y_2)
    return accurate_x, accurate_y


def initial_cte(pose, waypoints):
    
    #From projects, localize x and y and determine coefficients
    x_localize, y_localize = local_coordinates(pose, waypoints)
    
    #Coefficients (C)
    C = np.polyfit(x_localize, y_localize, 3)
    error = np.polyval(C, 5.0)
    return error


class Controller(object):
    def __init__(self, S):
        self.yaw_controller = YawController(
            S.wheel_base, S.steer_ratio,
            0.1, S.max_lat_accel, S.max_steer_angle)
        
        #Initial paramaters
        self.S = S
        self.driverless_mass = self.S.vehicle_mass + self.S.fuel_capacity * GAS_DENSITY
        self.pid_steer = PID(.6, .0022, .26, -S.max_steer_angle, S.max_steer_angle)
        self.last_cte = 0
        self.p1 = .0001
        self.p2 = .0001


    def control(self, twist_cmd, c_v, t_delta, pose, waypoints):
        #Create cte from waypoints
        cte = initial_cte(pose, waypoints)
        cte_delta = (cte - self.last_cte) / t_delta
        self.last_cte = cte

        # update velocity for linear and angular
        vel_linear = abs(twist_cmd.twist.linear.x)
        vel_linear = vel_linear / (1 + self.p1 * cte * cte + self.p2 * cte_delta * cte_delta)
        vel_anglular = twist_cmd.twist.angular.z
        velocity_error = vel_linear - c_v.twist.linear.x

        #FIXME: DEFINITION OF ACCEL_ANGULAR MISSING
        accel_angular = vel_anglular
        # Steering prediction
        steer = self.yaw_controller.get_steering(vel_linear, accel_angular, c_v.twist.linear.x)
        

        # Steer correction
        #FIXME: INCORRECT NUMBER OF VARIABLES
        steer_correction = self.pid_steer.step(steer, cte_delta * t_delta)
        #steer_correction = self.pid_steer.step(cte, t_delta, steer, self.S.max_steer_angle)
        steer = steer_correction + steer

        # Throttle/brake prediction
        # Acceleration (a)
        a = velocity_error / t_delta

        # Update sensitive to direction
        throttle = 0
        brake = 0

        #positive accel keep under accel limit
        if a > 0:
            a = min(a, self.S.accel_limit)
        
        # Decel must be kept under decel limit
        else:
            a = max(a, self.S.decel_limit)
        torque = self.driverless_mass * a * self.S.wheel_radius

        # accel/decel until limits or ideal state
        if a > 0:
            throttle = min(1, torque / (self.driverless_mass * self.S.accel_limit * self.S.wheel_radius))
        else:
            brake = min(abs(torque), (self.driverless_mass * abs(self.S.decel_limit) * self.S.wheel_radius))

        # if deadband, set throttle and brake to 0 (cruise)
        if abs(a) < self.S.brake_deadband:
            throttle = 0
            brake = 0

        # Return throttle, brake, steer
        return throttle, brake, steer

    # Reset for post-manual control/Stop
    def reset(self):
        self.pid_steer.reset()


# In[ ]:



