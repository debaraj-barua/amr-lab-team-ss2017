#!/usr/bin/env python

PACKAGE = 'amr_navigation'

from math import atan2, copysign
from velocity_controller import VelocityController, Velocity
from velocity_controller import get_shortest_angle, get_distance
import numpy as np
import rospy

class OmniVelocityController(VelocityController):
    """
    An implementation of velocity controller that drives the robot in all directions.

    The base is assumed to have 3 degrees of freedom. The controller directs the
    robot along the straight line connecting the current and target pose and combines linear
    and angular motion.

    The robot drives at a constant (max) velocity until it is close to the goal pose, 
    then it de accelerates and stops at the goal with zero velocity.
    """

    def __init__(self, l_max_vel, l_tolerance, l_max_acc, a_max_vel, a_tolerance, a_max_acc):
        self._l_max_vel = l_max_vel
        self._l_max_acc = l_max_acc
        self._l_tolerance = l_tolerance

        self._a_max_vel = a_max_vel
        self._a_max_acc = a_max_acc
        self._a_tolerance = a_tolerance

    def compute_velocity(self, actual_pose):
        # compute remaining distances
        linear_dist = get_distance(self._target_pose, actual_pose)
        angular_dist = get_shortest_angle(self._target_pose.theta, actual_pose.theta)

        if (abs(linear_dist) < self._l_tolerance and
                    abs(angular_dist) < self._a_tolerance):
            self._linear_complete = True
            self._angular_complete = True
            return Velocity()

        # Calculate linear distance from goal where the robot should start to de-accelerate
        l_time_for_de_acceleration = self._l_max_vel / self._l_max_acc
        l_distance_for_de_acceleration = 0.5 * self._l_max_acc * (l_time_for_de_acceleration ** 2)

        # Time required based on max velocity and de-acceleration
        linear_time = abs(linear_dist) / self._l_max_vel if \
            linear_dist > l_distance_for_de_acceleration else l_time_for_de_acceleration

        # Get position with respect to the bot. The angle is given with respect to the frame
        theta = actual_pose.theta * -1
        multiplication_matrix = np.array([[np.cos(theta), -np.sin(theta),0], [np.sin(theta), np.cos(theta),0], [0,0,1]])

        target_pos_bot = multiplication_matrix.dot(np.array([self._target_pose.x, self._target_pose.y,1]))
        target_pos_bot_x, target_pos_bot_y = target_pos_bot[0], target_pos_bot[1]

        current_pos_bot = multiplication_matrix.dot(np.array([actual_pose.x, actual_pose.y,1]))
        current_pos_bot_x, current_pos_bot_y = current_pos_bot[0], current_pos_bot[1]

        dx_bot = target_pos_bot_x - current_pos_bot_x
        dy_bot = target_pos_bot_y - current_pos_bot_y

        linear_vel_x = dx_bot / linear_time
        linear_vel_y = dy_bot/linear_time


        # Calculate angular distance from goal where the robot should start to de-accelerate
        a_time_for_de_acceleration = self._a_max_vel / self._a_max_acc
        a_distance_for_de_acceleration = 0.5 * self._a_max_acc * (a_time_for_de_acceleration ** 2)

        angular_time = abs(angular_dist) / self._a_max_vel if \
            angular_dist > a_distance_for_de_acceleration else a_time_for_de_acceleration

        angular_vel = angular_dist / angular_time

        return Velocity(linear_vel_x,linear_vel_y,angular_vel)