#!/usr/bin/env python

PACKAGE = 'amr_navigation'

from math import atan2, copysign
from velocity_controller import VelocityController, Velocity
from velocity_controller import get_shortest_angle, get_distance
import numpy as np
import rospy

class OmniVelocityController(VelocityController):
    """
    A simple implementation of velocity controller that drives the robot as if
    it had a differential drive base.

    The base is assumed to have 2 degrees of freedom, i.e. can mave forwards
    and rotate. The controller tries to orient the robot towards the goal and
    then move it forwards until it is reached.

    The robot drives at a constant (max) velocity until it has almost reached
    the goal pose, then it switches to the minimum velocity.
    """

    def __init__(self, l_max_vel, l_tolerance, a_max_vel, a_tolerance):
        self._l_max_vel = l_max_vel
        self._l_tolerance = l_tolerance
        self._a_max_vel = a_max_vel
        self._a_tolerance = a_tolerance

    def compute_velocity(self, actual_pose):
        # Displacement and orientation to the target in world frame:
        dx = self._target_pose.x - actual_pose.x
        dy = self._target_pose.y - actual_pose.y
        # Step 1: compute remaining distances
        linear_dist = get_distance(self._target_pose, actual_pose)
        angular_dist = get_shortest_angle(self._target_pose.theta, actual_pose.theta)

        min_time = abs(linear_dist) / self._a_max_vel

        # Get position with respect to the bot
        theta = actual_pose.theta * -1
        matrix = np.array([[np.cos(theta), -np.sin(theta),0], [np.sin(theta), np.cos(theta),0], [0,0,1]])

        target_pos_bot = matrix.dot(np.array([self._target_pose.x, self._target_pose.y,1]))
        target_pos_bot_x = target_pos_bot[0]
        target_pos_bot_y = target_pos_bot[1]

        current_pos_bot = matrix.dot(np.array([actual_pose.x, actual_pose.y,1]))
        current_pos_bot_x = current_pos_bot[0]
        current_pos_bot_y = current_pos_bot[1]

        dx_bot = target_pos_bot_x - current_pos_bot_x
        dy_bot = target_pos_bot_y - current_pos_bot_y

        linear_vel_x = dx_bot / min_time
        linear_vel_y = dy_bot/min_time

        angular_vel = angular_dist/min_time

        if (abs(linear_dist) < self._l_tolerance and
                    abs(angular_dist) < self._a_tolerance):
            self._linear_complete = True
            self._angular_complete = True
            return Velocity()

        return Velocity(linear_vel_x,linear_vel_y,angular_vel)

        """
        if (abs(linear_dist) < self._l_tolerance and
                    abs(angular_dist) < self._a_tolerance):
            self._linear_complete = True
            self._angular_complete = True
            return Velocity()

        if abs(linear_dist > self._l_tolerance):
            angular_dist = get_shortest_angle(atan2(dy, dx), actual_pose.theta)
            # We still need to drive to the target, therefore we first need
            # to make sure that we are oriented towards it.

        # Step 2: compute velocities
        linear_vel, angular_vel = 0, 0

        if abs(linear_dist) > self._l_tolerance:
            linear_vel = (self._l_max_vel if abs(linear_dist) > 5 * self._l_tolerance else
                          self._l_tolerance)
        if abs(angular_dist) > self._a_tolerance:
            angular_vel = (self._a_max_vel if abs(angular_dist) > 5 * self._a_tolerance else
                           self._a_tolerance)

        if abs(angular_dist) > self._a_tolerance * 5:
            # We need to rotate a lot, so stand still and rotate with max velocity.
            return Velocity(0, 0, copysign(angular_vel, angular_dist))
        else:
            # We need to rotate just a bit (or do not need at all),
            # so it is fine to combine with linear motion if needed.
            return Velocity(copysign(linear_vel, linear_dist),
                            0,
                            copysign(angular_vel, angular_dist))
        """