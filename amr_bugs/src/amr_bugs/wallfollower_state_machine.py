#!/usr/bin/env python

"""
This module provides a single construct() function which produces a Smach state
machine that implements wallfollowing behavior.

The state machine contains three states:
    * findWall:     initial state - drives until a wall is detected
    * alignWall     aligning state - used to align at convex corners or walls in front of robot 
    * followWall    following state - used to follow a straight wall, robust against sensor noise, curls around concave corners

The constructed state machine has three attached methods:
    * set_ranges(ranges): this function should be called to update the range
                          readings
    * get_twist(): returns a twist message that could be directly passed to the
                   velocity publisher
    * set_config(config): updates the machine userdata with the new config

The constructed state machine is preemptable, i.e. each state checks whether
a preemption is requested and returns 'preempted' if that is the case.
"""

PACKAGE = 'amr_bugs'

import math

import roslib
import rospy

roslib.load_manifest(PACKAGE)
import smach
from preemptable_state import PreemptableState
from math import log
from math import cos
from math import pi
from types import MethodType
from geometry_msgs.msg import Twist

__all__ = ['construct']


def search(ud):
    """
    State machine starts from this state. The state ends when an obstacle is found.
    :param ud: user data
    :return: "found_obstacle" when it finds one
    """
    ud.velocity = (ud.max_forward_velocity, 0, 0)

    if ud.front_min < ud.clearance:
        ud.velocity = (0, 0, 0)
        return "found_obstacle"


def align(ud):
    """
    Align with the wall based on mode
    :param ud: user data
    :return: "aligned" when it is aligned with the wall
    """
    ud.velocity = (0, 0, ud.default_rotational_speed * ud.direction * 5)

    if ud.front_min >= ud.clearance:
        ud.velocity = (0, 0, 0)
        return "aligned"

def follow(ud):
    """
    Follow the wall
    :param ud: user data
    :return: "found_unaligned" when not aligned with the wall
    """
    x_velocity = ud.max_forward_velocity
    y_velocity = 0
    angular_velocity = 0

    if ud.front_min < ud.clearance:
        ud.velocity = (0, 0, 0)
        return "found_unaligned"

    if ud.side_min < ud.clearance:
        y_velocity = ud.max_forward_velocity * ud.direction

    if ud.top_corner > ud.clearance * 2.:
        angular_velocity = ud.default_rotational_speed * ud.direction * -1.

    ud.velocity = (x_velocity, y_velocity, angular_velocity)

def set_ranges(self, ranges):
    """
    This function will be attached to the constructed wallfollower machine.
    Its argument is a list of Range messages as received by a sonar callback. 
    For left hand side wallfollowing, the sensor values are mirrored (sides are swapped).
    """

    self.userdata.ranges = map(lambda s: s.range, ranges)

    self.userdata.front_min = min(ranges[2].range, ranges[3].range, ranges[4].range, ranges[5].range)

    self.userdata.side_min = min(ranges[7].range, ranges[8].range) if self.userdata.direction == 1 \
        else min(ranges[0].range, ranges[15].range)

    self.userdata.top_corner = min(ranges[5].range, ranges[6].range) if self.userdata.direction == 1 \
        else min(ranges[1].range, ranges[2].range)


def get_twist(self):
    """
    This function will be attached to the constructed wallfollower machine.
    It creates a Twist message that could be directly published by a velocity
    publisher. The values for the velocity components are fetched from the
    machine userdata.
    """
    twist = Twist()
    twist.linear.x = self.userdata.velocity[0]
    twist.linear.y = self.userdata.velocity[1]
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = self.userdata.velocity[2]

    # ============================= YOUR CODE HERE =============================
    # Instructions: although this function is implemented, you may need to
    #               slightly tweak it if you decided to handle wallfolllowing
    #               mode in "the smart way".
    # Hint: state machine userdata is accessible in this function as well, for
    #       example you can read the current wallfollowing mode with
    #
    #           self.userdata.mode
    #
    # ==========================================================================

    return twist


def set_config(self, config):
    """
    This function will be attached to the constructed wallfollower machine.
    It updates the relevant fields in the machine userdata.
    Its argument is the config object that comes from ROS dynamic reconfigure
    client. self.userdata.direction sets a velocity sign depending on the mode.
    """
    self.userdata.mode = config['mode']
    self.userdata.clearance = config['clearance']
    if self.userdata.mode == 1:
        self.userdata.direction = 1
    else:
        self.userdata.direction = -1
    return config


def construct():
    sm = smach.StateMachine(outcomes=['preempted'])
    # Attach helper functions
    sm.set_ranges = MethodType(set_ranges, sm, sm.__class__)
    sm.get_twist = MethodType(get_twist, sm, sm.__class__)
    sm.set_config = MethodType(set_config, sm, sm.__class__)
    # Set initial values in userdata
    sm.userdata.velocity = (0, 0, 0)
    sm.userdata.mode = 1
    sm.userdata.clearance = 0.3
    sm.userdata.ranges = None
    sm.userdata.max_forward_velocity = 0.3
    sm.userdata.default_rotational_speed = 0.5
    sm.userdata.direction = 1

    with sm:
        smach.StateMachine.add("SEARCH",
                               PreemptableState(search,
                                                input_keys=["clearance", "max_forward_velocity", "front_min"],
                                                output_keys=["velocity"],
                                                outcomes=["found_obstacle"]),
                               transitions={"found_obstacle": "ALIGN"})

        smach.StateMachine.add("ALIGN",
                               PreemptableState(align,
                                                input_keys=["clearance", "direction",
                                                            "front_min", "default_rotational_speed"],
                                                output_keys=["velocity"],
                                                outcomes=["aligned"]),
                               transitions={"aligned": "FOLLOW"})

        smach.StateMachine.add("FOLLOW",
                               PreemptableState(follow,
                                                input_keys=["clearance", "direction",
                                                            "max_forward_velocity", "default_rotational_speed",
                                                            "front_min", "side_min", "top_corner"],
                                                output_keys=["velocity"],
                                                outcomes=["found_unaligned"]),
                               transitions={"found_unaligned": "ALIGN"})

        pass
    return sm
