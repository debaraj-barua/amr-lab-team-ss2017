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


# =============================== YOUR CODE HERE ===============================
# Instructions: write a function for each state of wallfollower state machine.
#               The function should have exactly one argument (userdata
#               dictionary), which you should use to access the input ranges
#               and to provide the output velocity.
#               The function should have at least one 'return' statement, which
#               returns one of the possible outcomes of the state.
#               The function should not block (i.e. have infinite loops), but
#               rather it should implement just one iteration (check
#               conditions, compute velocity), because it will be called
#               regularly from the state machine.
#
# Hint: below is an example of a state that moves the robot forward until the
#       front sonar readings are less than the desired clearance. It assumes
#       that the corresponding variables ('front_min' and 'clearance') are
#       available in the userdata dictionary.
#
#           def search(ud):
#               if ud.front_min < ud.clearance:
#                   return 'found_obstacle'
#               ud.velocity = (1, 0, 0)
# ==============================================================================

def search(ud):
    front_min = min(ud.ranges[3], ud.ranges[4])
    if front_min < ud.clearance:
        ud.velocity = (0, 0, 0)
        return 'found_obstacle'

    ud.velocity = (0.2, 0, 0)


def align_with_wall(ud):
    clearance_tolerance = 0.1
    front_min = min(ud.ranges[3], ud.ranges[4])

    if front_min < ud.clearance:
        angular_velocity = -0.1 if ud.mode == 0 else 0.1
        ud.velocity = (0, 0, angular_velocity)

        rate = rospy.Rate(100)
        rate.sleep()
        while not rospy.is_shutdown():
            side_sonar_1 = ud.ranges[0] if ud.mode == 0 else ud.ranges[7]
            side_sonar_2 = ud.ranges[15] if ud.mode == 0 else ud.ranges[8]

            rospy.loginfo("{0} {1}".format(side_sonar_1, side_sonar_2))
            if (math.fabs(side_sonar_1 - side_sonar_2) < 1e-3):
                ud.velocity = (0, 0, 0)
                break

            rate.sleep()

        while not rospy.is_shutdown():
            if (ud.mode == 0):
                distance_with_wall = min(ud.ranges[0], ud.ranges[15])

                if (math.fabs(distance_with_wall - ud.clearance) < clearance_tolerance):
                    break

                y_velocity = -0.1 if distance_with_wall > ud.clearance else 0.1
                ud.velocity = (0, y_velocity, 0)

            else:
                distance_with_wall = min(ud.ranges[7], ud.ranges[8])

                if (math.fabs(distance_with_wall - ud.clearance) < clearance_tolerance):
                    break

                y_velocity = 0.1 if distance_with_wall > ud.clearance else -0.1
                ud.velocity = (0, y_velocity, 0)

            rate.sleep()

    return "aligned_with_wall"


def follow_wall(ud):
    clearance_tolerance = 0.1

    front_min = min(ud.ranges[3], ud.ranges[4])
    if front_min < ud.clearance:
        ud.velocity = (0, 0, 0)
        return 'found_corner'

    if ud.mode == 1 and ud.ranges[8] < ud.clearance + clearance_tolerance \
            and ud.ranges[7] > ud.ranges[8] * 2:
        ud.velocity = (0, 0, 0)
        return "found_convex"

    ud.velocity = (0.2, 0, 0)


def align_with_corner(ud):
    front_min = min(ud.ranges[3], ud.ranges[4])
    right_min = min(ud.ranges[7], ud.ranges[8])
    left_min = min(ud.ranges[0], ud.ranges[15])

    if front_min < ud.clearance:
        angular_velocity = 0.1 if left_min > right_min else -0.1
        ud.velocity = (0, 0, angular_velocity)

        rate = rospy.Rate(100)
        rate.sleep()

        while not rospy.is_shutdown():
            rospy.loginfo("{0} {1}".format(ud.ranges[11], ud.ranges[12]))
            if (math.fabs(ud.ranges[11] - ud.ranges[12]) < 1e-3):  # Back sensors will be at right
                ud.velocity = (0, 0, 0)
                break
            rate.sleep()

    return "corner_aligned"


def align(ud):
    if (ud.turn == "left"):
        ud.velocity = (0, 0, .1)

        # rospy.loginfo("{0} {1}".format(ud.right_1, ud.right_2))
        if (ud.front_min > ud.clearance and
                    math.fabs(ud.right_1 - ud.right_2) < 1e-5):
            ud.velocity = (0, 0, 0)
            return "aligned"
    else:
        ud.velocity = (0, 0, -.1)

        # rospy.loginfo("{0} {1}".format(ud.right_1, ud.right_2))
        if (ud.front_min > ud.clearance and
                    math.fabs(ud.left_1 - ud.left_2) < 1e-5):
            ud.velocity = (0, 0, 0)
            return "aligned"


def forward(ud):
    if ud.front_min < ud.clearance:
        ud.velocity = (0, 0, 0)

        if (ud.left > ud.right):
            ud.turn = "left"
        else:
            ud.turn = "right"

        return "found_corner"

    elif ud.mode == 0 and ud.left_1 - ud.left_2 > 1.0:
        ud.turn = "left"
        return "found_corner"

    elif ud.mode == 1 and ud.right_1 - ud.right_2 > 1.0:
        ud.turn = "right"
        return "found_corner"

    y_velocity = 0.

    if ud.mode == 0:
        if ud.left - ud.clearance > 0.1:
            y_velocity = 0.1
        elif ud.clearance - ud.left > -0.1:
            y_velocity = 0.1
    else:
        if ud.right - ud.clearance > 0.1:
            y_velocity = -0.1
        elif ud.clearance - ud.right > 0.1:
            y_velocity = 0.1

    ud.velocity = (0.2, y_velocity, 0)


def set_ranges(self, ranges):
    """
    This function will be attached to the constructed wallfollower machine.
    Its argument is a list of Range messages as received by a sonar callback. 
    For left hand side wallfollowing, the sensor values are mirrored (sides are swapped).
    """

    self.userdata.ranges = map(lambda s: s.range, ranges)


    # ============================= YOUR CODE HERE =============================
    # Instructions: store the ranges from a ROS message into the userdata
    #               dictionary of the state machine.
    #               'ranges' is a list or Range messages (that should be
    #               familiar to you by now). It implies that to access the
    #               actual range reading of, say, sonar number 3, you need to
    #               write:
    #
    #                   ranges[3].range
    #
    #               For example, to create an item called 'front_min', which
    #               contains the minimum between the ranges reported by the two
    #               front sonars, you would write the following:
    #
    #                   self.userdata.front_min = min(ranges[3].range, ranges[4].range)
    #
    # Hint: you can just store the whole array of the range readings, but to
    #       simplify the code in your state functions, you may compute
    #       additional values, e.g. the difference between the reading of the
    #       side sonars, or the minimum of all sonar readings, etc.
    #
    # Hint: you can access all the variables stored in userdata. This includes
    #       the current settings of the wallfollower (that is clearance and the
    #       mode of wallfollowing). Think about how you could make your state
    #       functions independent of wallfollowing mode by smart preprocessing
    #       of the sonar readings.
    # ==========================================================================


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
        smach.StateMachine.add('SEARCH',
                               PreemptableState(search,
                                                input_keys=["ranges", "clearance", "mode"],
                                                output_keys=["velocity"],
                                                outcomes=["found_obstacle"]),
                               transitions={'found_obstacle': 'ALIGN_WITH_WALL'})

        smach.StateMachine.add('ALIGN_WITH_WALL',
                               PreemptableState(align_with_wall,
                                                input_keys=["ranges", "clearance", "mode"],
                                                output_keys=["velocity"],
                                                outcomes=["aligned_with_wall"]),
                               transitions={'aligned_with_wall': 'FOLLOW_WALL'})

        smach.StateMachine.add('FOLLOW_WALL',
                               PreemptableState(follow_wall,
                                                input_keys=["ranges", "clearance", "mode"],
                                                output_keys=["velocity"],
                                                outcomes=["found_corner", "found_convex"]),
                               transitions={"found_corner": "ALIGN_WITH_CORNER",
                                            "found_convex": "preempted"})

        smach.StateMachine.add('ALIGN_WITH_CORNER',
                               PreemptableState(align_with_corner,
                                                input_keys=["ranges", "clearance", "mode"],
                                                output_keys=["velocity"],
                                                outcomes=["corner_aligned"]),
                               transitions={'corner_aligned': 'FOLLOW_WALL'})

        smach.StateMachine.add('ALIGN',
                               PreemptableState(align,
                                                input_keys=["front_min", "right_top",
                                                            "right_bottom", "clearance",
                                                            "right_1", "right_2",
                                                            "left_1", "left_2",
                                                            "turn"],
                                                output_keys=['velocity'],
                                                outcomes=['aligned']),
                               transitions={'aligned': 'FORWARD'})

        smach.StateMachine.add('FORWARD',
                               PreemptableState(forward,
                                                input_keys=['front_min', 'clearance',
                                                            "right_1", "right_2",
                                                            "left_1", "left_2",
                                                            "right_top", "right_bottom",
                                                            "left_top", "left_bottom",
                                                            "mode", "left", "right"],
                                                output_keys=['velocity', "turn"],
                                                outcomes=['found_corner']),
                               transitions={'found_corner': 'ALIGN'})

        pass
        # =========================== YOUR CODE HERE ===========================
        # Instructions: construct the state machine by adding the states that
        #               you have implemented.
        #               Below is an example how to add a state:
        #
        #                   smach.StateMachine.add('SEARCH',
        #                                          PreemptableState(search,
        #                                                           input_keys=['front_min', 'clearance'],
        #                                                           output_keys=['velocity'],
        #                                                           outcomes=['found_obstacle']),
        #                                          transitions={'found_obstacle': 'ANOTHER_STATE'})
        #
        #               First argument is the state label, an arbitrary string
        #               (by convention should be uppercase). Second argument is
        #               an object that implements the state. In our case an
        #               instance of the helper class PreemptableState is
        #               created, and the state function in passed. Moreover,
        #               we have to specify which keys in the userdata the
        #               function will need to access for reading (input_keys)
        #               and for writing (output_keys), and the list of possible
        #               outcomes of the state. Finally, the transitions are
        #               specified. Normally you would have one transition per
        #               state outcome.
        #
        # Note: The first state that you add will become the initial state of
        #       the state machine.
        # ======================================================================
    return sm