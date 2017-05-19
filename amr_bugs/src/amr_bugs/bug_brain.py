#!/usr/bin/env python


# =============================== YOUR CODE HERE ===============================
# Instructions: complete the currently empty BugBrain class. A new instance of
#               this class will be created for each new move_to command. The
#               constructor receives the goal specification and the mode of
#               wallfollowing (left (0) or right (1)) that is currently in use.
#               All the remaining functions receive the current position and
#               orientation of the robot.
#
# Hint: you can create a class member variable at any place in your code (not
#       only in __init__) by assigning a value to it, e.g.:
#
#           self.some_member_variable = 2012
#
# Hint: you could use the 'planar' library to avoid implementing geometrical
#       functions that check the distance between a point and a line, or any
#       other helper functions that you need. To use its classes add the
#       following import statements on top of the file:
#
#            from planar import Point, Vec2
#            from planar.c import Line
#            from math import degrees
#
#       As discussed in the lab class, you will need to install the library by
#       executing `sudo pip install planar` in the terminal.
#
# Hint: all the member variables whose names start with 'wp_' (which stands for
#       'waypoint') will be automagically visualized in RViz as points of
#       different colors. Similarly, all the member variables whose names
#       start with 'ln_' (which stands for 'line') will be visualized as lines
#       in RViz. The only restriction is that the objects stored in these
#       variables should indeed be points and lines.
#       The valid points are:
#
#           self.wp_one = (1, 2)
#           self.wp_two = [1, 2]
#           self.wp_three = Point(x, y) # if you are using 'planar'
#
#       The valid lines are (assuming that p1 and p2 are valid points):
#
#           self.ln_one = (p1, p2)
#           self.ln_two = [p1, p2]
#           self.ln_three = Line.from_points([p1, p2]) # if you are using 'planar'

import rospy
import math
from planar import Point
from planar.c import Line


class BugBrain:
    TOLERANCE = 0.3

    def __init__(self, goal_x, goal_y, side):
        # Set goal point
        self.wp_goal = Point(goal_x, goal_y)
        self.mode = side

        # Track points where the bot starts following the wall
        self.following_points = []

        # Track points where it leaves the wall and goes straight
        self.leaving_points = []

        # Track repeated points. Goal is unreachable if the bot comes back to a repeated point
        self.visited_twice = []

    def follow_wall(self, x, y, theta):
        """
        This function is called when the state machine enters the wallfollower
        state.
        """
        # Set start point if not set already
        if not hasattr(self, "wp_start"):
            self.wp_start = Point(x, y)
            self.ln_path = Line.from_points([self.wp_start, self.wp_goal])

        self.following_points.append(Point(x, y))

    def leave_wall(self, x, y, theta):
        """
        This function is called when the state machine leaves the wallfollower
        state.
        """
        self.leaving_points.append(Point(x, y))

    def is_goal_unreachable(self, x, y, theta):
        """
        This function is regularly called from the wallfollower state to check
        the brain's belief about whether the goal is unreachable.
        """
        # The goal is unreachable if it has been visited twice before
        # and followed both straight line to goal and the wall.
        # The time checking is done to handle repeated calls for the bot from same position.
        for item in self.visited_twice:
            if Point(x, y).distance_to(item[0]) <= self.TOLERANCE \
                    and math.fabs(item[1] - rospy.get_time()) > 5:
                return True
        return False

    def add_repeated_points(self, point):
        """
        Lists points visited twice if not listed already. Also keeps track of time of visit. 
        """
        already_listed = False
        for item in self.visited_twice:
            distance = point.distance_to(item[0])
            if distance <= self.TOLERANCE:
                already_listed = True
                break

        if not already_listed:
            self.visited_twice.append((point, rospy.get_time()))

    def is_time_to_leave_wall(self, x, y, theta):
        """
        This function is regularly called from the wallfollower state to check
        the brain's belief about whether it is the right time (or place) to
        leave the wall and move straight to the goal.
        """
        if hasattr(self, "ln_path"):
            current_point = Point(x, y)
            distance_from_line = math.fabs(self.ln_path.distance_to(current_point))
            distance_from_start_following_point = math.fabs(current_point.distance_to(self.following_points[-1]))

            # Leave the wall when back on track and not if it just started following the wall
            if distance_from_line <= self.TOLERANCE and \
                            distance_from_start_following_point >= self.TOLERANCE * 2:

                # Don't leave the wall if it has been left before at same point
                visited_before = False
                for point in self.leaving_points:
                    distance_from_point = current_point.distance_to(point)
                    if distance_from_point <= self.TOLERANCE * 3:
                        visited_before = True
                        # Keep track of the point to decide on unreachable goal
                        self.add_repeated_points(point)
                        break

                if not visited_before:
                    return True

        return False

# ==============================================================================
