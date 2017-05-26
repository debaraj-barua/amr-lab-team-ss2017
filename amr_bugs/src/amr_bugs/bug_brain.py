#!/usr/bin/env python


#=============================== YOUR CODE HERE ===============================
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

from planar import Point, Vec2
from planar.c import Line
#from math import degrees
import rospy


"""
#==============================================================================
#                        Class Variables
#                       ------------------
#
# @c wp_goal_point      : Stores goal coordinates
# @c side               : Stores side
# @c leave_point_list   : Stores coordinates of all the points where bug left the wall
# @c begin_point_list   : Stores coordinates of all the points where bug started wall follow
# @c now                : Stores rostime (used to avoid multiple storage of same point)
# @c wp_start_wall_point: Stores last point where the bug started following the wall
# @c ln_goal_line       : Stores straight line path to goal
# @c wp_current_point   : Stores current position as a point
#==============================================================================
"""

class BugBrain:

    TOLERANCE = 0.3

    def __init__(self, goal_x, goal_y, side):
        self.side=side
        self.wp_goal_point=Vec2(goal_x,goal_y)
        self.leave_point_list=[]
        self.begin_point_list=[]
        self.now = rospy.get_rostime()

        
    def follow_wall(self, x, y, theta):
        """
        This function is called when the state machine enters the wallfollower
        state.
        """
        self.wp_start_wall_point=Vec2(x,y)
        flag=True
        
        """
        Check if current wall following start point is already inserted into list  
        "self.begin_point_list".
        If not, append to list
        """
        for x in range(len(self.begin_point_list)):
            wp_point=self.begin_point_list[x][0]
            if abs(wp_point.distance_to( self.wp_start_wall_point)<=1):
                flag=False
     
        if (flag==True) :     
            rospy.loginfo("Inserting Begining point to list")
            self.begin_point_list.append((self.wp_start_wall_point,0))
        
        
        rospy.loginfo("Start Wall Follow.")
        self.now = rospy.get_rostime()
        
        
        """
        If this is first hit on any wall, estimate straight line to goal, 
        use this straight line for checking position later
        """
        if len(self.leave_point_list)<1:
            rospy.loginfo(" Plot Line...........")
            self.ln_goal_line=Line.from_points([self.wp_start_wall_point, self.wp_goal_point])
            
        pass

    def leave_wall(self, x, y, theta):
        """
        This function is called when the state machine leaves the wallfollower
        state.
        """
        # compute and store necessary variables

        """
        Storing point where bug leaves the wall
        """        
        self.wp_left_wall_point=Vec2(x,y)
        pass

    def is_goal_unreachable(self, x, y, theta):
        """
        This function is regularly called from the wallfollower state to check
        the brain's belief about whether the goal is unreachable.
        """
        current_point=Vec2(x,y)
                                    
        """
        Calculate time difference to avoid storing same point as repeated laps
        """
        next_time = rospy.get_rostime()
        time_diff=abs(self.now.secs-next_time.secs)

        
        """
        For all points on the begin_point_list, check if current position has 
        been reached before. If so, increase count of that point (increase lap).
        
        Goal is unreachable, if bug reaches same point after 3rd lap around the goal
        """        
        
        for x in range(len(self.begin_point_list)):
            wp_point=self.begin_point_list[x][0]
            if abs(wp_point.distance_to(current_point)<=self.TOLERANCE and time_diff>20):
                self.now = rospy.get_rostime()
                self.begin_point_list[x]=(self.begin_point_list[x][0],self.begin_point_list[x][1]+1)
                #self.count_turns=self.count_turns+1
                rospy.loginfo("Total Number of Approach Wall:")
                rospy.loginfo(len(self.begin_point_list))    
                rospy.loginfo("Max Lap:")
                rospy.loginfo(self.begin_point_list[x][1])
                if (self.begin_point_list[x][1]>=2):
                    return True
            
        return False

    def is_time_to_leave_wall(self, x, y, theta):
        """
        This function is regularly called from the wallfollower state to check
        the brain's belief about whether it is the right time (or place) to
        leave the wall and move straight to the goal.
        """

        self.wp_current_point=Vec2(x,y)
        
        """
        When bug reaches goal line during wall follow, check if that path has been taken
        earlier. 
        If taken, proceed with wall follow. 
        Else, leave wall and take staright line to goal
        """
        
        if (abs(self.ln_goal_line.distance_to(self.wp_current_point))<=self.TOLERANCE and 
                    abs(self.wp_current_point.distance_to(self.wp_start_wall_point))>1):
            flag=True
            rospy.loginfo("Checking previous paths taken..")
            for i in range(len(self.leave_point_list)):
                wp_point=self.leave_point_list[i]

                if (abs(wp_point.distance_to(self.wp_current_point))<=1):
                    flag=False
                    break                    
            if(flag==True):
                rospy.loginfo("Leaving wall")
                self.leave_point_list.append(self.wp_current_point)
                return True
            
        return False

#==============================================================================
