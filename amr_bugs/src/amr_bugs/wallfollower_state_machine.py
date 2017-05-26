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

"""
#==============================================================================
#                       State Descriptions
#                       ------------------
#   1) SEARCH            : Used to search for any wall, used during initialization
#   2) ALIGN             : Used to alignt to a wall, accroding to mode selected
#   3) FOLLOW_WALL       : Used to follow a wall once aligned 
#   4) TRANSITION_SEARCH : If mode is changed during wall following, this is 
#                          used to search for wall in the required direction
#   5) CONVEX            : Used ro navigate door ways, i.e, convex turns
#   5) CONCAVE           : Used to navigate corners, i.e., concave turns 
#==============================================================================
"""

PACKAGE = 'amr_bugs'

import roslib
roslib.load_manifest(PACKAGE)
import smach
import rospy
from preemptable_state import PreemptableState
from math import copysign
from types import MethodType
from geometry_msgs.msg import Twist


__all__ = ['construct']

#=============================== YOUR CODE HERE ===============================
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
#==============================================================================

#==============================================================================
#    SEARCH            : Used to search for any wall, used during initialization
#==============================================================================
def search(ud):
       rospy.loginfo("Searching")
       if ud.front_min < ud.clearance:
           ud.velocity = (0, 0, 0)
           return 'found_wall'
       ud.velocity = (0.3, 0, 0)
      
def set_ranges(self, ranges):
    """
    This function will be attached to the constructed wallfollower machine.
    Its argument is a list of Range messages as received by a sonar callback. 
    For left hand side wallfollowing, the sensor values are mirrored (sides are swapped).
    """


    #============================= YOUR CODE HERE =============================
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
    #==========================================================================
    
    self.userdata.front_1 = ranges[3].range
    self.userdata.front_2 = ranges[4].range
    self.userdata.front_min = min(ranges[3].range, ranges[4].range)
    self.userdata.right_1 = ranges[7].range
    self.userdata.right_2 = ranges[8].range
    self.userdata.left_1 = ranges[0].range
    self.userdata.left_2 = ranges[15].range
    self.userdata.front_right = min(ranges[5].range, ranges[6].range)
    self.userdata.front_left = min(ranges[1].range, ranges[2].range)
    self.userdata.back_right = min(ranges[9].range, ranges[10].range)
    self.userdata.back_left = min(ranges[13].range, ranges[14].range)
    
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

    #============================= YOUR CODE HERE =============================
    # Instructions: although this function is implemented, you may need to
    #               slightly tweak it if you decided to handle wallfolllowing
    #               mode in "the smart way".
    # Hint: state machine userdata is accessible in this function as well, for
    #       example you can read the current wallfollowing mode with
    #
    #           self.userdata.mode
    #
    #==========================================================================

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


"""
Defining new methods::
"""

#==============================================================================
# TRANSITION_SEARCH : If mode is changed during wall following, this is
#                     used to search for wall in the required direction
#==============================================================================
def transition_search(userdata):
        rospy.loginfo("Transitioning")
        tolerance=0.05
        #==============================================================================
        #         Find direction to be alligned to, left or right
        #==============================================================================
        if(userdata.direction==1):  #follow on right
            side_1=userdata.right_1
            side_2=userdata.right_2
            angular_velo=-userdata.default_rotational_speed
        else:  # follow on left
            side_1=userdata.left_1
            side_2=userdata.left_2
            angular_velo=userdata.default_rotational_speed    
        
        if (max(side_1,side_2) < userdata.clearance+tolerance and abs(side_1-side_2) < tolerance):
           userdata.velocity = (0, 0, 0)
           return 'found_wall'
        userdata.velocity = (0, 0, angular_velo)

#   ALIGN             : Used to alignt to a wall, accroding to mode selected
def align(userdata):
    
    tolerance=0.05
    #==============================================================================
    #         Find direction to be alligned to, left or right
    #==============================================================================
    if userdata.direction==1:
                rospy.loginfo("Aligning to Right")
                angular_speed = userdata.default_rotational_speed
         
                if (abs(userdata.right_1-userdata.right_2) < tolerance and 
                    max(userdata.right_1,userdata.right_2)<=userdata.clearance+tolerance):
                    
                    rospy.loginfo("Aligned")
                    userdata.velocity = (0,0,0)
                    return "aligned"
                else:
                    userdata.velocity = (0,0,angular_speed)

    else:
                rospy.loginfo("Aligning to Left")
                angular_speed = -userdata.default_rotational_speed
                
                if (abs(userdata.left_1-userdata.left_2) < tolerance and 
                    min(userdata.left_1,userdata.left_2)<=userdata.clearance+tolerance):
                    
                    rospy.loginfo("Aligned")
                    userdata.velocity = (0,0,0)
                    return "aligned"
                else:
                    userdata.velocity = (0,0,angular_speed)

#==============================================================================
#   FOLLOW_WALL       : Used to follow a wall once aligned   
#==============================================================================
def follow_wall(userdata):
        f_min=userdata.front_min
        tolerance=0.1
        
        #==============================================================================
        #         Find direction to be alligned to, left or right
        #==============================================================================
        if(userdata.direction==1):  #follow on right
            side_1=userdata.right_1
            side_2=userdata.right_2
            f_corner=userdata.front_right
            angular_velo=userdata.default_rotational_speed
            back_side=userdata.back_right
        else:  # follow on left
            side_1=userdata.left_1
            side_2=userdata.left_2
            f_corner=userdata.front_left
            angular_velo=-userdata.default_rotational_speed  
            back_side=userdata.back_left
        
        #==============================================================================
        #         Checking ahead for corners, doors and lost wall conditions
        #==============================================================================
        
        if(f_min<userdata.clearance): #Checking for the corner and go to turn
            return 'concave'
        if(f_min>userdata.clearance and 
            min(side_1,side_2)>userdata.clearance*3 and 
            f_corner>userdata.clearance*5):    #if it get lost then go find a wall
            return 'no_wall'
        if(abs(f_corner)>userdata.clearance*5 or max(side_1,side_2)>userdata.clearance+tolerance):  #if there is opening then go to door
            #return 'convex'          
             rospy.loginfo("Convex Turning")
             userdata.velocity=(userdata.max_forward_velocity,
                                     userdata.clearance-min(side_1,side_2),
                                     (side_2-side_1)*1.7 )

        
         
        #==============================================================================
        #         Aligning Robot for uneven walls        
        #==============================================================================

        #align lateral offsets to wall
        if (((max(side_2,side_1)>userdata.clearance+tolerance) and back_side>userdata.clearance+tolerance)
            or (min(side_2,side_1)<userdata.clearance+tolerance and back_side<userdata.clearance+tolerance)):  
               rospy.loginfo("Correcting Offsets")  
               userdata.velocity=(0.01, userdata.direction*userdata.clearance-min(side_1,side_2),userdata.direction*(side_2-side_1))          
               #userdata.velocity=(0.01, userdata.clearance-min(side_1,side_2) ,0)  

        #rotate while moving to maitain orientation
        if(abs(side_1-side_2)>0.02 and side_2<userdata.clearance+tolerance 
            and side_1<userdata.clearance+tolerance):   
            rospy.loginfo("Following with angular adjustments")            
            userdata.velocity=(0.3,0,
                            ((side_2-side_1)/abs(side_2-side_1))*angular_velo)
                            
        # go straight       
        elif side_1==userdata.clearance+tolerance :                                                      
            rospy.loginfo("Following Straight")
            userdata.velocity=(userdata.max_forward_velocity,0,0)
        
        
#==============================================================================
# Concave State is called to manoeuvre corners with walls straight ahead
#==============================================================================
def concave(userdata):        
    
        #==============================================================================
        #         Find direction to be alligned to, left or right
        #==============================================================================        
        if(userdata.direction==1):  #follow on right
            if(userdata.front_1<userdata.clearance*2 or 
                userdata.front_2<userdata.clearance*2 or 
                userdata.front_right<userdata.clearance*2 ):
                    
                userdata.velocity=(0,0,userdata.default_rotational_speed)
                
            else:
                userdata.velocity=(0,0,0)
                return 'navigated'
        
        else:   #follow on left
            if(userdata.front_1<userdata.clearance*2 or 
                userdata.front_2<userdata.clearance*2 or 
                userdata.front_left<userdata.clearance*2 ):
                
                userdata.velocity=(0,0,-userdata.default_rotational_speed)
                
            else:
                userdata.velocity=(0,0,0)
                return 'navigated'
        
#==============================================================================
# Convex State is called to manoeuvre corners with gaps in walls (like doors)
#==============================================================================
def convex(userdata):        
    
        #==============================================================================
        #         Find direction to be alligned to, left or right
        #==============================================================================        
        if(userdata.direction==1):  #follow on right
        #check fot the clearance with repect to  the corner sensors 
            if(userdata.front_right>userdata.clearance*3 or max(userdata.right_2,userdata.right_1)>userdata.clearance):
                    
#==============================================================================
#                 userdata.velocity=(userdata.max_forward_velocity,
#                                     userdata.max_forward_velocity/7,
#                                     -userdata.default_rotational_speed)
#==============================================================================
                 rospy.loginfo("Navigating Convex Right")
                 
                 #if(min(userdata.left_1,userdata.left_2))
                 
                 userdata.velocity=(userdata.max_forward_velocity,
                                    #(userdata.clearance-min(userdata.left_1,userdata.left_2))*0.05,
                                    0,
                                    (userdata.right_2-userdata.right_1)*1.7 )
                
                
            else:
                userdata.velocity=(0,0,0)
                return 'navigated'
        
        else:   #follow on left
            if(userdata.front_left>userdata.clearance*2 or max(userdata.left_2,userdata.left_1)>userdata.clearance):
                
#==============================================================================
#                 userdata.velocity=(userdata.max_forward_velocity,
#                                     -userdata.max_forward_velocity/7,
#                                     userdata.default_rotational_speed) 
#                 
#==============================================================================
                
                 userdata.velocity=(userdata.max_forward_velocity,
                                    #(userdata.clearance-min(userdata.left_1,userdata.left_2))*0.05,
                                    0,
                                    (userdata.left_2-userdata.left_1)*1.7 ) 
                
            else:
                userdata.velocity=(0,0,0)
                return 'navigated'
"""
def stop(userdata):
   
    rospy.loginfo("Stopping")
    userdata.velocity = (0, 0, 0)
    return 'stop'
"""
def construct():
    sm = smach.StateMachine(outcomes=['preempted'])
    # Attach helper functions
    sm.set_ranges = MethodType(set_ranges, sm, sm.__class__)
    sm.get_twist = MethodType(get_twist, sm, sm.__class__)
    sm.set_config = MethodType(set_config, sm, sm.__class__)
    # Set initial values in userdata
    sm.userdata.velocity = (0, 0, 0)
    sm.userdata.mode = 1
    sm.userdata.clearance = 0.7
    sm.userdata.ranges = None
    sm.userdata.max_forward_velocity = 0.3
    sm.userdata.default_rotational_speed = 0.5
    sm.userdata.direction = 1

    with sm:
        #pass
        #=========================== YOUR CODE HERE ===========================
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
        #======================================================================
        
        #==============================================================================
        # SEARCH            : Used to search for any wall, used during initialization        
        #==============================================================================
        smach.StateMachine.add('SEARCH',PreemptableState(search,input_keys=['front_min','clearance'],
                                                                   output_keys=['velocity'],
                                                                   outcomes=['found_wall']),
                                                                    transitions={'found_wall': 'ALIGN'})
        
        #==============================================================================
        # ALIGN             : Used to alignt to a wall, accroding to mode selected        
        #==============================================================================
        smach.StateMachine.add('ALIGN',PreemptableState(align,input_keys=['front_min', 'clearance',
                                                                            'left_1','left_2',
                                                                            'right_1','right_2',
                                                                            'direction','default_rotational_speed'],
                                                                   output_keys=['velocity'],
                                                                   outcomes=['aligned']),
                                                                   transitions={'aligned': 'FOLLOW_WALL'})
        
        #==============================================================================
        #    FOLLOW_WALL       : Used to follow a wall once aligned   
        #==============================================================================
        smach.StateMachine.add('FOLLOW_WALL',PreemptableState(follow_wall,input_keys=['front_min', 'clearance',
                                                                                      'left_1','left_2',
                                                                                      'right_1','right_2',
                                                                                      'front_right','front_left',
                                                                                      'direction','default_rotational_speed',
                                                                                      'max_forward_velocity','back_right','back_left'],
                                                                   output_keys=['velocity'],
                                                                   outcomes=['concave','no_wall','convex','align']),
                                                                   transitions={'concave': 'CONCAVE',
                                                                                 'no_wall':'TRANSITION_SEARCH',
                                                                                 'convex':'CONVEX',
                                                                                 'align' : 'ALIGN'})
                                                                                 
        #==============================================================================
        # TRANSITION_SEARCH : If mode is changed during wall following, this is
        #                     used to search for wall in the required direction
        #==============================================================================                                                                                 
        smach.StateMachine.add('TRANSITION_SEARCH',PreemptableState(transition_search,input_keys=['clearance','front_min',
                                                                                      'left_1','left_2',
                                                                                      'right_1','right_2',
                                                                                      'direction','default_rotational_speed'],
                                                                   output_keys=['velocity'],
                                                                   outcomes=['found_wall']),
                                                                    transitions={'found_wall': 'ALIGN'})  
        
        #==============================================================================
        # Concave State is called to manoeuvre corners with walls straight ahead
        #==============================================================================                                                           
        smach.StateMachine.add('CONCAVE', PreemptableState(concave,input_keys=['front_min', 'clearance',
                                                                               'front_1','front_2',
                                                                                'left_1','left_2',
                                                                                'right_1','right_2',
                                                                                'front_right','front_left',
                                                                                'direction','default_rotational_speed'],
                                                                    output_keys=['velocity'],
                                                                    outcomes=['navigated']),
                                                                    transitions={'navigated': 'FOLLOW_WALL' })

        #==============================================================================
        # Convex State is called to manoeuvre corners with gaps in walls (like doors)
        #==============================================================================                                                                    
        smach.StateMachine.add('CONVEX', PreemptableState(convex,input_keys=['front_min', 'clearance','max_forward_velocity',
                                                                               'front_1','front_2',
                                                                                'left_1','left_2',
                                                                                'right_1','right_2',
                                                                                'front_right','front_left',
                                                                                'direction','default_rotational_speed'],
                                                                    output_keys=['velocity'],
                                                                    outcomes=['navigated']),
                                                                    transitions={'navigated': 'FOLLOW_WALL' })                                                                    
        
        """
        #STOP state was used as a redundant temporary state for undefined states
        smach.StateMachine.add('STOP',PreemptableState(stop,input_keys=['front_min','clearance'],
                                                                   output_keys=['velocity'],
                                                                   outcomes=['stop']),
                                                                    transitions={'stop': 'STOP'})
        """           
    
    return sm
