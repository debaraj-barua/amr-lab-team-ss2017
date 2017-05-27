#!/usr/bin/env python
"""
#==============================================================================
#                       State Descriptions
#                       ------------------
#   1) SEARCH            : Used to search for any wall, used during initialization
#   2) ALIGN             : Used to alignt to a wall, accroding to mode selected
#   3) FOLLOW_WALL       : Used to follow a wall once aligned & also navigate 
                           convex turns
#   4) TRANSITION_SEARCH : If mode is changed during wall following, this is 
#                          used to search for wall in the required direction
#   5) CONCAVE           : Used to navigate corners, i.e., concave turns 
#==============================================================================
"""

PACKAGE = 'amr_bugs'

import roslib
roslib.load_manifest(PACKAGE)
import smach
import rospy
from preemptable_state import PreemptableState
#from math import copysign
from types import MethodType
from geometry_msgs.msg import Twist


__all__ = ['construct']

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
          
            rospy.loginfo("Convex Turning")
            rospy.logwarn(userdata.direction*(userdata.clearance-min(side_1,side_2)))
            userdata.velocity=(userdata.max_forward_velocity,
                                     userdata.direction*(userdata.clearance-min(side_1,side_2)),
                                     userdata.direction*(side_2-side_1)*1.7 )
             
         
        #==============================================================================
        #         Aligning Robot for uneven walls        
        #==============================================================================

        #align lateral offsets to wall
        if (((max(side_2,side_1)>userdata.clearance+tolerance) and back_side>userdata.clearance+tolerance)
            or (min(side_2,side_1)<userdata.clearance+tolerance and back_side<userdata.clearance+tolerance)):  
               
               rospy.loginfo("Correcting Offsets")  
               rospy.logwarn(userdata.direction*(userdata.clearance-min(side_1,side_2)))
               
               userdata.velocity=(0.01, userdata.direction*(userdata.clearance-min(side_1,side_2)),userdata.direction*(side_2-side_1)) 

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
                                                                   outcomes=['concave','no_wall','align']),
                                                                   transitions={'concave': 'CONCAVE',
                                                                                 'no_wall':'TRANSITION_SEARCH',
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

      
    return sm
