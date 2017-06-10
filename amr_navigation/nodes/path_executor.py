#!/usr/bin/env python

PACKAGE = 'amr_navigation'
NODE = 'path_executor'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from actionlib import SimpleActionClient, SimpleActionServer
from nav_msgs.msg import Path
from amr_msgs.msg import MoveToAction, MoveToGoal, ExecutePathAction, \
                         ExecutePathFeedback, ExecutePathResult

#==============================================================================
#                        Class Variables
#                       ------------------
#
# @c _continue          : Stores whether execute_cb should continue or stop
# @c _skip_unreachable  : Stores boolean value for skip unreachable flag
# @c _path_result       : Stores a boolean array for each path goal
# @c _poses             : Stores array if goal poses
# @c _poses_idx         : Stores current pose id
# @c _client            : Client object of MoveToAction
# @c _server            : Server object for ExecutePathAction
#==============================================================================


class PathExecutor:
    _continue=True
    _skip_unreachable=True
    _path_result=ExecutePathResult()
     
    def __init__(self):
        self._poses = []
        self._poses_idx = 0
        self._client = SimpleActionClient('/bug2/move_to', MoveToAction)
        self._action_name = rospy.get_name() + "/execute_path"
        self._server = SimpleActionServer(self._action_name, ExecutePathAction, execute_cb=self.execute_cb, auto_start = False)
        self._server.start()

    def execute_cb(self, goal):
        """
        This funciton is used to execute the move to goal
        """
        self._path_result.visited=[]                #Initialize the path result 
        self._skip_unreachable=goal.skip_unreachable
        self._client.wait_for_server()

        self._poses = goal.path.poses
        self._poses_idx = 0                         #Start with the first goal
        self._continue= True
        rospy.loginfo('Starting Number of Poses: %s'%len(self._poses))
        move = MoveToGoal()
        move.target_pose.pose = self._poses[self._poses_idx].pose
        
        """
        Send the goal to the _client and once done, go to callback function 
        "move_to_done_cb"
        """
        self._client.send_goal(move, done_cb=self.move_to_done_cb)
        
        while (self._continue==True):
            """
            Check if current goal is preempted by other user action.
            If so, break out of loop
            """
            if self._server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._server.set_preempted()
                self._continue= False
                break
                
    def move_to_done_cb(self, state, result):
        """
        This call back function is called once one goal is completed.
        This checks if there are any other goal remaining to be executed and
        recurrsively calls itself.
        """
        feedback = ExecutePathFeedback()
     
        feedback.pose=self._poses[self._poses_idx]
        
        """
        #======================================================================
        #* If         state is 3, i.e., goal was successful, publish feedback 
        #             and move to next goal
        #                      
        #* Else       publish feedback and check _skip_unreachable flag, 
        #             * If True                go to next pose 
        #             * Else                   break
        #======================================================================
        """
        if(state == 3):
            feedback.reached = True 
            self._path_result.visited.append(feedback.reached)
            self._server.publish_feedback(feedback)
    
            self._poses_idx = self._poses_idx + 1
            """
            * If                    more pose available, move to next goal
            * Else                  break
            """
            if(self._poses_idx < len(self._poses)):
                move = MoveToGoal()
                move.target_pose.pose = self._poses[self._poses_idx].pose
                self._client.send_goal(move, done_cb=self.move_to_done_cb)
            else:
                self._continue=False
                self._server.set_succeeded(self._path_result)
            
        else:
            feedback.reached = False
            self._path_result.visited.append(False)
            self._server.publish_feedback(feedback)
            
            if(self._skip_unreachable==True):
                self._poses_idx = self._poses_idx + 1
                
                """
                * If                    more pose available, move to next goal
                * Else                  break
                """
                if(self._poses_idx < len(self._poses)):
                    move = MoveToGoal()
                    move.target_pose.pose = self._poses[self._poses_idx].pose
                    self._client.send_goal(move, done_cb=self.move_to_done_cb)
                else:
                    self._continue=False
                    self._server.set_succeeded(self._path_result)
            else:
                rospy.loginfo('Unreachable')
                self._continue=False
                self._server.set_succeeded(self._path_result)
                

if __name__ == '__main__':
    rospy.init_node(NODE)
    pe = PathExecutor()
    rospy.spin()
