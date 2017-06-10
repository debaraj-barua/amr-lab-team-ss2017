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


class PathExecutor:

    def __init__(self):
        self._poses = []
        self._poses_idx = 0

        self._client = SimpleActionClient('/bug2/move_to', MoveToAction)

        self._action_name = rospy.get_name() + "/execute_path"
        self._server = SimpleActionServer(self._action_name, ExecutePathAction, execute_cb=self.execute_cb, auto_start = False)
        self._server.start()

    def execute_cb(self, goal):
        self._client.wait_for_server()

        self._poses = goal.path.poses
        self._poses_idx = 0

        move = MoveToGoal()
        move.target_pose.pose = self._poses[self._poses_idx].pose

        self._client.send_goal(move, done_cb=self.move_to_done_cb)

        rospy.spin()

    def move_to_done_cb(self, state, result):
        # Send feedback
        feedback = ExecutePathFeedback()
        feedback.pose=self._poses[self._poses_idx]
        feedback.reached = True if state == 3 else False
        self._server.publish_feedback(feedback)

        self._poses_idx = self._poses_idx + 1

        if(self._poses_idx < len(self._poses)):
            move = MoveToGoal()
            move.target_pose.pose = self._poses[self._poses_idx].pose

            self._client.send_goal(move, done_cb=self.move_to_done_cb)
        else:
            self._server.set_succeeded(ExecutePathResult())

if __name__ == '__main__':
    rospy.init_node(NODE)
    pe = PathExecutor()
    rospy.spin()
