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
        self._client = SimpleActionClient('/bug2/move_to', MoveToAction)

        self._action_name = rospy.get_name() + "/execute_path"
        self._server = SimpleActionServer(self._action_name, ExecutePathAction, execute_cb=self.execute_cb, auto_start = False)
        self._server.start()

    def execute_cb(self, goal):
        self._client.wait_for_server()

        rospy.loginfo("Received {0} poses".format(len(goal.path.poses)))

        move = MoveToGoal()
        move.target_pose.pose = goal.path.poses[0].pose

        self._client.send_goal(move)

        self._server.set_succeeded(ExecutePathResult())

    def move_to_done_cb(self, state, result):
        pass


if __name__ == '__main__':
    rospy.init_node(NODE)
    pe = PathExecutor()
    rospy.spin()
