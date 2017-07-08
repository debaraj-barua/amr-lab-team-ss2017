#!/usr/bin/env python

PACKAGE = 'amr_localization'
NODE = 'pose_likelihood_server'

import roslib

roslib.load_manifest(PACKAGE)
import rospy
import tf
import numpy as np
import math

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose2D
from amr_srvs.srv import GetMultiplePoseLikelihood, GetMultiplePoseLikelihoodResponse, GetNearestOccupiedPointOnBeam, \
    GetNearestOccupiedPointOnBeamRequest, SwitchRanger


class PoseLikelihoodServerNode:
    """
    This is a port of the AMR Python PoseLikelihoodServerNode
    """

    def __init__(self):

        rospy.init_node(NODE)
        # Wait until SwitchRanger service (and hence stage node) becomes available.
        rospy.loginfo('Waiting for the /switch_ranger service to be advertised...');
        rospy.wait_for_service('/switch_ranger')
        try:
            switch_ranger = rospy.ServiceProxy('/switch_ranger', SwitchRanger)
            # Make sure that the hokuyo laser is available and enable them (aka switch on range scanner)
            switch_ranger('scan_front', True)
        except rospy.ServiceException, e:
            rospy.logerror("Service call failed: %s" % e)

        """
            Expose GetMultiplePoseLikelihood Service here,
            subscribe for /scan_front,
            create client for /occupancy_query_server/get_nearest_occupied_point_on_beam service

            http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)
        """

        # Expose the service
        self._likelihood_server = rospy.Service('/pose_likelihood_server/get_pose_likelihood',
                                                GetMultiplePoseLikelihood,
                                                self._handle_multiple_pose_likelihood_request)

        # Subscribe for laser scan
        self._scan_front_subscriber = rospy.Subscriber('/scan_front',
                                                       LaserScan, self._scan_front_callback, queue_size=1)

        # Create client for occupied point on beam
        self._occupied_beam_client = rospy.ServiceProxy('/occupancy_query_server/get_nearest_occupied_point_on_beam',
                                                        GetNearestOccupiedPointOnBeam)

        # Transform base link to laser front link
        base_frame_id = '/base_link'
        laser_front_frame_id = '/base_laser_front_link'
        self._tf = tf.TransformListener()
        self._tf.waitForTransform(base_frame_id, laser_front_frame_id,
                                  rospy.Time(), rospy.Duration(10),
                                  polling_sleep_duration=rospy.Duration(0.01))

        time = self._tf.getLatestCommonTime(laser_front_frame_id, base_frame_id)
        position, quaternion = self._tf.lookupTransform(base_frame_id, laser_front_frame_id, time)

        self.pos_x = position[0]
        self.pos_y = position[1]

        self.pos_yaw = tf.transformations.euler_from_quaternion(quaternion)[2]

        rospy.loginfo('Started [pose_likelihood_server] node.')

        pass

    def _scan_front_callback(self, data):
        self.ranges = data.ranges
        self.angle_min = data.angle_min
        self.range_max = data.range_max
        self.angle_increment = data.angle_increment

    def _handle_multiple_pose_likelihood_request(self, request):
        likelihood = []

        return likelihood

    """
    ============================== YOUR CODE HERE ==============================
    Instructions:   implemenent the pose likelihood server node including a
                    constructor which should create all needed servers, clients,
                    and subscribers, and appropriate callback functions.
                    GetNearestOccupiedPointOnBeam service allows to query
                    multiple beams in one service request. Use this feature to
                    simulate all the laser beams with one service call, otherwise
                    the time spent on communication with the server will be too
                    long.

    Hint: refer to the sources of the previous assignments or to the ROS
          tutorials to see examples of how to create servers, clients, and
          subscribers.
    
    Hint: in the laser callback it is enough to just store the incoming laser
          readings in a class member variable so that they could be accessed
          later while processing a service request.
  
    Hint: the GetNearestOccupiedPointOnBeam service may return arbitrary large
          distance, do not forget to clamp it to [0..max_range] interval.


    Look at the tf library capabilities, you might need it to find transform
    from the /base_link to /base_laser_front_link.
    Here's an example how to use the transform lookup:

        time = self._tf.getLatestCommonTime(frame_id, other_frame_id)
        position, quaternion = self._tf.lookupTransform(frame_id,
                                                        other_frame_id,
                                                        time)
        yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
        x, y, yaw = position[0], position[1], yaw

    You might need other functions for transforming routine, you can find
    a brief api description
        http://mirror.umd.edu/roswiki/doc/diamondback/api/tf/html/python/tf_python.html
    """


if __name__ == '__main__':
    w = PoseLikelihoodServerNode()
    rospy.spin()
