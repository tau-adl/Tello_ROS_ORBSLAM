#!/usr/bin/env python

from __future__ import print_function

import rospy

from geometry_msgs.msg import Twist, PoseStamped, Point, Pose
from visualization_msgs.msg import Marker

import time

class MarkerConverter(object):

    def __init__(self):
        # Initialize ROS
        rospy.init_node('marker_conveter', anonymous=False)

        self.id                = rospy.get_param('~ID')

        self.pose_publisher = rospy.Publisher('/ccmslam/pose'+self.id, PoseStamped, queue_size=1)
        self.pose_orb_publisher = rospy.Publisher('/orb_slam2_mono/pose', PoseStamped, queue_size=1)



        self.pose_stamped = PoseStamped()

        # rospy.Subscriber("/ccmslam/ClientMarkerMap"+self.id, Marker, self.marker_convert)
        rospy.Subscriber("/ccmslam/ServerMarkerMap"+self.id, Marker, self.marker_convert)

        rospy.spin()

        rospy.loginfo("Quiting marker_conveter")


    def marker_convert(self, marker_data):
        if len(marker_data.points) > 0:
            point_unrotated = marker_data.points[0]
            point_rotated = Point(point_unrotated.z, point_unrotated.x, point_unrotated.y)
            self.pose_stamped.header = marker_data.header
            self.pose_stamped.pose.position = point_rotated
            self.pose_publisher.publish(self.pose_stamped)
            self.pose_orb_publisher.publish(self.pose_stamped)

  



if __name__ == '__main__':
    marker_conveter = MarkerConverter()
