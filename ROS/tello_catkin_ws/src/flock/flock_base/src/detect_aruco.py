#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import cv2
from cv_bridge import CvBridge
import tf
import tf.transformations as tft
import numpy as np

# Transformation notation:
# Tst == T_source_target
# vector_target = T_source_target * vector_source


def rvec_and_tvec_to_matrix(rvec, tvec):
    """Rodrigues rotation and translation vector to 4x4 matrix"""
    t_matrix = tft.translation_matrix(tvec)
    R, _ = cv2.Rodrigues(rvec)
    r_matrix = tft.identity_matrix()
    r_matrix[:3, :3] = R
    return np.dot(t_matrix, r_matrix)


def tf_to_matrix(ros_transform):
    """ROS transform to 4x4 matrix"""
    t, q = ros_transform
    t_matrix = tft.translation_matrix(t)
    r_matrix = tft.quaternion_matrix(q)
    return np.dot(t_matrix, r_matrix)


def matrix_to_tf(T):
    """4x4 matrix to ROS transform"""
    t = tft.translation_from_matrix(T)
    q = tft.quaternion_from_matrix(T)
    return t, q


def pose_to_matrix(p):
    """geometry_msgs.msg.Pose to 4x4 matrix"""
    t_matrix = tft.translation_matrix([p.position.x, p.position.y, p.position.z])
    r_matrix = tft.quaternion_matrix([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
    return np.dot(t_matrix, r_matrix)


def matrix_to_pose(matrix):
    """4x4 matrix to geometry_msgs.msg.Pose"""
    t, q = matrix_to_tf(matrix)
    return Pose(Point(*t), Quaternion(*q))


class DetectArUco(object):

    # Clyde's Tello calibration data
    _camera_matrix = np.array(
        [[921.170702, 0.000000, 459.904354],
         [0.000000, 919.018377, 351.238301],
         [0.000000, 0.000000, 1.000000]])
    _distortion = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])

    # Markers are 18cm x 18cm
    _marker_length = 0.18

    # ID of the first marker we see
    _first_marker_id = -1

    # Pose of the first marker is fixed
    _first_marker_pose = Pose(Point(0, 0, 1), Quaternion(0.5, -0.5, -0.5, 0.5))

    # Transform odom => coordinate frame of the first marker
    _Tom = tft.inverse_matrix(pose_to_matrix(_first_marker_pose))

    def __init__(self):
        # ArUco data -- we're using 6x6 ArUco images
        self._aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self._aruco_parameters = cv2.aruco.DetectorParameters_create()

        # Initialize ROS
        rospy.init_node('detect_aruco_node', anonymous=False)

        # ROS publishers
        self._image_pub = rospy.Publisher('image_marked', Image, queue_size=10)
        self._rviz_markers_pub = rospy.Publisher('rviz_markers', MarkerArray, queue_size=10)

        # ROS OpenCV bridge
        self._cv_bridge = CvBridge()

        # ROS transform managers
        self._tf_listener = tf.TransformListener()
        self._tf_broadcaster = tf.TransformBroadcaster()

        # Get base_link => camera_frame transform
        self._tf_listener.waitForTransform("base_link", "camera_frame", rospy.Time(), rospy.Duration(4))
        self._Tcb = tf_to_matrix(self._tf_listener.lookupTransform("base_link", "camera_frame", rospy.Time()))

        # Now that we're initialized, set up subscriptions and spin
        rospy.Subscriber("image_raw", Image, self.image_callback)
        rospy.spin()

    def image_callback(self, msg):
        # Convert ROS image ==> OpenCV Mat
        color_mat = self._cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Color => gray for detection
        gray_mat = cv2.cvtColor(color_mat, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray_mat, self._aruco_dict, parameters=self._aruco_parameters)

        # Stop if no markers were detected
        if ids is None:
            return

        # Draw borders on the color image
        # color_mat = cv2.aruco.drawDetectedMarkers(color_mat, corners)

        # Publish the marked up image
        self._image_pub.publish(self._cv_bridge.cv2_to_imgmsg(color_mat, 'bgr8'))

        if self._first_marker_id < 0:
            # Grab the first marker we see
            self._first_marker_id = ids[0][0]
            rospy.loginfo('First marker has id %d' % self._first_marker_id)
        else:
            # Stop if the first marker wasn't detected
            found_first_marker = False
            for index in range(len(ids)):
                if ids[index][0] == self._first_marker_id:
                    found_first_marker = True
                    break
            if not found_first_marker:
                return

        # Compute transformations, each is marker_frame => camera_frame
        rvecs, tvecs, object_points = cv2.aruco.estimatePoseSingleMarkers(corners, self._marker_length, self._camera_matrix, self._distortion)

        # Compute the pose of the drone
        Tbo = tft.identity_matrix()
        for index in range(len(ids)):
            if ids[index][0] == self._first_marker_id:
                # Tob = Tcb * Tmc * Tom
                Tmc = rvec_and_tvec_to_matrix(rvecs[index][0], tvecs[index][0])
                Tob = self._Tcb.dot(Tmc).dot(self._Tom)

                # sendTransform expects child=target and parent=source
                # We can't flip source and target because we want odom to be the parent of base_link
                # Instead, invert Tob to get Tbo
                Tbo = tft.inverse_matrix(Tob)
                t, q = matrix_to_tf(Tbo)
                self._tf_broadcaster.sendTransform(t, q, rospy.Time.now(), child='base_link', parent='odom')
                break

        # Compute the pose of the other markers, and publish a MarkerArray message for display in rviz
        markerArray = MarkerArray()
        for index in range(len(ids)):
            marker = Marker()
            marker.id = ids[index][0]
            marker.header.frame_id = 'odom'
            if ids[index][0] == self._first_marker_id:
                # This one is easy
                marker.pose = self._first_marker_pose
            else:
                # Compute the pose of this marker
                # Tmo = Tbo * Tcb * Tmc
                Tmc = rvec_and_tvec_to_matrix(rvecs[index][0], tvecs[index][0])
                Tmo = Tbo.dot(self._Tcb).dot(Tmc)
                marker.pose = matrix_to_pose(Tmo)
            marker.type = marker.CUBE
            marker.action = marker.ADD  # TODO DELETE markers that aren't visible
            marker.scale = Vector3(0.1, 0.1, 0.01)
            marker.color = ColorRGBA(1., 1., 0., 1.)
            markerArray.markers.append(marker)
        self._rviz_markers_pub.publish(markerArray)


if __name__ == '__main__':
    driver = DetectArUco()
