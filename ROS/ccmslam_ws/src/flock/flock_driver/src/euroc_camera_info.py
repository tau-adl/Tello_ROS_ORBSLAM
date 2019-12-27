#!/usr/bin/env python

from __future__ import print_function

import rospy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32 

import time

# Camera.fx: 462.880355
# Camera.fy: 461.901489
# Camera.cx: 365.972337
# Camera.cy: 247.068837
# # camera_matrix:
# #   data: [462.880355, 0.000000, 365.972337, 0.000000, 461.901489, 247.068837, 0.000000, 0.000000, 1.000000]

#           # [ fx        0           cx]
#           # [ 0         fy          cy]
#           # [ 0         0           1 ]


# Camera.k1: -0.283525
# Camera.k2: 0.074111
# Camera.p1: 0.000145
# Camera.p2: -0.000337
# Camera.k3: 0.0
# # distortion_coefficients:
# # data: [-0.283525, 0.074111, 0.000145, -0.000337, 0.000000]


# Camera.width: 752
# Camera.height: 480

# # Camera frames per second
# # Camera.fps: 60.0
# Camera.fps: 20.0

# # Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
# Camera.RGB: 1

# image_width: 752
# image_height: 480
# camera_name: narrow_stereo
# camera_matrix:
#   rows: 3
#   cols: 3
#   data: [462.880355, 0.000000, 365.972337, 0.000000, 461.901489, 247.068837, 0.000000, 0.000000, 1.000000]
# distortion_model: plumb_bob
# distortion_coefficients:
#   rows: 1
#   cols: 5
#   data: [-0.283525, 0.074111, 0.000145, -0.000337, 0.000000]
# rectification_matrix:
#   rows: 3
#   cols: 3
#   data: [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
# projection_matrix:
#   rows: 3
#   cols: 4
#   data: [361.042694, 0.000000, 360.074173, 0.000000, 0.000000, 423.122498, 248.546022, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]

camera_info = CameraInfo()


camera_info.height = 480
camera_info.width = 752
camera_info.distortion_model = 'plumb_bob'
camera_info.D = [-0.283525, 0.074111, 0.000145, -0.000337, 0.000000]
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]
camera_info.K = [462.880355, 0.000000, 365.972337, 0.000000, 461.901489, 247.068837, 0.000000, 0.000000, 1.000000]

# Rectification matrix (stereo cameras only)
# A rotation matrix aligning the camera coordinate system to the ideal
# stereo image plane so that epipolar lines in both stereo images are
# parallel.
camera_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1] # 3x3 row-major matrix
# Projection/camera matrix
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]
camera_info.P = [361.042694, 0.000000, 360.074173, 0.000000, 0.000000, 423.122498, 248.546022, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
# Binning refers here to any camera setting which combines rectangular
#  neighborhoods of pixels into larger "super-pixels." It reduces the
#  resolution of the output image to
#  (width / binning_x) x (height / binning_y).
# The default values binning_x = binning_y = 0 is considered the same
#  as binning_x = binning_y = 1 (no subsampling).
camera_info.binning_x	=	1
camera_info.binning_y	=	1
camera_info.roi.do_rectify = True

def publish_info(image):
    global camera_info
    global camera_info_pub
    camera_info.header = image.header
    # camera_info.header.stamp = rospy.Time.now()
    camera_info_pub.publish(camera_info)
    camera_picture_pub.publish(image)

if __name__ == '__main__':
    # driver = telloCameraInfo()
    camera_info_pub = rospy.Publisher('euroc/camera_info', CameraInfo, queue_size=1)
    camera_picture_pub = rospy.Publisher('euroc/image_raw', Image, queue_size=1)
    rospy.Subscriber("cam0/image_raw", Image, publish_info)
    rospy.init_node('euroc_camera_info')
    TIME_TO_PUBLISH = 3
    while not rospy.is_shutdown():
        pass
        # camera_info.header.stamp = rospy.Time.now()
        # camera_info_pub.publish(camera_info)
        # time.sleep(TIME_TO_PUBLISH)
    rospy.loginfo("Quiting euroc_camera_info")

