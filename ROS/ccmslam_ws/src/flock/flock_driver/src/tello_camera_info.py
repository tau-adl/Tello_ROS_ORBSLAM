#!/usr/bin/env python

from __future__ import print_function

import rospy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32 

import time

# image_width: 960
# image_height: 720
# camera_name: narrow_stereo
# camera_matrix:
#   rows: 3
#   cols: 3
#   data: [ 924.873180, 0  , 486.997346, 
#           0  , 923.504522, 364.308527, 
#           0  , 0  , 1.000000  ]

					# [ fx        0           cx]
					# [ 0         fy          cy]
					# [ 0         0           1 ]
# distortion_model: plumb_bob
# distortion_coefficients:
#   rows: 1
#   cols: 5
#   data: [-0.034749, 0.071514, 0.000363, 0.003131, 0]
# rectification_matrix:
#   rows: 3
#   cols: 3
#   data: [ 1.000000, 0, 0, 
#           0, 1.000000, 0, 
#           0, 0, 1.000000]
# projection_matrix:
#   rows: 3
#   cols: 4
#   data: [   921.967102, 0  , 489.492281, 0, 
#             0  , 921.018890, 364.508536, 0, 
#             0  , 0  , 1.000000  , 0]

camera_info = CameraInfo()


camera_info.height = 720
camera_info.width = 960
camera_info.distortion_model = 'plumb_bob'
camera_info.D = [-0.034749, 0.071514, 0.000363, 0.003131, 0]
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]
camera_info.K = [ 924.873180, 0, 486.997346, 0, 923.504522, 364.308527, 0, 0, 1 ]

# Rectification matrix (stereo cameras only)
# A rotation matrix aligning the camera coordinate system to the ideal
# stereo image plane so that epipolar lines in both stereo images are
# parallel.
camera_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1] # 3x3 row-major matrix
# Projection/camera matrix
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]
camera_info.P = [921.967102, 0  , 489.492281, 0, 0  , 921.018890, 364.508536, 0, 0  , 0  , 1.000000  , 0]
# Binning refers here to any camera setting which combines rectangular
#  neighborhoods of pixels into larger "super-pixels." It reduces the
#  resolution of the output image to
#  (width / binning_x) x (height / binning_y).
# The default values binning_x = binning_y = 0 is considered the same
#  as binning_x = binning_y = 1 (no subsampling).
camera_info.binning_x	=	1
camera_info.binning_y	=	1
# camera_info.roi.do_rectify = True

def publish_info(msg):
    global camera_info
    global camera_info_pub
    camera_info.header = msg.header
    # camera_info.header.stamp = rospy.Time.now()
    camera_info_pub.publish(camera_info)

if __name__ == '__main__':
    # driver = telloCameraInfo()
    camera_info_pub = rospy.Publisher('camera/camera_info', CameraInfo, queue_size=1)
    rospy.Subscriber("camera/image_raw", Image, publish_info)
    rospy.init_node('tello_camera_info')
    TIME_TO_PUBLISH = 3
    while not rospy.is_shutdown():
        pass
        # camera_info.header.stamp = rospy.Time.now()
        # camera_info_pub.publish(camera_info)
        # time.sleep(TIME_TO_PUBLISH)
    rospy.loginfo("Quiting tello_camera_info")

