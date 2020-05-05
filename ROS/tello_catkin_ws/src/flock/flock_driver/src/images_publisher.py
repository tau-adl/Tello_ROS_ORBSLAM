#!/usr/bin/env python

from __future__ import print_function

import rospy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32 

import time
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError


if __name__ == '__main__':
    rospy.init_node('images_publisher')
    # driver = telloCameraInfo()

    try:
        pose_topic_name = rospy.get_param('~CAMERA_TOPIC_NAME')
    except KeyError:
        pose_topic_name = '/camera/image_raw'

    # try:
    image_dir = rospy.get_param(rospy.get_name() + '/IMAGES_DIR')
    # except KeyError:
    #     image_dir = '~/ROS/tello_catkin_ws/camera_calibaration_data/Tello/calibrationdata_57A6A7'

    image_pub = rospy.Publisher(pose_topic_name, Image, queue_size=1)

    bridge = CvBridge()

    
    TIME_TO_PUBLISH = 1
    file_list = [element for element in os.listdir(image_dir) if '.png' in element.lower()]
    while not rospy.is_shutdown():
        for file in file_list:
            if rospy.is_shutdown():
                break
            file_path = os.path.join(image_dir, file)
            print(file)
            cv_image = cv2.imread(file_path)
            # print(type(cv_image))
            image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            start_time = time.time()
            while time.time() - start_time < TIME_TO_PUBLISH:
                rospy.sleep(0.1)
                # pass

    rospy.loginfo("Quiting images_publisher")

