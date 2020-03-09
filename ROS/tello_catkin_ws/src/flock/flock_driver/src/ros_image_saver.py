#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Empty
import math
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import time
import os
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class RosImageSaver(object):
    """Wrapper class to enable the GUI."""

    def __init__(self):
        # Instantiate CvBridge
        self.bridge = CvBridge()
        rospy.init_node('RosImageSaver', anonymous=False)
        try: 
            self.id = rospy.get_param('~ID')
        except KeyError:
            self.id = ''
        self.publish_prefix = "tello{}/".format(self.id)
        self.picture_dir = rospy.get_param('~PICTURE_DIR')
        self.take_picture_flag = False
        # Define your image topic
        image_topic = self.publish_prefix+'camera/image_raw'
        # Set up your subscriber and define its callback
        rospy.Subscriber(image_topic, Image, self.image_callback)

        rospy.Subscriber(self.publish_prefix+'take_picure', Empty, self.take_picure_callback)

        rospy.Subscriber(self.publish_prefix+'real_world_pos', PoseStamped, self.real_world_pos_callback)

        # Spin until ctrl + c
        rospy.spin()

    def real_world_pos_callback(self, msg):
        self.real_world_position = msg.pose.position
        self.orientation_deg = self.quatenrion_to_euler_point(msg.pose.orientation)
        
    def quatenrion_to_euler_point(self, quatenrion):
        euler_list = euler_from_quaternion([quatenrion.x, quatenrion.y, quatenrion.z, quatenrion.w])
        euler = Point()
        euler.x = self.rad_to_deg(euler_list[0])
        euler.y = self.rad_to_deg(euler_list[1])
        euler.z = self.rad_to_deg(euler_list[2])
        return euler

    def rad_to_deg(self, rad):
        return rad / math.pi * 180.0

    def deg_to_rad(self, deg):
        return deg * math.pi / 180.0

    def image_callback(self, msg):
        
        if self.take_picture_flag:
            self.take_picture_flag = False
            try:
                # Convert your ROS Image message to OpenCV2
                cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError, e:
                print(e)
            else:
                # Save your OpenCV2 image as a jpeg 
                time_str = time.strftime("%Y-%m-%d - %H-%M-%S")
                # time_str = 'a'
                final_str = 'a' + time_str
                # final_str += ' ' + "x={}".format(self.real_world_position.x)
                # final_str += ' ' + "y={}".format(self.real_world_position.y)
                # final_str += ' ' + "z={}".format(self.real_world_position.z)
                # final_str += ' ' + "yaw={}".format(self.orientation_deg.z)

                # out_path = os.path.join(self.picture_dir, final_str)
                out_path = final_str
                out_path += '.jpeg'
                print("Taking picture to {}".format(out_path))
                cv2.imwrite(out_path, cv2_img)

    def take_picure_callback(self, msg):
        
        self.take_picture_flag = True
  
   

if __name__ == '__main__':
    driver = RosImageSaver()