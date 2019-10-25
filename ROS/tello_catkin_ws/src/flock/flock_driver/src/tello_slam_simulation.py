#!/usr/bin/env python

from __future__ import print_function

import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import Int32 
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from std_msgs.msg import Empty, Bool, Int32, Float32
from flock_msgs.msg import Flip, FlightData
import signal
import time
import math
from math import sin, cos, atan, pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random
import numpy as np
import threading

class telloSlamSimulation(object):

    def __init__(self, initial_x=0, initial_y=0, initial_z=0, initial_yaw=0):
        # Initialize ROS
        rospy.init_node('tello_slam_simulation', anonymous=False)


        # rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, self.slam_callback)
        # rospy.Subscriber('/tello/command_pos', Point, self.command_pos_callback)
        # rospy.Subscriber('/tello/allow_slam_control', Bool, self.allow_slam_control_callback)
        # rospy.Subscriber('flight_data', FlightData, self.flightdata_callback)
        # rospy.Subscriber('takeoff', Empty, self.takeoff_callback)
        # rospy.Subscriber('/tello/calibrate_real_world_scale', Empty, self.calibrate_real_word_scale_callback)
        # rospy.Subscriber('/tello/scan_room', Bool, self.scan_room_callback)

        self.noise_var = 0.003

        self.real_world_scale = 4.0
        self.flight_data = FlightData()
        self.pose_stamped = PoseStamped()
        self.pose_stamped.pose.position.x = initial_x
        self.pose_stamped.pose.position.y = initial_y
        self.pose_stamped.pose.position.z = initial_z
        self.orientation_euler_deg = Point()
        self.orientation_euler_deg.z = initial_yaw
        self.orientation_euler_rad = self.euler_deg_to_euler_rad(self.orientation_euler_deg)
        self.pose_stamped.pose.orientation = self.euler_rad_point_to_quatenrion(self.orientation_euler_rad)

        self.angle_deg = 12
        self.angle_radian = self.deg_to_rad(self.angle_deg)

        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.throttle = 0

        self.throttle_factor = 0.6 # 30 cm for second
        self.pitch_factor = 0.8 # 30 cm for second
        self.roll_factor = 0.8 # 30 cm for second
        self.yaw_factor = 45 # degree for second


        self.last_time_rx_command = time.time()

        self.pose_publisher = rospy.Publisher('/orb_slam2_mono/pose2', PoseStamped, queue_size=1)
        self.flightdata_publisher = rospy.Publisher('flight_data2', FlightData, queue_size=1)
        # self.pose_publisher = rospy.Subscriber('/tello/command_pos', Point,, queue_size=1)
        # self.pose_publisher = rospy.Subscriber('/tello/allow_slam_control', Bool, queue_size=1)
        # self.pose_publisher = rospy.Subscriber('/flight_data', FlightData, queue_size=1)
        # self.pose_publisher = rospy.Subscriber('takeoff', Empty, queue_size=1)
        # self.pose_publisher = rospy.Subscriber('/tello/calibrate_real_world_scale', Empty, queue_size=1)
        # self.pose_publisher = rospy.Subscriber('/tello/scan_room', Bool, queue_size=1)


        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('takeoff', Empty, self.takeoff_callback)
        rospy.Subscriber('/tello/real_world_scale', Float32, self.real_world_scale_callback)

        worker = threading.Thread(target=self.periodic_publish)
        worker.start()


        # Spin until interrupted
        rospy.spin()

        rospy.loginfo("Quiting tello_slam_simulation")

    def real_world_scale_callback(self, msg):
        self.real_world_scale = msg.data


    def rad_to_deg(self, rad):
        return rad / math.pi * 180.0

    def deg_to_rad(self, deg):
        return deg * math.pi / 180.0

    def euler_deg_to_euler_rad(self, point_deg):
        point_rad = Point()
        point_rad.x = self.deg_to_rad(point_deg.x)
        point_rad.y = self.deg_to_rad(point_deg.y)
        point_rad.z = self.deg_to_rad(point_deg.z)
        return point_rad

    def euler_rad_point_to_quatenrion(self, euler_point):
        return self.orientation_to_quaternion(euler_point.x, euler_point.y, euler_point.z)

    def euler_deg_point_to_quatenrion(self, euler_point):
        return self.orientation_to_quaternion(self.deg_to_rad(euler_point.x), self.deg_to_rad(euler_point.y), self.deg_to_rad(euler_point.z))

    def point_copy(self, p):
        return Point(p.x, p.y, p.z)

    def orientation_to_quaternion(self, pitch, roll, yaw):
        quaternion_list = quaternion_from_euler(pitch, roll, yaw)
        quaternion = Quaternion()
        quaternion.x = quaternion_list[0]
        quaternion.y = quaternion_list[1]
        quaternion.z = quaternion_list[2]
        quaternion.w = quaternion_list[3]
        return quaternion

    def update_altitude(self):
        # self.flight_data.altitude = round(self.pose_stamped.pose.position.z * self.real_world_scale, 1)
        self.flight_data.altitude = round(self.pose_stamped.pose.position.z, 1)


    def periodic_publish(self):
        rate = rospy.Rate(30)
        self.last_time_rx_command = time.time()
        while not rospy.is_shutdown():
            position = self.point_copy(self.pose_stamped.pose.position)
            time_passed = time.time() - self.last_time_rx_command
            self.pose_stamped.pose.position.z += self.throttle * time_passed * self.throttle_factor
            self.update_altitude()
            speed_x = self.pitch * cos(self.orientation_euler_deg.z) * self.pitch_factor + self.roll  * sin(self.orientation_euler_deg.z) * self.roll_factor
            speed_y = -self.roll * cos(self.orientation_euler_deg.z) * self.roll_factor + self.pitch * sin(self.orientation_euler_deg.z) * self.pitch_factor

            self.pose_stamped.pose.position.x += speed_x * time_passed
            self.pose_stamped.pose.position.y += speed_y * time_passed
            

            self.orientation_euler_deg.z += -self.yaw * time_passed * self.yaw_factor

            copy = self.point_copy(self.pose_stamped.pose.position)
            copy_orientation_euler_deg_z = self.orientation_euler_deg.z

            self.pose_stamped.pose.position.x += self.noise_var * np.random.randn()
            self.pose_stamped.pose.position.y += self.noise_var * np.random.randn()
            self.pose_stamped.pose.position.z += self.noise_var * np.random.randn()
            self.orientation_euler_deg.z += self.noise_var * 10 * np.random.randn()

            if self.orientation_euler_deg.z > 180:
                self.orientation_euler_deg.z -= 360
            elif self.orientation_euler_deg.z < -180:
                self.orientation_euler_deg.z += 360
            

            self.pose_stamped.pose.orientation = self.euler_deg_point_to_quatenrion(self.orientation_euler_deg)
            self.publish_pose()
            self.pose_stamped.pose.position = copy
            self.orientation_euler_deg.z = copy_orientation_euler_deg_z
            self.last_time_rx_command = time.time()
            rate.sleep()

    def cmd_vel_callback(self, msg):
        self.pitch = self.clip_thresold_value(msg.linear.x, 0.05, 1)
        self.roll = self.clip_thresold_value(-msg.linear.y, 0.05, 1)
        self.throttle = self.clip_thresold_value(msg.linear.z, 0.09, 1)
        self.yaw = self.clip_thresold_value(-msg.angular.z, 0.03, 1)

    def sign(self, x):
        return 2 * (x > 0) - 1

    def clip_value(self, val, threshold_val):
        if abs(val) > threshold_val:
            val = self.sign(val) * threshold_val
        return val

    def threshold_value(self, val, threshold_val):
        if abs(val) < threshold_val:
            return 0.0
        else:
            return val

    def clip_thresold_value(self, val, min_val, max_val):
        return self.threshold_value(self.clip_value(val, max_val), min_val)

    def takeoff_callback(self, msg):
        self.pose_stamped.pose.position.z = 0.8 #/ self.real_world_scale
        self.update_altitude()
        rospy.loginfo("Altitude Set to {}".format(self.flight_data.altitude))
        self.publish_pose()

    def publish_pose(self):
        copy = self.point_copy(self.pose_stamped.pose.position)
        copy.x = self.pose_stamped.pose.position.x * math.cos(-self.angle_radian) + self.pose_stamped.pose.position.z * math.sin(-self.angle_radian)
        copy.y = self.pose_stamped.pose.position.y
        copy.z = self.pose_stamped.pose.position.x * (-math.sin(-self.angle_radian)) + self.pose_stamped.pose.position.z * math.cos(-self.angle_radian)

        copy2 = self.point_copy(self.pose_stamped.pose.position)

        self.pose_stamped.pose.position = copy
        self.pose_stamped.pose.position.x /= self.real_world_scale
        self.pose_stamped.pose.position.y /= self.real_world_scale
        self.pose_stamped.pose.position.z /= self.real_world_scale

        self.pose_stamped.header.seq += 1
        self.pose_stamped.header.stamp = rospy.Time.now()
        self.pose_stamped.header.frame_id = 'map'
        self.pose_publisher.publish(self.pose_stamped)
        self.flightdata_publisher.publish(self.flight_data)
        self.pose_stamped.pose.position = copy2

   



if __name__ == '__main__':
    driver = telloSlamSimulation()
