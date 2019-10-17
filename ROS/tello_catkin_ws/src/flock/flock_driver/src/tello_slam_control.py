#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import Empty, Bool, Int32, Float32
from flock_msgs.msg import Flip, FlightData
import time
import threading
import math


class TelloSlamControler(object):

    def __init__(self):
        # Initialize ROS
        rospy.init_node('tello_slam_control', anonymous=False)
        

        # ROS subscriptions
        rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, self.slam_callback)
        rospy.Subscriber('/command_x', Point, self.command_x_callback)
        rospy.Subscriber('/tello/allow_slam_control', Bool, self.allow_slam_control_callback)
        rospy.Subscriber('flight_data', FlightData, self.flightdata_callback)


        self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.delta_pub = rospy.Publisher('/tello/delta_pos', Point, queue_size = 1)
        self.real_world_pub = rospy.Publisher('/tello/real_world_pos', Point, queue_size = 1)
        self.real_world_scale_pub = rospy.Publisher('/tello/real_world_scale', Float32, queue_size = 1)
        self.rotated_pos_pub = rospy.Publisher('/tello/rotated_pos', Point, queue_size = 1)


        self.command_x = Point()
        self.twist = Twist()
        self.position = Point() 
        self.real_world = Point()
        self.position_0 = Point()
        self.pos_error = Point()
        self.real_world_scale = 3.96
        self.pos_error_prev = Point()
        self.pos_err_derivative = Point()
        self.pos_err_filtered_derivative = Point()
        self.Kp = Point(0.3, 0.3, 0.3)
        self.Kd = Point(1, 1, 1)

        self.allow_slam_control = False

        self.last_time_received_pose = time.time() - 5

        self.rate = 30

        self.angle = 12.0
        self.angle_radian = self.angle / 180.0 * math.pi

        self.control_mult_factor_x = 2
        self.control_mult_factor_y = 2
        self.control_mult_factor_z = 4

        self.time_threshold = 1

        self.counter_before_map_exists_flag = 0
        self.counter_before_no_map_flag = 0
        self.MAX_COUNT_BEFORE_NO_MAP_EXISTS_FLAG = 6 * self.rate
        self.MAX_COUNT_BEFORE_MAP_EXISTS_FLAG = 10 * self.rate

        self.caution_speed_threshold = Point(0.3, 0.3, 0.2)

        self.delta_pos = Point()


        self.map_exists_flag = False

        self.last_position = 0
        self.altitude = 0


        

        self.last_time_published = time.time()
        self.thread = threading.Thread(target=self.peridoc_timer, args=())
        self.thread.start()
        

        # Spin until interrupted
        rospy.spin()

    def peridoc_timer(self):

        # if its the same position
        if self.last_position == self.position:
            # increment counter before map not exists
            self.counter_before_no_map_flag += 1
            self.counter_before_map_exists_flag = 0
            # if flag is higher than threshold
            if self.counter_before_no_map_flag == self.MAX_COUNT_BEFORE_NO_MAP_EXISTS_FLAG:
                # take flag off, and initiate counter for map exists
                self.map_exists_flag = False
                # print("map_exists_flag={}".format(self.map_exists_flag))
                self.counter_before_map_exists_flag = 0

        else:
            # received position which is different from last position
            self.last_position = self.position
            # increment counter
            self.counter_before_map_exists_flag += 1
            self.counter_before_no_map_flag = 0
            if self.counter_before_map_exists_flag == self.MAX_COUNT_BEFORE_MAP_EXISTS_FLAG:
                self.map_exists_flag = True
                self.counter_before_no_map_flag = 0
                # print("map_exists_flag={}".format(self.map_exists_flag))

                

        # print("map_exists_flag={}, counter_exists={} counter_not_exists={}".format(self.map_exists_flag, self.counter_before_map_exists_flag, self.counter_before_no_map_flag))
        # print("last_position={}, position={}".format(self.last_position, self.position))

        # if self.map_exists_flag:

        if time.time() - self.last_time_received_pose < self.time_threshold:
            self.twist.linear = self.calculate_control_algorithm()

            # sign_delta_x = 2 * (self.delta_pos.x > 0) - 1
            # sign_delta_y = 2 * (self.delta_pos.y > 0) - 1
            sign_delta_z = 2 * (self.delta_pos.z > 0) - 1

            # self.twist.linear.x = sign_delta_x * min(self.control_mult_factor_x*abs(self.delta_pos.x), self.caution_speed_threshold.x)
            # self.twist.linear.y = sign_delta_y * min(self.control_mult_factor_y*abs(self.delta_pos.y), self.caution_speed_threshold.y)
            self.twist.linear.z = sign_delta_z * min(self.control_mult_factor_z*abs(self.delta_pos.z), self.caution_speed_threshold.z)

            # self.twist.linear = self.calculate_control_algorithm()

            self.twist.linear = self.clip_point(self.calculate_control_algorithm(), self.caution_speed_threshold)


        else:
            self.twist.linear.x = 0
            self.twist.linear.y = 0
            self.twist.linear.z = 0
            self.twist.angular.x = 0
            self.twist.angular.y = 0
            self.twist.angular.z = 0

        # print("map_exists_flag={} counter_before_no_map_flag={} counter_before_map_exists_flag={}".format(self.map_exists_flag, self.counter_before_no_map_flag, self.counter_before_map_exists_flag))

        if self.allow_slam_control:
            self.pub_twist.publish(self.twist)

        if not rospy.is_shutdown():
            new_timer = max(1.0/self.rate - (time.time() - self.last_time_published), 1/self.rate/2)
            self.last_time_published = time.time()
            threading.Timer(new_timer, self.peridoc_timer).start()

    def flightdata_callback(self, flight_data):
        self.altitude = flight_data.altitude

    def sign(self, x):
        return 2 * (x > 0) - 1

    def clip_point(self, p, threshold_point):
        if abs(p.x) > threshold_point.x:
            p.x = self.sign(p.x) * threshold_point.x
        if abs(p.y) > threshold_point.y:
            p.y = self.sign(p.y) * threshold_point.y
        if abs(p.z) > threshold_point.z:
            p.z = self.sign(p.z) * threshold_point.z
        return p

    def point_copy(self, p):
        return Point(p.x, p.y, p.z)

    def calculate_control_algorithm(self):
        point = Point()

        self.pos_error.x = self.command_x.x - self.real_world.x
        self.pos_error.y = self.command_x.y - self.real_world.y
        self.pos_error.z = self.command_x.z - self.real_world.z

        self.pos_err_derivative.x = self.pos_error.x - self.pos_error_prev.x
        self.pos_err_derivative.y = self.pos_error.y - self.pos_error_prev.y
        self.pos_err_derivative.z = self.pos_error.z - self.pos_error_prev.z

        self.pos_err_filtered_derivative.x = 0.5648*self.pos_err_filtered_derivative.x + 12.75*self.pos_err_derivative.x;
        self.pos_err_filtered_derivative.y = 0.5648*self.pos_err_filtered_derivative.y + 12.75*self.pos_err_derivative.y;
        self.pos_err_filtered_derivative.z = 0.5648*self.pos_err_filtered_derivative.z + 12.75*self.pos_err_derivative.z;

        point.x = self.Kp.x*self.pos_error.x + self.Kd.x*self.pos_err_filtered_derivative.x;
        point.y = self.Kp.y*self.pos_error.y + self.Kd.y*self.pos_err_filtered_derivative.y;
        point.z = self.Kp.z*self.pos_error.z + self.Kd.z*self.pos_err_filtered_derivative.z;

        self.pos_error_prev = self.point_copy(self.pos_error)

        return point

    def command_x_callback(self, command_x):
        self.command_x = command_x

    def allow_slam_control_callback(self, msg):
        self.allow_slam_control = (msg.data == 1)
        if self.allow_slam_control:
            self.position_0 = self.point_copy(self.position)
            try:
                if self.altitude > 0.2:
                    self.real_world_scale = self.altitude / self.position.z
                    self.real_world_scale_pub.publish(self.real_world_scale)
            except ZeroDivisionError:
                self.real_world_scale = 1
            

        


    def slam_callback(self, slam_msg):
        # if self.command_x.x == 0 and self.command_x.y == 0 and self.command_x.z == 0:
            # return

        # self.position = slam_msg.pose.position
        self.slam_pos = slam_msg.pose.position

        self.position.x = self.slam_pos.x * math.cos(self.angle_radian) + self.slam_pos.z * math.sin(self.angle_radian)
        self.position.y = self.slam_pos.y
        self.position.z = self.slam_pos.x * (-math.sin(self.angle_radian)) + self.slam_pos.z * math.cos(self.angle_radian)

        self.real_world.x = (self.position.x - self.position_0.x)* self.real_world_scale
        self.real_world.y = (self.position.y - self.position_0.y)* self.real_world_scale
        self.real_world.z = (self.position.z - self.position_0.z)* self.real_world_scale

        self.delta_pos.x = self.command_x.x  -   self.real_world.x 
        self.delta_pos.y = self.command_x.y  -   self.real_world.y
        self.delta_pos.z = self.command_x.z  -   self.real_world.z
        self.last_time_received_pose = time.time()

        self.delta_pub.publish(self.delta_pos)
        self.real_world_scale_pub.publish(self.real_world_scale)
        self.real_world_pub.publish(self.real_world)
        self.rotated_pos_pub.publish(self.position)


        # print("delta_x={} delta_y={} delta_z={}".format(delta_x, delta_y, delta_z))



   

if __name__ == '__main__':
    driver = TelloSlamControler()
