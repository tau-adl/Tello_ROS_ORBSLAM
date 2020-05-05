#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose, Quaternion, PoseArray
from std_msgs.msg import Empty, Bool, Int32, Float32
from flock_msgs.msg import Flip, FlightData
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import threading
import math
import numpy as np
from copy import deepcopy


class TelloSlamControler(object):

    def __init__(self):
        

        # Initialize ROS
        rospy.init_node('tello_slam_control', anonymous=False)

        
        # rospy.Subscriber('land', Empty, self.land_callback)

        self.time_of_takeoff = time.time() - 1000

        try: 
            self.id = rospy.get_param('~ID')
        except KeyError:
            self.id = ''

        
        try:
            self.pose_topic_name = rospy.get_param('~POSE_TOPIC_NAME')
        except KeyError:
            self.pose_topic_name = '/ccmslam/PoseOutClient'+str(self.id)


        self.publish_prefix = "tello{}/".format(self.id)



        self.pub_twist = rospy.Publisher(self.publish_prefix+'cmd_vel', Twist, queue_size = 1)
        self.delta_pub = rospy.Publisher(self.publish_prefix+'delta_pos', Point, queue_size = 1)
        self.real_world_pub = rospy.Publisher(self.publish_prefix+'real_world_pos', PoseStamped, queue_size = 1)
        self.real_world_scale_pub = rospy.Publisher(self.publish_prefix+'real_world_scale', Float32, queue_size = 1)
        self.rotated_pos_pub = rospy.Publisher(self.publish_prefix+'rotated_pos', Point, queue_size = 1)
        self.slam_orientation_pub = rospy.Publisher(self.publish_prefix+'orientation', Point, queue_size = 1)
        self.move_up_publisher = rospy.Publisher(self.publish_prefix+'move_up', Float32, queue_size = 1)
        self.pose_trail_publisher = rospy.Publisher(self.publish_prefix+'pose_trail', PoseArray, queue_size = 1)
        self.path_publisher = rospy.Publisher(self.publish_prefix+'path_trail', Path, queue_size = 1)


        self.path_trail_msg = Path()
        self.command_pos = Point()
        self.command_orientation_deg = Point()
        self.twist = Twist()
        self.rotated_pos = Point() 
        self.real_world = PoseStamped()
        self.rotated_position_0 = Point()
        self.pos_error = Point()
        self.real_world_scale = 5.21
        self.pos_error_prev = Point()
        self.rotation_error_prev = 0
        self.pos_err_derivative = Point()
        self.pos_err_filtered_derivative = Point()
        self.rotation_err_filtered_derivative = 0
        self.Kp = Point(0.6, 0.6, 1.5)
        self.Kd = Point(1.0, 1.0, 1.5)
        self.Kp_yaw = 0.015 # 0.005
        self.Kd_yaw = 0.01 # 0.2 = 360 deg / 38 sec = 9 deg/sec

        self.orientation_buffer_len = 10
        self.orientation_buffer = self.orientation_buffer_len*[0.0]

        self.allow_slam_control = False

        self.last_time_received_pose = time.time() - 5

        self.land_flag = False

        self.rate = 30

        self.angle = 12.0
        self.angle_radian = self.angle / 180.0 * math.pi

        self.control_mult_factor_x = 2
        self.control_mult_factor_y = 2
        self.control_mult_factor_z = 4

        self.time_threshold = 1


        self.last_time_added_pose = time.time()
        self.time_until_add_pose = 0.3
        self.stamped_pose_trail_list = 30*[PoseStamped()]
        self.pose_trail_list = []
        self.pose_trail_msg = PoseArray()


        self.counter_before_map_exists_flag = 0
        self.counter_before_no_map_flag = 0
        self.MAX_COUNT_BEFORE_NO_MAP_EXISTS_FLAG = 6 * self.rate
        self.MAX_COUNT_BEFORE_MAP_EXISTS_FLAG = 10 * self.rate

        # self.caution_speed_threshold = Point(0.25, 0.25, 0.3)
        self.caution_speed_threshold = Point(0.3, 0.3, 0.5)
        self.caution_speed_yaw = 0.1 # 0.34


        self.map_exists_flag = False

        self.last_position = 0
        self.altitude = 0

        self.rotated_pos_z_ground = 0

        self.calib_altitude_high = 1.4
        self.calib_altitude_low = 0.6
        self.calib_altitude_min = -20
        self.calib_altitude_rate = 0.3

        self.last_time_saved_trajectory = time.time()

        self.slam_pos = Point()
        self.slam_orientation_deg = Point()

        rospy.loginfo('Received Kp = x={} y={} z={} yaw={}'.format(self.Kp.x, self.Kp.y, self.Kp.z, self.Kp_yaw))
        rospy.loginfo('Received Kd = x={} y={} z={} yaw={}'.format(self.Kd.x, self.Kd.y, self.Kd.z, self.Kd_yaw))




        # ROS subscriptions
        # rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, self.slam_callback)
        rospy.Subscriber(self.pose_topic_name,  PoseStamped, self.slam_callback)
        rospy.Subscriber(self.publish_prefix+'command_pos', Pose, self.command_pos_callback)
        rospy.Subscriber(self.publish_prefix+'allow_slam_control', Bool, self.allow_slam_control_callback)
        rospy.Subscriber(self.publish_prefix+'flight_data', FlightData, self.flightdata_callback)
        rospy.Subscriber(self.publish_prefix+'takeoff', Empty, self.takeoff_callback)
        rospy.Subscriber(self.publish_prefix+'calibrate_real_world_scale', Empty, self.calibrate_real_word_scale_callback)
        rospy.Subscriber(self.publish_prefix+'scan_room', Bool, self.scan_room_callback)

        rospy.Subscriber(self.publish_prefix+'kd', Pose, self.kd_callback)
        rospy.Subscriber(self.publish_prefix+'kp', Pose, self.kp_callback)
        rospy.Subscriber(self.publish_prefix+'land', Empty, self.land_callback)

        self.last_time_published = time.time()
        self.thread = threading.Thread(target=self.peridoc_timer, args=())
        self.thread.start()
        

        # Spin until interrupted
        rospy.spin()


    def land_callback(self, msg):
        self.land_flag = True
        self.allow_slam_control = False
        self.pub_twist.publish(self.speed_to_twist())

    def save_trajectory_to_log(self):
        if time.time() - self.last_time_saved_trajectory > 1:
            with open('log_file_trajectory.txt', 'a') as file:
                file.write("%.3f,%.3f,%.3f,%.1f,%.3f,%.3f,%.3f,%.1f\n"%(self.real_world.pose.position.x, self.real_world.pose.position.y, self.real_world.pose.position.z, self.slam_orientation_deg.z, self.command_pos.x, self.command_pos.y, self.command_pos.z, self.command_orientation_deg.z))
            self.last_time_saved_trajectory = time.time()


    def peridoc_timer(self):

        if time.time() - self.last_time_received_pose < self.time_threshold:
            # self.twist.linear = self.calculate_control_algorithm()

            # sign_delta_x = 2 * (self.pos_error.x > 0) - 1
            # sign_delta_y = 2 * (self.pos_error.y > 0) - 1
            # sign_delta_z = 2 * (self.pos_error.z > 0) - 1

            # self.twist.linear.x = sign_delta_x * min(self.control_mult_factor_x*abs(self.pos_error.x), self.caution_speed_threshold.x)
            # self.twist.linear.y = sign_delta_y * min(self.control_mult_factor_y*abs(self.pos_error.y), self.caution_speed_threshold.y)
            # self.twist.linear.z = sign_delta_z * min(self.control_mult_factor_z*abs(self.pos_error.z), self.caution_speed_threshold.z)

            # self.twist.linear = self.calculate_control_algorithm()

            linear, yaw = self.calculate_control_algorithm()

            self.twist.linear = self.clip_point(linear, self.caution_speed_threshold)
            self.twist.angular.z = self.clip_value(yaw, self.caution_speed_yaw)


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
        self.save_trajectory_to_log()

        if not rospy.is_shutdown():
            new_timer = max(1.0/self.rate - (time.time() - self.last_time_published), 1/self.rate/2)
            self.last_time_published = time.time()
            threading.Timer(new_timer, self.peridoc_timer).start()

    def flightdata_callback(self, flight_data):
        self.altitude = flight_data.altitude

    def takeoff_callback(self, msg):
        self.time_of_takeoff = time.time()
        self.land_flag = False

    def sign(self, x):
        return 2 * (x > 0) - 1

    def clip_value(self, val, threshold_val):
        if abs(val) > threshold_val:
            val = self.sign(val) * threshold_val
        return val

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
        speed = Point()


        self.pos_error.x = self.command_pos.x - self.real_world.pose.position.x
        self.pos_error.y = self.command_pos.y - self.real_world.pose.position.y
        self.pos_error.z = self.command_pos.z - self.real_world.pose.position.z

        pos_error_copy = self.point_copy(self.pos_error)

        max_val = 100000000
        if abs(self.command_orientation_deg.z - self.slam_orientation_deg.z) < max_val:
            error = self.command_orientation_deg.z - self.slam_orientation_deg.z
            max_val = abs(error)
        if abs(self.command_orientation_deg.z - self.slam_orientation_deg.z + 360) < max_val:
            error = self.command_orientation_deg.z - self.slam_orientation_deg.z + 360
            max_val = abs(error)
        if abs(self.command_orientation_deg.z - self.slam_orientation_deg.z - 360) < max_val:
            error = self.command_orientation_deg.z - self.slam_orientation_deg.z - 360
            max_val = abs(error)

        self.rotation_error_deg = error



        x_rotated = pos_error_copy.x*math.cos(self.orientation_alpha_rad) + pos_error_copy.y*math.sin(self.orientation_alpha_rad)
        y_rotated = pos_error_copy.y*math.cos(self.orientation_alpha_rad) - pos_error_copy.x*math.sin(self.orientation_alpha_rad)


        self.pos_error.x = x_rotated
        self.pos_error.y = y_rotated

        # self.pos_error.x = pos_error_copy.x
        # self.pos_error.y = pos_error_copy.y


        self.delta_pub.publish(self.pos_error)


        self.pos_err_derivative.x = self.pos_error.x - self.pos_error_prev.x
        self.pos_err_derivative.y = self.pos_error.y - self.pos_error_prev.y
        self.pos_err_derivative.z = self.pos_error.z - self.pos_error_prev.z
        self.rotation_err_derivative = self.rotation_error_deg - self.rotation_error_prev

        self.pos_err_filtered_derivative.x = 0.5648*self.pos_err_filtered_derivative.x + 12.75*self.pos_err_derivative.x
        self.pos_err_filtered_derivative.y = 0.5648*self.pos_err_filtered_derivative.y + 12.75*self.pos_err_derivative.y
        self.pos_err_filtered_derivative.z = 0.5648*self.pos_err_filtered_derivative.z + 12.75*self.pos_err_derivative.z
        self.rotation_err_filtered_derivative = 0.5648*self.rotation_err_filtered_derivative + 12.75*self.rotation_err_derivative

        speed.x = self.Kp.x*self.pos_error.x + self.Kd.x*self.pos_err_filtered_derivative.x
        speed.y = self.Kp.y*self.pos_error.y + self.Kd.y*self.pos_err_filtered_derivative.y
        speed.z = self.Kp.z*self.pos_error.z + self.Kd.z*self.pos_err_filtered_derivative.z
        # speed.z = 0


        # yaw = -(self.Kp_yaw*self.rotation_error_deg)

        yaw = (self.Kp_yaw*self.rotation_error_deg + self.Kd_yaw*self.rotation_err_filtered_derivative)
        # yaw = 0


        self.pos_error_prev = self.point_copy(self.pos_error)
        self.rotation_error_prev = self.rotation_error_deg

        # rospy.loginfo('command_pos = {} real_world = {} pos_error = {} pos_err_derivative = {} pos_err_filtered_derivative = {} point = {}'.format(self.command_pos, self.real_world,
        #     self.pos_error, self.pos_err_derivative, self.pos_err_filtered_derivative, point))

        return speed, yaw

    def rad_to_deg(self, rad):
        return rad / math.pi * 180.0

    def deg_to_rad(self, deg):
        return deg * math.pi / 180.0

    def euler_rad_point_to_quatenrion(self, euler_point):
        return self.orientation_to_quaternion(euler_point.x, euler_point.y, euler_point.z)

    def euler_deg_point_to_quatenrion(self, euler_point):
        return self.orientation_to_quaternion(self.deg_to_rad(euler_point.x), self.deg_to_rad(euler_point.y), self.deg_to_rad(euler_point.z))

    def orientation_to_quaternion(self, pitch, roll, yaw):
        quaternion_list = quaternion_from_euler(pitch, roll, yaw)
        quaternion = Quaternion()
        quaternion.x = quaternion_list[0]
        quaternion.y = quaternion_list[1]
        quaternion.z = quaternion_list[2]
        quaternion.w = quaternion_list[3]
        return quaternion

    def quatenrion_to_euler_point(self, quatenrion):
        euler_list = euler_from_quaternion([quatenrion.x, quatenrion.y, quatenrion.z, quatenrion.w])
        euler = Point()
        euler.x = self.rad_to_deg(euler_list[0])
        euler.y = self.rad_to_deg(euler_list[1])
        euler.z = self.rad_to_deg(euler_list[2])
        return euler
      

    def command_pos_callback(self, command_pos):
        self.command_pos = command_pos.position
        self.command_orientation_deg = self.quatenrion_to_euler_point(command_pos.orientation)
        # self.command_pos = self.rotate_point(self.command_pos, self.slam_orientation_rad_list[2])

    def kd_callback(self, msg):
        self.Kd.x = msg.position.x
        self.Kd.y = msg.position.y
        self.Kd.z = msg.position.z
        self.Kd_yaw = msg.orientation.z
        rospy.loginfo('Received Kd = x={} y={} z={} yaw={}'.format(self.Kd.x, self.Kd.y, self.Kd.z, self.Kd_yaw))

    def kp_callback(self, msg):
        self.Kp.x = msg.position.x
        self.Kp.y = msg.position.y
        self.Kp.z = msg.position.z
        self.Kp_yaw = msg.orientation.z
        rospy.loginfo('Received Kp = x={} y={} z={} yaw={}'.format(self.Kp.x, self.Kp.y, self.Kp.z, self.Kp_yaw))

    def allow_slam_control_callback(self, msg):
        self.allow_slam_control = (msg.data == 1)
        if self.allow_slam_control:
            pass
            # self.rotated_position_0 = self.point_copy(self.rotated_pos)
            # try:
            #     if self.altitude > 0.2:
                    # self.potential_real_world_scale = self.altitude / (self.rotated_pos.z - self.rotated_position_0.z)
                    # if not self.calculated_real_world_scale_flag:
                        # first time calculating real_world_scale
                        # self.real_world_scale = self.potential_real_world_scale
                        # self.real_world_scale_pub.publish(self.real_world_scale)
                        # self.calculated_real_world_scale_flag = True


                    # else:
                        # update only of the real_world_scale is higher than a threshold
                        # if abs(self.real_world_scale-self.potential_real_world_scale) > 1:
                        #     self.real_world_scale_pub.publish(self.real_world_scale)
            # except ZeroDivisionError:
            #     pass

        else:
            self.pub_twist.publish(Twist())


    def speed_to_twist(self, pitch=0, roll=0, throttle=0, yaw=0):
        twist = Twist()
        twist.linear.x = pitch
        twist.linear.y = -roll
        twist.linear.z = throttle
        twist.angular.z = -yaw
        return twist

    def rotate_point(self, point, alpha):
        copy = self.point_copy(point)
        out_point = self.point_copy(point)
        out_point.x = copy.x * math.cos(alpha) + copy.y * math.sin(alpha)
        out_point.y = copy.y * math.cos(alpha) + copy.x * math.sin(alpha)
        return out_point


    def slam_callback(self, slam_msg):
        # if self.command_pos.x == 0 and self.command_pos.y == 0 and self.command_pos.z == 0:
            # return

        # self.rotated_pos = slam_msg.pose.position
        self.slam_pos = slam_msg.pose.position
        self.slam_quaternion = slam_msg.pose.orientation 
        self.slam_orientation_rad_list = euler_from_quaternion([self.slam_quaternion.x, self.slam_quaternion.y, self.slam_quaternion.z, self.slam_quaternion.w])
        self.slam_orientation_rad = Point(self.slam_orientation_rad_list[0], self.slam_orientation_rad_list[1], self.slam_orientation_rad_list[2])
        self.orientation_buffer.pop(0)
        self.orientation_buffer.append(self.slam_orientation_rad.z)
        # self.orientation_alpha_rad = sum(self.orientation_buffer) / len(self.orientation_buffer)
        self.orientation_alpha_rad  = np.median(self.orientation_buffer)
        self.slam_orientation_deg.x = self.slam_orientation_rad.x / math.pi * 180
        self.slam_orientation_deg.y = self.slam_orientation_rad.y / math.pi * 180
        self.slam_orientation_deg.z = self.orientation_alpha_rad / math.pi * 180
        self.slam_orientation_pub.publish(self.slam_orientation_deg)

        self.rotated_pos.x = self.slam_pos.x * math.cos(self.angle_radian) + self.slam_pos.z * math.sin(self.angle_radian)
        self.rotated_pos.y = self.slam_pos.y
        self.rotated_pos.z = self.slam_pos.x * (-math.sin(self.angle_radian)) + self.slam_pos.z * math.cos(self.angle_radian)


        # self.copy_rotated_pos = self.point_copy(self.rotated_pos)

        # self.copy_rotated_pos.x = self.rotated_pos.x * math.cos(self.slam_orientation_rad_list[2]) + self.rotated_pos.y * math.sin(self.slam_orientation_rad_list[2])
        # self.copy_rotated_pos.y = self.rotated_pos.y * math.cos(self.slam_orientation_rad_list[2]) + self.rotated_pos.x * math.sin(self.slam_orientation_rad_list[2])

        # if self.altitude == 0 and time.time() - self.time_of_takeoff < 1:
            # just taking off, check the z_0 position
            # self.rotated_pos_z_ground = self.rotated_pos.z
            # self.rotated_position_0 = self.point_copy(self.rotated_pos)
            # self.rotated_position_0.z = self.rotated_pos.z

        slam_orientation_rad_copy = self.point_copy(self.slam_orientation_rad)
        slam_orientation_rad_copy.y = slam_orientation_rad_copy.y - self.angle_radian
        


        self.real_world.header = slam_msg.header
        self.real_world.pose.position.x = (self.rotated_pos.x - self.rotated_position_0.x)* self.real_world_scale
        self.real_world.pose.position.y = (self.rotated_pos.y - self.rotated_position_0.y)* self.real_world_scale
        self.real_world.pose.position.z = (self.rotated_pos.z - self.rotated_position_0.z)* self.real_world_scale

        self.real_world.pose.orientation = slam_msg.pose.orientation
        # self.real_world.pose.orientation = self.euler_rad_point_to_quatenrion(slam_orientation_rad_copy)

        # self.real_world = self.rotate_point(self.real_world, self.slam_orientation_rad_list[2])

        self.last_time_received_pose = time.time()

        self.real_world_scale_pub.publish(self.real_world_scale)
        self.real_world_pub.publish(self.real_world)
        self.rotated_pos_pub.publish(self.rotated_pos)


        if time.time() - self.last_time_added_pose > self.time_until_add_pose:
            self.stamped_pose_trail_list.append(deepcopy(self.real_world))
            self.stamped_pose_trail_list.pop(0)
            self.pose_trail_list = [element.pose for element in self.stamped_pose_trail_list]
            self.pose_trail_msg.poses = self.pose_trail_list
            self.pose_trail_msg.header.frame_id = 'world'
            self.pose_trail_msg.header.stamp = rospy.Time.now()
            self.pose_trail_publisher.publish(self.pose_trail_msg)

            # self.path_trail_msg.header = deepcopy(slam_msg.header)
            self.path_trail_msg.header.seq += 1
            self.path_trail_msg.header.frame_id = 'world'
            self.path_trail_msg.header.stamp = rospy.Time.now()
            self.path_trail_msg.poses = deepcopy(self.stamped_pose_trail_list)
            self.path_publisher.publish(self.path_trail_msg)


            self.last_time_added_pose = time.time()



    def move_up(self, distance):
        self.move_up_publisher.publish(distance)


    def calibrate_real_word_scale_callback(self, msg):
        # this will begin a process to calibrate the self.real_world_scale using the altitude.
        rospy.loginfo('starting calibration of real world scale')


        # publish to stay in place
        self.pub_twist.publish(self.speed_to_twist())
        # if self.altitude > self.calib_altitude_high:
        #     rospy.loginfo('altitude is {} > {} - calibration failed'.format(self.altitude, self.calib_altitude_high))
        #     return
        # elif self.altitude < self.calib_altitude_min:
        #     rospy.loginfo('altitude is {} < {} - calibration failed'.format(self.altitude, self.calib_altitude_low))
        #     return
        # else:
        #     rospy.loginfo('altitude is {} < {} - starting raise'.format(self.altitude, self.calib_altitude_high))
        # go to altitude = self.calib_altitude_high
        ground_altitude = self.altitude # say altitude suppose to be 0.5 meter.
        rospy.loginfo('ground altitude is {} - starting raise'.format(ground_altitude))

        # while self.altitude < self.calib_altitude_high:
        t = time.time()
        while self.altitude < ground_altitude + 0.6:
            self.pub_twist.publish(self.speed_to_twist(throttle=self.calib_altitude_rate))
            if self.altitude < self.calib_altitude_min or self.land_flag:
                self.pub_twist.publish(self.speed_to_twist())
                rospy.loginfo('altitude is {} > {} - calibration failed'.format(self.altitude, self.calib_altitude_low))
                return
            time.sleep(0.2)
            rospy.loginfo('altitude is {}, desired height is {}'.format(self.altitude, self.calib_altitude_low))
            if time.time() - t > 1:
                if ground_altitude == self.altitude:
                    rospy.loginfo('altitude is fixed, something is wrong.'.format(self.altitude, self.calib_altitude_low))
                    self.pub_twist.publish(self.speed_to_twist())
                    return

        # self.move_up(0.8)
        # time.sleep(3)
        # self.move_up(0.3)
        # time.sleep(2)
            
        # stop
        # wait without blocking
        t = time.time()
        for i in range(3):
            self.speed_to_twist()
        time.sleep(2)
        # while time.time() - t < 2.0:
            # self.pub_twist.publish(self.speed_to_twist())
        self.upper_altitude = self.altitude
        self.upper_z = self.rotated_pos.z
        rospy.loginfo('altitude is {} - z is {} - Stopping'.format(self.upper_altitude, self.upper_z))
        # get rotated slam z position at altitude=self.calib_altitude_high
        # go to altitude = self.calib_altitude_low
        rospy.loginfo('altitude is {} - starting descent'.format(self.altitude))
        t = time.time()
        # while self.altitude > self.calib_altitude_low:
        while self.altitude > ground_altitude + 0.2:
            self.pub_twist.publish(self.speed_to_twist(throttle=-self.calib_altitude_rate))
            # while time.time() - t < 0.05:
            if self.land_flag:
                self.pub_twist.publish(self.speed_to_twist())
                rospy.loginfo('altitude is {} - calibration failed'.format(self.altitude, self.calib_altitude_low))
                return
                # pass
            time.sleep(0.2)
            # t = time.time()
        # stop
        # wait without blocking
        t = time.time()
        while time.time() - t < 2.0:
            time.sleep(0.4)
            self.pub_twist.publish(self.speed_to_twist())
        # get rotated slam z position at altitude=self.calib_altitude_low
        self.lower_altitude = self.altitude
        self.lower_z = self.rotated_pos.z
        rospy.loginfo('altitude is {} - z is {} - Stopping'.format(self.lower_altitude, self.lower_z))
        try:
            self.real_world_scale = (self.upper_altitude - self.lower_altitude) / (self.upper_z - self.lower_z)
        except ZeroDivisionError:
            rospy.loginfo('Error: ZeroDivisionError: lower_z = {}, upper_z = {}, rotated_position_0.z={}'.format(self.lower_z, self.upper_z, self.real_world_scale, self.rotated_position_0.z))
        self.real_world_scale_pub.publish(self.real_world_scale)
        self.rotated_position_0.z = self.lower_z - self.calib_altitude_low/self.real_world_scale
        rospy.loginfo('lower_z = {}, upper_z = {}, real_world_scale={}, rotated_position_0.z={}'.format(self.lower_z, self.upper_z, self.real_world_scale, self.rotated_position_0.z))


    def scan_room_callback(self, msg):
        direction = msg.data * 2 - 1
        rospy.loginfo('starting room scan')
        self.pub_twist.publish(self.speed_to_twist())

        time_threshold_lost_camera = 0.3

        # wait until there is map
        rospy.loginfo('wait until there is map')
        while time.time() - self.last_time_received_pose > time_threshold_lost_camera:
            pass

        # remember that starting orientation
        starting_orientation = self.slam_orientation_deg.z + 180 # 0-360 degree
        rospy.loginfo('starting orientation = {}'.format(starting_orientation))
        number_of_times_stuck = 0
        throttle_dir = 1

        # start yawing
        while abs((self.slam_orientation_deg.z + 180 + 10*direction)%360 - starting_orientation) > 1:
            self.pub_twist.publish(self.speed_to_twist(yaw=-0.2*direction))
            # check if need to abort.
            if self.allow_slam_control:
                self.pub_twist.publish(self.speed_to_twist())
                rospy.loginfo('finished with error')
                return

            # check if didn't received position lately.
            if time.time() - self.last_time_received_pose > time_threshold_lost_camera:
                # lost map - rotate back
                while time.time() - self.last_time_received_pose > time_threshold_lost_camera:
                    # rotate until returned to map
                    rospy.loginfo('lost_camera = orientation = {}'.format(self.slam_orientation_deg.z + 180))
                    self.pub_twist.publish(self.speed_to_twist(yaw=0.2*direction))
                    # check if need to abort.
                    if self.allow_slam_control:
                        self.pub_twist.publish(self.speed_to_twist())
                        rospy.loginfo('finished with error')
                        return


                t = time.time()
                # increment number of times lost map
                number_of_times_stuck += 1
                if number_of_times_stuck == 10:
                    # if number of times lost map is grater than a threshild, than go a bit up or down
                    number_of_times_stuck = 0
                    while time.time() - t < 0.4:
                        self.pub_twist.publish(self.speed_to_twist(throttle=0.3*throttle_dir))
                        # check if need to abort.
                        if self.allow_slam_control or self.altitude > 1.7:
                            self.pub_twist.publish(self.speed_to_twist())
                            rospy.loginfo('finished with error')
                            return
                    # change direction for next time
                    throttle_dir = -throttle_dir

                # wait a little to catch the map
                while time.time() - t < 1:
                    self.pub_twist.publish(self.speed_to_twist())
                    if self.allow_slam_control:
                        self.pub_twist.publish(self.speed_to_twist())
                        rospy.loginfo('finished with error')
                        return

        rospy.loginfo('finished')
        self.pub_twist.publish(self.speed_to_twist())


if __name__ == '__main__':
    driver = TelloSlamControler()
