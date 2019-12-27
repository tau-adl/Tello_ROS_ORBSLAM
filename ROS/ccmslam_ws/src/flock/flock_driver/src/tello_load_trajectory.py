#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose, Quaternion
from std_msgs.msg import Empty, Bool, Int32, Float32, String
from flock_msgs.msg import Flip, FlightData
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import threading
import math

# rostopic pub /tello/trajectory_path std_msgs/String "data: '/home/arkadiros/ROS/tello_catkin_ws/src/flock/flock_driver/src/trajectory_list.txt'" 
# rostopic pub /tello/trajectory_path std_msgs/String "data: '/home/arkadiros/ROS/tello_catkin_ws/src/flock/flock_driver/src/salon_route_from_entrance.csv'"


class TelloLoadTrajectory(object):

    def __init__(self):
        # Initialize ROS
        rospy.init_node('tello_load_trajectory', anonymous=False)
        

        # ROS subscriptions
        rospy.Subscriber('/tello/trajectory_path', String, self.trajectory_path_callback)
        rospy.Subscriber('/tello/real_world_pos', PoseStamped, self.real_world_pos_callback) 
        rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, self.slam_callback)
        rospy.Subscriber('/tello/command_pos', Pose, self.command_pos_callback)

        self.command_pos_publisher = rospy.Publisher('/tello/command_pos', Pose, queue_size = 1)
        self.path_publisher = rospy.Publisher('/tello/path', Path, queue_size = 1)

        self.trajectory_list = []


        self.command_pos = Pose()
        self.received_command_pos = Pose()
        self.slam_pos = Pose()
        self.command_pos_orientation_deg = Point()
        self.threshold = Point(0.15, 0.15, 0.15)
        self.threshold_orientation = Point(10, 10, 10)
        self.real_world_pos = Point(10, 10, 10)

        self.path = Path()
        self.pose_stamped = PoseStamped()

        self.received_trajectory_flag = False
        self.current_trajectory_index = 0

        self.main()

        # Spin until interrupted
        rospy.spin()

    def command_pos_callback(self, command_pos):
        self.received_command_pos = command_pos
        # rospy.loginfo("Received Command {}".format(self.received_command_pos))

    def rad_to_deg(self, rad):
        return rad / math.pi * 180.0

    def deg_to_rad(self, deg):
        return deg * math.pi / 180.0

    def slam_callback(self, slam_msg):
        self.slam_pos = slam_msg.pose.position
        self.slam_quaternion = slam_msg.pose.orientation 
        self.slam_orientation_deg = self.quatenrion_point_to_euler_degree(self.slam_quaternion)

    def real_world_pos_callback(self, msg):
        self.real_world_pos = msg.pose.position

    def quatenrion_point_to_euler_degree(self, slam_quaternion):
        rad = self.quatenrion_point_to_euler(slam_quaternion)
        return Point(self.rad_to_deg(rad.x), self.rad_to_deg(rad.y), self.rad_to_deg(rad.z))

    def quatenrion_point_to_euler(self, orientation_point):
        return self.quaternion_to_orientation(orientation_point.x, orientation_point.y, orientation_point.z, orientation_point.w)

    def euler_point_deg_to_rad(self, point_deg):
        return Point(self.deg_to_rad(point_deg.x), self.deg_to_rad(point_deg.y), self.deg_to_rad(point_deg.z))


    def euler_point_deg_to_quatenrion(self, euler_point_deg):
        return self.euler_point_to_quatenrion(self.euler_point_deg_to_rad(euler_point_deg))

    def euler_point_to_quatenrion(self, euler_point):
        return self.orientation_to_quaternion(euler_point.x, euler_point.y, euler_point.z)

    def quaternion_to_orientation(self, x, y, z, w):
        euler_list = euler_from_quaternion([x, y, z, w])
        euler = Point()
        euler.x = euler_list[0]
        euler.y = euler_list[1]
        euler.z = euler_list[2]
        return euler

    def orientation_to_quaternion(self, pitch, roll, yaw):
        quaternion_list = quaternion_from_euler(pitch, roll, yaw)
        quaternion = Quaternion()
        quaternion.x = quaternion_list[0]
        quaternion.y = quaternion_list[1]
        quaternion.z = quaternion_list[2]
        quaternion.w = quaternion_list[3]
        # rospy.loginfo("quaternion={}".format(quaternion))
        return quaternion

    def load_from_csv(self, msg_path):
        path = msg_path.data
        print(path)
        file = open(path, 'r')
        out = []
        data = file.readlines()
        for line in data:
            temp_lst = line.strip().replace('\n','').split(',')
            final_lst = [float(element) for element in temp_lst]
            out.append(final_lst)

        # list_of_lists = [element.strip().replace('\n','').split(',') for element in data]
        # out = [float(coordinate) for coordinate in lst for lst in list_of_lists]
        return out

    def trajectory_path_callback(self, trajectory_path):
        self.trajectory_path = trajectory_path
        self.trajectory_list = self.load_from_csv(self.trajectory_path)
        self.received_trajectory_flag = True
        self.current_trajectory_index = 0
        self.command_pos = self.received_command_pos
        rospy.loginfo("Trajectory have Gained Control")

    def find_min_distance_in_orientation(self, ori1, ori2):
        max_val = 100000000
        if abs(ori1 - ori2) < max_val:
            error = ori1 - ori2
            max_val = abs(error)
        if abs(ori1 - ori2 + 360) < max_val:
            error = ori1 - ori2 + 360
            max_val = abs(error)
        if abs(ori1 - ori2 - 360) < max_val:
            error = ori1 - ori2 - 360
            max_val = abs(error)
        return max_val


    def main(self):
        while not rospy.is_shutdown():
            if self.received_trajectory_flag:

                if not self.command_pos == self.received_command_pos:
                    self.received_trajectory_flag = False
                    rospy.loginfo("Trajectory have Lost control")
                    continue

                if abs(self.command_pos.position.x - self.real_world_pos.x) < self.threshold.x:
                    if abs(self.command_pos.position.y - self.real_world_pos.y) < self.threshold.y:
                        if abs(self.command_pos.position.z - self.real_world_pos.z) < self.threshold.z:
                            if self.find_min_distance_in_orientation(self.command_pos_orientation_deg.z, self.slam_orientation_deg.z) < self.threshold_orientation.z:
                                self.current_trajectory_index += 1

                if self.current_trajectory_index < len(self.trajectory_list):
                    self.command_pos.position.x = self.trajectory_list[self.current_trajectory_index][0]
                    self.command_pos.position.y = self.trajectory_list[self.current_trajectory_index][1]
                    self.command_pos.position.z = self.trajectory_list[self.current_trajectory_index][2]
                    self.command_pos_orientation_deg.z = self.trajectory_list[self.current_trajectory_index][3]
                    self.command_pos.orientation = self.euler_point_deg_to_quatenrion(self.command_pos_orientation_deg)


                    self.command_pos_publisher.publish(self.command_pos)

                    self.pose_stamped.header.seq += 1
                    self.pose_stamped.header.stamp = rospy.Time.now()
                    self.pose_stamped.header.frame_id = 'map'
                    pose_stamped = PoseStamped()
                    pose_stamped.header.seq = self.pose_stamped.header.seq
                    pose_stamped.header.stamp = rospy.Time.now()
                    # self.pose_stamped.header.stamp = rospy.Time.now()
                    pose_stamped.header.frame_id = 'map'

                    pose_stamped.pose.position.x = self.command_pos.position.x
                    pose_stamped.pose.position.y = self.command_pos.position.y
                    pose_stamped.pose.position.z = self.command_pos.position.z

                    pose_stamped.pose.orientation.x = self.command_pos.orientation.x
                    pose_stamped.pose.orientation.y = self.command_pos.orientation.y
                    pose_stamped.pose.orientation.z = self.command_pos.orientation.z
                    pose_stamped.pose.orientation.w = self.command_pos.orientation.w

                    self.path.header = pose_stamped.header
                    self.path.poses.append(pose_stamped)
                    self.pose_stamped = pose_stamped
                    self.path_publisher.publish(self.path)

                else:
                    self.received_trajectory_flag = False
                    rospy.loginfo("Trajectory Finished")

                    # rospy.loginfo("self.command_pos_orientation_deg = {} {}".format(self.command_pos_orientation_deg.z, self.command_pos.orientation))
                # time.sleep(1)
            time.sleep(1)


if __name__ == '__main__':
    driver = TelloLoadTrajectory()
