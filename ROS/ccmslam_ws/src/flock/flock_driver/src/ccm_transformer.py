#!/usr/bin/env python

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
import tf.transformations
import tf2_geometry_msgs
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty, Bool, Int32, Float32 
from copy import deepcopy
import Tkinter as tki
import math
import struct

class CcmTransformer(object):
    def __init__(self):
        rospy.init_node('ccm_transformer', anonymous=False)


        self.position = Point()
        self.orientation = Point()
        self.pose_0_transformed = PoseStamped()
        self.pose_1_transformed = PoseStamped()
        self.pose_0 = PoseStamped()
        self.pose_1 = PoseStamped()
        self.position_transformed_0 = Point()
        self.position_transformed_1 = Point()
        self.orientation_deg_transformed_0 = Point()
        self.orientation_deg_transformed_1 = Point()

        self.rotated_position_0 = Point()
        self.rotated_position_1 = Point()

        self.transform_msg_0 = TransformStamped()
        self.transform_msg_1 = TransformStamped()

        self.transformer_state_0 = False
        self.transformer_state_1 = False

        self.tello_pose_pub_0 = rospy.Publisher('/tello0/pose', PoseStamped, queue_size = 1)
        self.tello_pose_pub_1 = rospy.Publisher('/tello1/pose', PoseStamped, queue_size = 1)

        self.tello_pose_transform_pub_0 = rospy.Publisher('/tello0/pose_transform', PoseStamped, queue_size = 1)
        self.tello_pose_transform_pub_1 = rospy.Publisher('/tello1/pose_transform', PoseStamped, queue_size = 1)

        self.point_cloud_transformed_pub_0 = rospy.Publisher('/tello0/point_cloud', PointCloud2, queue_size = 5)
        self.point_cloud_transformed_pub_1 = rospy.Publisher('/tello1/point_cloud', PointCloud2, queue_size = 5)

        self.slam_z0_0_pub = rospy.Publisher('/tello0/slam_z0_transformed', Point, queue_size = 1)
        self.slam_z0_1_pub = rospy.Publisher('/tello1/slam_z0_transformed', Point, queue_size = 1)

        rospy.Subscriber('/tello0/TransformerState', Bool, self.transformer_state_0_callback)
        rospy.Subscriber('/tello1/TransformerState', Bool, self.transformer_state_1_callback)

        rospy.Subscriber('/ccmslam/PoseOutClient0', PoseStamped, self.pose0_callback)
        rospy.Subscriber('/ccmslam/PoseOutClient1', PoseStamped, self.pose1_callback)
        rospy.Subscriber('/ccmslam/TransOutServer0/', TransformStamped, self.transform0_callback)
        rospy.Subscriber('/ccmslam/TransOutServer1/', TransformStamped, self.transform1_callback)

        rospy.Subscriber('/ccmslam/ClientMapPointsMap0/', PointCloud2, self.point_cloud_callback_0)
        rospy.Subscriber('/ccmslam/ClientMapPointsMap1/', PointCloud2, self.point_cloud_callback_1)

        rospy.Subscriber('/tello0/slam_z0/', Point, self.slam_z0_0_callback)
        rospy.Subscriber('/tello1/slam_z0/', Point, self.slam_z0_1_callback)

        


        rospy.spin()


    def quatenrion_point_to_euler_degree(self, slam_quaternion):
        if not type(slam_quaternion) == Quaternion:
            raise ValueError("Did not receive Quaternion. received {} instead".format(type(slam_quaternion)))
        rad = self.quatenrion_point_to_euler(slam_quaternion)
        return Point(self.rad_to_deg(rad.x), self.rad_to_deg(rad.y), self.rad_to_deg(rad.z))

    def quatenrion_point_to_euler(self, orientation_point):
        # print

        # if not type(orientation_point) == Quaternion:
        #     raise ValueError("Did not receive Quaternion. received {} instead".format(type(orientation_point)))
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

    def clip_angle(self, angle, PI=180):
        # Retreive supplementary turns (in radians)
        while(angle >= 2*PI):
            angle -= 2*PI
        while(angle <= -2*PI):
            angle += 2*PI
        # Select shortest rotation to reach the target
        if(angle > PI):
            angle -= 2*PI
        elif(angle < -PI):
            angle += 2*PI
        return angle

    def rad_to_deg(self, rad):
        return rad / math.pi * 180.0

    def deg_to_rad(self, deg):
        return deg * math.pi / 180.0


    def qv_mult(self, q1, v1):
        # v1 = tf.transformations.unit_vector(v1)
        q2 = list(v1)
        q2.append(0.0)
        return tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(q1, q2), tf.transformations.quaternion_conjugate(q1))[:3]

    def quat_to_vect(self, q):
        return [q.x, q.y, q.z, q.w]

    def point_to_vect(self, v):
        return [v.x, v.y, v.z]

    def extract_from_pose_msg(self, pose_msg):
        '''
            arg pose_msg - pose of type PoseStamped
            the function extracts and converts the pose and
            returns RPY, Position, Quaternion
        '''
        if not type(pose_msg) == PoseStamped:
            raise ValueError("Did not receive PoseStamped. received {} instead".format(type(pose_msg)))


        # pose_msg is PoseStamped
        if not type(pose_msg.pose) == Pose:
            raise ValueError("Did not receive Pose. received {} instead".format(type(pose_msg.pose)))

        if not type(pose_msg.pose.orientation) == Quaternion:
            # q = Quaternion(pose_msg.pose.orientation[0], pose_msg.pose.orientation[1], pose_msg.pose.orientation[2], pose_msg.pose.orientation[3])
            raise ValueError("Did not receive Quaternion. received {} instead".format(type(pose_msg.pose.orientation)))
        # else:
        q = pose_msg.pose.orientation

        position = pose_msg.pose.position
        orientation_deg = self.quatenrion_point_to_euler_degree(q)
        # orientation_deg.x = self.clip_angle(orientation_deg.x)
        # orientation_deg.y = self.clip_angle(orientation_deg.y)
        # orientation_deg.z = self.clip_angle(orientation_deg.z)
        orientation_quat = deepcopy(pose_msg.pose.orientation)
        return orientation_deg, position, orientation_quat

    def apply_coordinate_position_rotation(self, position):
        out = Point()
        out.x = position.z
        out.y = -position.x
        out.z = -position.y
        return out


    def apply_coordinate_rotation(self, pose_msg):
        '''
            new_pitch = old_roll
            new_roll = -old_yaw
            new_yaw = old_pitch
            
            new_x = old z
            new_y = -old_x
            new_z = -old_y
        '''
        orientation_deg, position, orientation_quat = self.extract_from_pose_msg(pose_msg)
        rotated_orientation_deg = Point()
        # rotated_orientation_deg.x = orientation_deg.y # new_pitch = old_roll
        # rotated_orientation_deg.y = -orientation_deg.z # new_roll = -old_yaw
        # rotated_orientation_deg.z = orientation_deg.x # new_yaw = old_pitch

        rotated_orientation_deg.x = orientation_deg.x
        rotated_orientation_deg.y = orientation_deg.z
        rotated_orientation_deg.z = orientation_deg.y

        rotated_position = Point()
        rotated_position = self.apply_coordinate_position_rotation(position)

        pose_rotated = PoseStamped()
        pose_rotated.header = pose_msg.header
        pose_rotated.pose.position = rotated_position
        pose_rotated.pose.orientation = self.euler_point_deg_to_quatenrion(rotated_orientation_deg)

        return pose_rotated

    def apply_transform_to_pose(self, pose_msg, transform_msg):
        '''
            this function receives a pose_msg and transform_msg and 
            returns PoseStamped transformed.
        '''
        pose_transformed = PoseStamped()
        pose_transformed.header = pose_msg.header
        try:
            transform_scale = float(transform_msg.child_frame_id)
        except:
            transform_scale = -1
        quat_multiplied = quaternion_multiply(self.quat_to_vect(pose_msg.pose.orientation), 
                                                tf.transformations.quaternion_conjugate(self.quat_to_vect(transform_msg.transform.rotation)))
        pose_transformed.pose.orientation = Quaternion(quat_multiplied[0], quat_multiplied[1], quat_multiplied[2], quat_multiplied[3])

        rot = transform_msg.transform.rotation
        q = [rot.x, rot.y, rot.z, rot.w]
        pos = pose_msg.pose.position
        v = [pos.x, pos.y, pos.z]
        # print("v = {}".format(v))

        new_v = self.qv_mult(q, v)
        # print("new_v = {}".format(new_v))
        pose_transformed.pose.position.x = new_v[0]*transform_scale + transform_msg.transform.translation.x
        pose_transformed.pose.position.y = new_v[1]*transform_scale + transform_msg.transform.translation.y
        pose_transformed.pose.position.z = new_v[2]*transform_scale + transform_msg.transform.translation.z

        return pose_transformed

    def pose0_callback(self, pose_msg):
        self.pose_0 = deepcopy(pose_msg)
        if self.transformer_state_0 == 1:
            self.pose_0 = self.apply_transform_to_pose(self.pose_0, self.transform_msg_0)
        self.tello_pose_transform_pub_0.publish(self.pose_0)
        self.pose_0 = self.apply_coordinate_rotation(self.pose_0)
        self.tello_pose_pub_0.publish(self.pose_0)

    def pose1_callback(self, pose_msg):
        self.pose_1 = deepcopy(pose_msg)
        if self.transformer_state_1 == 1:
            self.pose_1 = self.apply_transform_to_pose(self.pose_1, self.transform_msg_1)
        self.tello_pose_transform_pub_1.publish(self.pose_1)
        self.pose_1 = self.apply_coordinate_rotation(self.pose_1)
        self.tello_pose_pub_1.publish(self.pose_1)


    def transform0_callback(self, msg):
        '''
            this function only saves the transformation.
        '''
        self.transform_msg_0 = deepcopy(msg)

    def transform1_callback(self, msg):
        '''
            this function only saves the transformation.
        '''
        self.transform_msg_1 = deepcopy(msg)


    def transformer_state_0_callback(self, msg):
        self.transformer_state_0 = msg.data
        rospy.loginfo("Change Transformer 0 State to %d", self.transformer_state_0)
        # self.slam_z0_0_pub.publish(self.rotated_position_1)

    def transformer_state_1_callback(self, msg):
        self.transformer_state_1 = msg.data
        try:
            rospy.loginfo("Change Transformer 1 State to %d", self.transformer_state_1)
        except Exception as e:
            print(e)
            print(self.transformer_state_1)
        if self.transformer_state_1 == True:
            self.slam_z0_1_pub.publish(self.rotated_position_0)
        else:
            self.slam_z0_1_pub.publish(self.rotated_position_1)


    def print_pose(self, name, Pose):
        print(name)
        deg = Pose[0]
        pos = Pose[1]
        quat = Pose[2]
        print("position: x={}, y={}, z={}".format(pos.x, pos.y, pos.z))
        print("quarternion: x={}, y={}, z={}, w={}".format(quat.x, quat.y, quat.z, quat.w))
        print("angles: roll={}, pitch={}, yaw={}".format(deg.x, deg.y, deg.z))


    def float_from_4_bytes(self, bytes_list):
        '''
            this function receives 4 bytes and returns float
        '''
        return struct.unpack('f', bytes_list)[0]

    def point_cloud_callback(self, point_cloud_msg):
        '''
            this function receives point cloud message, and returns list of Point's.
        '''
        fields_str = str(point_cloud_msg.fields)
        # file.write(fields_str+'\n')

        # point_cloud_msg.data is list of uint8[].
        # every 4 elements, is one float32
        # convert cloud.data to list where each element in a 4 byte list
        list_of_4_bytes = [point_cloud_msg.data[x:x+4] for x in range(0, len(point_cloud_msg.data), 4)]
        # convert every 4 bytes to a float32
        list_of_floats = [self.float_from_4_bytes(element) for element in list_of_4_bytes]
        # convert every float to string, and make sure there are only 4 zeros after the point (anti float32....)
        # list_of_strings = ['%.4f' % element for element in list_of_floats]
        # print(list_of_strings)

        ccm_slam_mode = 'rgb' in fields_str
        # print("fields_str={}".format(fields_str))

        # convert to list of points:
        list_of_points = [Point(list_of_floats[x], list_of_floats[x+1], list_of_floats[x+2]) for x in range(0, len(list_of_floats), 3+ccm_slam_mode)]
        # list_of_points = [point for point in list_of_points if (not point == Point() and 0.3 < point.z < 0.5)]
        list_of_points = [point for point in list_of_points if (not point == Point())]

        # print('len(list_of_points)={}'.format(len(list_of_points)))

        return list_of_points

    def apply_transform_to_point(self, point, transform_msg):
        '''
            this function receives a point and transform_msg and 
            returns PoseStamped transformed.
        '''
        transform_scale = float(transform_msg.child_frame_id)
        point_transformed = Point()
        rot = transform_msg.transform.rotation
        q = [rot.x, rot.y, rot.z, rot.w]
        v = [point.x, point.y, point.z]
        # print("v = {}".format(v))

        new_v = self.qv_mult(q, v)
        # print("new_v = {}".format(new_v))
        point_transformed.x = new_v[0]*transform_scale + transform_msg.transform.translation.x
        point_transformed.y = new_v[1]*transform_scale + transform_msg.transform.translation.y
        point_transformed.z = new_v[2]*transform_scale + transform_msg.transform.translation.z

        # point_transformed = self.apply_coordinate_position_rotation(point_transformed)

        return point_transformed

    def from_list_of_points_to_point_cloud(self, list_of_points):
        point_cloud = PointCloud2()
        for point in list_of_points:
            point_cloud.data += struct.pack('f', point.x)
            point_cloud.data += struct.pack('f', point.y)
            point_cloud.data += struct.pack('f', point.z)
            # point_cloud.data += struct.pack('f', 0)
        return point_cloud

    def point_cloud_callback_0(self, point_cloud_msg):
        # print("point_cloud_msg", len(point_cloud_msg.data), point_cloud_msg.height, point_cloud_msg.width, point_cloud_msg.point_step)
        list_of_points = self.point_cloud_callback(point_cloud_msg)
        list_of_transformed_points = [self.apply_transform_to_point(point, self.transform_msg_0) for point in list_of_points]
        point_cloud_out_msg = self.from_list_of_points_to_point_cloud(list_of_transformed_points)
        point_cloud_out_msg.header = point_cloud_msg.header
        point_cloud_out_msg.header.frame_id = 'world'
        point_cloud_out_msg.fields = point_cloud_msg.fields[:3]
        # point_cloud_out_msg.fields = point_cloud_msg.fields
        # print("type(point_cloud_msg.fields)={}".format(type(point_cloud_msg.fields)))
        # for i, element in enumerate(point_cloud_msg.fields):
        #     print("type(point_cloud_msg.fields[i]={}".format(type(element)))
        #     print("point_cloud_msg.fields[i]={}".format(element))
        # # print("point_cloud_msg.fields={}".format(point_cloud_msg.fields))
        point_cloud_out_msg.is_dense = True
        point_cloud_out_msg.height = len(list_of_transformed_points)
        point_cloud_out_msg.width = 1
        point_cloud_out_msg.point_step = 12
        point_cloud_out_msg.is_bigendian = point_cloud_msg.is_bigendian
        # print("point_cloud_out_msg", len(point_cloud_out_msg.data), point_cloud_out_msg.height, point_cloud_out_msg.width, point_cloud_out_msg.point_step)
        self.point_cloud_transformed_pub_0.publish(point_cloud_out_msg)
    
    def point_cloud_callback_1(self, point_cloud_msg):
        list_of_points = self.point_cloud_callback(point_cloud_msg)
        list_of_transformed_points = [self.apply_transform_to_point(point, self.transform_msg_1) for point in list_of_points]
        point_cloud_out_msg = self.from_list_of_points_to_point_cloud(list_of_transformed_points)
        point_cloud_out_msg.header = point_cloud_msg.header
        point_cloud_out_msg.header.frame_id = 'world'
        point_cloud_out_msg.fields = point_cloud_msg.fields[:3]
        point_cloud_out_msg.is_dense = True
        # point_cloud_out_msg.height = point_cloud_msg.height
        point_cloud_out_msg.height = len(list_of_transformed_points)
        point_cloud_out_msg.width = 1
        point_cloud_out_msg.point_step = 12
        point_cloud_out_msg.is_bigendian = point_cloud_msg.is_bigendian
        self.point_cloud_transformed_pub_1.publish(point_cloud_out_msg)

    def slam_z0_0_callback(self, msg):
        self.rotated_position_0 = msg

    def slam_z0_1_callback(self, msg):
        self.rotated_position_1 = msg







if __name__ == '__main__':
    # root = tki.Tk()
    A = CcmTransformer()
    # root.mainloop()