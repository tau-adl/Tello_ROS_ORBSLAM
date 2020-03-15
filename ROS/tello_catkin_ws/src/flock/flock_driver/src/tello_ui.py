#!/usr/bin/env python

import rospy
import Tkinter as tki
import tkFileDialog
from Tkinter import Toplevel, Scale
import threading
import os
import time
import platform
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Empty, Bool, Int32, Float32
from flock_msgs.msg import Flip, FlightData
import signal
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# PACKAGE = "orb_slam2_ros"
# import roslib;roslib.load_manifest(PACKAGE)
# import dynamic_reconfigure.client



class TelloUI(object):
    """Wrapper class to enable the GUI."""

    def __init__(self, root):

        rospy.init_node('tello_ui', anonymous=False)

        # rospy.Subscriber('/command_pos', Point, self.command_pos_callback)

        signal.signal(signal.SIGINT, self.onClose)
        signal.signal(signal.SIGTERM, self.onClose)

        self.quit_flag = False

        try: 
            self.id                = rospy.get_param('~ID')
        except KeyError:
            self.id = ''
        self.publish_prefix = "tello{}/".format(self.id)



        try:
            self.pose_topic_name = rospy.get_param('~POSE_TOPIC_NAME')
        except KeyError:
            self.pose_topic_name = '/ccmslam/PoseOutClient'+str(self.id)


        self.point_command_pos = Point(0.0, 0.0, 1.0)
        self.point_command_pos_yaw = 0.0
        self.command_pos = Pose()
        self.rotated_pos = Point()
        self.slam_pos = Point()
        self.twist_manual_control = Twist()
        self.real_world_pos = Point()
        self.delta_pos = Point()
        self.orientation_degree = Point()

        self.allow_slam_control = False

        self.current_mux = 0
        # initialize the root window and image panel
        self.root = root

        self.panel = None
       # self.panel_for_pose_handle_show = None

        # self.client = dynamic_reconfigure.client.Client("orb_slam2_mono")

        self.use_merge_coordinates = False
       

        # create buttons
        self.land = True

        self.column = 0
        self.row = 0
        self.frame_column = 0
        self.frame_row = 0

        self.angle_delta_x = 0
        self.angle_delta_y = 0
        self.angle_delta_z = 0
        self.angle = 12.0
        self.angle_radian = self.angle / 180.0 * math.pi

        self.real_world_scale = 3.9636
        self.altitude = 0

        self.init_command_pos_frame_flag = False
        self.init_real_world_frame_flag = False
        self.init_slam_pose_frame_flag = False
        self.init_delta_frame_flag = False
        self.init_speed_frame_flag = False
        self.init_info_frame_flag = False
        self.init_manual_control_frame_flag = False
        self.init_angle_calc_frame_flag = False
        self.init_rotated_frame_flag = False

        self.trajectory_threshold = Point(0.25, 0.25, 0.25)
        self.trajectory_orientation_threshold = Point(3, 3, 3)
        self.trajectory_list = []
        self.last_trajectory = []

        self.lock = threading.Lock()

        self.default_bg = '#d9d9d9'



        self.left_frame = tki.Frame(self.root, relief=tki.SUNKEN)
        self.left_frame.grid(row=0, column=0)

        self.init_command_pos_frame(self.left_frame)

        self.init_real_world_frame(self.left_frame)

        self.init_rotated_frame(self.left_frame)

        self.init_slam_pose_frame(self.left_frame)

        self.init_delta_frame(self.left_frame)

        self.init_speed_frame(self.left_frame)

        self.init_info_frame(self.left_frame)

        self.init_manual_control_frame(self.left_frame)

        # self.init_angle_calc_frame()

        # self.init_kd_kp_frame()

        self.number_of_trajectory_points_ui = 10

        self.right_frame = tki.Frame(self.root, relief=tki.SUNKEN)
        self.right_frame.grid(row=0, column=1)
        self.row = 0
        self.column = 0

        self.init_merge_map_frame(self.right_frame)
        self.init_trajectory_frame(self.right_frame)





        self.update_command_pos_to_gui()





        self.row += 1
        self.column = 0


        # set a callback to handle when the window is closed
        self.root.wm_title("TELLO Controller"+str(self.id))
        self.root.wm_protocol("WM_DELETE_WINDOW", self.onClose)

        self.kd = Pose()
        self.kp = Pose()

        # rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, self.slam_callback)
        rospy.Subscriber(self.pose_topic_name, PoseStamped, self.slam_callback)
        rospy.Subscriber(self.publish_prefix+'delta_pos', Point, self.delta_pos_callback)
        rospy.Subscriber(self.publish_prefix+'cmd_vel', Twist, self.speed_callback)
        rospy.Subscriber(self.publish_prefix+'flight_data', FlightData, self.flightdata_callback)
        rospy.Subscriber(self.publish_prefix+'allow_slam_control', Bool, self.allow_slam_control_callback)
        rospy.Subscriber(self.publish_prefix+'real_world_scale', Float32, self.real_world_scale_callback)
        rospy.Subscriber(self.publish_prefix+'real_world_pos', PoseStamped, self.real_world_pos_callback) 
        rospy.Subscriber(self.publish_prefix+'rotated_pos', Point, self.rotated_pos_callback)
        rospy.Subscriber(self.publish_prefix+'command_pos', Pose, self.command_pos_callback)
        rospy.Subscriber(self.publish_prefix+'orientation', Point, self.orientation_callback)

        self.command_pos_publisher = rospy.Publisher(self.publish_prefix+'command_pos', Pose, queue_size = 1)
        self.pub_takeoff = rospy.Publisher(self.publish_prefix+'takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher(self.publish_prefix+'land', Empty, queue_size=1)
        self.pub_allow_slam_control = rospy.Publisher(self.publish_prefix+'allow_slam_control', Bool, queue_size=1)
        self.cmd_val_publisher = rospy.Publisher(self.publish_prefix+'cmd_vel', Twist, queue_size = 1)
        self.calibrate_real_world_scale_publisher = rospy.Publisher(self.publish_prefix+'calibrate_real_world_scale', Empty, queue_size = 1)
        self.scan_room_publisher = rospy.Publisher(self.publish_prefix+'scan_room', Bool, queue_size = 1)
        self.kd_publisher = rospy.Publisher(self.publish_prefix+'kd', Pose, queue_size = 1)
        self.kp_publisher = rospy.Publisher(self.publish_prefix+'kp', Pose, queue_size = 1)
        self.kp_publisher = rospy.Publisher(self.publish_prefix+'kp', Pose, queue_size = 1)
        self.pub_mux =  rospy.Publisher('tello_mux', Int32, queue_size = 1)
        self.path_publisher = rospy.Publisher(self.publish_prefix+'path', Path, queue_size = 1)
        self.take_picure_publisher = rospy.Publisher(self.publish_prefix+'take_picure', Empty, queue_size=1)
        self.merge_coordinates_pub = rospy.Publisher(self.publish_prefix+'TransformerState', Bool, queue_size=1)
        

        self.publish_command()




    def nothing(self):
        rospy.loginfo("nothing")
        return


    def init_command_pos_frame(self, root_frame):
        self.init_command_pos_frame_flag = True
        self.frame_command = tki.Frame(root_frame, relief=tki.SUNKEN, borderwidth = 1)
        rospy.loginfo("command_pos_frame in row={} column={}".format(self.row, self.column))
        self.frame_command.grid(row=self.row, column=self.column)

        self.current_frame = self.frame_command

        self.title_label_command_pos = tki.Label(self.current_frame, text="Command Position[Meters]", font=("Helvetica", 13))
        self.title_label_command_pos.grid(row=self.frame_row, column=1)#, padx=3, pady=3)

        self.frame_row += 1

        self.command_label_x = tki.Label(self.current_frame, text="X[m]")
        self.command_label_x.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.command_label_y = tki.Label(self.current_frame, text="Y[m]")
        self.command_label_y.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.command_label_z = tki.Label(self.current_frame, text="Z[m]")
        self.command_label_z.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.command_label_yaw = tki.Label(self.current_frame, text="Yaw[Degree]")
        self.command_label_yaw.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.frame_row += 1
        self.frame_column = 0

        self.command_strigvar_x = tki.StringVar()
        self.command_entry_x = tki.Entry(self.current_frame, width=9, textvariable=self.command_strigvar_x)
        self.command_entry_x.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.command_entry_x.delete(0, tki.END)
        self.command_entry_x.insert(0, "0.0")

        self.frame_column += 1

        self.command_strigvar_y = tki.StringVar()
        self.command_entry_y = tki.Entry(self.current_frame, width=9, textvariable=self.command_strigvar_y)
        self.command_entry_y.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.command_entry_y.delete(0, tki.END)
        self.command_entry_y.insert(0, "0.0")

        self.frame_column += 1

        self.command_strigvar_z = tki.StringVar()
        self.command_entry_z = tki.Entry(self.current_frame, width=9, textvariable=self.command_strigvar_z)
        self.command_entry_z.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.command_entry_z.delete(0, tki.END)
        self.command_entry_z.insert(0, "1.0")

        self.frame_column += 1

        self.command_strigvar_yaw = tki.StringVar()
        self.command_entry_yaw = tki.Entry(self.current_frame, width=9, textvariable=self.command_strigvar_yaw)
        self.command_entry_yaw.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.command_entry_yaw.delete(0, tki.END)
        self.command_entry_yaw.insert(0, "0.0")

        self.frame_column += 1

        self.frame_row += 1
        self.frame_column = 0

        self.btn_takeoff = tki.Button(self.current_frame, text="Takeoff!", command=self.takeoff, bg='yellow')
        self.btn_takeoff.grid(row=self.frame_row, column=0)#, padx=3, pady=3)

        self.btn_publish_command = tki.Button(self.current_frame, text="Publish Command!", command=self.publish_command)
        self.btn_publish_command.grid(row=self.frame_row, column=1)#, padx=3, pady=3)

        self.btn_land = tki.Button(self.current_frame, text="Land!", command=self.land_callback, fg='green', bg='white')
        self.btn_land.grid(row=self.frame_row, column=2)#, padx=3, pady=3)

        # self.btn_reset_map = tki.Button(self.current_frame, text="Reset Map!", command=self.reset_map_callback)
        # self.btn_reset_map.grid(row=self.frame_row, column=3)#, padx=3, pady=3)

        self.frame_row += 1

        # self.btn_scan_room_right = tki.Button(self.current_frame, text="Scan Room Right!", command=self.scan_room_right_callback)
        # self.btn_scan_room_right.grid(row=self.frame_row, column=0)#, padx=3, pady=3)

        # self.btn_scan_room_left = tki.Button(self.current_frame, text="Scan Room Left!", command=self.scan_room_left_callback)
        # self.btn_scan_room_left.grid(row=self.frame_row, column=1)#, padx=3, pady=3)

        self.btn_stay_in_place = tki.Button(self.current_frame, text="Stay In Place!", command=self.stay_in_place)
        self.btn_stay_in_place.grid(row=self.frame_row, column=0)#, padx=3, pady=3)

        self.btn_calibrate_z = tki.Button(self.current_frame, text="Calibrate Z!", command=self.calibrate_z_callback, bg='yellow')
        self.btn_calibrate_z.grid(row=self.frame_row, column=1)#, padx=3, pady=3)

        self.row += 1
        self.column = 0

    def init_slam_pose_frame(self, root_frame):
        self.init_slam_pose_frame_flag = True
        self.frame_column = 0
        self.frame_row = 0

        self.frame_pose = tki.Frame(root_frame, relief=tki.SUNKEN, borderwidth = 1)
        rospy.loginfo("slam_pose_frame in row={} column={}".format(self.row, self.column))
        self.frame_pose.grid(row=self.row, column=self.column)

        self.current_frame = self.frame_pose

        self.title_label_pose = tki.Label(self.current_frame, text="Slam Pose", font=("Helvetica", 13))
        self.title_label_pose.grid(row=self.frame_row, column=1)#, padx=3, pady=3)

        self.frame_row += 1

        self.slam_pose_label_x = tki.Label(self.current_frame, text="X")
        self.slam_pose_label_x.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.slam_pose_label_y = tki.Label(self.current_frame, text="Y")
        self.slam_pose_label_y.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.slam_pose_label_z = tki.Label(self.current_frame, text="Z")
        self.slam_pose_label_z.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.frame_row += 1
        self.frame_column = 0

        self.slam_pose_strigvar_x = tki.StringVar()
        self.slam_pose_entry_x = tki.Entry(self.current_frame, width=9, textvariable=self.slam_pose_strigvar_x)
        self.slam_pose_entry_x.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.slam_pose_entry_x.delete(0, tki.END)
        self.slam_pose_entry_x.insert(0, "0.0")

        self.frame_column += 1

        self.slam_pose_strigvar_y = tki.StringVar()
        self.slam_pose_entry_y = tki.Entry(self.current_frame, width=9, textvariable=self.slam_pose_strigvar_y)
        self.slam_pose_entry_y.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.slam_pose_entry_y.delete(0, tki.END)
        self.slam_pose_entry_y.insert(0, "0.0")

        self.frame_column += 1

        self.slam_pose_strigvar_z = tki.StringVar()
        self.slam_pose_entry_z = tki.Entry(self.current_frame, width=9, textvariable=self.slam_pose_strigvar_z)
        self.slam_pose_entry_z.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.slam_pose_entry_z.delete(0, tki.END)
        self.slam_pose_entry_z.insert(0, "0.0")


        self.frame_column += 1
        self.row += 1
        self.column = 0

    def init_delta_frame(self, root_frame):
        self.init_delta_frame_flag = True
        self.frame_column = 0
        self.frame_row = 0

        self.frame_delta = tki.Frame(root_frame, relief=tki.SUNKEN, borderwidth = 1)
        rospy.loginfo("delta_frame in row={} column={}".format(self.row, self.column))


        self.frame_delta.grid(row=self.row, column=self.column)

        self.current_frame = self.frame_delta

        self.frame_delta_header = tki.Frame(self.current_frame, relief=tki.SUNKEN)
        self.frame_delta_header.grid(row=0, column=0)

        self.title_label_delta = tki.Label(self.frame_delta_header, text="Delta Between Command and Real World [Meters]", font=("Helvetica", 13))
        self.title_label_delta.grid(row=0, column=0)#, padx=3, pady=3)

        self.frame_row += 1

        self.frame_delta_2 = tki.Frame(self.current_frame, relief=tki.SUNKEN)
        self.frame_delta_2.grid(row=1, column=0)

        self.current_frame = self.frame_delta_2

        self.delta_label_x = tki.Label(self.current_frame, text="X")
        self.delta_label_x.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.delta_label_y = tki.Label(self.current_frame, text="Y")
        self.delta_label_y.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.delta_label_z = tki.Label(self.current_frame, text="Z")
        self.delta_label_z.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.frame_row += 1
        self.frame_column = 0

        self.delta_strigvar_x = tki.StringVar()
        self.delta_entry_x = tki.Entry(self.current_frame, width=9, textvariable=self.delta_strigvar_x)
        self.delta_entry_x.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.delta_entry_x.delete(0, tki.END)
        self.delta_entry_x.insert(0, "0.0")

        self.frame_column += 1

        self.delta_strigvar_y = tki.StringVar()
        self.delta_entry_y = tki.Entry(self.current_frame, width=9, textvariable=self.delta_strigvar_y)
        self.delta_entry_y.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.delta_entry_y.delete(0, tki.END)
        self.delta_entry_y.insert(0, "0.0")

        self.frame_column += 1

        self.delta_strigvar_z = tki.StringVar()
        self.delta_entry_z = tki.Entry(self.current_frame, width=9, textvariable=self.delta_strigvar_z)
        self.delta_entry_z.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.delta_entry_z.delete(0, tki.END)
        self.delta_entry_z.insert(0, "0.0")

        self.frame_column += 1
        self.row += 1
        self.column = 0
    
    def init_speed_frame(self, root_frame):
        self.init_speed_frame_flag = True
        self.frame_column = 0
        self.frame_row = 0

        self.frame_speed = tki.Frame(root_frame, relief=tki.SUNKEN, borderwidth = 1)
        rospy.loginfo("speed_frame in row={} column={}".format(self.row, self.column))
        self.frame_speed.grid(row=self.row, column=self.column)

        self.current_frame = self.frame_speed

        self.title_label_speed = tki.Label(self.current_frame, text="Speed", font=("Helvetica", 13))
        self.title_label_speed.grid(row=self.frame_row, column=1)#, padx=3, pady=3)

        self.frame_row += 1

        self.speed_label_pitch = tki.Label(self.current_frame, text="Pitch")
        self.speed_label_pitch.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.speed_label_roll = tki.Label(self.current_frame, text="Roll")
        self.speed_label_roll.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.speed_label_throttle = tki.Label(self.current_frame, text="Throttle")
        self.speed_label_throttle.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.speed_label_yaw = tki.Label(self.current_frame, text="Yaw")
        self.speed_label_yaw.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1


        self.frame_row += 1
        self.frame_column = 0

        self.speed_strigvar_pitch = tki.StringVar()
        self.speed_entry_pitch = tki.Entry(self.current_frame, width=9, textvariable=self.speed_strigvar_pitch)
        self.speed_entry_pitch.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.speed_entry_pitch.delete(0, tki.END)
        self.speed_entry_pitch.insert(0, "0.0")

        self.frame_column += 1

        self.speed_strigvar_roll = tki.StringVar()
        self.speed_entry_roll = tki.Entry(self.current_frame, width=9, textvariable=self.speed_strigvar_roll)
        self.speed_entry_roll.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.speed_entry_roll.delete(0, tki.END)
        self.speed_entry_roll.insert(0, "0.0")

        self.frame_column += 1

        self.speed_strigvar_throttle = tki.StringVar()
        self.speed_entry_throttle = tki.Entry(self.current_frame, width=9, textvariable=self.speed_strigvar_throttle)
        self.speed_entry_throttle.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.speed_entry_throttle.delete(0, tki.END)
        self.speed_entry_throttle.insert(0, "0.0")

        self.frame_column += 1

        self.speed_strigvar_yaw = tki.StringVar()
        self.speed_entry_yaw = tki.Entry(self.current_frame, width=9, textvariable=self.speed_strigvar_yaw)
        self.speed_entry_yaw.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.speed_entry_yaw.delete(0, tki.END)
        self.speed_entry_yaw.insert(0, "0.0")

        self.row += 1
        self.column = 0

    def init_info_frame(self, root_frame):
        self.init_info_frame_flag = True
        self.frame_column = 0
        self.frame_row = 0

        self.frame_etc = tki.Frame(root_frame, relief=tki.SUNKEN, borderwidth = 1)
        rospy.loginfo("info_frame in row={} column={}".format(self.row, self.column))
        self.frame_etc.grid(row=self.row, column=self.column)

        self.current_frame = self.frame_etc


        self.altitude_label = tki.Label(self.current_frame, text="Altitude")
        self.altitude_label.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)

        self.frame_column += 1

        self.battery_label = tki.Label(self.current_frame, text="Battery %")
        self.battery_label.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)

        self.frame_column += 1

        # self.flight_time_remaining_label = tki.Label(self.current_frame, text="flight time remaining")
        # self.flight_time_remaining_label.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)

        self.frame_column += 1

        self.allow_slam_control_btn = tki.Button(self.current_frame, text="Toggle Slam Control!",  command=self.allow_slam_control_btn_callback, bg='yellow')
        self.allow_slam_control_btn.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)


        self.frame_row += 1
        self.frame_column = 0

        self.altitude_strigvar = tki.StringVar()
        self.altitude_entry = tki.Entry(self.current_frame, width=9, textvariable=self.altitude_strigvar)
        self.altitude_entry.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.altitude_entry.delete(0, tki.END)
        self.altitude_entry.insert(0, "0.0")

        self.frame_column += 1

        self.battery_strigvar = tki.StringVar()
        self.battery_entry = tki.Entry(self.current_frame, width=9, textvariable=self.battery_strigvar)
        self.battery_entry.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.battery_entry.delete(0, tki.END)
        self.battery_entry.insert(0, "0.0")

        self.frame_column += 1

        # self.flight_time_remaining_strigvar = tki.StringVar()
        # self.flight_time_remaining_entry = tki.Entry(self.current_frame, width=9, textvariable=self.flight_time_remaining_strigvar)
        # self.flight_time_remaining_entry.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        # self.flight_time_remaining_entry.delete(0, tki.END)
        # self.flight_time_remaining_entry.insert(0, "0.0")

        self.frame_column += 1

        self.allow_slam_control_strigvar = tki.StringVar()
        self.allow_slam_control_entry = tki.Entry(self.current_frame, width=9, textvariable=self.allow_slam_control_strigvar)
        self.allow_slam_control_entry.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.allow_slam_control_entry.delete(0, tki.END)
        self.allow_slam_control_entry.insert(0, "{}".format(self.allow_slam_control))

        self.row += 1
        self.column = 0

    def init_manual_control_frame(self, root_frame):
        self.init_manual_control_frame_flag = True
        self.frame_column = 0
        self.frame_row = 0

        self.frame_manual_control = tki.Frame(root_frame, relief=tki.SUNKEN, borderwidth = 1)
        rospy.loginfo("manual_control_frame in row={} column={}".format(self.row, self.column))
        self.frame_manual_control.grid(row=self.row, column=self.column)

        self.current_frame = self.frame_manual_control

        self.title_label_manual_control = tki.Label(self.current_frame, text="Manual Control", font=("Helvetica", 13))
        self.title_label_manual_control.grid(row=self.frame_row, column=1)#, padx=3, pady=3)

        self.manual_control_set_btn = tki.Button(self.current_frame, text="Manual Control Set!",  command=self.manual_control_set_callback, bg='yellow')
        self.manual_control_set_btn.grid(row=self.frame_row, column=2)#, padx=3, pady=3)

        self.manual_control_clear_btn = tki.Button(self.current_frame, text="Manual Control Clear!",  command=self.manual_control_clear_callback)
        self.manual_control_clear_btn.grid(row=self.frame_row, column=3)#, padx=3, pady=3)

        self.frame_row += 1

        self.manual_control_label_pitch = tki.Label(self.current_frame, text="Pitch")
        self.manual_control_label_pitch.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.manual_control_label_roll = tki.Label(self.current_frame, text="Roll")
        self.manual_control_label_roll.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.manual_control_label_throttle = tki.Label(self.current_frame, text="Throttle")
        self.manual_control_label_throttle.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.manual_control_label_yaw = tki.Label(self.current_frame, text="Yaw")
        self.manual_control_label_yaw.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.frame_row += 1
        self.frame_column = 0

        self.manual_control_strigvar_pitch = tki.StringVar()
        self.manual_control_entry_pitch = tki.Entry(self.current_frame, width=9, textvariable=self.manual_control_strigvar_pitch)
        self.manual_control_entry_pitch.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.manual_control_entry_pitch.delete(0, tki.END)
        self.manual_control_entry_pitch.insert(0, "0.0")

        self.frame_column += 1

        self.manual_control_strigvar_roll = tki.StringVar()
        self.manual_control_entry_roll = tki.Entry(self.current_frame, width=9, textvariable=self.manual_control_strigvar_roll)
        self.manual_control_entry_roll.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.manual_control_entry_roll.delete(0, tki.END)
        self.manual_control_entry_roll.insert(0, "0.0")

        self.frame_column += 1

        self.manual_control_strigvar_throttle = tki.StringVar()
        self.manual_control_entry_throttle = tki.Entry(self.current_frame, width=9, textvariable=self.manual_control_strigvar_throttle)
        self.manual_control_entry_throttle.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.manual_control_entry_throttle.delete(0, tki.END)
        self.manual_control_entry_throttle.insert(0, "0.0")

        self.frame_column += 1

        self.manual_control_strigvar_yaw = tki.StringVar()
        self.manual_control_entry_yaw = tki.Entry(self.current_frame, width=9, textvariable=self.manual_control_strigvar_yaw)
        self.manual_control_entry_yaw.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.manual_control_entry_yaw.delete(0, tki.END)
        self.manual_control_entry_yaw.insert(0, "0.0")

        self.frame_column += 1

        self.row += 1
        self.column = 0

    def init_angle_calc_frame(self, root_frame):
        self.init_angle_calc_frame_flag = True
        self.frame_column = 0
        self.frame_row = 0

        self.frame_angle_calc = tki.Frame(root_frame, relief=tki.SUNKEN, borderwidth = 1)
        rospy.loginfo("angle_calc_frame in row={} column={}".format(self.row, self.column))
        self.frame_angle_calc.grid(row=self.row, column=self.column)

        self.current_frame = self.frame_angle_calc

        self.title_label_angle_calc = tki.Label(self.current_frame, text="Angel Calc", font=("Helvetica", 13))
        self.title_label_angle_calc.grid(row=self.frame_row, column=2)#, padx=3, pady=3)

        self.frame_row += 1

        self.angle_calc_label_x_moved = tki.Label(self.current_frame, text="X Moved")
        self.angle_calc_label_x_moved.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.angle_calc_label_y_moved = tki.Label(self.current_frame, text="Y Moved")
        self.angle_calc_label_y_moved.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.angle_calc_label_z_moved = tki.Label(self.current_frame, text="Z Moved")
        self.angle_calc_label_z_moved.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.angle_calc_label_angle = tki.Label(self.current_frame, text="Angle")
        self.angle_calc_label_angle.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.angle_calc_set_btn = tki.Button(self.current_frame, text="Calculate Angle!",  command=self.angle_calc_set_callback)
        self.angle_calc_set_btn.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)

        self.frame_column += 1


        self.frame_row += 1
        self.frame_column = 0

        self.angle_calc_strigvar_x_moved = tki.StringVar()
        self.angle_calc_entry_x_moved = tki.Entry(self.current_frame, width=9, textvariable=self.angle_calc_strigvar_x_moved)
        self.angle_calc_entry_x_moved.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.angle_calc_entry_x_moved.delete(0, tki.END)
        self.angle_calc_entry_x_moved.insert(0, "0.0")

        self.frame_column += 1

        self.angle_calc_strigvar_y_moved = tki.StringVar()
        self.angle_calc_entry_y_moved = tki.Entry(self.current_frame, width=9, textvariable=self.angle_calc_strigvar_y_moved)
        self.angle_calc_entry_y_moved.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.angle_calc_entry_y_moved.delete(0, tki.END)
        self.angle_calc_entry_y_moved.insert(0, "0.0")

        self.frame_column += 1

        self.angle_calc_strigvar_z_moved = tki.StringVar()
        self.angle_calc_entry_z_moved = tki.Entry(self.current_frame, width=9, textvariable=self.angle_calc_strigvar_z_moved)
        self.angle_calc_entry_z_moved.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.angle_calc_entry_z_moved.delete(0, tki.END)
        self.angle_calc_entry_z_moved.insert(0, "0.0")

        self.frame_column += 1

        self.angle_calc_strigvar_angle = tki.StringVar()
        self.angle_calc_entry_angle = tki.Entry(self.current_frame, width=9, textvariable=self.angle_calc_strigvar_angle)
        self.angle_calc_entry_angle.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.angle_calc_entry_angle.delete(0, tki.END)
        self.angle_calc_entry_angle.insert(0, "0.0")
        self.angle_calc_strigvar_angle.set('%.4f'%(self.angle))  

        self.frame_column += 1

        self.angle_calc_clear_btn = tki.Button(self.current_frame, text="Clear Angle!",  command=self.angle_calc_clear_callback)
        self.angle_calc_clear_btn.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)

        self.frame_column += 1

        self.row += 1
        self.column = 0
    
    def init_rotated_frame(self, root_frame):
        self.init_rotated_frame_flag = True
        self.frame_column = 0
        self.frame_row = 0

        self.frame_rotated = tki.Frame(root_frame, relief=tki.SUNKEN, borderwidth = 1)
        rospy.loginfo("rotated_frame in row={} column={}".format(self.row, self.column))
        self.frame_rotated.grid(row=self.row, column=self.column)

        self.title_label_rotated = tki.Label(self.frame_rotated, text="Rotated SLAM Coordinates", font=("Helvetica", 13))
        self.title_label_rotated.grid(row=self.frame_row, column=1)#, padx=3, pady=3)

        self.frame_row += 1

        self.rotated_label_x = tki.Label(self.frame_rotated, text="X")
        self.rotated_label_x.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.rotated_label_y = tki.Label(self.frame_rotated, text="Y")
        self.rotated_label_y.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.rotated_label_z = tki.Label(self.frame_rotated, text="Z")
        self.rotated_label_z.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.rotated_label_orientation = tki.Label(self.frame_rotated, text="Orientation")
        self.rotated_label_orientation.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.frame_row += 1
        self.frame_column = 0

        self.rotated_strigvar_x = tki.StringVar()
        self.rotated_entry_x = tki.Entry(self.frame_rotated, width=9, textvariable=self.rotated_strigvar_x)
        self.rotated_entry_x.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.rotated_entry_x.delete(0, tki.END)
        self.rotated_entry_x.insert(0, "0.0")

        self.frame_column += 1

        self.rotated_strigvar_y = tki.StringVar()
        self.rotated_entry_y = tki.Entry(self.frame_rotated, width=9, textvariable=self.rotated_strigvar_y)
        self.rotated_entry_y.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.rotated_entry_y.delete(0, tki.END)
        self.rotated_entry_y.insert(0, "0.0")

        self.frame_column += 1

        self.rotated_strigvar_z = tki.StringVar()
        self.rotated_entry_z = tki.Entry(self.frame_rotated, width=9, textvariable=self.rotated_strigvar_z)
        self.rotated_entry_z.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.rotated_entry_z.delete(0, tki.END)
        self.rotated_entry_z.insert(0, "0.0")

        self.frame_column += 1

        self.rotated_strigvar_orientation = tki.StringVar()
        self.rotated_entry_orientation = tki.Entry(self.frame_rotated, width=9, textvariable=self.rotated_strigvar_orientation)
        self.rotated_entry_orientation.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.rotated_entry_orientation.delete(0, tki.END)
        self.rotated_entry_orientation.insert(0, "0.0")

        self.frame_column += 1

        self.row += 1
        self.column = 0

    def init_real_world_frame(self, root_frame):
        self.init_real_world_frame_flag = True
        self.frame_column = 0
        self.frame_row = 0

        self.frame_real_world = tki.Frame(root_frame, relief=tki.SUNKEN, borderwidth = 1)
        rospy.loginfo("real_world_frame in row={} column={}".format(self.row, self.column))
        self.frame_real_world.grid(row=self.row, column=self.column)

        self.title_label_real_world = tki.Label(self.frame_real_world, text="Real World Position[Meters]", font=("Helvetica", 13))
        self.title_label_real_world.grid(row=self.frame_row, column=1)#)#, padx=3, pady=3)

        # self.btn_change_mux_toggle = tki.Button(self.frame_real_world, text="Change Mux!", command=self.change_mux)
        # self.btn_change_mux_toggle.grid(row=self.frame_row, column=3)#)#, padx=3, pady=3)

        self.frame_row += 1

        self.real_world_label_x = tki.Label(self.frame_real_world, text="X")
        self.real_world_label_x.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.real_world_label_y = tki.Label(self.frame_real_world, text="Y")
        self.real_world_label_y.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.real_world_label_z = tki.Label(self.frame_real_world, text="Z")
        self.real_world_label_z.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.real_world_label_scale = tki.Label(self.frame_real_world, text="Altitude Scale")
        self.real_world_label_scale.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.frame_row += 1
        self.frame_column = 0

        self.real_world_strigvar_x = tki.StringVar()
        self.real_world_entry_x = tki.Entry(self.frame_real_world, width=9, textvariable=self.real_world_strigvar_x)
        self.real_world_entry_x.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.real_world_entry_x.delete(0, tki.END)
        self.real_world_entry_x.insert(0, "0.0")

        self.frame_column += 1

        self.real_world_strigvar_y = tki.StringVar()
        self.real_world_entry_y = tki.Entry(self.frame_real_world, width=9, textvariable=self.real_world_strigvar_y)
        self.real_world_entry_y.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.real_world_entry_y.delete(0, tki.END)
        self.real_world_entry_y.insert(0, "0.0")

        self.frame_column += 1

        self.real_world_strigvar_z = tki.StringVar()
        self.real_world_entry_z = tki.Entry(self.frame_real_world, width=9, textvariable=self.real_world_strigvar_z)
        self.real_world_entry_z.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.real_world_entry_z.delete(0, tki.END)
        self.real_world_entry_z.insert(0, "0.0")

        self.frame_column += 1

        self.real_world_strigvar_scale = tki.StringVar()
        self.real_world_entry_scale = tki.Entry(self.frame_real_world, width=9, textvariable=self.real_world_strigvar_scale)
        self.real_world_entry_scale.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.real_world_entry_scale.delete(0, tki.END)
        self.real_world_entry_scale.insert(0, "0.0")

        self.frame_column += 1

        self.row += 1
        self.column = 0
    
    def init_kd_kp_frame(self, root_frame):
        self.init_kd_kp_frame_flag = True
        self.frame_column = 0
        self.frame_row = 0

        self.kd_kp_frame = tki.Frame(root_frame, relief=tki.SUNKEN, borderwidth = 1)
        rospy.loginfo("kd_kp_frame in row={} column={}".format(self.row, self.column))
        self.frame_kd_kp.grid(row=self.row, column=self.column)

        self.current_frame = self.frame_kd_kp


        self.title_label_kd = tki.Label(self.current_frame, text="Kd", font=("Helvetica", 13))
        self.title_label_kd.grid(row=self.frame_row, column=1)#, padx=3, pady=3)

        self.kp_kd_btn = tki.Button(self.current_frame, text="Publish Kd/Kp!",  command=self.kd_kp_callback)
        self.kp_kd_btn.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)


        self.frame_row += 1

        self.kd_label_x = tki.Label(self.current_frame, text="X")
        self.kd_label_x.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.kd_label_y = tki.Label(self.current_frame, text="Y")
        self.kd_label_y.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.kd_label_z = tki.Label(self.current_frame, text="Z")
        self.kd_label_z.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.kd_label_yaw = tki.Label(self.current_frame, text="Yaw")
        self.kd_label_yaw.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1


        self.frame_row += 1
        self.frame_column = 0

        self.kd_strigvar_x = tki.StringVar()
        self.kd_entry_x = tki.Entry(self.current_frame, width=9, textvariable=self.kd_strigvar_x)
        self.kd_entry_x.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.kd_entry_x.delete(0, tki.END)
        self.kd_entry_x.insert(0, "1.0")

        self.frame_column += 1

        self.kd_strigvar_y = tki.StringVar()
        self.kd_entry_y = tki.Entry(self.current_frame, width=9, textvariable=self.kd_strigvar_y)
        self.kd_entry_y.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.kd_entry_y.delete(0, tki.END)
        self.kd_entry_y.insert(0, "1.0")

        self.frame_column += 1

        self.kd_strigvar_z = tki.StringVar()
        self.kd_entry_z = tki.Entry(self.current_frame, width=9, textvariable=self.kd_strigvar_z)
        self.kd_entry_z.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.kd_entry_z.delete(0, tki.END)
        self.kd_entry_z.insert(0, "1.5")

        self.frame_column += 1

        self.kd_strigvar_yaw = tki.StringVar()
        self.kd_entry_yaw = tki.Entry(self.current_frame, width=9, textvariable=self.kd_strigvar_yaw)
        self.kd_entry_yaw.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.kd_entry_yaw.delete(0, tki.END)
        self.kd_entry_yaw.insert(0, "0.01")

        self.frame_column = 0
        self.frame_row += 1

        self.title_label_kp = tki.Label(self.current_frame, text="Kp", font=("Helvetica", 13))
        self.title_label_kp.grid(row=self.frame_row, column=1)#, padx=3, pady=3)

        self.frame_row += 1

        self.kp_label_x = tki.Label(self.current_frame, text="X")
        self.kp_label_x.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.kp_label_y = tki.Label(self.current_frame, text="Y")
        self.kp_label_y.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.kp_label_z = tki.Label(self.current_frame, text="Z")
        self.kp_label_z.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1

        self.kp_label_yaw = tki.Label(self.current_frame, text="Yaw")
        self.kp_label_yaw.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        self.frame_column += 1


        self.frame_row += 1
        self.frame_column = 0

        self.kp_strigvar_x = tki.StringVar()
        self.kp_entry_x = tki.Entry(self.current_frame, width=9, textvariable=self.kp_strigvar_x)
        self.kp_entry_x.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.kp_entry_x.delete(0, tki.END)
        self.kp_entry_x.insert(0, "0.6")

        self.frame_column += 1

        self.kp_strigvar_y = tki.StringVar()
        self.kp_entry_y = tki.Entry(self.current_frame, width=9, textvariable=self.kp_strigvar_y)
        self.kp_entry_y.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.kp_entry_y.delete(0, tki.END)
        self.kp_entry_y.insert(0, "0.6")

        self.frame_column += 1

        self.kp_strigvar_z = tki.StringVar()
        self.kp_entry_z = tki.Entry(self.current_frame, width=9, textvariable=self.kp_strigvar_z)
        self.kp_entry_z.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.kp_entry_z.delete(0, tki.END)
        self.kp_entry_z.insert(0, "1.5")

        self.frame_column += 1

        self.kp_strigvar_yaw = tki.StringVar()
        self.kp_entry_yaw = tki.Entry(self.current_frame, width=9, textvariable=self.kp_strigvar_yaw)
        self.kp_entry_yaw.grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
        self.kp_entry_yaw.delete(0, tki.END)
        self.kp_entry_yaw.insert(0, "0.015")

        self.row += 1
        self.column = 0

    def init_merge_map_frame(self, root_frame):
        

        self.init_merge_map_frame_flag = True
        self.frame_column = 0
        self.frame_row = 0

        self.frame_merge_map = tki.Frame(root_frame, relief=tki.SUNKEN, borderwidth = 1)
        rospy.loginfo("merge_map_frame in row={} column={}".format(self.row, self.column))
        self.frame_merge_map.grid(row=self.row, column=self.column)
        # self.frame_merge_map = self.frame_trajectory_main

        self.current_frame = self.frame_merge_map

        self.toogle_merge_coordinates_btn = tki.Button(self.current_frame, text="Toggle Use Merged Coordinates",  command=self.toggle_merge_coordinates_publish, bg='yellow')
        self.toogle_merge_coordinates_btn.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)

        self.frame_row += 1

        self.row += 1
        self.column = 0



    
    def init_trajectory_frame(self, root_frame):
        
        self.init_trajectory_frame_flag = True
        

        self.frame_trajectory_main = tki.Frame(root_frame, relief=tki.SUNKEN, borderwidth = 1)
        rospy.loginfo("trajectory_frame in row={} column={}".format(self.row, self.column))
        self.frame_trajectory_main.grid(row=self.row, column=self.column)

        self.frame_trajectory_1 = tki.Frame(self.frame_trajectory_main, relief=tki.SUNKEN, borderwidth = 1)
        self.frame_trajectory_1.grid(row=0, column=0)

        self.current_frame = self.frame_trajectory_1

        self.frame_column = 0
        self.frame_row = 0

        self.title_label_trajectory = tki.Label(self.current_frame, text="Trajectory Control", font=("Helvetica", 13))
        self.title_label_trajectory.grid(row=self.frame_row, column=0)#, padx=3, pady=3)

        self.frame_row += 1

        self.trajectory_set_btn = tki.Button(self.current_frame, text="Publish Trajectory!",  command=self.trajectory_publish_callback, bg='yellow')
        self.trajectory_set_btn.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)

        self.frame_row += 1

        self.trajectory_load_btn = tki.Button(self.current_frame, text="Load File",  command=self.trajectory_load_callback)
        self.trajectory_load_btn.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)

        self.frame_row += 1

        self.trajectory_reload_last_btn = tki.Button(self.current_frame, text="Reload Last",  command=self.load_last_trajectory_callback)
        self.trajectory_reload_last_btn.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
        
        self.frame_row += 1

        self.trajectory_pushup_btn = tki.Button(self.current_frame, text="Push Up!",  command=self.trajectory_pushup_callback)
        self.trajectory_pushup_btn.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)


        self.frame_row += 1

        self.trajectory_kill_btn = tki.Button(self.current_frame, text="Kill Trajectory!",  command=self.trajectory_kill_callback)
        self.trajectory_kill_btn.grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)


        self.frame_trajectory_2 = tki.Frame(self.frame_trajectory_main, relief=tki.SUNKEN, borderwidth = 1)
        self.frame_trajectory_2.grid(row=1, column=0)        

        self.current_frame = self.frame_trajectory_2

        self.row += 1
        self.column = 1


        self.frame_trajectory_points = tki.Frame(self.current_frame, relief=tki.SUNKEN, borderwidth = 1)
        self.frame_trajectory_points.grid(row=self.row, column=self.column)

        self.current_frame = self.frame_trajectory_points
        self.frame_row = 0

        self.frame_column += 1

        self.trajectory_x_label = tki.Label(self.current_frame, text="X")
        self.trajectory_x_label.grid(row=self.frame_row, column=self.frame_column, padx=2, pady=5)
        self.frame_column += 1

        self.trajectory_y_label = tki.Label(self.current_frame, text="Y")
        self.trajectory_y_label.grid(row=self.frame_row, column=self.frame_column, padx=2, pady=2)
        self.frame_column += 1

        self.trajectory_z_label = tki.Label(self.current_frame, text="Z")
        self.trajectory_z_label.grid(row=self.frame_row, column=self.frame_column, padx=2, pady=2)
        self.frame_column += 1

        self.trajectory_yaw_label = tki.Label(self.current_frame, text="YAW")
        self.trajectory_yaw_label.grid(row=self.frame_row, column=self.frame_column, padx=2, pady=2)
        self.frame_column = 0
        self.frame_row += 1

        self.trajectory_idx_label = []
        self.trajectory_strigvar_x = []
        self.trajectory_entry_x = []
        self.trajectory_strigvar_y = []
        self.trajectory_entry_y = []
        self.trajectory_strigvar_z = []
        self.trajectory_entry_z = []
        self.trajectory_strigvar_yaw = []
        self.trajectory_entry_yaw = []

        for trajectory_idx in range(self.number_of_trajectory_points_ui):

            self.trajectory_idx_label.append(tki.Label(self.current_frame, text="{}".format(trajectory_idx)))
            self.trajectory_idx_label[trajectory_idx].grid(row=self.frame_row, column=self.frame_column)#, padx=3, pady=3)
            self.frame_column += 1

            self.trajectory_strigvar_x.append(tki.StringVar())
            self.trajectory_entry_x.append(tki.Entry(self.current_frame, width=5, textvariable=self.trajectory_strigvar_x[trajectory_idx]))
            self.trajectory_entry_x[trajectory_idx].grid(row=self.frame_row, column=self.frame_column)#, padx=2, pady=2)
            self.trajectory_entry_x[trajectory_idx].delete(0, tki.END)
            self.trajectory_entry_x[trajectory_idx].insert(0, "0.0")
            self.frame_column += 1

            self.trajectory_strigvar_y.append(tki.StringVar())
            self.trajectory_entry_y.append(tki.Entry(self.current_frame, width=5, textvariable=self.trajectory_strigvar_y[trajectory_idx]))
            self.trajectory_entry_y[trajectory_idx].grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
            self.trajectory_entry_y[trajectory_idx].delete(0, tki.END)
            self.trajectory_entry_y[trajectory_idx].insert(0, "0.0")
            self.frame_column += 1

            self.trajectory_strigvar_z.append(tki.StringVar())
            self.trajectory_entry_z.append(tki.Entry(self.current_frame, width=5, textvariable=self.trajectory_strigvar_z[trajectory_idx]))
            self.trajectory_entry_z[trajectory_idx].grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
            self.trajectory_entry_z[trajectory_idx].delete(0, tki.END)
            self.trajectory_entry_z[trajectory_idx].insert(0, "1.0")
            self.frame_column += 1

            self.trajectory_strigvar_yaw.append(tki.StringVar())
            self.trajectory_entry_yaw.append(tki.Entry(self.current_frame, width=5, textvariable=self.trajectory_strigvar_yaw[trajectory_idx]))
            self.trajectory_entry_yaw[trajectory_idx].grid(row=self.frame_row, column=self.frame_column)#, padx=5, pady=5)
            self.trajectory_entry_yaw[trajectory_idx].delete(0, tki.END)
            self.trajectory_entry_yaw[trajectory_idx].insert(0, "0")
            self.frame_column += 1

            self.frame_row += 1
            self.frame_column = 0

        self.row += 1
        self.column = 0

    

    def calibrate_z_callback(self):
        self.calibrate_real_world_scale_publisher.publish()
        self.btn_calibrate_z.configure(fg = 'red', bg = 'black')

    def scan_room_left_callback(self):
        rospy.loginfo('pressed Scan Room Left!')
        self.scan_room_publisher.publish(True)

    def scan_room_right_callback(self):
        rospy.loginfo('pressed Scan Room Right!')
        self.scan_room_publisher.publish(False)

    def allow_slam_control_btn_callback(self):
        if self.allow_slam_control:
            self.allow_slam_control_btn.configure(bg='yellow', fg='black')
            self.btn_calibrate_z.configure(bg='yellow')
        else:
            self.allow_slam_control_btn.configure(bg='black', fg='yellow')
            self.btn_calibrate_z.configure(bg=self.default_bg)
        self.pub_allow_slam_control.publish(not self.allow_slam_control)

    def real_world_scale_callback(self, msg):
        self.real_world_scale = float(msg.data)
        if self.init_real_world_frame_flag:
            self.real_world_strigvar_scale.set('%.4f'%(self.real_world_scale)) 
        self.btn_calibrate_z.configure(fg = 'green', bg = 'yellow')


    def real_world_pos_callback(self, msg):
        self.real_world_pos = msg.pose.position
        if self.init_real_world_frame_flag:
            self.real_world_strigvar_x.set('%.4f'%(self.real_world_pos.x))
            self.real_world_strigvar_y.set('%.4f'%(self.real_world_pos.y))
            self.real_world_strigvar_z.set('%.4f'%(self.real_world_pos.z))

    def flightdata_callback(self, flight_data):
        self.altitude = flight_data.altitude
        if self.init_info_frame_flag:
            self.altitude_strigvar.set('%.4f'%(self.altitude))
            self.battery_strigvar.set('%.2f'%(flight_data.battery_percent))
            # self.flight_time_remaining_strigvar.set('%.2f'%(flight_data.estimated_flight_time_remaining))
        # try:
            # if self.altitude > 0.2:
                # self.real_world_scale = self.altitude / self.rotated_pos.z
        # except ZeroDivisionError:
            # self.real_world_scale = 1
        # self.real_world_strigvar_scale.set('%.4f'%(self.real_world_scale))  

    def allow_slam_control_callback(self, msg):
        self.allow_slam_control = (msg.data == 1)
        if self.init_speed_frame_flag:
            self.allow_slam_control_strigvar.set(self.allow_slam_control)

    def angle_calc_clear_callback(self):
        self.angle_delta_x = float(self.slam_pose_strigvar_x.get())
        self.angle_delta_y = float(self.slam_pose_strigvar_y.get())
        self.angle_delta_z = float(self.slam_pose_strigvar_z.get())


    def kd_kp_callback(self):
        self.kd.position.x = float(self.kd_strigvar_x.get())
        self.kd.position.y = float(self.kd_strigvar_y.get())
        self.kd.position.z = float(self.kd_strigvar_z.get())
        self.kd.orientation.z = float(self.kd_strigvar_yaw.get())

        self.kp.position.x = float(self.kp_strigvar_x.get())
        self.kp.position.y = float(self.kp_strigvar_y.get())
        self.kp.position.z = float(self.kp_strigvar_z.get())
        self.kp.orientation.z = float(self.kp_strigvar_yaw.get())

        self.kd_publisher.publish(self.kd)
        self.kp_publisher.publish(self.kp)

    def toggle_merge_coordinates_publish(self):
        self.use_merge_coordinates = not(self.use_merge_coordinates)
        self.merge_coordinates_pub.publish(Bool(self.use_merge_coordinates))
        if self.use_merge_coordinates:
            self.toogle_merge_coordinates_btn.configure(fg='red', bg='black')
        else:
            self.toogle_merge_coordinates_btn.configure(bg='yellow', fg='black')
        print("toggle_merge_coordinates_publish", self.use_merge_coordinates)

    def trajectory_publish_callback(self):
        self.update_trajectory_list_from_gui()
        self.last_trajectory = [element for element in self.trajectory_list]
        self.update_gui_trajectory()
        self.trajectory_kill = False
        trajectory_thread = threading.Thread(target=self.trajectory_thread, args=())
        trajectory_thread.start()
        self.trajectory_set_btn.configure(fg='green', bg=self.default_bg)

    def load_last_trajectory_callback(self):
        self.lock.acquire()
        self.trajectory_list = [element for element in self.last_trajectory]
        self.update_gui_trajectory()
        self.lock.release()

    def trajectory_pushup_callback(self):
        self.lock.acquire()
        self.trajectory_list.pop(0)
        self.update_gui_trajectory()
        self.lock.release()
        self.take_picure()

    def take_picure(self):
        self.take_picure_publisher.publish()

    def trajectory_load_callback(self):
        trajectory_path = tkFileDialog.askopenfilename(initialdir = "~/ROS/ccmslam_ws/src/flock/flock_driver/src/",title = "Select Trajectory file",filetypes = (("csv files","*.csv"),("all files","*.*")))    
        if trajectory_path == '':
            return
        self.trajectory_list = self.load_trajectory_from_csv(trajectory_path)
        print("trajectory_list={}".format(self.trajectory_list))
        self.update_gui_trajectory()

    def trajectory_kill_callback(self):
        self.trajectory_kill = True
        self.trajectory_set_btn.configure(fg='black', bg='yellow')

    def update_trajectory_list_from_gui(self):
        out = []
        try:
            for trajectory_idx in range(self.number_of_trajectory_points_ui):
                line_list = []
                line_list.append(float(self.trajectory_strigvar_x[trajectory_idx].get()))
                line_list.append(float(self.trajectory_strigvar_y[trajectory_idx].get()))
                line_list.append(float(self.trajectory_strigvar_z[trajectory_idx].get()))
                line_list.append(float(self.trajectory_strigvar_yaw[trajectory_idx].get()))


                out.append(line_list)
            for trajectory_idx in range(self.number_of_trajectory_points_ui, len(self.trajectory_list)):
                out.append(self.trajectory_list[trajectory_idx])
        except ValueError:   
            rospy.loginfo('Trajectory length is {} which is less than {}'.format(len(self.trajectory_list), self.number_of_trajectory_points_ui))
        self.trajectory_list = out

    def load_trajectory_from_csv(self, trajectory_path):
        print(trajectory_path)
        file = open(trajectory_path, 'r')
        out = []
        # data = file.readlines()

        for line in file:
            temp_lst = line.strip().replace('\n','').split(',')
            x_y_z = [float(element) for element in temp_lst]
            out.append(x_y_z)
        file.close()
        return out


    def update_gui_trajectory(self):
        for trajectory_idx in range(self.number_of_trajectory_points_ui):
            self.trajectory_entry_x[trajectory_idx].delete(0, tki.END)
            self.trajectory_entry_y[trajectory_idx].delete(0, tki.END)
            self.trajectory_entry_z[trajectory_idx].delete(0, tki.END)
            self.trajectory_entry_yaw[trajectory_idx].delete(0, tki.END)
            if trajectory_idx < len(self.trajectory_list):
                self.trajectory_entry_x[trajectory_idx].insert(0, "%.1f" % self.trajectory_list[trajectory_idx][0])
                self.trajectory_entry_y[trajectory_idx].insert(0, "%.1f" % self.trajectory_list[trajectory_idx][1])
                self.trajectory_entry_z[trajectory_idx].insert(0, "%.1f" % self.trajectory_list[trajectory_idx][2])
                self.trajectory_entry_yaw[trajectory_idx].insert(0, "%.1f" % self.trajectory_list[trajectory_idx][3])


    def trajectory_thread(self):

        degree_point = Point()

        rospy.loginfo("Started trajectory_thread with {}".format(self.trajectory_list))

        path_msg = Path()
        command_pos = Pose()

        self.lock.acquire()
        rospy.loginfo("Started trajectory_thread with {}".format(self.trajectory_list))



        for trajectory_idx, trajectory_line in enumerate(self.trajectory_list):
            pose_stamped = PoseStamped()
            pose_stamped.header.seq = trajectory_idx
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = 'world'

            pose_stamped.pose.position.x = trajectory_line[0]
            pose_stamped.pose.position.y = trajectory_line[1]
            pose_stamped.pose.position.z = trajectory_line[2]

            command_yaw = trajectory_line[3]
            degree_point.z = command_yaw
            command_pos.orientation = self.euler_point_deg_to_quatenrion(degree_point)

            pose_stamped.pose.orientation = command_pos.orientation
            path_msg.header = pose_stamped.header
            path_msg.poses.append(pose_stamped)
        self.lock.release()
        self.path_publisher.publish(path_msg)

        time.sleep(0.5)

        command_pos = self.received_command_pos
        command_yaw = self.point_command_pos_yaw

        while not rospy.is_shutdown():
            time.sleep(0.2)
            # if not (command_pos == self.received_command_pos and command_yaw == self.point_command_pos_yaw):
            #     rospy.loginfo("Trajectory have Lost control")
            #     return

            if self.quit_flag:
                rospy.loginfo("Trajectory Quit due to Quit GUI")
                self.trajectory_set_btn.configure(fg='black', bg='yellow')
                return


            if self.land:
                rospy.loginfo("Trajectory Quit due to Landing")
                self.trajectory_set_btn.configure(fg='black', bg='yellow')
                return

            if self.trajectory_kill:
                rospy.loginfo("Trajectory Quit due to Killing Command")
                self.trajectory_set_btn.configure(fg='black', bg='yellow')
                return

            if abs(command_pos.position.x - self.real_world_pos.x) < self.trajectory_threshold.x:
                if abs(command_pos.position.y - self.real_world_pos.y) < self.trajectory_threshold.y:
                    if abs(command_pos.position.z - self.real_world_pos.z) < self.trajectory_threshold.z:
                        if self.find_min_distance_in_orientation(command_yaw, self.orientation_degree.z) < self.trajectory_orientation_threshold.z:
                            self.trajectory_pushup_callback()


            if len(self.trajectory_list) > 0:
                command_pos.position.x = self.trajectory_list[0][0]
                command_pos.position.y = self.trajectory_list[0][1]
                command_pos.position.z = self.trajectory_list[0][2]
                command_yaw = self.trajectory_list[0][3]
                degree_point.z = command_yaw
                command_pos.orientation = self.euler_point_deg_to_quatenrion(degree_point)


                self.command_pos_publisher.publish(command_pos)

            else:
                rospy.loginfo("Trajectory Finished")
                self.trajectory_set_btn.configure(fg='black', bg='yellow')
                return

            time.sleep(1)
        return

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


    def angle_calc_set_callback(self):
        x = float(self.angle_calc_strigvar_x_moved.get())
        z = float(self.angle_calc_strigvar_z_moved.get())
        tan_angle = z/x
        self.angle_radian = math.atan(tan_angle)
        self.angle = self.angle_radian*180/math.pi
        self.angle_calc_strigvar_angle.set('%.4f'%(self.angle))  
        print("x={} z={} z/x={} angle={}".format(x, z, tan_angle, self.angle))

    def orientation_callback(self, orientation_point):
        self.orientation_degree = orientation_point
        if self.init_rotated_frame_flag:
            self.rotated_strigvar_orientation.set('%.4f'%(self.orientation_degree.z))

    def delta_pos_callback(self, delta_pos):
        self.delta_pos = delta_pos
        if self.init_delta_frame_flag:
            self.delta_strigvar_x.set('%.4f'%(self.delta_pos.x))
            self.delta_strigvar_y.set('%.4f'%(self.delta_pos.y))
            self.delta_strigvar_z.set('%.4f'%(self.delta_pos.z))

    def rotated_pos_callback(self, rotated_pos):
        self.rotated_pos = rotated_pos
        if self.init_rotated_frame_flag:
            self.rotated_strigvar_x.set('%.4f'%(self.rotated_pos.x))
            self.rotated_strigvar_y.set('%.4f'%(self.rotated_pos.y))
            self.rotated_strigvar_z.set('%.4f'%(self.rotated_pos.z))

    def point_copy(self, p):
        return Point(p.x, p.y, p.z)

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

    def command_pos_callback(self, command_pos):
        self.received_command_pos = command_pos
        self.point_command_pos = self.point_copy(command_pos.position)
        orientation_deg = self.quatenrion_point_to_euler_degree(command_pos.orientation)
        self.point_command_pos_yaw =  orientation_deg.z
        # rospy.loginfo("self.point_command_pos_yaw = {} {} {}".format(self.point_command_pos_yaw, command_pos.orientation, orientation_deg))
        self.update_command_pos_to_gui()  

    def slam_callback(self, slam_msg):
        self.slam_pos = slam_msg.pose.position
        if self.init_slam_pose_frame_flag:
            self.slam_pose_strigvar_x.set('%.4f'%(self.slam_pos.x))
            self.slam_pose_strigvar_y.set('%.4f'%(self.slam_pos.y))
            self.slam_pose_strigvar_z.set('%.4f'%(self.slam_pos.z))    

        if self.init_angle_calc_frame_flag:
            self.angle_calc_strigvar_x_moved.set('%.4f'%(self.slam_pos.x - self.angle_delta_x))
            self.angle_calc_strigvar_y_moved.set('%.4f'%(self.slam_pos.y - self.angle_delta_y))
            self.angle_calc_strigvar_z_moved.set('%.4f'%(self.slam_pos.z - self.angle_delta_z))  
        if self.manual_control_clear_btn.cget('background') == 'black':
            if self.manual_control_clear_btn.cget('foreground') == 'yellow':
                self.manual_control_clear_btn.configure(bg = 'black', fg = 'red')
            else:
                self.manual_control_clear_btn.configure(bg = 'black', fg = 'yellow')
        if self.allow_slam_control_btn.cget('background') == 'black':
            if self.allow_slam_control_btn.cget('foreground') == 'yellow':
                self.allow_slam_control_btn.configure(bg = 'black', fg = 'red')
            else:
                self.allow_slam_control_btn.configure(bg = 'black', fg = 'yellow')


    def update_command_pos_from_gui(self):
        if self.init_command_pos_frame_flag:
            self.point_command_pos.x = float(self.command_strigvar_x.get())
            self.point_command_pos.y = float(self.command_strigvar_y.get())
            self.point_command_pos.z = float(self.command_strigvar_z.get())
            self.point_command_pos_yaw = float(self.command_strigvar_yaw.get())
        self.command_pos.position = self.point_command_pos
        yaw_rad = self.deg_to_rad(self.point_command_pos_yaw)
        orientation = quaternion_from_euler(0, 0, yaw_rad)
        self.command_pos.orientation.x = orientation[0]
        self.command_pos.orientation.y = orientation[1]
        self.command_pos.orientation.z = orientation[2]
        self.command_pos.orientation.w = orientation[3]


    def rad_to_deg(self, rad):
        return rad / math.pi * 180.0

    def deg_to_rad(self, deg):
        return deg * math.pi / 180.0

    def update_command_pos_to_gui(self):
        if self.init_command_pos_frame_flag:
            self.command_strigvar_x.set('%.4f'%(self.point_command_pos.x))
            self.command_strigvar_y.set('%.4f'%(self.point_command_pos.y))
            self.command_strigvar_z.set('%.4f'%(self.point_command_pos.z))
            self.command_strigvar_yaw.set('%.4f'%(self.point_command_pos_yaw))

    def publish_command(self):
        self.update_command_pos_from_gui()
        self.command_pos_publisher.publish(self.command_pos)
        time.sleep(0.2)
        self.command_pos_publisher.publish(self.command_pos)
        time.sleep(0.2)
        self.command_pos_publisher.publish(self.command_pos)

    def stay_in_place(self):
        self.point_command_pos.x = self.real_world_pos.x
        self.point_command_pos.y = self.real_world_pos.y
        self.point_command_pos.z = self.real_world_pos.z
        self.point_command_pos_yaw = self.orientation_degree.z
        self.update_command_pos_to_gui()
        self.publish_command()

    def speed_callback(self, twist_msg):
        if self.init_speed_frame_flag:
            self.speed_strigvar_pitch.set('%.4f'%(twist_msg.linear.x))
            self.speed_strigvar_roll.set('%.4f'%(-twist_msg.linear.y))
            self.speed_strigvar_throttle.set('%.4f'%(twist_msg.linear.z))
            self.speed_strigvar_yaw.set('%.4f'%(-twist_msg.angular.z))

    def manual_control_set_callback(self):
        try:
            self.twist_manual_control.linear.x = float(self.manual_control_strigvar_pitch.get())
            self.twist_manual_control.linear.y = -float(self.manual_control_strigvar_roll.get())
            self.twist_manual_control.linear.z = float(self.manual_control_strigvar_throttle.get())
            self.twist_manual_control.angular.z = -float(self.manual_control_strigvar_yaw.get())
        except ValueError:
            self.twist_manual_control = Twist()
        self.cmd_val_publisher.publish(self.twist_manual_control)
        self.manual_control_set_btn.configure(fg = 'green', bg=self.default_bg)
        self.manual_control_clear_btn.configure(bg = 'black', fg = 'yellow')
        self.btn_calibrate_z.configure(bg=self.default_bg)

    def manual_control_clear_callback(self):
        self.twist_manual_control.linear.x = 0
        self.twist_manual_control.linear.y = 0
        self.twist_manual_control.linear.z = 0
        self.twist_manual_control.angular.z = 0
        self.cmd_val_publisher.publish(self.twist_manual_control)
        self.manual_control_set_btn.configure(fg = 'black',  bg = 'yellow')
        self.manual_control_clear_btn.configure(bg = self.default_bg, fg = 'black')
        self.btn_calibrate_z.configure(bg='yellow')

    def reset_map_callback(self):
        self.client.update_configuration({"reset_map": True})


    def takeoff(self):
        self.land = False
        self.pub_takeoff.publish()
        self.btn_takeoff.configure(fg='green', bg='white')
        self.btn_land.configure(fg='black', bg='yellow')


    def change_mux(self):
        self.current_mux = 1-self.current_mux
        self.pub_mux.publish(self.current_mux)

    def land_callback(self):
        self.land = True
        self.pub_land.publish()
        self.btn_takeoff.configure(fg='black', bg='yellow')
        self.btn_land.configure(fg='green', bg='white')


    def onClose(self, *args):
        """
        set the stop event, cleanup the camera, and allow the rest of
        
        the quit process to continue
        """
        print("[INFO] closing...")
        # self.stopEvent.set()
        self.quit_flag = True
        time.sleep(1)
        self.root.quit()
        self.root.destroy()
        # rospy.signal_shutdown('Exited UI')


if __name__ == '__main__':
    root = tki.Tk()
    my_gui = TelloUI(root)
    root.mainloop()

