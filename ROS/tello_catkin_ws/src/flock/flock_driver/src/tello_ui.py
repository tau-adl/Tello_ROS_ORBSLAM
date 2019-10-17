#!/usr/bin/env python

import rospy
import Tkinter as tki
from Tkinter import Toplevel, Scale
import threading
import os
import time
import platform
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import Empty, Bool, Int32, Float32
from flock_msgs.msg import Flip, FlightData
import signal
import math


class TelloUI(object):
    """Wrapper class to enable the GUI."""

    def __init__(self, root):

        rospy.init_node('tello_ui', anonymous=False)

        # rospy.Subscriber('/command_x', Point, self.command_x_callback)

        signal.signal(signal.SIGINT, self.onClose)
        signal.signal(signal.SIGTERM, self.onClose)



        self.point_command_x = Point()
        self.rotated_pos = Point()
        self.slam_pos = Point()
        self.twist_manual_control = Twist()
        self.real_world_pos = Point()
        self.delta_pos = Point()

        self.allow_slam_control = False

        # initialize the root window and image panel
        self.root = root

        self.panel = None
       # self.panel_for_pose_handle_show = None

        # create buttons

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


        self.init_command_x_frame()

        self.init_real_world_frame()

        self.init_slam_pose_frame()

        self.init_delta_frame()

        self.init_speed_frame()

        self.init_info_frame()

        self.init_manual_control_frame()

        self.init_angle_calc_frame()

        self.init_rotated_frame()


        #############################################################################
        

        #############################################################################



        #############################################################################

        

        #############################################################################

        

        #############################################################################

        

        #############################################################################


        self.row += 1
        self.column = 0



        # start a thread that constantly pools the video sensor for
        # the most recently read frame
        # self.stopEvent = threading.Event()
        # self.thread = threading.Thread(target=self.videoLoop, args=())
        # self.thread.start()

        # set a callback to handle when the window is closed
        self.root.wm_title("TELLO Controller")
        self.root.wm_protocol("WM_DELETE_WINDOW", self.onClose)

        rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, self.slam_callback)
        rospy.Subscriber('/tello/delta_pos', Point, self.delta_pos_callback)
        rospy.Subscriber('cmd_vel', Twist, self.speed_callback)
        rospy.Subscriber('flight_data', FlightData, self.flightdata_callback)
        rospy.Subscriber('/tello/allow_slam_control', Bool, self.allow_slam_control_callback)
        rospy.Subscriber('/tello/real_world_scale', Float32, self.real_world_scale_callback)
        rospy.Subscriber('/tello/real_world_pos', Point, self.real_world_pos_callback) 
        rospy.Subscriber('/tello/rotated_pos', Point, self.rotated_pos_callback)

        self.command_x_publisher = rospy.Publisher('command_x', Point, queue_size = 1)
        self.pub_takeoff = rospy.Publisher('takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('land', Empty, queue_size=1)
        self.pub_allow_slam_control = rospy.Publisher('/tello/allow_slam_control', Bool, queue_size=1)
        self.cmd_val_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

        
        # self.root.mainloop()
        # rospy.spin()

        # self.onClose()


        # the auto-takeoff thread will start if the 'takeoff' button on command window is clicked 
        # self.auto_takeoff_thread = threading.Thread(target=self._autoTakeoff)
        # the sending_command will send command to tello every 5 seconds
        # self.sending_command_thread = threading.Thread(target = self._sendingCommand)
        # self.get_GUI_Image_thread = threading.Thread(target = self._getGUIImage)

    def init_command_x_frame(self):
        self.frame_command = tki.Frame(self.root, relief=tki.SUNKEN)
        self.frame_command.grid(row=self.row, column=self.column)

        self.title_label_command_x = tki.Label(self.frame_command, text="Command X", font=("Helvetica", 13))
        self.title_label_command_x.grid(row=self.frame_row, column=1, padx=10, pady=5)

        self.frame_row += 1

        self.command_label_x = tki.Label(self.frame_command, text="X")
        self.command_label_x.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.command_label_y = tki.Label(self.frame_command, text="Y")
        self.command_label_y.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.command_label_z = tki.Label(self.frame_command, text="Z")
        self.command_label_z.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.frame_row += 1
        self.frame_column = 0

        self.command_strigvar_x = tki.StringVar()
        self.command_entry_x = tki.Entry(self.frame_command, width=15, textvariable=self.command_strigvar_x)
        self.command_entry_x.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.command_entry_x.delete(0, tki.END)
        self.command_entry_x.insert(0, "0.0")

        self.frame_column += 1

        self.command_strigvar_y = tki.StringVar()
        self.command_entry_y = tki.Entry(self.frame_command, width=15, textvariable=self.command_strigvar_y)
        self.command_entry_y.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.command_entry_y.delete(0, tki.END)
        self.command_entry_y.insert(0, "0.0")

        self.frame_column += 1

        self.command_strigvar_z = tki.StringVar()
        self.command_entry_z = tki.Entry(self.frame_command, width=15, textvariable=self.command_strigvar_z)
        self.command_entry_z.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.command_entry_z.delete(0, tki.END)
        self.command_entry_z.insert(0, "0.0")

        self.frame_column += 1

        self.frame_row += 1
        self.frame_column = 0

        self.btn_takeoff = tki.Button(self.frame_command, text="Takeoff!", command=self.takeoff)
        self.btn_takeoff.grid(row=self.frame_row, column=0, padx=10, pady=5)

        self.btn_publish_command = tki.Button(self.frame_command, text="Publish Command!", command=self.publish_command)
        self.btn_publish_command.grid(row=self.frame_row, column=1, padx=10, pady=5)

        self.btn_land = tki.Button(self.frame_command, text="Land!", command=self.land)
        self.btn_land.grid(row=self.frame_row, column=2, padx=10, pady=5)

        self.frame_row += 1

        self.btn_stay_in_place = tki.Button(self.frame_command, text="Stay In Place!", command=self.stay_in_place)
        self.btn_stay_in_place.grid(row=self.frame_row, column=1, padx=10, pady=5)

        self.row += 1
        self.column = 0

    def init_slam_pose_frame(self):
        self.frame_column = 0
        self.frame_row = 0

        self.frame_pose = tki.Frame(self.root, relief=tki.SUNKEN)
        self.frame_pose.grid(row=self.row, column=self.column)

        self.title_label_pose = tki.Label(self.frame_pose, text="Slam Pose", font=("Helvetica", 13))
        self.title_label_pose.grid(row=self.frame_row, column=1, padx=10, pady=5)

        self.frame_row += 1

        self.pose_label_x = tki.Label(self.frame_pose, text="X")
        self.pose_label_x.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.pose_label_y = tki.Label(self.frame_pose, text="Y")
        self.pose_label_y.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.pose_label_z = tki.Label(self.frame_pose, text="Z")
        self.pose_label_z.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.frame_row += 1
        self.frame_column = 0

        self.pose_strigvar_x = tki.StringVar()
        self.pose_entry_x = tki.Entry(self.frame_pose, width=15, textvariable=self.pose_strigvar_x)
        self.pose_entry_x.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.pose_entry_x.delete(0, tki.END)
        self.pose_entry_x.insert(0, "0.0")

        self.frame_column += 1

        self.pose_strigvar_y = tki.StringVar()
        self.pose_entry_y = tki.Entry(self.frame_pose, width=15, textvariable=self.pose_strigvar_y)
        self.pose_entry_y.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.pose_entry_y.delete(0, tki.END)
        self.pose_entry_y.insert(0, "0.0")

        self.frame_column += 1

        self.pose_strigvar_z = tki.StringVar()
        self.pose_entry_z = tki.Entry(self.frame_pose, width=15, textvariable=self.pose_strigvar_z)
        self.pose_entry_z.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.pose_entry_z.delete(0, tki.END)
        self.pose_entry_z.insert(0, "0.0")

        self.frame_column += 1
        self.row += 1
        self.column = 0

    def init_delta_frame(self):
        self.frame_column = 0
        self.frame_row = 0

        self.frame_delta = tki.Frame(self.root, relief=tki.SUNKEN)
        self.frame_delta.grid(row=self.row, column=self.column)

        self.current_frame = self.frame_delta

        self.title_label_dela = tki.Label(self.current_frame, text="Delta", font=("Helvetica", 13))
        self.title_label_dela.grid(row=self.frame_row, column=1, padx=10, pady=5)

        self.frame_row += 1

        self.delta_label_x = tki.Label(self.current_frame, text="X")
        self.delta_label_x.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.delta_label_y = tki.Label(self.current_frame, text="Y")
        self.delta_label_y.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.delta_label_z = tki.Label(self.current_frame, text="Z")
        self.delta_label_z.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.frame_row += 1
        self.frame_column = 0

        self.delta_strigvar_x = tki.StringVar()
        self.delta_entry_x = tki.Entry(self.current_frame, width=15, textvariable=self.delta_strigvar_x)
        self.delta_entry_x.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.delta_entry_x.delete(0, tki.END)
        self.delta_entry_x.insert(0, "0.0")

        self.frame_column += 1

        self.delta_strigvar_y = tki.StringVar()
        self.delta_entry_y = tki.Entry(self.current_frame, width=15, textvariable=self.delta_strigvar_y)
        self.delta_entry_y.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.delta_entry_y.delete(0, tki.END)
        self.delta_entry_y.insert(0, "0.0")

        self.frame_column += 1

        self.delta_strigvar_z = tki.StringVar()
        self.delta_entry_z = tki.Entry(self.current_frame, width=15, textvariable=self.delta_strigvar_z)
        self.delta_entry_z.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.delta_entry_z.delete(0, tki.END)
        self.delta_entry_z.insert(0, "0.0")

        self.frame_column += 1
        self.row += 1
        self.column = 0
    
    def init_speed_frame(self):
        self.frame_column = 0
        self.frame_row = 0

        self.frame_speed = tki.Frame(self.root, relief=tki.SUNKEN)
        self.frame_speed.grid(row=self.row, column=self.column)

        self.current_frame = self.frame_speed

        self.title_label_speed = tki.Label(self.current_frame, text="Speed", font=("Helvetica", 13))
        self.title_label_speed.grid(row=self.frame_row, column=1, padx=10, pady=5)

        self.frame_row += 1

        self.speed_label_pitch = tki.Label(self.current_frame, text="Pitch")
        self.speed_label_pitch.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.speed_label_roll = tki.Label(self.current_frame, text="Roll")
        self.speed_label_roll.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.speed_label_throttle = tki.Label(self.current_frame, text="Throttle")
        self.speed_label_throttle.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.speed_label_yaw = tki.Label(self.current_frame, text="Yaw")
        self.speed_label_yaw.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1


        self.frame_row += 1
        self.frame_column = 0

        self.speed_strigvar_pitch = tki.StringVar()
        self.speed_entry_pitch = tki.Entry(self.current_frame, width=15, textvariable=self.speed_strigvar_pitch)
        self.speed_entry_pitch.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.speed_entry_pitch.delete(0, tki.END)
        self.speed_entry_pitch.insert(0, "0.0")

        self.frame_column += 1

        self.speed_strigvar_roll = tki.StringVar()
        self.speed_entry_roll = tki.Entry(self.current_frame, width=15, textvariable=self.speed_strigvar_roll)
        self.speed_entry_roll.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.speed_entry_roll.delete(0, tki.END)
        self.speed_entry_roll.insert(0, "0.0")

        self.frame_column += 1

        self.speed_strigvar_throttle = tki.StringVar()
        self.speed_entry_throttle = tki.Entry(self.current_frame, width=15, textvariable=self.speed_strigvar_throttle)
        self.speed_entry_throttle.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.speed_entry_throttle.delete(0, tki.END)
        self.speed_entry_throttle.insert(0, "0.0")

        self.frame_column += 1

        self.speed_strigvar_yaw = tki.StringVar()
        self.speed_entry_yaw = tki.Entry(self.current_frame, width=15, textvariable=self.speed_strigvar_yaw)
        self.speed_entry_yaw.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.speed_entry_yaw.delete(0, tki.END)
        self.speed_entry_yaw.insert(0, "0.0")

        self.row += 1
        self.column = 0

    def init_info_frame(self):
        self.frame_column = 0
        self.frame_row = 0

        self.frame_etc = tki.Frame(self.root, relief=tki.SUNKEN)
        self.frame_etc.grid(row=self.row, column=self.column)

        self.current_frame = self.frame_etc


        self.altitude_label = tki.Label(self.current_frame, text="Altitude")
        self.altitude_label.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)

        self.frame_column += 1

        self.battery_label = tki.Label(self.current_frame, text="Battery %")
        self.battery_label.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)

        self.frame_column += 1

        self.flight_time_remaining_label = tki.Label(self.current_frame, text="flight time remaining")
        self.flight_time_remaining_label.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)

        self.frame_column += 1

        self.allow_slam_control_btn = tki.Button(self.current_frame, text="Toggle Slam Control!",  command=self.allow_slam_control_btn_callback)
        self.allow_slam_control_btn.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)


        self.frame_row += 1
        self.frame_column = 0

        self.altitude_strigvar = tki.StringVar()
        self.altitude_entry = tki.Entry(self.current_frame, width=15, textvariable=self.altitude_strigvar)
        self.altitude_entry.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.altitude_entry.delete(0, tki.END)
        self.altitude_entry.insert(0, "0.0")

        self.frame_column += 1

        self.battery_strigvar = tki.StringVar()
        self.battery_entry = tki.Entry(self.current_frame, width=15, textvariable=self.battery_strigvar)
        self.battery_entry.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.battery_entry.delete(0, tki.END)
        self.battery_entry.insert(0, "0.0")

        self.frame_column += 1

        self.flight_time_remaining_strigvar = tki.StringVar()
        self.flight_time_remaining_entry = tki.Entry(self.current_frame, width=15, textvariable=self.flight_time_remaining_strigvar)
        self.flight_time_remaining_entry.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.flight_time_remaining_entry.delete(0, tki.END)
        self.flight_time_remaining_entry.insert(0, "0.0")

        self.frame_column += 1

        self.allow_slam_control_strigvar = tki.StringVar()
        self.allow_slam_control_entry = tki.Entry(self.current_frame, width=15, textvariable=self.allow_slam_control_strigvar)
        self.allow_slam_control_entry.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.allow_slam_control_entry.delete(0, tki.END)
        self.allow_slam_control_entry.insert(0, "{}".format(self.allow_slam_control))

        self.row += 1
        self.column = 0

    def init_manual_control_frame(self):
        self.frame_column = 0
        self.frame_row = 0

        self.frame_manual_control = tki.Frame(self.root, relief=tki.SUNKEN)
        self.frame_manual_control.grid(row=self.row, column=self.column)

        self.current_frame = self.frame_manual_control

        self.title_label_manual_control = tki.Label(self.current_frame, text="Manual Control", font=("Helvetica", 13))
        self.title_label_manual_control.grid(row=self.frame_row, column=2, padx=10, pady=5)

        self.frame_row += 1

        self.manual_control_label_pitch = tki.Label(self.current_frame, text="Pitch")
        self.manual_control_label_pitch.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.manual_control_label_roll = tki.Label(self.current_frame, text="Roll")
        self.manual_control_label_roll.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.manual_control_label_throttle = tki.Label(self.current_frame, text="Throttle")
        self.manual_control_label_throttle.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.manual_control_label_yaw = tki.Label(self.current_frame, text="Yaw")
        self.manual_control_label_yaw.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.manual_control_set_btn = tki.Button(self.current_frame, text="Manual Control Set!",  command=self.manual_control_set_callback)
        self.manual_control_set_btn.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)

        self.frame_column += 1


        self.frame_row += 1
        self.frame_column = 0

        self.manual_control_strigvar_pitch = tki.StringVar()
        self.manual_control_entry_pitch = tki.Entry(self.current_frame, width=15, textvariable=self.manual_control_strigvar_pitch)
        self.manual_control_entry_pitch.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.manual_control_entry_pitch.delete(0, tki.END)
        self.manual_control_entry_pitch.insert(0, "0.0")

        self.frame_column += 1

        self.manual_control_strigvar_roll = tki.StringVar()
        self.manual_control_entry_roll = tki.Entry(self.current_frame, width=15, textvariable=self.manual_control_strigvar_roll)
        self.manual_control_entry_roll.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.manual_control_entry_roll.delete(0, tki.END)
        self.manual_control_entry_roll.insert(0, "0.0")

        self.frame_column += 1

        self.manual_control_strigvar_throttle = tki.StringVar()
        self.manual_control_entry_throttle = tki.Entry(self.current_frame, width=15, textvariable=self.manual_control_strigvar_throttle)
        self.manual_control_entry_throttle.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.manual_control_entry_throttle.delete(0, tki.END)
        self.manual_control_entry_throttle.insert(0, "0.0")

        self.frame_column += 1

        self.manual_control_strigvar_yaw = tki.StringVar()
        self.manual_control_entry_yaw = tki.Entry(self.current_frame, width=15, textvariable=self.manual_control_strigvar_yaw)
        self.manual_control_entry_yaw.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.manual_control_entry_yaw.delete(0, tki.END)
        self.manual_control_entry_yaw.insert(0, "0.0")

        self.frame_column += 1

        self.manual_control_clear_btn = tki.Button(self.current_frame, text="Manual Control Clear!",  command=self.manual_control_clear_callback)
        self.manual_control_clear_btn.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)

        self.frame_column += 1

        self.row += 1
        self.column = 0

    def init_angle_calc_frame(self):
        self.frame_column = 0
        self.frame_row = 0

        self.frame_angle_calc = tki.Frame(self.root, relief=tki.SUNKEN)
        self.frame_angle_calc.grid(row=self.row, column=self.column)

        self.current_frame = self.frame_angle_calc

        self.title_label_angle_calc = tki.Label(self.current_frame, text="Angel Calc", font=("Helvetica", 13))
        self.title_label_angle_calc.grid(row=self.frame_row, column=2, padx=10, pady=5)

        self.frame_row += 1

        self.angle_calc_label_x_moved = tki.Label(self.current_frame, text="X Moved")
        self.angle_calc_label_x_moved.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.angle_calc_label_y_moved = tki.Label(self.current_frame, text="Y Moved")
        self.angle_calc_label_y_moved.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.angle_calc_label_z_moved = tki.Label(self.current_frame, text="Z Moved")
        self.angle_calc_label_z_moved.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.angle_calc_label_angle = tki.Label(self.current_frame, text="Angle")
        self.angle_calc_label_angle.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.angle_calc_set_btn = tki.Button(self.current_frame, text="Calculate Angle!",  command=self.angle_calc_set_callback)
        self.angle_calc_set_btn.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)

        self.frame_column += 1


        self.frame_row += 1
        self.frame_column = 0

        self.angle_calc_strigvar_x_moved = tki.StringVar()
        self.angle_calc_entry_x_moved = tki.Entry(self.current_frame, width=15, textvariable=self.angle_calc_strigvar_x_moved)
        self.angle_calc_entry_x_moved.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.angle_calc_entry_x_moved.delete(0, tki.END)
        self.angle_calc_entry_x_moved.insert(0, "0.0")

        self.frame_column += 1

        self.angle_calc_strigvar_y_moved = tki.StringVar()
        self.angle_calc_entry_y_moved = tki.Entry(self.current_frame, width=15, textvariable=self.angle_calc_strigvar_y_moved)
        self.angle_calc_entry_y_moved.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.angle_calc_entry_y_moved.delete(0, tki.END)
        self.angle_calc_entry_y_moved.insert(0, "0.0")

        self.frame_column += 1

        self.angle_calc_strigvar_z_moved = tki.StringVar()
        self.angle_calc_entry_z_moved = tki.Entry(self.current_frame, width=15, textvariable=self.angle_calc_strigvar_z_moved)
        self.angle_calc_entry_z_moved.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.angle_calc_entry_z_moved.delete(0, tki.END)
        self.angle_calc_entry_z_moved.insert(0, "0.0")

        self.frame_column += 1

        self.angle_calc_strigvar_angle = tki.StringVar()
        self.angle_calc_entry_angle = tki.Entry(self.current_frame, width=15, textvariable=self.angle_calc_strigvar_angle)
        self.angle_calc_entry_angle.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.angle_calc_entry_angle.delete(0, tki.END)
        self.angle_calc_entry_angle.insert(0, "0.0")
        self.angle_calc_strigvar_angle.set('%.4f'%(self.angle))  

        self.frame_column += 1

        self.angle_calc_clear_btn = tki.Button(self.current_frame, text="Clear Angle!",  command=self.angle_calc_clear_callback)
        self.angle_calc_clear_btn.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)

        self.frame_column += 1

        self.row += 1
        self.column = 0
    
    def init_rotated_frame(self):
        self.frame_column = 0
        self.frame_row = 0

        self.frame_rotated = tki.Frame(self.root, relief=tki.SUNKEN)
        self.frame_rotated.grid(row=self.row, column=self.column)

        self.title_label_rotated = tki.Label(self.frame_rotated, text="Rotated Coardinates", font=("Helvetica", 13))
        self.title_label_rotated.grid(row=self.frame_row, column=1, padx=10, pady=5)

        self.frame_row += 1

        self.rotated_label_x = tki.Label(self.frame_rotated, text="X")
        self.rotated_label_x.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.rotated_label_y = tki.Label(self.frame_rotated, text="Y")
        self.rotated_label_y.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.rotated_label_z = tki.Label(self.frame_rotated, text="Z")
        self.rotated_label_z.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.frame_row += 1
        self.frame_column = 0

        self.rotated_strigvar_x = tki.StringVar()
        self.rotated_entry_x = tki.Entry(self.frame_rotated, width=15, textvariable=self.rotated_strigvar_x)
        self.rotated_entry_x.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.rotated_entry_x.delete(0, tki.END)
        self.rotated_entry_x.insert(0, "0.0")

        self.frame_column += 1

        self.rotated_strigvar_y = tki.StringVar()
        self.rotated_entry_y = tki.Entry(self.frame_rotated, width=15, textvariable=self.rotated_strigvar_y)
        self.rotated_entry_y.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.rotated_entry_y.delete(0, tki.END)
        self.rotated_entry_y.insert(0, "0.0")

        self.frame_column += 1

        self.rotated_strigvar_z = tki.StringVar()
        self.rotated_entry_z = tki.Entry(self.frame_rotated, width=15, textvariable=self.rotated_strigvar_z)
        self.rotated_entry_z.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.rotated_entry_z.delete(0, tki.END)
        self.rotated_entry_z.insert(0, "0.0")

        self.frame_column += 1
        self.row += 1
        self.column = 0

    def init_real_world_frame(self):
        self.frame_column = 0
        self.frame_row = 0

        self.frame_real_world = tki.Frame(self.root, relief=tki.SUNKEN)
        self.frame_real_world.grid(row=self.row, column=self.column)

        self.title_label_real_world = tki.Label(self.frame_real_world, text="Real World[cm]", font=("Helvetica", 13))
        self.title_label_real_world.grid(row=self.frame_row, column=1, padx=10, pady=5)

        self.frame_row += 1

        self.real_world_label_x = tki.Label(self.frame_real_world, text="X")
        self.real_world_label_x.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.real_world_label_y = tki.Label(self.frame_real_world, text="Y")
        self.real_world_label_y.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.real_world_label_z = tki.Label(self.frame_real_world, text="Z")
        self.real_world_label_z.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.real_world_label_scale = tki.Label(self.frame_real_world, text="Altitude Scale")
        self.real_world_label_scale.grid(row=self.frame_row, column=self.frame_column, padx=10, pady=5)
        self.frame_column += 1

        self.frame_row += 1
        self.frame_column = 0

        self.real_world_strigvar_x = tki.StringVar()
        self.real_world_entry_x = tki.Entry(self.frame_real_world, width=15, textvariable=self.real_world_strigvar_x)
        self.real_world_entry_x.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.real_world_entry_x.delete(0, tki.END)
        self.real_world_entry_x.insert(0, "0.0")

        self.frame_column += 1

        self.real_world_strigvar_y = tki.StringVar()
        self.real_world_entry_y = tki.Entry(self.frame_real_world, width=15, textvariable=self.real_world_strigvar_y)
        self.real_world_entry_y.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.real_world_entry_y.delete(0, tki.END)
        self.real_world_entry_y.insert(0, "0.0")

        self.frame_column += 1

        self.real_world_strigvar_z = tki.StringVar()
        self.real_world_entry_z = tki.Entry(self.frame_real_world, width=15, textvariable=self.real_world_strigvar_z)
        self.real_world_entry_z.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.real_world_entry_z.delete(0, tki.END)
        self.real_world_entry_z.insert(0, "0.0")

        self.frame_column += 1

        self.real_world_strigvar_scale = tki.StringVar()
        self.real_world_entry_scale = tki.Entry(self.frame_real_world, width=15, textvariable=self.real_world_strigvar_scale)
        self.real_world_entry_scale.grid(row=self.frame_row, column=self.frame_column, padx=5, pady=5)
        self.real_world_entry_scale.delete(0, tki.END)
        self.real_world_entry_scale.insert(0, "0.0")

        self.frame_column += 1

        self.row += 1
        self.column = 0
    
    def allow_slam_control_btn_callback(self):
        self.pub_allow_slam_control.publish(not self.allow_slam_control)

    def real_world_scale_callback(self, msg):
        self.real_world_scale = float(msg.data)
        self.real_world_strigvar_scale.set('%.4f'%(self.real_world_scale)) 

    def real_world_pos_callback(self, msg):
        self.real_world_pos = msg
        self.real_world_strigvar_x.set('%.4f'%(self.real_world_pos.x))
        self.real_world_strigvar_y.set('%.4f'%(self.real_world_pos.y))
        self.real_world_strigvar_z.set('%.4f'%(self.real_world_pos.z))

    def flightdata_callback(self, flight_data):
        self.altitude = flight_data.altitude
        self.altitude_strigvar.set('%.4f'%(self.altitude))
        self.battery_strigvar.set('%.2f'%(flight_data.battery_percent))
        self.flight_time_remaining_strigvar.set('%.2f'%(flight_data.estimated_flight_time_remaining))
        # try:
            # if self.altitude > 0.2:
                # self.real_world_scale = self.altitude / self.rotated_pos.z
        # except ZeroDivisionError:
            # self.real_world_scale = 1
        # self.real_world_strigvar_scale.set('%.4f'%(self.real_world_scale))  

    def allow_slam_control_callback(self, msg):
        self.allow_slam_control = (msg.data == 1)
        self.allow_slam_control_strigvar.set(self.allow_slam_control)

    def angle_calc_clear_callback(self):
        self.angle_delta_x = float(self.pose_strigvar_x.get())
        self.angle_delta_y = float(self.pose_strigvar_y.get())
        self.angle_delta_z = float(self.pose_strigvar_z.get())

    def angle_calc_set_callback(self):
        x = float(self.angle_calc_strigvar_x_moved.get())
        z = float(self.angle_calc_strigvar_z_moved.get())
        tan_angle = z/x
        self.angle_radian = math.atan(tan_angle)
        self.angle = self.angle_radian*180/math.pi
        self.angle_calc_strigvar_angle.set('%.4f'%(self.angle))  
        print("x={} z={} z/x={} angle={}".format(x, z, tan_angle, self.angle))


    def delta_pos_callback(self, delta_pos):
        self.delta_pos = delta_pos
        self.delta_strigvar_x.set('%.4f'%(self.delta_pos.x))
        self.delta_strigvar_y.set('%.4f'%(self.delta_pos.y))
        self.delta_strigvar_z.set('%.4f'%(self.delta_pos.z))

    def rotated_pos_callback(self, rotated_pos):
        self.rotated_pos = rotated_pos
        self.rotated_strigvar_x.set('%.4f'%(self.rotated_pos.x))
        self.rotated_strigvar_y.set('%.4f'%(self.rotated_pos.y))
        self.rotated_strigvar_z.set('%.4f'%(self.rotated_pos.z))

        

    def slam_callback(self, slam_msg):
        self.slam_pos = slam_msg.pose.position

        # self.rotated_pos.x = self.slam_pos.x * math.cos(self.angle_radian) + self.slam_pos.z * math.sin(self.angle_radian)
        # self.rotated_pos.y = self.slam_pos.y
        # self.rotated_pos.z = self.slam_pos.x * (-math.sin(self.angle_radian)) + self.slam_pos.z * math.cos(self.angle_radian)

        # self.real_world_pos.x = self.rotated_pos.x * self.real_world_scale
        # self.real_world_pos.y = self.rotated_pos.y * self.real_world_scale
        # self.real_world_pos.z = self.rotated_pos.z * self.real_world_scale

        self.pose_strigvar_x.set('%.4f'%(self.slam_pos.x))
        self.pose_strigvar_y.set('%.4f'%(self.slam_pos.y))
        self.pose_strigvar_z.set('%.4f'%(self.slam_pos.z))    

        self.angle_calc_strigvar_x_moved.set('%.4f'%(self.slam_pos.x - self.angle_delta_x))
        self.angle_calc_strigvar_y_moved.set('%.4f'%(self.slam_pos.y - self.angle_delta_y))
        self.angle_calc_strigvar_z_moved.set('%.4f'%(self.slam_pos.z - self.angle_delta_z))  
        self.command_x_publisher.publish(self.point_command_x)

        # self.rotated_strigvar_x.set('%.4f'%(self.rotated_pos.x))
        # self.rotated_strigvar_y.set('%.4f'%(self.rotated_pos.y))
        # self.rotated_strigvar_z.set('%.4f'%(self.rotated_pos.z))

        # self.real_world_strigvar_x.set('%.4f'%(self.real_world_pos.x))
        # self.real_world_strigvar_y.set('%.4f'%(self.real_world_pos.y))
        # self.real_world_strigvar_z.set('%.4f'%(self.real_world_pos.z))
    
    def publish_command(self):
        self.point_command_x.x = float(self.command_strigvar_x.get())
        self.point_command_x.y = float(self.command_strigvar_y.get())
        self.point_command_x.z = float(self.command_strigvar_z.get())
        self.command_x_publisher.publish(self.point_command_x)

    def stay_in_place(self):
        self.command_strigvar_x.set('%.4f'%(self.real_world_pos.x))
        self.command_strigvar_y.set('%.4f'%(self.real_world_pos.y))
        self.command_strigvar_z.set('%.4f'%(self.real_world_pos.z))
        self.command_x_publisher.publish(self.real_world_pos)

    def speed_callback(self, twist_msg):
        self.speed_strigvar_pitch.set('%.4f'%(twist_msg.linear.x))
        self.speed_strigvar_roll.set('%.4f'%(-twist_msg.linear.y))
        self.speed_strigvar_throttle.set('%.4f'%(twist_msg.linear.z))
        self.speed_strigvar_yaw.set('%.4f'%(-twist_msg.angular.z))

    def manual_control_set_callback(self):
        self.twist_manual_control.linear.x = float(self.manual_control_strigvar_pitch.get())
        self.twist_manual_control.linear.y = -float(self.manual_control_strigvar_roll.get())
        self.twist_manual_control.linear.z = float(self.manual_control_strigvar_throttle.get())
        self.twist_manual_control.angular.z = -float(self.manual_control_strigvar_yaw.get())
        self.cmd_val_publisher.publish(self.twist_manual_control)

    def manual_control_clear_callback(self):
        self.twist_manual_control.linear.x = 0
        self.twist_manual_control.linear.y = 0
        self.twist_manual_control.linear.z = 0
        self.twist_manual_control.angular.z = 0
        self.cmd_val_publisher.publish(self.twist_manual_control)



    def takeoff(self):
        self.pub_takeoff.publish()

    def land(self):
        self.pub_land.publish()


    def onClose(self, *args):
        """
        set the stop event, cleanup the camera, and allow the rest of
        
        the quit process to continue
        """
        print("[INFO] closing...")
        # self.stopEvent.set()
        self.root.quit()
        # rospy.signal_shutdown('Exited UI')

if __name__ == '__main__':
    root = tki.Tk()
    my_gui = TelloUI(root)
    root.mainloop()

