#!/usr/bin/env python
from __future__ import print_function
import rospy

import math
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty 
# from geometry_msgs.msg import PoseStamped, Point, TransformStamped, Quaternion
from geometry_msgs.msg import *
from sensor_msgs.msg import Image
import struct
import signal
import matplotlib
matplotlib.use('TkAgg')
import numpy as np
import threading
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import Tkinter as tki
from cv_bridge import CvBridge, CvBridgeError
# import ImageTk
from PIL import Image as ImagePIL
from PIL import ImageTk as ImageTkPIL
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
import tf.transformations
import tf2_geometry_msgs
import traceback
from copy import deepcopy



class CloudMapSaverForServer(object):

    def __init__(self, root):


        # Initialize ROS
        rospy.init_node('cloud_map_saver_for_server', anonymous=False)
        self.once = False

        try: 
            self.id                = rospy.get_param('~ID')
        except KeyError:
            self.id = ''

        signal.signal(signal.SIGINT, self.onClose)
        signal.signal(signal.SIGTERM, self.onClose)

        self.root = root

        self.bridge = CvBridge()

        # self.file_path = rospy.get_param('~OUT_FILE_PATH')
        self.cloud0_topic_name = rospy.get_param('~CLOUD0_TOPIC_NAME')
        self.cloud1_topic_name = rospy.get_param('~CLOUD1_TOPIC_NAME')
        # self.trigger_topic_name = rospy.get_param('~TRIGGER_TOPIC_NAME')
        self.pose0_topic_name = rospy.get_param('~POSE0_TOPIC_NAME')
        self.pose1_topic_name = rospy.get_param('~POSE1_TOPIC_NAME')
        self.camera0_topic_name = rospy.get_param('~CAMERA0_TOPIC_NAME')
        self.camera1_topic_name = rospy.get_param('~CAMERA1_TOPIC_NAME')
        # self.cloud_server_topic_name = rospy.get_param('~CLOUD_SERVER_TOPIC_NAME')

        self.received_image = False

        # self.updateImageGui = threading.Thread(target = self._getGUIImage)

        self.root.wm_title("TELLO Viewer Server")
        self.root.wm_protocol("WM_DELETE_WINDOW", self.onClose)

        self.list_of_pure_lines = []
        self.list_of_pure_lines2 = []
        



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

        
        self.x_min = -0.5
        self.x_max = 3.5
        self.y_min = -2
        self.y_max = 2
        self.z_min = -2
        self.z_max = 2

        self.scale = (self.z_max - self.z_min)/5.0


        self.move_up = tki.Button(self.root, text="Up", command=self.move_up_callback)
        self.move_up.grid(row=0, column=1, padx=3, pady=3)

        self.move_down = tki.Button(self.root, text="Down", command=self.move_down_callback)
        self.move_down.grid(row=2, column=1, padx=3, pady=3)

        self.move_left = tki.Button(self.root, text="Left", command=self.move_left_callback)
        self.move_left.grid(row=1, column=0, padx=3, pady=3)

        self.move_right = tki.Button(self.root, text="Right", command=self.move_right_callback)
        self.move_right.grid(row=1, column=2, padx=3, pady=3)

        self.zoom_in = tki.Button(self.root, text="Zoom In", command=self.zoom_in_callback)
        self.zoom_in.grid(row=0, column=0, padx=3, pady=3)

        self.zoom_out = tki.Button(self.root, text="Zoom Out", command=self.zoom_out_callback)
        self.zoom_out.grid(row=2, column=2, padx=3, pady=3)

        self.scale_button = tki.Button(self.root, text="Zoom In", command=self.zoom_in_callback)
        self.scale_button.grid(row=0, column=0, padx=3, pady=3)

        self.scale_strigvar = tki.StringVar()
        self.scale_entry = tki.Entry(self.root, width=9, textvariable=self.scale_strigvar)
        self.scale_entry.grid(row=0, column=3, padx=3, pady=3)
        self.scale_entry.delete(0, tki.END)
        self.scale_entry.insert(0, str(self.scale))

        
        row = 3

        ##########################################

        self.command_label_x_client0 = tki.Label(self.root, text="x0")
        self.command_label_x_client0.grid(row=row, column=0, padx=3, pady=3)

        self.command_label_y_client0 = tki.Label(self.root, text="y0")
        self.command_label_y_client0.grid(row=row, column=1, padx=3, pady=3)

        self.command_label_z_client0 = tki.Label(self.root, text="z0")
        self.command_label_z_client0.grid(row=row, column=2, padx=3, pady=3)

        row += 1

        self.command_strigvar_x_client0 = tki.StringVar()
        self.command_entry_x_client0 = tki.Entry(self.root, width=9, textvariable=self.command_strigvar_x_client0)
        self.command_entry_x_client0.grid(row=row, column=0, padx=3, pady=3)
        self.command_entry_x_client0.delete(0, tki.END)
        self.command_entry_x_client0.insert(0, "0.0")

        self.command_strigvar_y_client0 = tki.StringVar()
        self.command_entry_y_client0 = tki.Entry(self.root, width=9, textvariable=self.command_strigvar_y_client0)
        self.command_entry_y_client0.grid(row=row, column=1, padx=3, pady=3)
        self.command_entry_y_client0.delete(0, tki.END)
        self.command_entry_y_client0.insert(0, "0.0")

        self.command_strigvar_z_client0 = tki.StringVar()
        self.command_entry_z_client0 = tki.Entry(self.root, width=9, textvariable=self.command_strigvar_z_client0)
        self.command_entry_z_client0.grid(row=row, column=2, padx=3, pady=3)
        self.command_entry_z_client0.delete(0, tki.END)
        self.command_entry_z_client0.insert(0, "0.0")

        row += 1

        ##########################################

        self.command_label_pitch_client0 = tki.Label(self.root, text="Pitch0[Deg]")
        self.command_label_pitch_client0.grid(row=row, column=0, padx=3, pady=3)

        self.command_label_roll_client0 = tki.Label(self.root, text="Roll0[Deg]")
        self.command_label_roll_client0.grid(row=row, column=1, padx=3, pady=3)

        self.command_label_yaw_client0 = tki.Label(self.root, text="Yaw0[Deg]")
        self.command_label_yaw_client0.grid(row=row, column=2, padx=3, pady=3)

        row += 1

        self.command_strigvar_pitch_client0 = tki.StringVar()
        self.command_entry_pitch_client0 = tki.Entry(self.root, width=9, textvariable=self.command_strigvar_pitch_client0)
        self.command_entry_pitch_client0.grid(row=row, column=0, padx=3, pady=3)
        self.command_entry_pitch_client0.delete(0, tki.END)
        self.command_entry_pitch_client0.insert(0, "0.0")

        self.command_strigvar_roll_client0 = tki.StringVar()
        self.command_entry_roll_client0 = tki.Entry(self.root, width=9, textvariable=self.command_strigvar_roll_client0)
        self.command_entry_roll_client0.grid(row=row, column=1, padx=3, pady=3)
        self.command_entry_roll_client0.delete(0, tki.END)
        self.command_entry_roll_client0.insert(0, "0.0")

        self.command_strigvar_yaw_client0 = tki.StringVar()
        self.command_entry_yaw_client0 = tki.Entry(self.root, width=9, textvariable=self.command_strigvar_yaw_client0)
        self.command_entry_yaw_client0.grid(row=row, column=2, padx=3, pady=3)
        self.command_entry_yaw_client0.delete(0, tki.END)
        self.command_entry_yaw_client0.insert(0, "0.0")

        row += 1

        ##########################################

        self.command_label_x_client1 = tki.Label(self.root, text="x1")
        self.command_label_x_client1.grid(row=row, column=0, padx=3, pady=3)

        self.command_label_y_client1 = tki.Label(self.root, text="y1")
        self.command_label_y_client1.grid(row=row, column=1, padx=3, pady=3)

        self.command_label_z_client1 = tki.Label(self.root, text="z1")
        self.command_label_z_client1.grid(row=row, column=2, padx=3, pady=3)

        row += 1

        self.command_strigvar_x_client1 = tki.StringVar()
        self.command_entry_x_client1 = tki.Entry(self.root, width=9, textvariable=self.command_strigvar_x_client1)
        self.command_entry_x_client1.grid(row=row, column=0, padx=3, pady=3)
        self.command_entry_x_client1.delete(0, tki.END)
        self.command_entry_x_client1.insert(0, "0.0")

        self.command_strigvar_y_client1 = tki.StringVar()
        self.command_entry_y_client1 = tki.Entry(self.root, width=9, textvariable=self.command_strigvar_y_client1)
        self.command_entry_y_client1.grid(row=row, column=1, padx=3, pady=3)
        self.command_entry_y_client1.delete(0, tki.END)
        self.command_entry_y_client1.insert(0, "0.0")

        self.command_strigvar_z_client1 = tki.StringVar()
        self.command_entry_z_client1 = tki.Entry(self.root, width=9, textvariable=self.command_strigvar_z_client1)
        self.command_entry_z_client1.grid(row=row, column=2, padx=3, pady=3)
        self.command_entry_z_client1.delete(0, tki.END)
        self.command_entry_z_client1.insert(0, "0.0")

        row += 1

        ##########################################

        self.command_label_pitch_client1 = tki.Label(self.root, text="Pitch1[Deg]")
        self.command_label_pitch_client1.grid(row=row, column=0, padx=3, pady=3)

        self.command_label_roll_client1 = tki.Label(self.root, text="Roll1[Deg]")
        self.command_label_roll_client1.grid(row=row, column=1, padx=3, pady=3)

        self.command_label_yaw_client1 = tki.Label(self.root, text="Yaw1[Deg]")
        self.command_label_yaw_client1.grid(row=row, column=2, padx=3, pady=3)

        row += 1

        self.command_strigvar_pitch_client1 = tki.StringVar()
        self.command_entry_pitch_client1 = tki.Entry(self.root, width=9, textvariable=self.command_strigvar_pitch_client1)
        self.command_entry_pitch_client1.grid(row=row, column=0, padx=3, pady=3)
        self.command_entry_pitch_client1.delete(0, tki.END)
        self.command_entry_pitch_client1.insert(0, "0.0")

        self.command_strigvar_roll_client1 = tki.StringVar()
        self.command_entry_roll_client1 = tki.Entry(self.root, width=9, textvariable=self.command_strigvar_roll_client1)
        self.command_entry_roll_client1.grid(row=row, column=1, padx=3, pady=3)
        self.command_entry_roll_client1.delete(0, tki.END)
        self.command_entry_roll_client1.insert(0, "0.0")

        self.command_strigvar_yaw_client1 = tki.StringVar()
        self.command_entry_yaw_client1 = tki.Entry(self.root, width=9, textvariable=self.command_strigvar_yaw_client1)
        self.command_entry_yaw_client1.grid(row=row, column=2, padx=3, pady=3)
        self.command_entry_yaw_client1.delete(0, tki.END)
        self.command_entry_yaw_client1.insert(0, "0.0")

        row += 1

        ##########################################


















        row = 3

        ##########################################

    

        self.transform_label_scale_client0 = tki.Label(self.root, text="trans_scale0")
        self.transform_label_scale_client0.grid(row=row, column=6, padx=3, pady=3)

        self.transform_label_x_client0 = tki.Label(self.root, text="trans_x0")
        self.transform_label_x_client0.grid(row=row, column=3, padx=3, pady=3)

        self.transform_label_y_client0 = tki.Label(self.root, text="trans_y0")
        self.transform_label_y_client0.grid(row=row, column=4, padx=3, pady=3)

        self.transform_label_z_client0 = tki.Label(self.root, text="trans_z0")
        self.transform_label_z_client0.grid(row=row, column=5, padx=3, pady=3)

        row += 1

        self.transform_strigvar_scale_client0 = tki.StringVar()
        self.transform_entry_scale_client0 = tki.Entry(self.root, width=9, textvariable=self.transform_strigvar_scale_client0)
        self.transform_entry_scale_client0.grid(row=row, column=6)
        self.transform_entry_scale_client0.delete(0, tki.END)
        self.transform_entry_scale_client0.insert(0, "0.0")

        self.transform_strigvar_x_client0 = tki.StringVar()
        self.transform_entry_x_client0 = tki.Entry(self.root, width=9, textvariable=self.transform_strigvar_x_client0)
        self.transform_entry_x_client0.grid(row=row, column=3, padx=3, pady=3)
        self.transform_entry_x_client0.delete(0, tki.END)
        self.transform_entry_x_client0.insert(0, "0.0")

        self.transform_strigvar_y_client0 = tki.StringVar()
        self.transform_entry_y_client0 = tki.Entry(self.root, width=9, textvariable=self.transform_strigvar_y_client0)
        self.transform_entry_y_client0.grid(row=row, column=4, padx=3, pady=3)
        self.transform_entry_y_client0.delete(0, tki.END)
        self.transform_entry_y_client0.insert(0, "0.0")

        self.transform_strigvar_z_client0 = tki.StringVar()
        self.transform_entry_z_client0 = tki.Entry(self.root, width=9, textvariable=self.transform_strigvar_z_client0)
        self.transform_entry_z_client0.grid(row=row, column=5, padx=3, pady=3)
        self.transform_entry_z_client0.delete(0, tki.END)
        self.transform_entry_z_client0.insert(0, "0.0")

        row += 1

        ##########################################

        self.transform_label_pitch_client0 = tki.Label(self.root, text="trans_Pitch0[Deg]")
        self.transform_label_pitch_client0.grid(row=row, column=3, padx=3, pady=3)

        self.transform_label_roll_client0 = tki.Label(self.root, text="trans_Roll0[Deg]")
        self.transform_label_roll_client0.grid(row=row, column=4, padx=3, pady=3)

        self.transform_label_yaw_client0 = tki.Label(self.root, text="trans_Yaw0[Deg]")
        self.transform_label_yaw_client0.grid(row=row, column=5, padx=3, pady=3)

        row += 1

        self.transform_strigvar_pitch_client0 = tki.StringVar()
        self.transform_entry_pitch_client0 = tki.Entry(self.root, width=9, textvariable=self.transform_strigvar_pitch_client0)
        self.transform_entry_pitch_client0.grid(row=row, column=3, padx=3, pady=3)
        self.transform_entry_pitch_client0.delete(0, tki.END)
        self.transform_entry_pitch_client0.insert(0, "0.0")

        self.transform_strigvar_roll_client0 = tki.StringVar()
        self.transform_entry_roll_client0 = tki.Entry(self.root, width=9, textvariable=self.transform_strigvar_roll_client0)
        self.transform_entry_roll_client0.grid(row=row, column=4, padx=3, pady=3)
        self.transform_entry_roll_client0.delete(0, tki.END)
        self.transform_entry_roll_client0.insert(0, "0.0")

        self.transform_strigvar_yaw_client0 = tki.StringVar()
        self.transform_entry_yaw_client0 = tki.Entry(self.root, width=9, textvariable=self.transform_strigvar_yaw_client0)
        self.transform_entry_yaw_client0.grid(row=row, column=5, padx=3, pady=3)
        self.transform_entry_yaw_client0.delete(0, tki.END)
        self.transform_entry_yaw_client0.insert(0, "0.0")

        row += 1

        ##########################################

        self.transform_label_scale_client1 = tki.Label(self.root, text="trans_scale1")
        self.transform_label_scale_client1.grid(row=row, column=6)

        self.transform_label_x_client1 = tki.Label(self.root, text="trans_x1")
        self.transform_label_x_client1.grid(row=row, column=3, padx=3, pady=3)

        self.transform_label_y_client1 = tki.Label(self.root, text="trans_y1")
        self.transform_label_y_client1.grid(row=row, column=4, padx=3, pady=3)

        self.transform_label_z_client1 = tki.Label(self.root, text="trans_z1")
        self.transform_label_z_client1.grid(row=row, column=5, padx=3, pady=3)

        row += 1

        self.transform_strigvar_scale_client1 = tki.StringVar()
        self.transform_entry_scale_client1 = tki.Entry(self.root, width=9, textvariable=self.transform_strigvar_scale_client1)
        self.transform_entry_scale_client1.grid(row=row, column=6)
        self.transform_entry_scale_client1.delete(0, tki.END)
        self.transform_entry_scale_client1.insert(0, "0.0")

        self.transform_strigvar_x_client1 = tki.StringVar()
        self.transform_entry_x_client1 = tki.Entry(self.root, width=9, textvariable=self.transform_strigvar_x_client1)
        self.transform_entry_x_client1.grid(row=row, column=3, padx=3, pady=3)
        self.transform_entry_x_client1.delete(0, tki.END)
        self.transform_entry_x_client1.insert(0, "0.0")

        self.transform_strigvar_y_client1 = tki.StringVar()
        self.transform_entry_y_client1 = tki.Entry(self.root, width=9, textvariable=self.transform_strigvar_y_client1)
        self.transform_entry_y_client1.grid(row=row, column=4, padx=3, pady=3)
        self.transform_entry_y_client1.delete(0, tki.END)
        self.transform_entry_y_client1.insert(0, "0.0")

        self.transform_strigvar_z_client1 = tki.StringVar()
        self.transform_entry_z_client1 = tki.Entry(self.root, width=9, textvariable=self.transform_strigvar_z_client1)
        self.transform_entry_z_client1.grid(row=row, column=5, padx=3, pady=3)
        self.transform_entry_z_client1.delete(0, tki.END)
        self.transform_entry_z_client1.insert(0, "0.0")

        row += 1

        ##########################################

        self.transform_label_pitch_client1 = tki.Label(self.root, text="trans_Pitch1[Deg]")
        self.transform_label_pitch_client1.grid(row=row, column=3, padx=3, pady=3)

        self.transform_label_roll_client1 = tki.Label(self.root, text="trans_Roll1[Deg]")
        self.transform_label_roll_client1.grid(row=row, column=4, padx=3, pady=3)

        self.transform_label_yaw_client1 = tki.Label(self.root, text="trans_Yaw1[Deg]")
        self.transform_label_yaw_client1.grid(row=row, column=5, padx=3, pady=3)

        row += 1

        self.transform_strigvar_pitch_client1 = tki.StringVar()
        self.transform_entry_pitch_client1 = tki.Entry(self.root, width=9, textvariable=self.transform_strigvar_pitch_client1)
        self.transform_entry_pitch_client1.grid(row=row, column=3, padx=3, pady=3)
        self.transform_entry_pitch_client1.delete(0, tki.END)
        self.transform_entry_pitch_client1.insert(0, "0.0")

        self.transform_strigvar_roll_client1 = tki.StringVar()
        self.transform_entry_roll_client1 = tki.Entry(self.root, width=9, textvariable=self.transform_strigvar_roll_client1)
        self.transform_entry_roll_client1.grid(row=row, column=4, padx=3, pady=3)
        self.transform_entry_roll_client1.delete(0, tki.END)
        self.transform_entry_roll_client1.insert(0, "0.0")

        self.transform_strigvar_yaw_client1 = tki.StringVar()
        self.transform_entry_yaw_client1 = tki.Entry(self.root, width=9, textvariable=self.transform_strigvar_yaw_client1)
        self.transform_entry_yaw_client1.grid(row=row, column=5, padx=3, pady=3)
        self.transform_entry_yaw_client1.delete(0, tki.END)
        self.transform_entry_yaw_client1.insert(0, "0.0")

        row += 1

        self.panel = tki.Label(self.root)
        self.panel.grid(row=1, column=3)

        self.orientation_deg = Point()

        self.during_plot = False

        self.list_of_pure_lines_0 = []

        self.list_of_pure_lines_1 = []

        # ROS subscriptions

        rospy.Subscriber(self.cloud0_topic_name, PointCloud2, self.point_cloud0_callback)
        rospy.Subscriber(self.cloud1_topic_name, PointCloud2, self.point_cloud1_callback)
        # rospy.Subscriber(self.trigger_topic_name, Empty, self.save_cloud_callback)
        rospy.Subscriber(self.pose0_topic_name, PoseStamped, self.pose0_callback)
        rospy.Subscriber(self.pose1_topic_name, PoseStamped, self.pose1_callback)
        # rospy.Subscriber(self.camera0_topic_name, Image, self.img0_callback)
        # rospy.Subscriber(self.camera1_topic_name, Image, self.img1_callback)

        rospy.Subscriber('/ccmslam/TransOutServer0/', TransformStamped, self.transform0_callback)
        rospy.Subscriber('/ccmslam/TransOutServer1/', TransformStamped, self.transform1_callback)

        # rospy.Subscriber(self.cloud_server_topic_name, PointCloud2, self.point_cloud_server1_callback)

        # self.updateImageGui.start()


        # Spin until interrupted
        # rospy.spin()

    def _getGUIImage(self):
        """
        Main operation to read frames from h264decoder and draw skeleton on 
        frames if the pose mode is opened
        """  
        # read the system of your computer

        image = ImagePIL.fromarray(self.cv_image)

        size = round(image.size[0]/2), round(image.size[1]/2)

        image.thumbnail(size, ImagePIL.ANTIALIAS)
        image = ImageTkPIL.PhotoImage(image)
        # self.panel = tki.Label(image=image)
        self.panel.config(image=image)
        self.panel.image = image


    def move_right_callback(self):
        self.z_max += self.scale
        self.z_min += self.scale
        self.plot_to_gui()

    def move_left_callback(self):
        self.z_max -= self.scale
        self.z_min -= self.scale
        self.plot_to_gui()

    def move_down_callback(self):
        self.x_max -= self.scale
        self.x_min -= self.scale
        self.plot_to_gui()

    def move_up_callback(self):
        self.x_max += self.scale
        self.x_min += self.scale
        self.plot_to_gui()

    def zoom_in_callback(self):
        self.x_min += self.scale
        self.x_max -= self.scale
        self.y_min += self.scale
        self.y_max -= self.scale
        self.z_min += self.scale
        self.z_max -= self.scale
        # self.scale = max(1, self.scale/1.1)
        self.scale = (self.z_max - self.z_min)/5
        self.scale_strigvar.set(str(self.scale))
        self.plot_to_gui()


    def zoom_out_callback(self):
        self.x_min -= self.scale
        self.x_max += self.scale
        self.y_min -= self.scale
        self.y_max += self.scale
        self.z_min -= self.scale
        self.z_max += self.scale
        self.scale = (self.z_max - self.z_min)/5
        self.scale_strigvar.set(str(self.scale))
        self.plot_to_gui()

    def img_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.received_image = True
            thread_tmp = threading.Thread(target=self._getGUIImage)
            thread_tmp.start()
        except CvBridgeError as e:
            # print(e)
            print(traceback.format_exc())




    def onClose(self, *args):
        """
        set the stop event, cleanup the camera, and allow the rest of
        
        the quit process to continue
        """
        rospy.loginfo('Closing Cloud Map')
        self.root.quit()
        self.root.destroy()
        # rospy.signal_shutdown('Exited UI')


    def plot_to_gui(self):

        self.during_plot = True
        # x=np.array ([1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        # v= np.array ([16,16.31925,17.6394,16.003,17.2861,17.3131,19.1259,18.9694,22.0003,22.81226])
        # p= np.array ([16.23697,     17.31653,     17.22094,     17.68631,     17.73641 ,    18.6368,
        #     19.32125,     19.31756 ,    21.20247  ,   22.41444   ,  22.11718  ,   22.12453])

        # xxx = [print(element) for element in self.list_of_pure_lines]

        fig = Figure(figsize=(5,5))
        a = fig.add_subplot(111)

        try:

            if len(self.list_of_pure_lines_0) > 0:
                x_0 = np.asarray([(float(element[0])) for element in self.list_of_pure_lines_0])
                y_0 = np.asarray([(float(element[1])) for element in self.list_of_pure_lines_0])
                z_0 = np.asarray([(float(element[2])) for element in self.list_of_pure_lines_0])


                a.scatter(y_0, x_0, color='black', s=0.2)

                a.arrow(-self.position_0.y, self.position_0.x, math.sin(self.deg_to_rad(-self.orientation_deg_0.z))/5, math.cos(self.deg_to_rad(-self.orientation_deg_0.z))/5, 
                    color='blue')


                # a.arrow(    -(self.position_0.y - self.position_transform_0.y), 
                #             (self.position_0.x - self.position_transform_0.x) , 
                #             math.sin(self.deg_to_rad(-(self.orientation_deg_0.z - self.orientation_transform_deg_0.z)))/4, 
                #             math.cos(self.deg_to_rad(-(self.orientation_deg_0.z - self.orientation_transform_deg_0.z)))/4, 
                #             color='blue')

                # a.arrow(    -self.position_transformed_0.y, 
                #             self.position_transformed_0.x, 
                #             math.sin(self.deg_to_rad(-self.orientation_deg_transformed_0.z))/4, 
                #             math.cos(self.deg_to_rad(-self.orientation_deg_transformed_0.z))/4, 
                #             color='blue')


        except Exception as e:
            # print(e)
            print(traceback.format_exc())

        try:

            if len(self.list_of_pure_lines_1) > 0:
                x_1 = np.asarray([(float(element[0]) ) for element in self.list_of_pure_lines_1])
                y_1 = np.asarray([(float(element[1])) for element in self.list_of_pure_lines_1])
                z_1 = np.asarray([(float(element[2]) ) for element in self.list_of_pure_lines_1])


                a.scatter(y_1, x_1, color='red', s=0.2)

                # a.arrow(    -(self.position_1.y - self.position_transform_1.y), 
                #             (self.position_1.x - self.position_transform_1.x) , 
                #             math.sin(self.deg_to_rad(-(self.orientation_deg_1.z - self.orientation_transform_deg_1.z)))/4, 
                #             math.cos(self.deg_to_rad(-(self.orientation_deg_1.z - self.orientation_transform_deg_1.z)))/4, 
                #             color='m')

                # a.arrow(    -self.position_transformed_1.y, 
                #             self.position_transformed_1.x, 
                #             math.sin(self.deg_to_rad(-self.orientation_deg_transformed_1.z))/4, 
                #             math.cos(self.deg_to_rad(-self.orientation_deg_transformed_1.z))/4, 
                #             color='m')

                a.arrow(-self.position_1.y, self.position_1.x, math.sin(self.deg_to_rad(-self.orientation_deg_1.z))/5, math.cos(self.deg_to_rad(-self.orientation_deg_1.z))/5, 
                    color='m')

        except Exception as e:
            # print(e)
            print(traceback.format_exc())

        a.axis(xmin=self.y_min, xmax=self.y_max)
        a.axis(ymin=self.x_min, ymax=self.x_max)
        # fig.xlim(self.y_min, self.y_min)
        # fig.ylim(self.z_min, self.z_min)
        # a.plot(p, range(2 +max(x)),color='blue')
        # a.invert_yaxis()

        a.set_title ("X-Y", fontsize=16)
        a.set_ylabel("X", fontsize=14)
        a.set_xlabel("Y", fontsize=14)



        # try:
        #     if len(self.list_of_pure_lines2) > 0:

        #         x = np.asarray([float(element[0]) for element in self.list_of_pure_lines2])
        #         y = np.asarray([float(element[1]) for element in self.list_of_pure_lines2])
        #         z = np.asarray([float(element[2]) for element in self.list_of_pure_lines2])

        #     # fig = Figure(figsize=(5,5))
        #     # a = fig.add_subplot(111)
        #         a.scatter(x, z, color='green', s=0.2)
        # except Exception as e:
        #     print(e)

        canvas = FigureCanvasTkAgg(fig, master=self.root)
        canvas.get_tk_widget().grid(row=1, column=1)
        canvas.draw()

        self.during_plot = False


    def pose0_callback(self, pose_msg):
        self.pose_0 = deepcopy(pose_msg)
        self.orientation_deg_0, self.position_0, self.orientation_quat_0 = self.pose_callback(pose_msg)

        self.command_strigvar_x_client0.set(str(self.position_0.x))
        self.command_strigvar_y_client0.set(str(self.position_0.y))
        self.command_strigvar_z_client0.set(str(self.position_0.z))
        
        self.command_strigvar_pitch_client0.set(str(self.orientation_deg_0.x))
        self.command_strigvar_roll_client0.set(str(self.orientation_deg_0.y))
        self.command_strigvar_yaw_client0.set(str(self.orientation_deg_0.z))



        # print("orientation_deg_0 = {}".format(self.orientation_deg_0))
        # self.plot_to_gui()

    def pose1_callback(self, pose_msg):
        self.pose_1 = deepcopy(pose_msg)
        self.orientation_deg_1, self.position_1, self.orientation_quat_1 = self.pose_callback(pose_msg)

        self.command_strigvar_x_client1.set(str(self.position_1.x))
        self.command_strigvar_y_client1.set(str(self.position_1.y))
        self.command_strigvar_z_client1.set(str(self.position_1.z))
        
        self.command_strigvar_pitch_client1.set(str(self.orientation_deg_1.x))
        self.command_strigvar_roll_client1.set(str(self.orientation_deg_1.y))
        self.command_strigvar_yaw_client1.set(str(self.orientation_deg_1.z))

        # print("orientation_deg_1 = {}".format(self.orientation_deg_1))
        # self.plot_to_gui()

    def pose_callback(self, pose_msg):
        if not type(pose_msg) == PoseStamped:
            raise ValueError("Did not receive PoseStamped. received {} instead".format(type(pose_msg)))


        # pose_msg is PoseStamped
        if not type(pose_msg.pose) == Pose:
            raise ValueError("Did not receive Pose. received {} instead".format(type(pose_msg.pose)))

        if not type(pose_msg.pose.orientation) == Quaternion:
            q = Quaternion(pose_msg.pose.orientation[0], pose_msg.pose.orientation[1], pose_msg.pose.orientation[2], pose_msg.pose.orientation[3])
            raise ValueError("Did not receive Quaternion. received {} instead".format(type(pose_msg.pose.orientation)))
        else:
            q = pose_msg.pose.orientation

        # print(type(pose_msg.pose.orientation))
        # print(pose_msg.pose.orientation)

        position = pose_msg.pose.position
        orientation_deg = self.quatenrion_point_to_euler_degree(q)
        orientation_deg.x = self.clip_angle(orientation_deg.x)
        orientation_deg.y = self.clip_angle(orientation_deg.y)
        orientation_deg.z = self.clip_angle(orientation_deg.z)
        orientation_quat = deepcopy(pose_msg.pose.orientation)
        # self.orientation_rad = self.euler_point_deg_to_rad(self.orientation_deg)
        # self.command_strigvar_pitch.set('%.4f'%(self.orientation_deg.x))
        # self.command_strigvar_roll.set('%.4f'%(self.orientation_deg.y))
        # self.command_strigvar_yaw.set('%.4f'%(self.orientation_deg.z))
        return orientation_deg, position, orientation_quat

    def transform_callback(self, msg):
        if not type(msg) == TransformStamped:
            raise ValueError("Did not receive TransformStamped. received {} instead".format(type(msg)))

        scale = float(msg.child_frame_id)
        position = msg.transform.translation

        # this suppose to be roll, so new_pitch = old_roll
        # this suppose to be pitch, so new_yaw = old_pitch
        # this suppose to be yaw, so new_roll = -old_yaw



        orientation_deg = self.quatenrion_point_to_euler_degree(msg.transform.rotation)
        orientation_deg.x = self.clip_angle(orientation_deg.x)
        orientation_deg.y = self.clip_angle(orientation_deg.y)
        orientation_deg.z = self.clip_angle(orientation_deg.z)

        pitch = orientation_deg.y
        yaw = orientation_deg.x
        roll = -orientation_deg.z

        q_vect = self.orientation_to_quaternion(pitch, roll, yaw)

        if not type(q_vect) == Quaternion:
            raise ValueError("Did not receive Quaternion. received {} instead".format(type(q_vect)))

        msg.transform.rotation = q_vect

        orientation_deg = self.quatenrion_point_to_euler_degree(msg.transform.rotation)
        orientation_deg.x = self.clip_angle(orientation_deg.x)
        orientation_deg.y = self.clip_angle(orientation_deg.y)
        orientation_deg.z = self.clip_angle(orientation_deg.z)

        orientation_quat = msg.transform.rotation
        # self.orientation_rad = self.euler_point_deg_to_rad(self.orientation_deg)
        # self.command_strigvar_pitch.set('%.4f'%(self.orientation_deg.x))
        # self.command_strigvar_roll.set('%.4f'%(self.orientation_deg.y))
        # self.command_strigvar_yaw.set('%.4f'%(self.orientation_deg.z))
        return orientation_deg, position, scale, orientation_quat

    def transform0_callback(self, msg):
        self.transform0 = msg
        self.orientation_transform_deg_0, self.position_transform_0, self.scale_transform_0, self.orientation_transform_quat_0 = self.transform_callback(msg)

        # self.transform_strigvar_scale_client0.set(str(self.scale_transform_0))
        # self.transform_strigvar_x_client0.set(str(self.position_transform_0.x))
        # self.transform_strigvar_y_client0.set(str(self.position_transform_0.y))
        # self.transform_strigvar_z_client0.set(str(self.position_transform_0.z))
        
        # self.transform_strigvar_pitch_client0.set(str(self.orientation_transform_deg_0.x))
        # self.transform_strigvar_roll_client0.set(str(self.orientation_transform_deg_0.y))
        # self.transform_strigvar_yaw_client0.set(str(self.orientation_transform_deg_0.z))

        # try:
        self.transform_pose0()
        # except:
            # pass


        self.transform_strigvar_scale_client0.set(str(self.scale_transform_0))
        self.transform_strigvar_x_client0.set(str(self.position_transformed_0.x))
        self.transform_strigvar_y_client0.set(str(self.position_transformed_0.y))
        self.transform_strigvar_z_client0.set(str(self.position_transformed_0.z))
        
        self.transform_strigvar_pitch_client0.set(str(self.orientation_transform_deg_0.x))
        self.transform_strigvar_roll_client0.set(str(self.orientation_transform_deg_0.y))
        self.transform_strigvar_yaw_client0.set(str(self.orientation_transform_deg_0.z))



        # print("orientation_transform_deg_0 = {}".format(self.orientation_transform_deg_0))

    def transform1_callback(self, msg):
        self.transform1 = msg
        self.orientation_transform_deg_1, self.position_transform_1, self.scale_transform_1, self.orientation_transform_quat_1 = self.transform_callback(msg)

        # self.transform_strigvar_scale_client1.set(str(self.scale_transform_1))
        # self.transform_strigvar_x_client1.set(str(self.position_transform_1.x))
        # self.transform_strigvar_y_client1.set(str(self.position_transform_1.y))
        # self.transform_strigvar_z_client1.set(str(self.position_transform_1.z))

        # self.transform_strigvar_pitch_client1.set(str(self.orientation_transform_deg_1.x))
        # self.transform_strigvar_roll_client1.set(str(self.orientation_transform_deg_1.y))
        # self.transform_strigvar_yaw_client1.set(str(self.orientation_transform_deg_1.z))

        self.transform_pose1()

        self.transform_strigvar_scale_client1.set(str(self.scale_transform_1))
        self.transform_strigvar_x_client1.set(str(self.position_transformed_1.x))
        self.transform_strigvar_y_client1.set(str(self.position_transformed_1.y))
        self.transform_strigvar_z_client1.set(str(self.position_transformed_1.z))

        self.transform_strigvar_pitch_client1.set(str(self.orientation_transform_deg_1.x))
        self.transform_strigvar_roll_client1.set(str(self.orientation_transform_deg_1.y))
        self.transform_strigvar_yaw_client1.set(str(self.orientation_transform_deg_1.z))

        # print("orientation_transform_deg_1 = {}".format(self.orientation_transform_deg_1))

    # rotate vector v1 by quaternion q1 
    def qv_mult(self, q1, v1):
        # v1 = tf.transformations.unit_vector(v1)
        q2 = list(v1)
        q2.append(0.0)
        return tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(q1, q2), tf.transformations.quaternion_conjugate(q1))[:3]

    def quat_to_vect(self, q):
        return [q.x, q.y, q.z, q.w]

    def point_to_vect(self, v):
        return [v.x, v.y, v.z]


    def transform_pose0(self):
        # self.transform0 - TransformStamped so TransformStamped.transform.translation / rotation
        # self.pose_0 - PoseStamped so PoseStamped.pose.position / orientation
        # self.pose_0_transformed - PoseStamped so PoseStamped.pose.position / orientation
        # self.quat_to_vect(v) - v is Quaternion
        # self.qv_mult(q, v) - q is [1,2,3,1] and v is [1,2,3]
        # self.pose_0_transformed = PoseStamped()

        quat_multiplied = quaternion_multiply(self.quat_to_vect(self.transform0.transform.rotation), self.quat_to_vect(self.pose_0.pose.orientation))
        self.pose_0_transformed.pose.orientation = Quaternion(quat_multiplied[0], quat_multiplied[1], quat_multiplied[2], quat_multiplied[3])

        rot = self.transform0.transform.rotation
        q = [rot.x, rot.y, rot.z, rot.w]
        pos = self.pose_0.pose.position
        v = [pos.x, pos.y, pos.z]

        new_v = self.qv_mult(q, v)
        self.pose_0_transformed.pose.position.x = new_v[0]*self.scale_transform_0 + self.transform0.transform.translation.x
        self.pose_0_transformed.pose.position.y = new_v[1]*self.scale_transform_0 + self.transform0.transform.translation.y
        self.pose_0_transformed.pose.position.z = new_v[2]*self.scale_transform_0 + self.transform0.transform.translation.z


        if not type(self.pose_0_transformed) == PoseStamped:
            raise ValueError("Did not receive PoseStamped. received {} instead".format(type(self.pose_0_transformed)))

        if not type(self.pose_0_transformed.pose.orientation) == Quaternion:
            raise ValueError("Did not receive Quaternion. received {} instead".format(type(self.elf.pose_0_transformed.pose.orientation)))


        # self.pose_0_transformed.position = 

        # self.pose_0_transformed = tf2_geometry_msgs.do_transform_pose(self.pose_0, self.transform0)
        self.orientation_deg_transformed_0, self.position_transformed_0, self.orientation_quat_transformed_0 = self.pose_callback(self.pose_0_transformed)


    def transform_pose1(self):
        self.pose_1_transformed = PoseStamped()
        quat_multiplied = quaternion_multiply(self.quat_to_vect(self.transform1.transform.rotation), self.quat_to_vect(self.pose_1.pose.orientation))
        self.pose_1_transformed.pose.orientation = Quaternion(quat_multiplied[0], quat_multiplied[1], quat_multiplied[2], quat_multiplied[3])

        rot = self.transform1.transform.rotation
        q = [rot.x, rot.y, rot.z, rot.w]
        pos = self.pose_1.pose.position
        v = [pos.x, pos.y, pos.z]

        new_v = self.qv_mult(q, v)
        self.pose_1_transformed.pose.position.x = new_v[0]*self.scale_transform_1 + self.transform1.transform.translation.x
        self.pose_1_transformed.pose.position.y = new_v[1]*self.scale_transform_1 + self.transform1.transform.translation.y
        self.pose_1_transformed.pose.position.z = new_v[2]*self.scale_transform_1 + self.transform1.transform.translation.z

        self.orientation_deg_transformed_1, self.position_transformed_1, self.orientation_quat_transformed_1 = self.pose_callback(self.pose_1_transformed)




    def quatenrion_point_to_euler_degree(self, slam_quaternion):
        if not type(slam_quaternion) == Quaternion:
            raise ValueError("Did not receive Quaternion. received {} instead".format(type(slam_quaternion)))
        rad = self.quatenrion_point_to_euler(slam_quaternion)
        return Point(self.rad_to_deg(rad.x), self.rad_to_deg(rad.y), self.rad_to_deg(rad.z))

    def quatenrion_point_to_euler(self, orientation_point):
        print

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

    def save_cloud_callback(self, msg):
        self.once = True

    def float_from_4_bytes(self, bytes_list):
        # print("bytes_list={}".format(bytes_list))
        return struct.unpack('f', bytes_list)[0]

    def point_cloud0_callback(self, point_cloud):
        self.list_of_pure_lines_0 = self.point_cloud_callback(point_cloud)
        if not self.during_plot:
            self.plot_to_gui()
        # self.plot_to_gui()


    def point_cloud1_callback(self, point_cloud):
        self.list_of_pure_lines_1 = self.point_cloud_callback(point_cloud)
        if not self.during_plot:
            self.plot_to_gui()
        # self.plot_to_gui()


    def point_cloud_callback(self, point_cloud):
        fields_str = str(point_cloud.fields)
        # file.write(fields_str+'\n')

        # point_cloud.data is list of uint8[].
        # every 4 elements, is one float32
        # convert cloud.data to list where each element in a 4 byte list
        list_of_4_bytes = [point_cloud.data[x:x+4] for x in range(0, len(point_cloud.data), 4)]
        # convert every 4 bytes to a float32
        list_of_floats = [self.float_from_4_bytes(element) for element in list_of_4_bytes]
        # convert every float to string, and make sure there are only 4 zeros after the point (anti float32....)
        list_of_strings = ['%.4f' % element for element in list_of_floats]
        # print(list_of_strings)

        # check if we are in orbslam mode or ccm_slam mode - there is difference in the structure of point_cloud.fields
        ccm_slam_mode = 'rgb' in fields_str

        # in ccmslam: [X, Y, Z, RGB]
        # in orbslam: [X, Y, Z]
        # so in ccmslam drop RGB
        list_of_lines = [list_of_strings[x:x+3] for x in range(0, len(list_of_strings), 3+ccm_slam_mode)]

        if ccm_slam_mode:
            color_id = [list_of_strings[x+3] for x in range(0, len(list_of_strings), 4)]
            # print(color_id)
        else:
            color_id = []

        # remove lines that are all zeros
        list_of_pure_lines = [element for element in list_of_lines if not element == ['0.0000', '0.0000', '0.0000']]
        return list_of_pure_lines

        # self.plot_to_gui()

        # if self.once:
        #     # write the points to file
        #     rospy.loginfo("Saving Point Cloud from topic {} to file {}".format(self.cloud_topic_name, self.file_path))
        #     with open(self.file_path, 'w') as file:
        #         [file.write(','.join(element)+'\n') for element in self.list_of_pure_lines]


    def point_cloud_server1_callback(self, point_cloud):
        fields_str = str(point_cloud.fields)
        # file.write(fields_str+'\n')

        # point_cloud.data is list of uint8[].
        # every 4 elements, is one float32
        # convert cloud.data to list where each element in a 4 byte list
        list_of_4_bytes = [point_cloud.data[x:x+4] for x in range(0, len(point_cloud.data), 4)]
        # convert every 4 bytes to a float32
        list_of_floats = [self.float_from_4_bytes(element) for element in list_of_4_bytes]
        # convert every float to string, and make sure there are only 4 zeros after the point (anti float32....)
        list_of_strings = ['%.4f' % element for element in list_of_floats]
        # print(list_of_strings)

        # check if we are in orbslam mode or ccm_slam mode - there is difference in the structure of point_cloud.fields
        ccm_slam_mode = 'rgb' in fields_str

        # in ccmslam: [X, Y, Z, RGB]
        # in orbslam: [X, Y, Z]
        # so in ccmslam drop RGB
        list_of_lines = [list_of_strings[x:x+3] for x in range(0, len(list_of_strings), 3+ccm_slam_mode)]

        # remove lines that are all zeros
        self.list_of_pure_lines2 = [element for element in list_of_lines if not element == ['0.0000', '0.0000', '0.0000']]

        # self.plot_to_gui()



            



if __name__ == '__main__':
    root = tki.Tk()
    my_gui = CloudMapSaverForServer(root)
    root.mainloop()