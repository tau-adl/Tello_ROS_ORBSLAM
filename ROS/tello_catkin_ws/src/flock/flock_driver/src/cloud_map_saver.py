#!/usr/bin/env python
from __future__ import print_function
import rospy

import math
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty 
from geometry_msgs.msg import PoseStamped, Point
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
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import traceback



class CloudMapSaver(object):

    def __init__(self, root):


        # Initialize ROS
        rospy.init_node('cloud_map_saver', anonymous=False)
        self.once = False

        try: 
            self.id                = rospy.get_param('~ID')
        except KeyError:
            self.id = ''

        signal.signal(signal.SIGINT, self.onClose)
        signal.signal(signal.SIGTERM, self.onClose)

        self.root = root

        self.bridge = CvBridge()

        self.file_path = rospy.get_param('~OUT_FILE_PATH')
        self.cloud_topic_name = rospy.get_param('~CLOUD_TOPIC_NAME')
        self.trigger_topic_name = rospy.get_param('~TRIGGER_TOPIC_NAME')
        self.pose_topic_name = rospy.get_param('~POSE_TOPIC_NAME')
        self.camera_topic_name = rospy.get_param('~CAMERA_TOPIC_NAME')
#        self.cloud_server_topic_name = rospy.get_param('~CLOUD_SERVER_TOPIC_NAME')

        self.received_image = False

        # self.updateImageGui = threading.Thread(target = self._getGUIImage)

        self.root.wm_title("TELLO Viewer"+str(self.id))
        self.root.wm_protocol("WM_DELETE_WINDOW", self.onClose)

        self.list_of_pure_lines = []
        self.list_of_pure_lines2 = []
        



        self.position = Point()
        self.orientation = Point()

        
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

        self.command_label_pitch = tki.Label(self.root, text="Pitch[Deg]")
        self.command_label_pitch.grid(row=3, column=0, padx=3, pady=3)

        self.command_label_roll = tki.Label(self.root, text="Roll[Deg]")
        self.command_label_roll.grid(row=3, column=1, padx=3, pady=3)

        self.command_label_yaw = tki.Label(self.root, text="Yaw[Deg]")
        self.command_label_yaw.grid(row=3, column=2, padx=3, pady=3)

        self.command_strigvar_pitch = tki.StringVar()
        self.command_entry_pitch = tki.Entry(self.root, width=9, textvariable=self.command_strigvar_pitch)
        self.command_entry_pitch.grid(row=4, column=0, padx=3, pady=3)
        self.command_entry_pitch.delete(0, tki.END)
        self.command_entry_pitch.insert(0, "0.0")

        self.command_strigvar_roll = tki.StringVar()
        self.command_entry_roll = tki.Entry(self.root, width=9, textvariable=self.command_strigvar_roll)
        self.command_entry_roll.grid(row=4, column=1, padx=3, pady=3)
        self.command_entry_roll.delete(0, tki.END)
        self.command_entry_roll.insert(0, "0.0")

        self.command_strigvar_yaw = tki.StringVar()
        self.command_entry_yaw = tki.Entry(self.root, width=9, textvariable=self.command_strigvar_yaw)
        self.command_entry_yaw.grid(row=4, column=2, padx=3, pady=3)
        self.command_entry_yaw.delete(0, tki.END)
        self.command_entry_yaw.insert(0, "0.0")

        self.panel = tki.Label(self.root)
        self.panel.grid(row=1, column=3)

        self.orientation_deg = Point()

        # ROS subscriptions

        rospy.Subscriber(self.cloud_topic_name, PointCloud2, self.point_cloud_callback)
        rospy.Subscriber(self.trigger_topic_name, Empty, self.save_cloud_callback)
        rospy.Subscriber(self.pose_topic_name, PoseStamped, self.pose_callback)
        rospy.Subscriber(self.camera_topic_name, Image, self.img_callback)

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
        # x=np.array ([1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        # v= np.array ([16,16.31925,17.6394,16.003,17.2861,17.3131,19.1259,18.9694,22.0003,22.81226])
        # p= np.array ([16.23697,     17.31653,     17.22094,     17.68631,     17.73641 ,    18.6368,
        #     19.32125,     19.31756 ,    21.20247  ,   22.41444   ,  22.11718  ,   22.12453])

        # xxx = [print(element) for element in self.list_of_pure_lines]

        fig = Figure(figsize=(5,5))
        a = fig.add_subplot(111)

        try:

            if len(self.list_of_pure_lines) > 0:
                x = np.asarray([float(element[0]) for element in self.list_of_pure_lines])
                y = np.asarray([-float(element[1]) for element in self.list_of_pure_lines])
                z = np.asarray([float(element[2]) for element in self.list_of_pure_lines])

            # self.x_min = min(x.min(), self.x_min)
            # self.x_max = max(x.max(), self.x_max)
            # self.y_min = min(y.min(), self.x_min)
            # self.y_max = max(y.max(), self.x_max)
            # self.z_min = min(z.min(), self.x_min)
            # self.z_max = max(z.max(), self.x_max)




                a.scatter(y, x, color='red', s=0.2)
            # a.scatter(self.position.y, self.position.x, color='blue')

            # x_arrow_final = self.position.x + math.cos(self.deg_to_rad(self.orientation_deg.z))*1
            # y_arrow_final = self.position.y + math.sin(self.deg_to_rad(self.orientation_deg.z))*1

            # x_arrow_initial = self.position.x - math.cos(self.deg_to_rad(self.orientation_deg.z))*1
            # y_arrow_initial = self.position.y - math.sin(self.deg_to_rad(self.orientation_deg.z))*1

            # a.plot([y_arrow_initial, y_arrow_final], [x_arrow_initial, x_arrow_final], color='blue')

            # a.arrow(self.position.y, self.position.x, math.sin(self.deg_to_rad(self.orientation_deg.z))/2, math.cos(self.deg_to_rad(self.orientation_deg.z))/2, 
            #     color='blue', head_width=0.01)

                # we want to look at the X-Y plane. so we want to replace the x and y.

                a.arrow(-self.position.y, self.position.x, math.sin(self.deg_to_rad(-self.orientation_deg.z))/5, math.cos(self.deg_to_rad(-self.orientation_deg.z))/5, 
                    color='blue')

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



        try:
            if len(self.list_of_pure_lines2) > 0:

                x = np.asarray([float(element[0]) for element in self.list_of_pure_lines2])
                y = np.asarray([float(element[1]) for element in self.list_of_pure_lines2])
                z = np.asarray([float(element[2]) for element in self.list_of_pure_lines2])

            # fig = Figure(figsize=(5,5))
            # a = fig.add_subplot(111)
                a.scatter(x, z, color='green', s=0.2)
        except Exception as e:
            print(e)

        # a.arrow(-self.position.y, self.position.x, math.sin(self.deg_to_rad(-self.orientation_deg.z))/5, math.cos(self.deg_to_rad(-self.orientation_deg.z))/5, 
        #     color='blue')

        # a.axis(xmin=self.z_min, xmax=self.z_max)
        # a.axis(ymin=self.x_min, ymax=self.x_max)
        # # fig.xlim(self.y_min, self.y_min)
        # # fig.ylim(self.z_min, self.z_min)
        # # a.plot(p, range(2 +max(x)),color='blue')
        # # a.invert_yaxis()

        # a.set_title ("X-Z", fontsize=16)
        # a.set_ylabel("Z", fontsize=14)
        # a.set_xlabel("X", fontsize=14)


        canvas = FigureCanvasTkAgg(fig, master=self.root)
        canvas.get_tk_widget().grid(row=1, column=1)
        canvas.draw()


    def pose_callback(self, pose_msg):
        self.position = pose_msg.pose.position
        self.orientation_deg = self.quatenrion_point_to_euler_degree(pose_msg.pose.orientation)
        self.orientation_deg.x = self.clip_angle(self.orientation_deg.x)
        self.orientation_deg.y = self.clip_angle(self.orientation_deg.y)
        self.orientation_deg.z = self.clip_angle(self.orientation_deg.z)
        self.orientation_rad = self.euler_point_deg_to_rad(self.orientation_deg)
        self.command_strigvar_pitch.set('%.4f'%(self.orientation_deg.x))
        self.command_strigvar_roll.set('%.4f'%(self.orientation_deg.y))
        self.command_strigvar_yaw.set('%.4f'%(self.orientation_deg.z))



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
        return struct.unpack('f', bytes_list)[0]

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
        self.list_of_pure_lines = [element for element in list_of_lines if not element == ['0.0000', '0.0000', '0.0000']]

        self.plot_to_gui()

        if self.once:
            # write the points to file
            rospy.loginfo("Saving Point Cloud from topic {} to file {}".format(self.cloud_topic_name, self.file_path))
            with open(self.file_path, 'w') as file:
                [file.write(','.join(element)+'\n') for element in self.list_of_pure_lines]


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
    my_gui = CloudMapSaver(root)
    root.mainloop()