#!/usr/bin/env python

from __future__ import print_function

import rospy
import threading

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Empty, Bool, Int32, Float32, String
from geometry_msgs.msg import PoseStamped
import time
import signal
import subprocess


class ImageMuxer(object):

    def __init__(self):
        # Initialize ROS
        rospy.init_node('image_muxer', anonymous=False)
        rospy.sleep(4)

        try: 
            self.network_card_name1 = rospy.get_param('~NETWORK_CARD_NAME1')
        except KeyError:
            self.network_card_name1 = '-------'

        try: 
            self.network_card_name2 = rospy.get_param('~NETWORK_CARD_NAME2')
        except KeyError:
            self.network_card_name2 = '-------'

        self.yaml_base_path = rospy.get_param('~yaml_base_path')

        self.current_mux = 0

        self.ssid1 = ''
        self.ssid2 = ''

        self.camera_info1 = ''
        self.camera_info2 = ''

        self.ssid_camera_info_dict = {'TELLO-57A6A7': self.camera_info_57A6A7(), 'TELLO-B5D81A': self.camera_info_B5D81A(), 
                            'TELLO-B62241': self.camera_info_B62241(), 'TELLO-58B4D9': self.camera_info_58B4D9() }

        self.buffer0 = []
        self.buffer1 = []
        self.frame_counter = 0
        self.MAX_FRAMES = 20
        self.quit_flag = False
        self.other_len = 0
        self.sleep_time = 1/65

        signal.signal(signal.SIGINT, self.set_quit_flag)
        signal.signal(signal.SIGTERM, self.set_quit_flag)

        # ROS subscriptions
        rospy.Subscriber("tello0/camera/image_raw", Image, self.get_image0)
        rospy.Subscriber("tello1/camera/image_raw", Image, self.get_image1)
        rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, self.slam_callback)

        rospy.Subscriber('tello_mux', Int32, self.change_mux)
        # rospy.Subscriber("flight_data", FlightData, self.flight_data_callback)

        self._image_pub = rospy.Publisher('camera/image_raw', Image, queue_size=10)
        self._camera_info_pub = rospy.Publisher('camera/camera_info', CameraInfo, queue_size=1)
        self.tello0_slam_pub = rospy.Publisher('/tello0/pose', PoseStamped, queue_size=10)
        self.tello1_slam_pub = rospy.Publisher('/tello1/pose', PoseStamped, queue_size=10)
        self.yaml_pub = rospy.Publisher('/orb_slam2_mono/camera_config', String, queue_size=1)

        # threading.Thread(target=self.publish_thread).start()

        threading.Thread(target=self.check_ssid_thread).start()

        # Spin until interrupted
        rospy.spin()

        rospy.loginfo("Quiting image_muxer")

    def set_quit_flag(self, *args):
        self.quit_flag = True


    def get_image0(self, data):
        if self.current_mux == 0:
            if not self.camera_info1 == '':
                self.camera_info1.header = data.header
                self._image_pub.publish(data)
                # self._camera_info_pub.publish(self.camera_info1)
            # self.buffer.append(data)
        # print('r0')

    def get_image1(self, data):
        if self.current_mux == 1:
            if not self.camera_info2 == '':
                self.camera_info2.header = data.header
                self._image_pub.publish(data)
                # self._camera_info_pub.publish(self.camera_info2)
            # self.buffer.append(data)

        # self.buffer1.append(data)
        # print('r1')

    def slam_callback(self, data):
        if self.current_mux == 0:
            self.tello0_slam_pub.publish(data)
        else:
            self.tello1_slam_pub.publish(data)

    def change_mux(self, data):
        self.current_mux = int(data.data)
        print('Changed  mux to {}.'.format(self.current_mux))

    def check_ssid_thread(self):
        while True:
            if self.quit_flag:
                rospy.loginfo('Quiting Image Muxer')
                return
            try:
                out = subprocess.check_output('iwgetid')
            except subprocess.CalledProcessError:
                time.sleep(1)
                continue
            for line in out.split('\n'):
                # rospy.loginfo(line)
                if ':' in line:
                    network_card_part, ssid_part = line.split(':') # 'wlp5s0    ESSID:"SSID23231"\n'
                    ssid = ssid_part.replace('"', '')
                    if '_' in ssid:
                        ssid = ssid.split('_')[0]
                    # self.ssid1 = ''
                    # self.ssid2 = ''
                    if self.network_card_name1 in network_card_part:
                        if not self.ssid1 == ssid:
                            self.ssid1 = ssid
                            self.camera_info1 = self.ssid_camera_info_dict.get(self.ssid1, '')
                            if 'TELLO' in self.ssid1:
                                self.tello_id1 = self.ssid1.replace('TELLO-', '')
                                self.yaml1 = self.yaml_base_path + 'Tello_' + self.tello_id1 + '.yaml'
                                self.yaml_pub.publish(self.yaml1 )
                                rospy.loginfo('publishing {}'.format(self.yaml1))
                            rospy.loginfo('{} connected to {}'.format(self.network_card_name1, self.ssid1))
                    else:
                        self.ssid1 = ''
                    if self.network_card_name2 in network_card_part:
                        if not self.ssid2 == ssid:
                            self.ssid2 = ssid
                            self.camera_info2 = self.ssid_camera_info_dict.get(self.ssid2, '')
                            if 'TELLO' in self.ssid2:
                                self.tello_id2 = self.ssid1.replace('TELLO-', '')
                                self.yaml2 = self.yaml_base_path + 'Tello_' + self.tello_id2 + '.yaml'
                                self.yaml_pub.publish(self.yaml2 )
                                rospy.loginfo('publishing {}'.format(self.yaml2))
                            # self.camera_info2 = self.ssid_camera_info_dict.get(self.ssid2, '')
                            rospy.loginfo('{} connected to {}'.format(self.network_card_name2, self.ssid2))
                    else:
                        self.ssid2 = ''

            time.sleep(1)
            

    def camera_info_57A6A7(self):
        camera_info = CameraInfo()
        camera_info.height = 720
        camera_info.width = 960
        camera_info.distortion_model = 'plumb_bob'
        camera_info.D = [-0.034749, 0.071514, 0.000363, 0.003131, 0]
        #     [fx  0 cx]
        # K = [ 0 fy cy]
        #     [ 0  0  1]
        camera_info.K = [ 924.873180, 0, 486.997346, 0, 923.504522, 364.308527, 0, 0, 1 ]

        # Rectification matrix (stereo cameras only)
        # A rotation matrix aligning the camera coordinate system to the ideal
        # stereo image plane so that epipolar lines in both stereo images are
        # parallel.
        camera_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1] # 3x3 row-major matrix
        # Projection/camera matrix
        #     [fx'  0  cx' Tx]
        # P = [ 0  fy' cy' Ty]
        #     [ 0   0   1   0]
        camera_info.P = [921.967102, 0  , 489.492281, 0, 0  , 921.018890, 364.508536, 0, 0  , 0  , 1.000000  , 0]
        # Binning refers here to any camera setting which combines rectangular
        #  neighborhoods of pixels into larger "super-pixels." It reduces the
        #  resolution of the output image to
        #  (width / binning_x) x (height / binning_y).
        # The default values binning_x = binning_y = 0 is considered the same
        #  as binning_x = binning_y = 1 (no subsampling).
        camera_info.binning_x   =   1
        camera_info.binning_y   =   1
        # camera_info.roi.do_rectify = True
        return camera_info

    def camera_info_B5D81A(self):
        camera_info = CameraInfo()
        camera_info.height = 720
        camera_info.width = 960
        camera_info.distortion_model = 'plumb_bob'
        camera_info.D = [-0.013707, 0.057347, 0.001189, -0.000620, 0.000000]
        #     [fx  0 cx]
        # K = [ 0 fy cy]
        #     [ 0  0  1]
        camera_info.K = [932.514995, 0.000000, 509.230011, 0.000000, 930.694048, 363.217342, 0.000000, 0.000000, 1.000000]

        # Rectification matrix (stereo cameras only)
        # A rotation matrix aligning the camera coordinate system to the ideal
        # stereo image plane so that epipolar lines in both stereo images are
        # parallel.
        camera_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1] # 3x3 row-major matrix
        # Projection/camera matrix
        #     [fx'  0  cx' Tx]
        # P = [ 0  fy' cy' Ty]
        #     [ 0   0   1   0]
        camera_info.P = [937.264587, 0.000000, 508.215333, 0.000000, 0.000000, 936.094482, 363.984413, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
        # Binning refers here to any camera setting which combines rectangular
        #  neighborhoods of pixels into larger "super-pixels." It reduces the
        #  resolution of the output image to
        #  (width / binning_x) x (height / binning_y).
        # The default values binning_x = binning_y = 0 is considered the same
        #  as binning_x = binning_y = 1 (no subsampling).
        camera_info.binning_x   =   1
        camera_info.binning_y   =   1
        # camera_info.roi.do_rectify = True
        return camera_info

    def camera_info_B62241(self):
        camera_info = CameraInfo()
        camera_info.height = 720
        camera_info.width = 960
        camera_info.distortion_model = 'plumb_bob'
        camera_info.D = [-0.026147, 0.071036, -0.001191, 0.000034, 0.000000]
        #     [fx  0 cx]
        # K = [ 0 fy cy]
        #     [ 0  0  1]
        camera_info.K = [936.038281, 0.000000, 489.072492, 0.000000, 934.410240, 355.865672, 0.000000, 0.000000, 1.000000]
        # camera_info.K = [0.0, 0.000000, 0.0, 0.000000, 0.0, 0.0, 0.000000, 0.000000, 0.000000]

        # Rectification matrix (stereo cameras only)
        # A rotation matrix aligning the camera coordinate system to the ideal
        # stereo image plane so that epipolar lines in both stereo images are
        # parallel.
        camera_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1] # 3x3 row-major matrix
        # Projection/camera matrix
        #     [fx'  0  cx' Tx]
        # P = [ 0  fy' cy' Ty]
        #     [ 0   0   1   0]
        camera_info.P = [937.920654, 0.000000, 488.955720, 0.000000, 0.000000, 935.798462, 355.122410, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
        # Binning refers here to any camera setting which combines rectangular
        #  neighborhoods of pixels into larger "super-pixels." It reduces the
        #  resolution of the output image to
        #  (width / binning_x) x (height / binning_y).
        # The default values binning_x = binning_y = 0 is considered the same
        #  as binning_x = binning_y = 1 (no subsampling).
        camera_info.binning_x   =   1
        camera_info.binning_y   =   1
        # camera_info.roi.do_rectify = True
        return camera_info

    def camera_info_58B4D9(self):
        camera_info = CameraInfo()
        camera_info.height = 720
        camera_info.width = 960
        camera_info.distortion_model = 'plumb_bob'
        camera_info.D = [-0.009413, 0.051197, 0.000404, 0.001424, 0.000000]
        #     [fx  0 cx]
        # K = [ 0 fy cy]
        #     [ 0  0  1]
        camera_info.K = [920.424491, 0.000000, 483.628078, 0.000000, 918.646459, 359.107873, 0.000000, 0.000000, 1.000000]
        # camera_info.K = [0.0, 0.000000, 0.0, 0.000000, 0.0, 0.0, 0.000000, 0.000000, 0.000000]

        # Rectification matrix (stereo cameras only)
        # A rotation matrix aligning the camera coordinate system to the ideal
        # stereo image plane so that epipolar lines in both stereo images are
        # parallel.
        camera_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1] # 3x3 row-major matrix
        # Projection/camera matrix
        #     [fx'  0  cx' Tx]
        # P = [ 0  fy' cy' Ty]
        #     [ 0   0   1   0]
        camera_info.P = [925.399780, 0.000000, 484.792705, 0.000000, 0.000000, 924.518188, 359.378385, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
        # Binning refers here to any camera setting which combines rectangular
        #  neighborhoods of pixels into larger "super-pixels." It reduces the
        #  resolution of the output image to
        #  (width / binning_x) x (height / binning_y).
        # The default values binning_x = binning_y = 0 is considered the same
        #  as binning_x = binning_y = 1 (no subsampling).
        camera_info.binning_x   =   1
        camera_info.binning_y   =   1
        # camera_info.roi.do_rectify = True
        return camera_info        

    # def publish_thread(self):
    #     while True:
    #         if self.quit_flag:
    #             break
    #         if self.current_mux == 0:
    #             if len(self.buffer0) > 0:
    #                 self._image_pub.publish(self.buffer0.pop(0))
    #                 self.frame_counter += 1
    #                 self.other_len = len(self.buffer1)
    #                 time.sleep(self.sleep_time)
    #                 # print('p0')

    #         else:
    #             if len(self.buffer1) > 0:
    #                 self._image_pub.publish(self.buffer1.pop(0))
    #                 self.frame_counter += 1
    #                 self.other_len = len(self.buffer0)
    #                 time.sleep(self.sleep_time)
    #                 # print('p1')

    #         if self.frame_counter >= self.MAX_FRAMES:
    #             if self.other_len > 0:
    #                 self.current_mux = 1 - self.current_mux
    #                 self.frame_counter = 0

            

        # print('exit from mux image publish thread.')


if __name__ == '__main__':
    driver = ImageMuxer()
