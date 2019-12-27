#!/usr/bin/env python

import threading
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Empty
from flock_msgs.msg import Flip, FlightData
from std_msgs.msg import Int32, Bool, String, Float32
# import av
import cv2
import numpy
import tellopy
from cv_bridge import CvBridge
import numpy as np
import libh264decoder
import signal



class FlockDriver(object):

    def __init__(self):
        # Initialize ROS
        rospy.init_node('flock_driver_node', anonymous=False)

        rospy.loginfo('flock_driver is starting')
        signal.signal(signal.SIGINT, self.cleanup)
        signal.signal(signal.SIGTERM, self.cleanup)


        self.camera_info = CameraInfo()
        self.camera_info.height = 720
        self.camera_info.width = 960
        self.camera_info.distortion_model = 'plumb_bob'
        self.camera_info.D = [-0.034749, 0.071514, 0.000363, 0.003131, 0]
        #     [fx  0 cx]
        # K = [ 0 fy cy]
        #     [ 0  0  1]
        self.camera_info.K = [ 924.873180, 0, 486.997346, 0, 923.504522, 364.308527, 0, 0, 1 ]

        # Rectification matrix (stereo cameras only)
        # A rotation matrix aligning the camera coordinate system to the ideal
        # stereo image plane so that epipolar lines in both stereo images are
        # parallel.
        self.camera_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1] # 3x3 row-major matrix
        # Projection/camera matrix
        #     [fx'  0  cx' Tx]
        # P = [ 0  fy' cy' Ty]
        #     [ 0   0   1   0]
        self.camera_info.P = [921.967102, 0  , 489.492281, 0, 0  , 921.018890, 364.508536, 0, 0  , 0  , 1.000000  , 0]
        # Binning refers here to any camera setting which combines rectangular
        #  neighborhoods of pixels into larger "super-pixels." It reduces the
        #  resolution of the output image to
        #  (width / binning_x) x (height / binning_y).
        # The default values binning_x = binning_y = 0 is considered the same
        #  as binning_x = binning_y = 1 (no subsampling).
        self.camera_info.binning_x   =   1
        self.camera_info.binning_y   =   1
        # self.camera_info.roi.do_rectify = True

        try:
            self.network_interface = rospy.get_param('~network_interface')
        except:
            self.network_interface = ''
        try:
            self.id                = rospy.get_param('~ID')
        except:
            self.id = 0

        # try:
        #     self.tello_ip = rospy.get_param('~TELLO_IP')
        # except:
        self.tello_ip = '192.168.10.1'

        self.publish_prefix = "tello{}/".format(self.id)

        # ROS publishers
        self._flight_data_pub = rospy.Publisher(self.publish_prefix+'flight_data', FlightData, queue_size=10)
        self._image_pub = rospy.Publisher(self.publish_prefix+'camera/image_raw', Image, queue_size=10)
        self._wifi_strength_data_pub = rospy.Publisher(self.publish_prefix+'wifi_strength', Int32, queue_size=10)
        self._light_strength_data_pub = rospy.Publisher(self.publish_prefix+'light_strength', Int32, queue_size=10)
        self._camera_info_pub = rospy.Publisher(self.publish_prefix+'camera/camera_info', CameraInfo, queue_size=10)

        

        # ROS subscriptions
        rospy.Subscriber(self.publish_prefix+'cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber(self.publish_prefix+'takeoff', Empty, self.takeoff_callback)
        rospy.Subscriber(self.publish_prefix+'land', Empty, self.land_callback)
        rospy.Subscriber(self.publish_prefix+'flip', Flip, self.flip_callback)
        rospy.Subscriber(self.publish_prefix+'zoom_mode', Bool, self.zoom_callback)
        rospy.Subscriber(self.publish_prefix+'exposure_level', Int32, self.exposure_callback)
        rospy.Subscriber(self.publish_prefix+'video_encoder_rate', Int32, self.video_encoder_rate_callback)
        rospy.Subscriber(self.publish_prefix+'log_level', String, self.log_level_callback)
        rospy.Subscriber(self.publish_prefix+'move_up', Float32, self.move_up_callback)
        

        # ROS OpenCV bridge
        self._cv_bridge = CvBridge()

        self.decoder = libh264decoder.H264Decoder()

        # Connect to the drone
        # self._drone = tellopy.TelloSDK(network_interface=self.network_interface, 
            # tello_ip='', local_port=7777, local_video_port=22222, tello_port=7777)
        self._drone = tellopy.TelloSDK(network_interface=self.network_interface)
        # self._drone = tellopy.Tello(network_interface=self.network_interface)
        self._drone.connect()
        # try:
        self._drone.wait_for_connection(60.0)

        self._drone.record_log_data()
        # except error.TelloError:
            # self.cleanup()
        rospy.loginfo('connected to drone')
        self._drone.start_video()
        # self._drone.set_loglevel('LOG_ERROR')

        # print('debug')
        # Listen to flight data messages
        self._drone.subscribe(self._drone.EVENT_FLIGHT_DATA, self.flight_data_callback)

        self._drone.subscribe(self._drone.EVENT_WIFI, self.wifi_strength_callback)
        self._drone.subscribe(self._drone.EVENT_LIGHT, self.light_strength_callback)

        # Start video thread
        # self._stop_request = threading.Event()

        # video_thread = threading.Thread(target=self.video_worker)
        # video_thread.start()
        self.packet_data = ""
        self._drone.subscribe(self._drone.EVENT_VIDEO_FRAME, self.videoFrameHandler)

        # rospy.on_shutdown(self.cleanup)


        # Spin until interrupted
        rospy.spin()

    def cleanup(self, sig=signal.SIGINT, frame=''):

        rospy.loginfo("Quiting flock_driver")

        if self._drone:
            # Force a landing
            self._drone.land()

            # Stop the video thread
            # self._stop_request.set()
            # video_thread.join(timeout=2)

            # Shut down the drone
            self._drone.quit()
            self._drone = None

    def wifi_strength_callback(self, event, sender, data, **args):
        # Publish what we have
        publish_type = Int32()
        publish_type.data = data[0]
        self._wifi_strength_data_pub.publish(publish_type)

    def light_strength_callback(self, event, sender, data, **args):
        # Publish what we have
        publish_type = Int32()
        publish_type.data = data[0]
        self._light_strength_data_pub.publish(publish_type)


    def flight_data_callback(self, event, sender, data, **args):
        flight_data = FlightData()

        # Battery state
        flight_data.battery_percent = data.battery_percentage
        flight_data.estimated_flight_time_remaining = data.drone_fly_time_left / 10.

        # Flight mode
        flight_data.flight_mode = data.fly_mode

        # Flight time
        flight_data.flight_time = data.fly_time

        # Very coarse velocity data
        # TODO do east and north refer to the body frame?
        # TODO the / 10. conversion might be wrong, verify
        flight_data.east_speed = -1. if data.east_speed > 30000 else data.east_speed / 10.
        flight_data.north_speed = -1. if data.north_speed > 30000 else data.north_speed / 10.
        flight_data.ground_speed = -1. if data.ground_speed > 30000 else data.ground_speed / 10.

        # Altitude
        flight_data.altitude = -1. if data.height > 30000 else data.height / 10.
        # flight_data.altitude = data.height

        # Equipment status
        flight_data.equipment = data.electrical_machinery_state
        flight_data.high_temperature = data.temperature_height

        # Some state indicators?
        flight_data.em_ground = data.em_ground
        flight_data.em_sky = data.em_sky
        flight_data.em_open = data.em_open

        flight_data.pitch = data.pitch
        flight_data.roll  = data.roll 
        flight_data.yaw   = data.yaw  
        # flight_data.templ = data.templ
        # flight_data.temph = data.temph
        # flight_data.tof   = data.tof  
        flight_data.agx   = data.agx  
        flight_data.agy   = data.agy  
        flight_data.agy   = data.agy  

        # Publish what we have
        self._flight_data_pub.publish(flight_data)

        # Debugging: is there data here? Print nonzero values
        if data.battery_low:
            print('battery_low is nonzero: %d' % data.battery_low)
        if data.battery_lower:
            print('battery_lower is nonzero: %d' % data.battery_lower)
        if data.battery_state:
            print('battery_state is nonzero: %d' % data.battery_state)
        if data.drone_battery_left:
            print('drone_battery_left is nonzero: %d' % data.drone_battery_left)
        if data.camera_state:
            print('camera_state is nonzero: %d' % data.camera_state)
        if data.down_visual_state:
            print('down_visual_state is nonzero: %d' % data.down_visual_state)
        if data.drone_hover:
            print('drone_hover is nonzero: %d' % data.drone_hover)
        if data.factory_mode:
            print('factory_mode is nonzero: %d' % data.factory_mode)
        if data.front_in:
            print('front_in is nonzero: %d' % data.front_in)
        if data.front_lsc:
            print('front_lsc is nonzero: %d' % data.front_lsc)
        if data.front_out:
            print('front_out is nonzero: %d' % data.front_out)
        if data.gravity_state:
            print('gravity_state is nonzero: %d' % data.gravity_state)
        if data.imu_calibration_state:
            print('imu_calibration_state is nonzero: %d' % data.imu_calibration_state)
        if data.imu_state:
            print('imu_state is nonzero: %d' % data.imu_state)
        if data.outage_recording:
            print('outage_recording is nonzero: %d' % data.outage_recording)
        if data.power_state:
            print('power_state is nonzero: %d' % data.power_state)
        if data.pressure_state:
            print('pressure_state is nonzero: %d' % data.pressure_state)
        if data.throw_fly_timer:
            print('throw_fly_timer is nonzero: %d' % data.throw_fly_timer)
        if data.wind_state:
            print('wind_state is nonzero: %d' % data.wind_state)
        if data.temperature_height:
            pass
        	# print('temperature_height is nonzero: %d' % data.temperature_height)

    def cmd_vel_callback(self, msg):
        self._drone.set_pitch(msg.linear.x)
        self._drone.set_roll(-msg.linear.y)     # Note sign flip
        self._drone.set_throttle(msg.linear.z)
        self._drone.set_yaw(-msg.angular.z)     # Note sign flip

        self._drone.set_high_speed_mode(msg.angular.x > 0)

    def takeoff_callback(self, msg):
        self._drone.takeoff()

    def land_callback(self, msg):
        self._drone.land()

    def flip_callback(self, msg):
        if msg.flip_command == Flip.flip_forward:
            self._drone.flip_forward()
        elif msg.flip_command == Flip.flip_back:
            self._drone.flip_back()
        elif msg.flip_command == Flip.flip_left:
            self._drone.flip_left()
        elif msg.flip_command == Flip.flip_right:
            self._drone.flip_right()
        elif msg.flip_command == Flip.flip_forwardleft:
            self._drone.flip_forwardleft()
        elif msg.flip_command == Flip.flip_forwardright:
            self._drone.flip_forwardright()
        elif msg.flip_command == Flip.flip_backleft:
            self._drone.flip_backleft()
        elif msg.flip_command == Flip.flip_backright:
            self._drone.flip_backright()

    def zoom_callback(self, msg):
        zoom = msg.data
        self._drone.set_video_mode(zoom)

    def exposure_callback(self, msg):
        exposure = msg.data
        if exposure > 3:
            print ("ERROR: publication of exposure is {} which is > 3".format(exposure))
            return
        self._drone.set_exposure(exposure)

    def video_encoder_rate_callback(self, msg):
        video_encoder_rate = msg.data
        self._drone.set_video_encoder_rate(video_encoder_rate)

    def log_level_callback(self, msg):
        """
        Set_loglevel controls the output messages. Valid levels are
        LOG_ERROR, LOG_WARN, LOG_INFO, LOG_DEBUG and LOG_ALL.
        """
        log_level = msg.data
        log_level_options = ['LOG_ERROR', 'LOG_WARN', 'LOG_INFO', 'LOG_DEBUG', 'LOG_ALL']
        if not log_level in log_level_options:
            print ("ERROR: publication of log_level is {} which is not in {}".format(log_level, log_level_options))
            return
        self._drone.set_loglevel(log_level)

        


        



    # def video_worker(self):

        # # Get video stream, open in PyAV
        # container = av.open(self._drone.get_video_stream())

        # # Decode h264
        # rospy.loginfo('starting video pipeline')
        # for frame in container.decode(video=0):

        #     # Convert PyAV frame => PIL image => OpenCV Mat
        #     color_mat = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)

        #     # Convert OpenCV Mat => ROS Image message and publish
        #     self._image_pub.publish(self._cv_bridge.cv2_to_imgmsg(color_mat, 'bgr8'))

        #     # Check for normal shutdown
        #     if self._stop_request.isSet():
        #         return


    def videoFrameHandler(self, event, sender, data):
        """
        Listens for video streaming (raw h264) from the Tello.

        Runs as a thread, sets self.frame to the most recent frame Tello captured.

        """
        # print("received {} bytes".format(len(data)))
        self.packet_data += data
        # print(len(data))
        # end of frame
        if len(data) != 1460:
            # print("trying to decode")
            for frame in self._h264_decode(self.packet_data):
                # print("decoded frame")  
                # Convert PyAV frame => PIL image => OpenCV Mat
                # color_mat = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
                color_mat = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                # Convert OpenCV Mat => ROS Image message and publish
                img_msg = self._cv_bridge.cv2_to_imgmsg(color_mat, 'bgr8')
                img_msg.header.stamp = rospy.Time.now()
                self._image_pub.publish(img_msg)
                # print("published frame")                
                self.camera_info.header = img_msg.header
                self._camera_info_pub.publish(self.camera_info)
                # print("published")

            self.packet_data = ""



    def _h264_decode(self, packet_data):
        """
        decode raw h264 format data from Tello
        
        :param packet_data: raw h264 data array
       
        :return: a list of decoded frame
        """
        res_frame_list = []
        frames = self.decoder.decode(packet_data)
        for framedata in frames:
            (frame, w, h, ls) = framedata
            if frame is not None:
                # print 'frame size %i bytes, w %i, h %i, linesize %i' % (len(frame), w, h, ls)

                frame = np.fromstring(frame, dtype=np.ubyte, count=len(frame), sep='')
                frame = (frame.reshape((h, ls / 3, 3)))
                frame = frame[:, :w, :]
                res_frame_list.append(frame)

        return res_frame_list

    def move_up_callback(self, msg):
        self._drone.move_up(msg.data)



if __name__ == '__main__':
    driver = FlockDriver()
