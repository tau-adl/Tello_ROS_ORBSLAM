#!/usr/bin/env python

from __future__ import print_function

import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import Int32 
from flock_msgs.msg import FlightData
import time

class telloLowBattery(object):

    def __init__(self):
        # Initialize ROS
        rospy.init_node('raw_image_fps_calc', anonymous=False)

        self.low_bat_warning = rospy.Publisher('low_battery_warning', Int32, queue_size=1)

        # ROS subscriptions
        # rospy.Subscriber("image_raw", Image, self.fpsCalculator)
        rospy.Subscriber("flight_data", FlightData, self.flight_data_callback)
        self.low_battery_flag = False
        self.t = time.time()
        self.TIME_TO_PRINT = 3


        # Spin until interrupted
        rospy.spin()

        rospy.loginfo("Quiting tello_low_battery")


    def flight_data_callback(self, flight_data):
        battery_percent = flight_data.battery_percent
        if not self.low_battery_flag:
            if battery_percent < 13:
                self.low_battery_flag = True
        else:
            if time.time() - self.t > self.TIME_TO_PRINT:
                rospy.loginfo('#######   Warning - Low Battery! {}%    #######'.format(battery_percent))
                self.t = time.time()
    #     flight_data.estimated_flight_time_remaining = data.drone_fly_time_left / 10.

    #     # Flight mode
    #     flight_data.flight_mode = data.fly_mode

    #     # Flight time
    #     flight_data.flight_time = data.fly_time

    #     # Very coarse velocity data
    #     # TODO do east and north refer to the body frame?
    #     # TODO the / 10. conversion might be wrong, verify
    #     flight_data.east_speed = -1. if data.east_speed > 30000 else data.east_speed / 10.
    #     flight_data.north_speed = -1. if data.north_speed > 30000 else data.north_speed / 10.
    #     flight_data.ground_speed = -1. if data.ground_speed > 30000 else data.ground_speed / 10.

    #     # Altitude
    #     flight_data.altitude = -1. if data.height > 30000 else data.height / 10.

    #     # Equipment status
    #     flight_data.equipment = data.electrical_machinery_state
    #     flight_data.high_temperature = data.temperature_height

    #     # Some state indicators?
    #     flight_data.em_ground = data.em_ground
    #     flight_data.em_sky = data.em_sky
    #     flight_data.em_open = data.em_open    



if __name__ == '__main__':
    driver = telloLowBattery()
