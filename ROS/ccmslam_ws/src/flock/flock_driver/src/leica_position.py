#!/usr/bin/env python

from __future__ import print_function

import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32 
from flock_msgs.msg import FlightData
import time

class telloLowBattery(object):

    def __init__(self):
        # Initialize ROS
        rospy.init_node('leica_position', anonymous=False)

        self.position_fixed = rospy.Publisher('/leica/position_fixed', PointStamped, queue_size=1)

        # ROS subscriptions
        # rospy.Subscriber("image_raw", Image, self.fpsCalculator)
        rospy.Subscriber("/leica/position", PointStamped, self.point_stamped)
        self.low_battery_flag = False
        self.t = time.time()
        self.TIME_TO_PRINT = 3


        # Spin until interrupted
        rospy.spin()

        rospy.loginfo("Quiting leica_position")


    def point_stamped(self, point):
        point.header.frame_id = "map"
        x = point.point.x
        y = point.point.y
        z = point.point.z
        point.point.y = x - 4.76291747561
        point.point.x = y - -1.82978867159
        point.point.z -= 1.18285997806
        self.position_fixed.publish(point)

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
