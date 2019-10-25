from math import sin, cos, atan, pi

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import Empty, Bool, Int32, Float32
from flock_msgs.msg import Flip, FlightData
from tf.transformations import euler_from_quaternion
import time
import threading
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random



        # x_ = xf - x
        # y_ = yf - y

        # saved_x = x
        # saved_y = y

        # x_rotated = round(x_*cos(alpha_rad) + y_*sin(alpha_rad), 3)
        # y_rotated = round(y_*cos(alpha_rad) - x_*sin(alpha_rad), 3)

        # pitch = min(x_rotated, 0.5)
        # roll  = min(y_rotated, 0.5)

        # x = x + round(pitch * cos(alpha_rad) - roll  * sin(alpha_rad), 3)
        # y = y + round(roll  * cos(alpha_rad) + pitch * sin(alpha_rad), 3)

        # print("started from ({},{}) to ({},{}), x_,y_=({},{}), rotated=({},{}) pitch={} roll={} new=({},{}) delta=({},{})".format(saved_x, saved_y, xf, yf, x_, y_, x_rotated, y_rotated, pitch, roll, x, y, xf-x, yf-y))
        # raw_input()

class Sim(object):
    def __init__(self, avg_n = 3, orientation_alpha_base=0):
        self.speed = Point()
        self.command_pos = Point(1, 0, 0)
        self.twist = Twist()
        self.rotated_pos = Point() 
        self.real_world = Point(0, 0, 0)
        self.rotated_position_0 = Point()
        self.pos_error = Point()
        self.real_world_scale = 3.96
        self.pos_error_prev = Point()
        self.rotation_error_prev = 0
        self.pos_err_derivative = Point()
        self.pos_err_filtered_derivative = Point()
        self.rotation_err_filtered_derivative = 0
        self.Kp = Point(0.7, 0.7, 0.7)
        self.Kd = Point(1, 1, 1)
        self.orientation_alpha_base = orientation_alpha_base
        self.avg_n = avg_n
        self.alpha_buffer = avg_n * [self.orientation_alpha_base]
        self.orientation_alpha = sum(self.alpha_buffer) / self.avg_n
        self.orientation_alpha_rad = self.orientation_alpha * pi / 180.0
        self.caution_speed_threshold = Point(0.15, 0.15, 0.2)


    def calculate_speed(self):


        self.pos_error.x = self.command_pos.x - self.real_world.x
        self.pos_error.y = self.command_pos.y - self.real_world.y
        self.pos_error.z = self.command_pos.z - self.real_world.z

        pos_error_copy = self.point_copy(self.pos_error)

        # self.rotation_error = self.slam_orientation.z - 0


        x_rotated = pos_error_copy.x*math.cos(self.orientation_alpha_rad) + pos_error_copy.y*math.sin(self.orientation_alpha_rad)
        y_rotated = pos_error_copy.y*math.cos(self.orientation_alpha_rad) - pos_error_copy.x*math.sin(self.orientation_alpha_rad)


        self.pos_error.x = x_rotated
        self.pos_error.y = y_rotated

        # self.delta_pub.publish(self.pos_error)


        self.pos_err_derivative.x = self.pos_error.x - self.pos_error_prev.x
        self.pos_err_derivative.y = self.pos_error.y - self.pos_error_prev.y
        self.pos_err_derivative.z = self.pos_error.z - self.pos_error_prev.z
        # self.rotation_err_derivative = self.rotation_error - self.rotation_error_prev

        self.pos_err_filtered_derivative.x = 0.5648*self.pos_err_filtered_derivative.x + 12.75*self.pos_err_derivative.x
        self.pos_err_filtered_derivative.y = 0.5648*self.pos_err_filtered_derivative.y + 12.75*self.pos_err_derivative.y
        self.pos_err_filtered_derivative.z = 0.5648*self.pos_err_filtered_derivative.z + 12.75*self.pos_err_derivative.z
        # self.rotation_err_filtered_derivative = 0.5648*self.rotation_err_filtered_derivative + 12.75*self.rotation_err_derivative

        self.speed.x = self.Kp.x*self.pos_error.x + self.Kd.x*self.pos_err_filtered_derivative.x
        self.speed.y = self.Kp.y*self.pos_error.y + self.Kd.y*self.pos_err_filtered_derivative.y
        # speed.z = self.Kp.z*self.pos_error.z + self.Kd.z*self.pos_err_filtered_derivative.z
        self.speed.z = 0


        self.speed = self.clip_point(self.speed, self.caution_speed_threshold)


    def calculate_real_world(self):
        self.real_world_copy = self.point_copy(self.real_world)
        self.real_world.x += round(self.speed.x * cos(self.orientation_alpha_rad) - self.speed.y  * sin(self.orientation_alpha_rad), 3)/2
        self.real_world.y += round(self.speed.y  * cos(self.orientation_alpha_rad) + self.speed.x * sin(self.orientation_alpha_rad), 3)/2
        self.alpha_buffer.pop(0)
        self.alpha_buffer.append(self.orientation_alpha_base + 2 * random.random())
        self.orientation_alpha = sum(self.alpha_buffer) / self.avg_n
        self.orientation_alpha_rad = self.orientation_alpha * pi / 180.0


    def clip_point(self, p, threshold_point):
        if abs(p.x) > threshold_point.x:
            p.x = self.sign(p.x) * threshold_point.x
        if abs(p.y) > threshold_point.y:
            p.y = self.sign(p.y) * threshold_point.y
        if abs(p.z) > threshold_point.z:
            p.z = self.sign(p.z) * threshold_point.z
        return p

    def point_copy(self, p):
        return Point(p.x, p.y, p.z)

    def sign(self, x):
        return 2 * (x > 0) - 1


if __name__ == '__main__':
    # x = 0
    # y = 0

    # xf = 1
    # yf = 2


    # pitch = 0
    # roll = 0

    # alpha = 45
    # alpha_rad = 45*pi/180

    sim = Sim(avg_n = 8, orientation_alpha_base = 0)

    plt.ion()
    plt.show()
    # fig, ax = plt.subplots()
    plt.axis([-2, 2, -2, 2])

    plt.scatter([sim.real_world.x], [sim.real_world.y], s=60, marker='o', c='b')
    plt.scatter([sim.command_pos.x], [sim.command_pos.y], s=60, marker='o', c='k')
    x_ = [sim.real_world.x, sim.real_world.x + cos(sim.orientation_alpha_rad)]
    y_ = [sim.real_world.y, sim.real_world.y + sin(sim.orientation_alpha_rad)]
    plt.plot(x_, y_)
    plt.pause(0.05)

    

    while True: 

        while abs(sim.real_world.x-sim.command_pos.x) > 0.08 or abs(sim.real_world.y-sim.command_pos.y) > 0.08:
            # raw_input('.')
            sim.calculate_speed()
            print('delta={},{} alpha={}'.format(sim.pos_error.x, sim.pos_error.y, sim.orientation_alpha))
            print('pitch={} roll={}'.format(sim.speed.x, sim.speed.y))
            sim.calculate_real_world()
            print('real world = {},{} command_pos= {},{}'.format(sim.real_world.x, sim.real_world.y, sim.command_pos.x, sim.command_pos.y))

            plt.scatter([sim.real_world.x], [sim.real_world.y], s=10, marker='o', c='b')
            plt.scatter([sim.command_pos.x], [sim.command_pos.y], s=60, marker='o', c='k')
            plt.pause(0.5)
            # line, = ax.plot([sim.real_world.x], [sim.real_world.y], 'bo')
            # line, = ax.plot([sim.command_pos.x], [sim.command_pos.y], 'go')
            # time.sleep(1)
        time.sleep(1)
        sim = Sim(avg_n = 8, orientation_alpha_base = random.randint(-179, 179))
        # sim.orientation_alpha_base = float(raw_input('base angle\n'))
        sim.orientation_alpha_base = random.randint(-179, 179)
        sim.real_world.x = float(random.randint(-1, 1))
        sim.real_world.y = float(random.randint(-1, 1))
        sim.command_pos.x = float(random.randint(-1, 1))
        sim.command_pos.y = float(random.randint(-1, 1))
        plt.clf()
        plt.axis([-2, 2, -2, 2])
        plt.scatter([sim.real_world.x], [sim.real_world.y], s=60, marker='o', c='b')
        plt.scatter([sim.command_pos.x], [sim.command_pos.y], s=60, marker='o', c='k')
        x_ = [sim.real_world.x, sim.real_world.x + cos(sim.orientation_alpha_rad)]
        y_ = [sim.real_world.y, sim.real_world.y + sin(sim.orientation_alpha_rad)]
        plt.plot(x_, y_)
        plt.pause(0.05)
