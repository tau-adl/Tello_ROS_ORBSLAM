#!/usr/bin/env python

from __future__ import print_function

# import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Image

import time
import pygame
import pygame.display
import pygame.key
import pygame.locals
import pygame.font

from cv_bridge import CvBridge, CvBridgeError

from subprocess import Popen, PIPE


prev_flight_data = None
video_player = None
video_recorder = None
font = None
wid = None
pygame_screen = None
bridge = None
date_fmt = '%Y-%m-%d_%H%M%S'

# import sys, select, termios, tty
# import keyboard

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
"""

# moveBindings = {
#         'i':(1,0,0,0),
#         'o':(1,0,0,-1),
#         'j':(0,0,0,1),
#         'l':(0,0,0,-1),
#         'u':(1,0,0,1),
#         ',':(-1,0,0,0),
#         '.':(-1,0,0,1),
#         'm':(-1,0,0,-1),
#         'O':(1,-1,0,0),
#         'I':(1,0,0,0),
#         'J':(0,1,0,0),
#         'L':(0,-1,0,0),
#         'U':(1,1,0,0),
#         '<':(-1,0,0,0),
#         '>':(-1,-1,0,0),
#         'M':(-1,1,0,0),
#         't':(0,0,1,0),
#         'b':(0,0,-1,0),
#     }

# speedBindings={
#         'q':(1.1,1.1),
#         'z':(.9,.9),
#         'w':(1.1,1),
#         'x':(.9,1),
#         'e':(1,1.1),
#         'c':(1,.9),
#     }

# def getKey():
#     tty.setraw(sys.stdin.fileno())
#     select.select([sys.stdin], [], [], 0)
#     keys = sys.stdin.read(1)
#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
#     for key in keys:
#         if key == '\x03'
#             raise KeyboardInterrupt
#         elif key == '\x1a':
#             exit()
#     return keys


# def vels(speed,turn):
#     return "currently:\tspeed %s\tturn %s " % (speed,turn)
def status_print(text):
    pygame.display.set_caption(text)

def videoFrameHandler(data):
    global video_player
    global video_recorder
    global wid
    global bridge

    try:
        data = bridge.imgmsg_to_cv2(data, "rgb8")
    except CvBridgeError as e:
        print(e)

    frame = pygame.surfarray.make_surface(data)
    frame = pygame.transform.rotate(frame, 270)
    frame = pygame.transform.flip(frame, 1, 0)

    pygame_screen.blit(frame, (0,0))
    pygame.display.update()


    # if video_player is None:
    #     cmd = [ 'mplayer', '-fps', '35', '-really-quiet' ]
    #     if wid is not None:
    #         cmd = cmd + [ '-wid', str(wid) ]
    #     video_player = Popen(cmd + ['-'], stdin=PIPE)
    #     print("Popen(cmd + ['-'], stdin=PIPE)")

    # try:
    #     video_player.stdin.write(data)
    # except IOError as err:
    #     status_print(str(err))
    #     video_player = None

    # try:
    #     if video_recorder:
    #         video_recorder.stdin.write(data)
    # except IOError as err:
    #     status_print(str(err))
    #     video_recorder = None

def set_command(keyname, twist):
    if keyname == 'w':
        twist.linear.z = 0.2
    elif keyname == 's':
        twist.linear.z = -0.2
    elif keyname == 'a':
        twist.angular.z = 0.2
    elif keyname == 'd':
        twist.angular.z = -0.2
    elif keyname == 'up':
        twist.linear.x = 0.2
    elif keyname == 'down':
        twist.linear.x = -0.2
    elif keyname == 'left':
        twist.linear.y = 0.2
    elif keyname == 'right':
        twist.linear.y = -0.2
    elif keyname == 'e':
        twist.angular.x = 1.0

    return twist

def raise_command(keyname, twist):
    if keyname == 'w':
        twist.linear.z += 0.05
    elif keyname == 's':
        twist.linear.z += -0.05
    elif keyname == 'a':
        twist.angular.z += 0.05
    elif keyname == 'd':
        twist.angular.z += -0.05
    elif keyname == 'up':
        twist.linear.x += 0.05
    elif keyname == 'down':
        twist.linear.x += -0.05
    elif keyname == 'left':
        twist.linear.y += 0.05
    elif keyname == 'right':
        twist.linear.y += -0.05
    elif keyname == 'e':
        twist.angular.x = 1.0

    return twist

def reset_command(keyname, twist):
    if keyname == 'w':
        twist.linear.z = 0.0
    elif keyname == 's':
        twist.linear.z = -0.0
    elif keyname == 'a':
        twist.angular.z = 0.0
    elif keyname == 'd':
        twist.angular.z = -0.0
    elif keyname == 'up':
        twist.linear.x = 0.0
    elif keyname == 'down':
        twist.linear.x = -0.0
    elif keyname == 'left':
        twist.linear.y = 0.0
    elif keyname == 'right':
        twist.linear.y = 0.0
    elif keyname == 'e':
        twist.angular.x = 0.0

    return twist

def limit_twist(twist):
    twist.linear.x = ((twist.linear.x > 0)*2-1) * min(abs(twist.linear.x), 0.3)
    twist.linear.y = ((twist.linear.y > 0)*2-1) * min(abs(twist.linear.y), 0.3)
    twist.angular.z = ((twist.angular.z > 0)*2-1) * min(abs(twist.angular.z), 0.3)
    twist.linear.z = ((twist.linear.z > 0)*2-1) * min(abs(twist.linear.z), 0.3)
    return twist

if __name__=="__main__":
    # settings = termios.tcgetattr(sys.stdin)

    time.sleep(1)

    pub_twist = rospy.Publisher('/tello/cmd_vel', Twist, queue_size = 1)
    pub_takeoff = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
    pub_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
    # rospy.Subscriber("camera/image_raw", Image, videoFrameHandler)
    rospy.Subscriber("/camera/image_raw", Image, videoFrameHandler)
    

    pygame.init()
    pygame.display.init()
    pygame_screen = pygame.display.set_mode((960, 720))
    pygame.font.init()

    bridge = CvBridge()

    font = pygame.font.SysFont("dejavusansmono", 32)
    if 'window' in pygame.display.get_wm_info():
        wid = pygame.display.get_wm_info()['window']
    print("Tello video WID:", wid)

    rospy.init_node('tello_keyboard')

    twist = Twist()

    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    t = time.time()

    list_of_pressed_keys = []

    try:
        while not rospy.is_shutdown():

            for keyname in list_of_pressed_keys:
                twist = raise_command(keyname, twist)

            for e in pygame.event.get():
                # WASD for movement
                if e.type == pygame.locals.KEYDOWN:
                    keyname = pygame.key.name(e.key)
                    print('+' + keyname)
                    if keyname == 'escape':
                        # drone.quit()
                        raise KeyboardInterrupt

                    if keyname == 'backspace':
                        pub_land.publish()
                        continue
                    elif keyname == 'tab':
                        pub_takeoff.publish()
                        continue
                    else:
                        if not keyname in list_of_pressed_keys:
                            list_of_pressed_keys.append(keyname)
                            twist = set_command(keyname, twist)
                        # twist = set_command(keyname, twist)
                        

                elif  e.type == pygame.locals.KEYUP:
                    keyname = pygame.key.name(e.key)
                    print('-' + keyname)
                    if keyname in list_of_pressed_keys:
                        list_of_pressed_keys.remove(keyname)
                    twist = reset_command(keyname, twist)

            

                
            

            twist = limit_twist(twist)
            pub_twist.publish(twist)

            time.sleep(0.033)  # loop with pygame.event.get() is too mush tight w/o some sleep
            # twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

            # if time.time() - t > 5:
            #     twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            #     twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            #     t = time.time()
        pygame.display.quit()
        pygame.quit()


    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub_twist.publish(twist)
        pub_land.publish()

        # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)