# Flock

Flock is a ROS driver for [DJI Tello](https://store.dji.com/product/tello) drones.

## Packages

* `flock` meta-package glue
* `flock_description` robot description files (TODO)
* `flock_driver` interface between the Tello hardware and ROS, not required for simulation
* `flock_base` base nodes
* `flock_rviz` extensions to rviz (TODO)
* `flock_gazebo` extensions to Gazebo (TODO)

## Nodes

### flock_driver

Provides a ROS wrapper around TelloPy. Not required for simulation.

#### Subscribed topics

* `~cmd_vel` [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
* `~takeoff` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~land` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~flip` flock_msgs/Flip

### flock_base

Provides teleop and (eventually) autonomous control.

#### Subscribed topics

* `~joy` [sensor_msgs/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html)

#### Published topics

* `~cmd_vel` [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
* `~takeoff` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~land` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* `~flip` flock_msgs/Flip

## Installation

### 1. Set up your Linux environment

Set up a Ubuntu 18.04 box or VM. This should include ffmpeg 3.4.4.
~~~
ffmpeg -version
~~~

### 2. Set up your Python environment

Use your favorite Python package manager to set up Python 2.7 and the following packages:

* numpy 1.15.2
* av 0.5.2
* opencv-python 3.4.3.18
* opencv-contrib-python 3.4.3.18
* tellopy 0.5.0

### 3. Install ROS

[Install ROS Melodic](http://wiki.ros.org/Installation/Ubuntu) with the `ros-melodic-desktop-full` option.
This will install Gazebo 9 and OpenCV 3.2, among other things.

Install these additional packages:
~~~
sudo apt install ros-melodic-joystick-drivers
~~~

### 4. Get Flock

Create a catkin workspace:
~~~
source /opt/ros/melodic/setup.bash
mkdir -p ~/flock_catkin_ws/src
cd ~/flock_catkin_ws/
catkin_make
source devel/setup.bash
~~~

Download and compile flock:
~~~
cd ~/flock_catkin_ws/src
git clone https://github.com/clydemcqueen/flock.git
cd ..
catkin_make
~~~

## Running Flock

### Test the environment

This script will connect to the drone and display a video feed in an OpenCV window.
It will also look for ArUco 6x6 markers and highlight them in green.
It does not require ROS.

Turn on the drone, connect to `TELLO-XXXXX` via wi-fi, and run this script:
~~~
python ~/flock_catkin_ws/src/flock/flock_driver/scripts/environment_test.py
~~~

### Teleop

This ROS launch file will allow you to fly the drone using a wired XBox One gamepad.

Turn on the drone, connect to `TELLO-XXXXX` via wi-fi, and launch ROS:
~~~
roslaunch flock_base teleop.launch
~~~

For left-handed operation:
~~~
roslaunch flock_base teleop.launch left_handed:=true
~~~

Controls:
* The left stick controls forward motion and yaw
* The right stick controls altitude and allows for strafing
* The small menu button takes off
* The small view button lands
* The 4 buttons Y, X, B and A will flip forward, left, right and back, respectively.
The Tello won't flip if the platform is unstable, so you may have a wait a second or two between flips.

An OpenCV window will pop up to show any 6x6 ArUco markers.

You can modify `teleop.launch` and/or `flock_base.py` to support other gamepads.

WARNING: the `flock_driver` node sends a `land` command just before it quits.
If you kill the ROS session normally (with a `Ctrl-C`) the drone should land as the ROS session dies.
However, if the `flock_driver` node dies unexpectedly while the drone is flying
you may have to re-launch and reconnect to get it to land.