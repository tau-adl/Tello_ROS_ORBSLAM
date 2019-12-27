cmake_minimum_required(VERSION 2.8.3)
project(flock_msgs)

## Find catkin and everything we need in ROS
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

## Generate code for messages we define
add_message_files(DIRECTORY msg FILES FlightData.msg Flip.msg)
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package and export dependencies
catkin_package(CATKIN_DEPENDS message_runtime)

## Set up include directories
include_directories(${catkin_INCLUDE_DIRS})
