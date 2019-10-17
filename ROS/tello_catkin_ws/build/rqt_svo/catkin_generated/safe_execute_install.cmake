execute_process(COMMAND "/home/arkadiros/ROS/tello_catkin_ws/build/rqt_svo/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/arkadiros/ROS/tello_catkin_ws/build/rqt_svo/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
