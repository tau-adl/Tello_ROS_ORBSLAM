# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "orb_slam2_ros: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(orb_slam2_ros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/arkadiros/ROS/tello_catkin_ws/src/orb_slam_2_ros/srv/SaveMap.srv" NAME_WE)
add_custom_target(_orb_slam2_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "orb_slam2_ros" "/home/arkadiros/ROS/tello_catkin_ws/src/orb_slam_2_ros/srv/SaveMap.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(orb_slam2_ros
  "/home/arkadiros/ROS/tello_catkin_ws/src/orb_slam_2_ros/srv/SaveMap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orb_slam2_ros
)

### Generating Module File
_generate_module_cpp(orb_slam2_ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orb_slam2_ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(orb_slam2_ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(orb_slam2_ros_generate_messages orb_slam2_ros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arkadiros/ROS/tello_catkin_ws/src/orb_slam_2_ros/srv/SaveMap.srv" NAME_WE)
add_dependencies(orb_slam2_ros_generate_messages_cpp _orb_slam2_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(orb_slam2_ros_gencpp)
add_dependencies(orb_slam2_ros_gencpp orb_slam2_ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS orb_slam2_ros_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(orb_slam2_ros
  "/home/arkadiros/ROS/tello_catkin_ws/src/orb_slam_2_ros/srv/SaveMap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/orb_slam2_ros
)

### Generating Module File
_generate_module_eus(orb_slam2_ros
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/orb_slam2_ros
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(orb_slam2_ros_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(orb_slam2_ros_generate_messages orb_slam2_ros_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arkadiros/ROS/tello_catkin_ws/src/orb_slam_2_ros/srv/SaveMap.srv" NAME_WE)
add_dependencies(orb_slam2_ros_generate_messages_eus _orb_slam2_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(orb_slam2_ros_geneus)
add_dependencies(orb_slam2_ros_geneus orb_slam2_ros_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS orb_slam2_ros_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(orb_slam2_ros
  "/home/arkadiros/ROS/tello_catkin_ws/src/orb_slam_2_ros/srv/SaveMap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orb_slam2_ros
)

### Generating Module File
_generate_module_lisp(orb_slam2_ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orb_slam2_ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(orb_slam2_ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(orb_slam2_ros_generate_messages orb_slam2_ros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arkadiros/ROS/tello_catkin_ws/src/orb_slam_2_ros/srv/SaveMap.srv" NAME_WE)
add_dependencies(orb_slam2_ros_generate_messages_lisp _orb_slam2_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(orb_slam2_ros_genlisp)
add_dependencies(orb_slam2_ros_genlisp orb_slam2_ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS orb_slam2_ros_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(orb_slam2_ros
  "/home/arkadiros/ROS/tello_catkin_ws/src/orb_slam_2_ros/srv/SaveMap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/orb_slam2_ros
)

### Generating Module File
_generate_module_nodejs(orb_slam2_ros
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/orb_slam2_ros
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(orb_slam2_ros_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(orb_slam2_ros_generate_messages orb_slam2_ros_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arkadiros/ROS/tello_catkin_ws/src/orb_slam_2_ros/srv/SaveMap.srv" NAME_WE)
add_dependencies(orb_slam2_ros_generate_messages_nodejs _orb_slam2_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(orb_slam2_ros_gennodejs)
add_dependencies(orb_slam2_ros_gennodejs orb_slam2_ros_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS orb_slam2_ros_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(orb_slam2_ros
  "/home/arkadiros/ROS/tello_catkin_ws/src/orb_slam_2_ros/srv/SaveMap.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orb_slam2_ros
)

### Generating Module File
_generate_module_py(orb_slam2_ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orb_slam2_ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(orb_slam2_ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(orb_slam2_ros_generate_messages orb_slam2_ros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arkadiros/ROS/tello_catkin_ws/src/orb_slam_2_ros/srv/SaveMap.srv" NAME_WE)
add_dependencies(orb_slam2_ros_generate_messages_py _orb_slam2_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(orb_slam2_ros_genpy)
add_dependencies(orb_slam2_ros_genpy orb_slam2_ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS orb_slam2_ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orb_slam2_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/orb_slam2_ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(orb_slam2_ros_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/orb_slam2_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/orb_slam2_ros
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(orb_slam2_ros_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orb_slam2_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/orb_slam2_ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(orb_slam2_ros_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/orb_slam2_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/orb_slam2_ros
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(orb_slam2_ros_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orb_slam2_ros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orb_slam2_ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/orb_slam2_ros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(orb_slam2_ros_generate_messages_py std_msgs_generate_messages_py)
endif()
