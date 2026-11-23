cmake_minimum_required(VERSION 3.0.2)
project(round2_vision)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES RecognizeTask.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs rospy std_msgs sensor_msgs geometry_msgs cv_bridge message_runtime
)

catkin_install_python(PROGRAMS
  scripts/vision_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
