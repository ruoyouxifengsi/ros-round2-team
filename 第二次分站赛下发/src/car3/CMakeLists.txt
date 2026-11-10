cmake_minimum_required(VERSION 2.8.3)

project(car3)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

catkin_install_python(PROGRAMS
  scripts/gripper_mimic.py
  scripts/grasp_attach.py
  scripts/arm_control_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 注意: launch/config/urdf/world/r1map 不装进 share —— devel/setup.bash 会把 src 加进
# ROS_PACKAGE_PATH, rospack find 解析到 src; 若再 install 到 devel/share 产生第二份拷贝,
# roslaunch 的 find_resource 会报 "multiple files named" (2026-09-19 踩过)
