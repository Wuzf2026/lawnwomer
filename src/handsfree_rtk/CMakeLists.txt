cmake_minimum_required(VERSION 3.0.2)

project(handsfree_rtk)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/ros_driver_um98x.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)