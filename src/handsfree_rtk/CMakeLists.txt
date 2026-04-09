cmake_minimum_required(VERSION 3.0.2)
project(handsfree_rtk)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  serial
)
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs serial
)
# Python驱动脚本
add_custom_target(umd98x_driver ALL
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/src/umd98x_driver.py
  WORKING_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/handsfree_rtk
)
# NTRIP客户端
add_custom_target(umd98x_ntrip ALL
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/src/umd98x_ntrip.py
  WORKING_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/handsfree_rtk
)
# 安装规则
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  FILES_MATCHING PATTERN "*.py"
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)