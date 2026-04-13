cmake_minimum_required(VERSION 3.0.2)
project(tools)

# 引入全局RK3588工具链
include(${CMAKE_SOURCE_DIR}/rk3588_toolchain.cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  lawnmower_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs lawnmower_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 安装Python脚本/launch/config
catkin_install_python(PROGRAMS
  scripts/param_config.py
  scripts/data_monitor.py
  scripts/status_query.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)