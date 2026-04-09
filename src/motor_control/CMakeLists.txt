cmake_minimum_required(VERSION 3.0.2)
project(motor_control)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  rk3588_sdk
)
catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs rk3588_sdk
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(motor_control_node src/motor_control_node.cpp)
target_link_libraries(motor_control_node
  ${catkin_LIBRARIES}
)
# 安装规则
install(TARGETS motor_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)