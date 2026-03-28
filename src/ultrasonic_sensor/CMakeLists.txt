cmake_minimum_required(VERSION 3.10.2)
project(ultrasonic_sensor)

# ARM64编译配置
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8.2-a -O2 -Wall")

# ROS1依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  rk3588_sdk
)

# 导出配置
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs rk3588_sdk
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/../rk3588_sdk/include
)

# 编译12路超声波节点
add_executable(ultrasonic_sensor_node
  src/ultrasonic_node.cpp
  src/ultrasonic_driver.cpp
)

# 链接RK3588 485库
target_link_libraries(ultrasonic_sensor_node
  ${catkin_LIBRARIES}
  rk3588_sdk
  pthread
)

# 安装规则
install(TARGETS ultrasonic_sensor_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)