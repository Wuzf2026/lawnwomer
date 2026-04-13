cmake_minimum_required(VERSION 3.0.2)
project(motor_control)

# 引入全局RK3588工具链（无需重复定义）
include(${CMAKE_SOURCE_DIR}/rk3588_toolchain.cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  lawnmower_msgs
)

# 串口依赖（RK3588 Ubuntu20.04需安装libserial-dev）
find_library(SERIAL_LIB serial)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs lawnmower_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/serial  # libserial头文件路径
)

# 编译电机控制节点
add_executable(motor_control_node src/motor_control_node.cpp)
target_link_libraries(motor_control_node
  ${catkin_LIBRARIES}
  ${SERIAL_LIB}
)

# 安装节点/launch/config
install(TARGETS motor_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)