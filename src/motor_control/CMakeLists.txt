cmake_minimum_required(VERSION 3.10)
project(motor_control)

# 引入RK3588工具链
include(${CMAKE_SOURCE_DIR}/../../rk3588_toolchain.cmake)

# ROS1依赖
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    std_msgs
    serial
    rk3588_sdk
)

# 导出配置
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp geometry_msgs std_msgs serial rk3588_sdk
)

# 包含目录
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/../rk3588_sdk/include
)

# 编译电机控制节点（Serial协议+485+轮速回采）
add_executable(motor_control_node
    src/motor_node.cpp
    src/serial_protocol.cpp
    src/motor_driver.cpp
    src/pid_controller.cpp
    src/rs485_comm.cpp
    src/serial_port.cpp
    src/motor_controller.cpp
)

# 链接依赖（RK3588 485/串口库）
target_link_libraries(motor_control_node
    ${catkin_LIBRARIES}
    rk3588_sdk
    pthread
)

# ARM64编译优化
target_compile_options(motor_control_node PRIVATE
    -march=armv8.2-a
    -O2
    -Wall
    -std=c++17
)

# 安装规则
install(TARGETS motor_control_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)