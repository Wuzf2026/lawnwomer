cmake_minimum_required(VERSION 3.10.2)
project(handsfree_rtk)

# ARM64编译优化
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8.2-a -O2 -Wall")

# ROS1 Noetic依赖
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  rk3588_sdk
)

# 导出配置
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp rospy sensor_msgs rk3588_sdk
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/../rk3588_sdk/include
)

# 编译C++驱动（UM982 USB适配）
add_executable(rtk_um982_node
    demo/c++/um982_driver.cpp
)

target_link_libraries(rtk_um982_node
  ${catkin_LIBRARIES}
  rk3588_sdk  # 依赖RK3588串口驱动
)

# 安装Python脚本（ROS1 Noetic Python3）
catkin_install_python(PROGRAMS
    scripts/ros_driver_um98x.py
    scripts/ros_driver_um98x_ntrip.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装规则
install(TARGETS rtk_um982_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch usb_rules/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)