cmake_minimum_required(VERSION 3.10.2)
project(lawnwomer_ws)

# 1. 基础配置（RK3588 ARM64 + ROS Noetic）
set(ROS_DISTRO noetic)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_C_COMPILER gcc)

# 2. ARM64 编译优化选项（RK3588 架构适配）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -march=armv8.2-a -O2 -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -march=armv8.2-a -O2 -Wall")

# 3. 查找 ROS 核心依赖（必须放在 catkin_init_workspace 前）
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  serial
)

# 4. 初始化 ROS 工作空间（ROS1 Noetic 核心指令）
catkin_init_workspace()

# 5. Python3 适配（Noetic 默认 Python3）
catkin_python_setup()

# 6. 全局包含目录（RK3588 系统头文件 + ROS 头文件）
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/aarch64-linux-gnu
  /usr/local/include
)

# 7. 全局链接库目录
link_directories(
  /usr/lib/aarch64-linux-gnu
  /usr/local/lib
  ${catkin_LIB_DIRS}
)

# 8. 导出依赖（供子包调用）
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs cv_bridge image_transport serial
  INCLUDE_DIRS include
  LIBRARIES rk3588_hardware
)
