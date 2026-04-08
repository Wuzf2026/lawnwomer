# RK3588 Toolchain file for cross-compilation
# 设置目标系统
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
# 设置交叉编译器路径（根据实际安装路径调整）
set(CMAKE_C_COMPILER /opt/atk-dlrk3588-toolchain/bin/aarch64-buildroot-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /opt/atk-dlrk3588-toolchain/bin/aarch64-buildroot-linux-gnu-g++)
set(CMAKE_AR /opt/atk-dlrk3588-toolchain/bin/aarch64-buildroot-linux-gnu-ar)
set(CMAKE_RANLIB /opt/atk-dlrk3588-toolchain/bin/aarch64-buildroot-linux-gnu-ranlib)
# 设置sysroot路径
set(CMAKE_SYSROOT /opt/atk-dlrk3588-toolchain/aarch64-buildroot-linux-gnu/sysroot)
# 设置搜索路径
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})
# 设置搜索策略
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)  # 程序在宿主机编译
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)   # 库文件在目标系统查找
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)   # 头文件在目标系统查找
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)   # 包配置在目标系统查找
# 设置C/C++标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# RK3588特定配置
set(RK3588 TRUE)
# 定义RK3588平台宏
add_definitions(-DRK3588)
# 链接选项（根据需要添加）
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L${CMAKE_SYSROOT}/usr/lib")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -L${CMAKE_SYSROOT}/usr/lib")
# 查找ROS相关库（在目标系统中）
set(ROS_ROOT ${CMAKE_SYSROOT}/opt/ros/noetic)
set(ROS_PACKAGE_PATH ${ROS_ROOT}/share)
# 设置catkin相关路径
set(catkin_DIR ${ROS_ROOT}/share/catkin/cmake)
set(roscpp_DIR ${ROS_ROOT}/share/roscpp/cmake)
set(rospy_DIR ${ROS_ROOT}/share/rospy/cmake)
set(sensor_msgs_DIR ${ROS_ROOT}/share/sensor_msgs/cmake)