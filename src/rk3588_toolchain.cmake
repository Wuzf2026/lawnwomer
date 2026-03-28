# RK3588 Ubuntu20.04 ARM64 工具链配置（ROS1 Noetic）
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 本地编译（RK3588板端Ubuntu20.04）
set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)

# 编译选项（适配RK3588，关闭CUDA，优化性能）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -march=armv8-a -mtune=cortex-a76")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -O2 -march=armv8-a -mtune=cortex-a76")

# ROS1 Noetic 路径配置
set(ROS_ROOT /opt/ros/noetic)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${ROS_ROOT})

# 禁用CUDA
set(CUDA_ENABLED OFF CACHE BOOL "Disable CUDA for RK3588" FORCE)

# 串口库依赖（RK3588串口）
find_package(serial REQUIRED)
include_directories(${SERIAL_INCLUDE_DIRS})