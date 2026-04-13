# RK3588交叉编译工具链配置
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 指定RK3588编译工具链路径（根据实际路径调整）
set(TOOLCHAIN_PATH /opt/rk3588_toolchain/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PATH}/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PATH}/bin/aarch64-linux-gnu-g++)

# ROS1 Noetic配置
set(ROS_DISTRO noetic)
set(ROS_ROOT /opt/ros/noetic)
include_directories(${ROS_ROOT}/include)
link_directories(${ROS_ROOT}/lib)

# 禁用CUDA和ROS2
add_definitions(-DNO_CUDA -DROS1_ONLY)

# 全局编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -fPIC")
set(CMAKE_BUILD_TYPE Release)

# RK3588硬件相关定义
add_definitions(-DRK3588_PLATFORM -DUBUNTU20_04)