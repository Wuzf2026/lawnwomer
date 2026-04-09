cmake_minimum_required(VERSION 2.8.3)
project(lawnwomer)
# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 查找catkin
find_package(catkin REQUIRED)
# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()
# 子目录
add_subdirectory(handsfree_rtk)
add_subdirectory(orbbec_camera)
add_subdirectory(Hesai_Lidar)
add_subdirectory(rk3588_sdk)
add_subdirectory(lawnmower_base)
add_subdirectory(motor_control)