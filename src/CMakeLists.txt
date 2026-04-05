cmake_minimum_required(VERSION 3.10)
project(lawnwomer)
# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 构建类型配置
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()
# 并行编译配置（使用所有可用核心）
if(NOT CMAKE_CACHE_STRING MATCHES "CMAKE_BUILD_PARALLEL_LEVEL")
    execute_process(
        COMMAND nproc
        OUTPUT_VARIABLE NPROC
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    set(CMAKE_BUILD_PARALLEL_LEVEL ${NPROC} CACHE STRING "Number of parallel jobs" FORCE)
endif()
# 设置catkin工作空间
set(CATKIN_DEVEL_PREFIX ${PROJECT_BINARY_DIR}/devel)
set(CMAKE_INSTALL_PREFIX ${PROJECT_BINARY_DIR}/install)
# 查找catkin核心组件
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    message_generation
    dynamic_reconfigure
)
# 查找第三方依赖
find_package(PCL 1.10 REQUIRED)
find_package(Boost 1.71 REQUIRED COMPONENTS system filesystem)
find_package(yaml-cpp REQUIRED)
# 设置包含目录
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
# 链接库配置
link_directories(
    ${PCL_LIBRARY_DIRS}
)
# 包含第三方包
add_subdirectory(third_party/handsfree_rtk)
add_subdirectory(third_party/orbbec_camera)
add_subdirectory(third_party/hesai_lidar)
add_subdirectory(third_party/rk3588_sdk)
# 包含自研包
add_subdirectory(lawnmower_base)
add_subdirectory(launch)
add_subdirectory(sensor_msgs)
add_subdirectory(param_config)
add_subdirectory(tools)
add_subdirectory(srv)
# 生成catkin配置
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES lawnwomer
    CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs dynamic_reconfigure
    DEPENDS PCL Boost
)