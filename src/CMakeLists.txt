cmake_minimum_required(VERSION 2.8.3)
project(lawnwomer)
# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 查找catkin
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2_ros
)
# 包含目录
include_directories(
    ${catkin_INCLUDE_DIRS}
)
# 子目录
add_subdirectory(third_party/handsfree_rtk)
add_subdirectory(third_party/orbbec_camera)
add_subdirectory(third_party/hesai_lidar)
add_subdirectory(third_party/rk3588_sdk)
add_subdirectory(lawnmower_base)
add_subdirectory(launch)
add_subdirectory(sensor_msgs)
add_subdirectory(param_config)
add_subdirectory(tools)
add_subdirectory(srv)
# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()
# 定义工程级变量
set(LAWNWOMER_SERVICE_FILE "${CMAKE_CURRENT_BINARY_DIR}/lawnwomer.service")
set(RK3588_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/rk3588_sdk/toolchainfile.cmake")
# 生成服务文件
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/config/lawnwomer.service.in"
    "${LAWNWOMER_SERVICE_FILE}"
    @ONLY
)
# 安装规则
install(FILES ${LAWNWOMER_SERVICE_FILE} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)