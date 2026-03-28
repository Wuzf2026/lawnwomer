cmake_minimum_required(VERSION 3.10.2)
project(orbbec_camera)

# ROS1 Noetic + ARM64编译配置
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8.2-a -O2 -Wall -fPIC")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8.2-a -O2 -Wall -fPIC")

# 启用RK3588硬件解码（适配Gemini335）
option(USE_RK_HW_DECODER "Use Rockchip hardware decoder" ON)
option(USE_NV_HW_DECODER "Use Nvidia hardware decoder" OFF)

# 检测RK3588 ARM64架构
execute_process(COMMAND uname -m OUTPUT_VARIABLE MACHINES)
execute_process(COMMAND getconf LONG_BIT OUTPUT_VARIABLE MACHINES_BIT)
if ((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "64"))
  set(HOST_PLATFORM "arm64")
endif ()

# ROS1依赖
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    cv_bridge
    image_transport
    camera_info_manager
    tf2_ros
    nodelet
    rk3588_sdk  # 依赖RK3588底层SDK
)

# 导出配置
catkin_package(
    INCLUDE_DIRS include SDK/include
    LIBRARIES orbbec_camera
    CATKIN_DEPENDS roscpp sensor_msgs cv_bridge rk3588_sdk
)

# 路径配置（适配RK3588 ARM64）
set(ORBBEC_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDK/lib/${HOST_PLATFORM})
set(ORBBEC_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDK/include/)
link_directories(${ORBBEC_LIBS_DIR})

# 包含目录
include_directories(
    include
    ${ORBBEC_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/../rk3588_sdk/include  # RK3588 SDK头文件
)

# 编译核心节点（Gemini335适配）
add_executable(ob_camera_node
    src/ob_camera_node.cpp
    src/ob_camera_node_driver.cpp
    src/rk_mpp_decoder.cpp  # RK硬件解码
    src/jpeg_decoder.cpp
    src/utils.cpp
    src/ros_sensor.cpp
)

# 链接依赖（ARM64 + RK3588 SDK）
target_link_libraries(ob_camera_node
    ${catkin_LIBRARIES}
    rk3588_sdk
    ${ORBBEC_LIBS_DIR}/libObTypes.so
    ${ORBBEC_LIBS_DIR}/libObDevice.so
    ${ORBBEC_LIBS_DIR}/libObSensor.so
)

# 安装规则
install(TARGETS ob_camera_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ config/ rviz/ SDK/lib/arm64/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)