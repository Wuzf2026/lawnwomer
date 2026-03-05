cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(multi_driver_ros)

#=======================================
# 通用配置
#=======================================
# 默认编译类型
if (CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
  add_definitions(-O3)
endif()

# C++版本配置
if(($ENV{ROS_DISTRO} STREQUAL "kilted"))
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
elseif(($ENV{ROS_DISTRO} STREQUAL "jazzy"))
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
elseif($ENV{ROS_DISTRO} STREQUAL "humble")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
  add_definitions(-std=c++14)
endif()

add_compile_options(-Wall)

#=======================================
# Hesai 相关配置
#=======================================
# Version information
set(VERSION_MAJOR 2)
set(VERSION_MINOR 0)
set(VERSION_TINY  11)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/Version.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/Version.h"
)

set(WITH_PTCS_USE ON CACHE BOOL "Use Ptcs")
# CUDA 相关配置
if(FIND_CUDA)
  find_package(CUDA)
endif()
if(${CUDA_FOUND})
  project(CommonModules LANGUAGES CXX CUDA)
  set(CUDA_SOURCE_PROPERTY_FORMAT OBJ)
  set(CUDA_SEPARABLE_COMPILATION ON)
  include_directories(${CUDA_INCLUDE_DIRS})
  set(CUDA_PROPAGATE_HOST_FLAGS OFF)
  set(CUDA_NVCC_FLAGS -arch=sm_61;-O3;-G;-g;)#根据具体GPU性能更改算力参数
  link_directories($ENV{CUDA_PATH}/lib/x64)
  add_definitions(-DUSE_CUDA)
  set(FIND_CUDA ON CACHE BOOL "Use Cuda")
else(${CUDA_FOUND})
  MESSAGE(STATUS "cuda not found!")
endif(${CUDA_FOUND})

# 点云类型配置
set(POINT_TYPE XYZI)

#=======================================
# 依赖项配置 (ROS1 核心)
#=======================================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  std_msgs
  message_generation
  geometry_msgs
  rospy
  camera_info_manager
  cv_bridge
  dynamic_reconfigure
  image_geometry
  image_transport
  message_filters
  std_srvs
  tf2
  tf2_ros
  pluginlib
  nodelet
  diagnostic_updater
)

#=======================================
# 消息生成
#=======================================
# Hesai 消息
add_message_files(
  FILES
  "UdpPacket.msg"
  "UdpFrame.msg"
  "LossPacket.msg"
  "Ptp.msg"
  "Firetime.msg"
)

# Orbbec 消息
add_message_files(
  FILES
  DeviceInfo.msg 
  Extrinsics.msg 
  Metadata.msg 
  IMUInfo.msg 
  DeviceStatus.msg
)

# Orbbec 服务
set(SERVICE_FILES
  GetBool.srv
  SetBool.srv
  GetCameraInfo.srv
  GetCameraParams.srv
  GetDeviceInfo.srv
  GetInt32.srv
  SetFilter.srv
  GetString.srv
  SetInt32.srv
  SetString.srv
  SetArrays.srv
)
add_service_files(FILES ${SERVICE_FILES})

# 生成所有消息
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

#=======================================
# Catkin 包配置
#=======================================
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES orbbec_camera
  CATKIN_DEPENDS
    roscpp
    roslib
    sensor_msgs
    std_msgs
    geometry_msgs
    rospy
    camera_info_manager
    cv_bridge
    dynamic_reconfigure
    image_geometry
    image_transport
    message_filters
    std_srvs
    tf2
    tf2_ros
    pluginlib
    nodelet
    diagnostic_updater
)

#=======================================
# 第三方依赖
#=======================================
# Boost
find_package(Boost REQUIRED COMPONENTS thread filesystem system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# YAML-CPP
find_package(yaml-cpp REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# Eigen3
find_package(Eigen3 REQUIRED)

# PkgConfig
find_package(PkgConfig REQUIRED)

# Threads
find_package(Threads REQUIRED)

#=======================================
# Orbbec 特有配置
#=======================================
# Orbbec 编译选项
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fPIC -g3")

# 硬件解码器选项
option(ENABLE_SANITIZER "Enable sanitizer options" OFF)
option(USE_RK_HW_DECODER "Use Rockchip hardware decoder" OFF)
option(USE_NV_HW_DECODER "Use Nvidia hardware decoder" OFF)

# 架构检测
execute_process(COMMAND uname -m OUTPUT_VARIABLE MACHINES)
execute_process(COMMAND getconf LONG_BIT OUTPUT_VARIABLE MACHINES_BIT)
if ((${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "64"))
  set(HOST_PLATFORM "x64")
elseif (${MACHINES} MATCHES "arm")
  set(HOST_PLATFORM "arm32")
elseif ((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "64"))
  set(HOST_PLATFORM "arm64")
elseif ((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "32"))
  set(HOST_PLATFORM "arm32")
endif ()

# Orbbec SDK 路径
set(ORBBEC_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDK/lib/${HOST_PLATFORM})
set(ORBBEC_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SDK/include/)
set(CMAKE_BUILD_RPATH "${CMAKE_BUILD_RPATH}:${ORBBEC_LIBS_DIR}")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH}:${ORBBEC_LIBS_DIR}")

# Nvidia 硬件解码器依赖
if (USE_NV_HW_DECODER)
  set(JETSON_MULTI_MEDIA_API_DIR /usr/src/jetson_multimedia_api)
  set(JETSON_MULTI_MEDIA_API_CLASS_DIR ${JETSON_MULTI_MEDIA_API_DIR}/samples/common/classes)
  set(JETSON_MULTI_MEDIA_API_INCLUDE_DIR ${JETSON_MULTI_MEDIA_API_DIR}/include/)
  set(LIBJPEG8B_INCLUDE_DIR ${JETSON_MULTI_MEDIA_API_INCLUDE_DIR}/libjpeg-8b)
  set(TEGRA_ARMABI /usr/lib/aarch64-linux-gnu/)
  set(NV_LIBRARIES
    -lnvjpeg -lnvbufsurface -lnvbufsurftransform -lyuv -lv4l2
  )
  list(APPEND NV_LIBRARIES
    -L${TEGRA_ARMABI} -L${TEGRA_ARMABI}/tegra)
endif ()

# Rockchip 硬件解码器依赖
if (USE_RK_HW_DECODER)
  pkg_search_module(RK_MPP REQUIRED rockchip_mpp)
  if (NOT RK_MPP_FOUND)
    message(FATAL_ERROR "Rockchip MPP not found")
  endif ()
  pkg_search_module(RGA librga)
  if (NOT RGA_FOUND)
    message(STATUS "Rockchip RGA not found, use libyuv instead")
    add_definitions(-DUSE_LIBYUV)
    add_compile_options(-lyuv)
  endif ()
endif ()

#=======================================
# 包含目录
#=======================================
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${ORBBEC_INCLUDE_DIR}
  include
)

# Hesai 额外包含
include_directories(
  src/manager
  src/msg/ros_msg
  src/msg/rs_msg
  src/utility
)

# Orbbec 硬件解码器包含目录
if (USE_NV_HW_DECODER)
  include_directories(
    ${JETSON_MULTI_MEDIA_API_INCLUDE_DIR}
    ${LIBJPEG8B_INCLUDE_DIR}
  )
endif ()
if (USE_RK_HW_DECODER)
  include_directories(
    ${RK_MPP_INCLUDE_DIRS}
    ${RGA_INCLUDE_DIR}
  )
endif ()

#=======================================
# 子目录构建 (Hesai SDK)
#=======================================
set(DISENABLE_TEST_CC ON CACHE BOOL "DISENABLE_TEST_CC")  
add_subdirectory(src/driver/HesaiLidar_SDK_2.0)

#=======================================
# 库定义
#=======================================
# Hesai 核心库
set(HESAI_LIB_TARGETS 
  hesai_sdk_lib
  log_lib
  lidar_lib
  ptcClient_lib
  source_lib
  platutils_lib
  serialClient_lib
  udpParser_lib
)
if(${CUDA_FOUND})
  list(APPEND HESAI_LIB_TARGETS udpParserGpu_lib)
endif()

# Orbbec 链接库
set(ORBBEC_LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  -lOrbbecSDK
  -L${ORBBEC_LIBS_DIR}
  Threads::Threads
  -lrt
  -ldw # for stack trace
  yaml-cpp
)
if (USE_RK_HW_DECODER)
  list(APPEND ORBBEC_LINK_LIBRARIES
    ${RK_MPP_LIBRARIES}
    ${RGA_LIBRARIES}
  )
elseif (USE_NV_HW_DECODER)
  list(APPEND ORBBEC_LINK_LIBRARIES
    ${NV_LIBRARIES}
  )
endif ()

#=======================================
# 可执行文件构建
#=======================================
# 1. Hesai ROS 节点
if(${CUDA_FOUND})
  add_executable(hesai_ros_driver_node
    node/hesai_ros_driver_node.cu
    src/manager/node_manager.cc
  )
else()
  add_executable(hesai_ros_driver_node
    node/hesai_ros_driver_node.cc
    src/manager/node_manager.cc
  )
endif()
target_link_libraries(hesai_ros_driver_node                   
  ${YAML_CPP_LIBRARIES}
  ${Boost_LIBRARIES}
  ${HESAI_LIB_TARGETS}
  ${catkin_LIBRARIES}
)              

# 2. Orbbec 库
set(ORBBEC_SOURCE_FILES
  src/d2c_viewer.cpp
  src/ob_camera_node.cpp
  src/ob_lidar_node.cpp
  src/ob_camera_node_driver.cpp
  src/ros_sensor.cpp
  src/ros_service.cpp
  src/utils.cpp
  src/ros_setup.cpp
  src/jpeg_decoder.cpp
)
if (USE_RK_HW_DECODER)
  add_definitions(-DUSE_RK_HW_DECODER)
  list(APPEND ORBBEC_SOURCE_FILES src/rk_mpp_decoder.cpp)
endif ()
if (USE_NV_HW_DECODER)
  add_definitions(-DUSE_NV_HW_DECODER)
  list(APPEND ORBBEC_SOURCE_FILES src/jetson_nv_decoder.cpp)
  # jetson_multimedia_api 源文件
  list(APPEND ORBBEC_SOURCE_FILES
    ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvBuffer.cpp
    ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvElement.cpp
    ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvElementProfiler.cpp
    ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvJpegDecoder.cpp
    ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvJpegEncoder.cpp
    ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvLogging.cpp
    ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvUtils.cpp
    ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvV4l2Element.cpp
    ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvV4l2ElementPlane.cpp
    ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvVideoDecoder.cpp
    ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvVideoEncoder.cpp
    ${JETSON_MULTI_MEDIA_API_CLASS_DIR}/NvBufSurface.cpp
  )
endif ()

# Orbbec 核心库
add_library(orbbec_camera ${ORBBEC_SOURCE_FILES})
target_link_libraries(orbbec_camera PRIVATE ${ORBBEC_LINK_LIBRARIES})
add_dependencies(orbbec_camera ${PROJECT_NAME}_generate_messages_cpp)

# Orbbec Nodelet
add_library(orbbec_camera_nodelet ${ORBBEC_SOURCE_FILES} src/ros_nodelet.cpp)
target_link_libraries(orbbec_camera_nodelet ${ORBBEC_LINK_LIBRARIES})
add_dependencies(orbbec_camera_nodelet ${PROJECT_NAME}_generate_messages_cpp)

# Orbbec 可执行文件宏
macro(add_orbbec_executable TARGET SOURCES)
  add_executable(${TARGET} ${SOURCES})
  target_link_libraries(${TARGET} ${ORBBEC_LINK_LIBRARIES} orbbec_camera)
endmacro()

# Orbbec 节点
add_orbbec_executable(list_devices_node src/list_devices_node.cpp)
add_orbbec_executable(list_depth_work_mode_node src/list_depth_work_mode.cpp)
add_orbbec_executable(list_camera_profile_mode_node src/list_camera_profile_mode.cpp)
add_orbbec_executable(set_device_ip src/set_device_ip.cpp)
add_orbbec_executable(orbbec_camera_node src/main.cpp)
add_orbbec_executable(service_benchmark_node scripts/service_benchmark_node.cpp)
add_orbbec_executable(image_sync_example_node examples/multi_camera_time_sync/image_sync_example_node.cpp)

#=======================================
# 安装配置
#=======================================
# 1. 可执行文件
install(TARGETS 
  hesai_ros_driver_node
  orbbec_camera_node
  list_devices_node
  list_depth_work_mode_node
  list_camera_profile_mode_node
  set_device_ip
  service_benchmark_node
  image_sync_example_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 2. 库文件
install(TARGETS 
  ${HESAI_LIB_TARGETS}
  orbbec_camera
  orbbec_camera_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 3. 脚本文件 (Python)
catkin_install_python(PROGRAMS
  scripts/common_benchmark_node.py
  scripts/service_benchmark_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 4. 配置文件/启动文件/Rviz
install(DIRECTORY
  launch
  rviz
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 5. Orbbec SDK 文件
install(DIRECTORY SDK/lib/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/SDK/lib)
install(DIRECTORY include DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/)
install(DIRECTORY ${ORBBEC_INCLUDE_DIR} DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/)
install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES LICENSE DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY ${ORBBEC_LIBS_DIR}/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/
  FILES_MATCHING PATTERN "*.so"
  PATTERN "*.so.*"
)

# 6. Udev 规则 (打包时安装)
if(DEFINED ENV{BUILDING_PACKAGE})
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/99-obsensor-ros1-libusb.rules
    DESTINATION /etc/udev/rules.d
  )
endif()

# 7. 复制 Orbbec 扩展文件
add_custom_target(copy_orbbec_files ALL
  COMMAND ${CMAKE_COMMAND} -E copy_directory  ${ORBBEC_LIBS_DIR}/extensions/  ${CATKIN_DEVEL_PREFIX}/lib/extensions/
)