cmake_minimum_required(VERSION 3.10)
project(lawnmower_base)

# 引入全局工具链配置
include(${CMAKE_SOURCE_DIR}/../../rk3588_toolchain.cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  rk3588_sdk
  orbbec_camera
  hesai_lidar
  handsfree_rtk
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lawnmower_base
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs rk3588_sdk orbbec_camera hesai_lidar handsfree_rtk
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/../rk3588_sdk/include
)

# 编译源文件
add_library(lawnmower_base
  src/orbbec_driver.cpp
  src/hesai_driver.cpp
  src/um982_driver.cpp
  src/sensor_data_fusion.cpp
)

add_executable(lawnmower_base_node src/lawnmower_base_node.cpp)
target_link_libraries(lawnmower_base_node
  ${catkin_LIBRARIES}
  lawnmower_base
  ${CMAKE_SOURCE_DIR}/../rk3588_sdk/lib/librk3588_sdk.so
  ${CMAKE_SOURCE_DIR}/../orbbec_camera/SDK/lib/libObSensor.so
  ${CMAKE_SOURCE_DIR}/../hesai_lidar/lib/libhesai_lidar.so
  ${CMAKE_SOURCE_DIR}/../handsfree_rtk/lib/libhandsfree_rtk.so
)

# 安装规则
install(TARGETS lawnmower_base lawnmower_base_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/lawnmower_base/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/ config/ scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)