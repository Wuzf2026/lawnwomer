cmake_minimum_required(VERSION 3.10.2)
project(sensor_data)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2 -march=armv8-a -fPIC")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED common io)
find_package(yaml-cpp REQUIRED)

# 修复 yaml-cpp 警告
set(yaml-cpp_INCLUDE_DIRS /usr/include/yaml-cpp)
set(yaml-cpp_LIBRARIES /usr/lib/aarch64-linux-gnu/libyaml-cpp.so)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sensor_data_driver sensor_fusion
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs pcl_ros pcl_conversions
  DEPENDS yaml-cpp PCL
)

include_directories(
  include
  include/hesai_lidar
  include/handsfree_rtk
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library(sensor_data_driver
  src/orbbec_driver.cpp
  src/hesai_imu_driver.cpp
  src/hesai_imu_api.cpp
  src/um982_gps_driver.cpp
  src/um982_gps_api.cpp
  src/data_collect.cpp
)
target_link_libraries(sensor_data_driver
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(sensor_fusion src/sensor_fusion.cpp)
target_link_libraries(sensor_fusion
  ${catkin_LIBRARIES}
  sensor_data_driver
  ${YAML_CPP_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(sensor_data_node src/sensor_data_node.cpp)
target_link_libraries(sensor_data_node
  ${catkin_LIBRARIES}
  sensor_data_driver
  sensor_fusion
  ${YAML_CPP_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS sensor_data_driver sensor_fusion sensor_data_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/sensor_data include/hesai_lidar include/handsfree_rtk
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})