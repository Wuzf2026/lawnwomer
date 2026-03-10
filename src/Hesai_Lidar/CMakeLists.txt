cmake_minimum_required(VERSION 3.0.2)
project(hesai_lidar)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs tf2_ros)
catkin_package(CATKIN_DEPENDS roscpp sensor_msgs tf2_ros)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(hesai_lidar_node src/hesai_lidar_node.cpp)
target_link_libraries(hesai_lidar_node ${catkin_LIBRARIES})

install(TARGETS hesai_lidar_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})