cmake_minimum_required(VERSION 3.0.2)
project(handsfree_rtk)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs nav_msgs tf2_ros
)
catkin_package(CATKIN_DEPENDS roscpp sensor_msgs nav_msgs tf2_ros)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(handsfree_rtk_node src/handsfree_rtk_node.cpp)
target_link_libraries(handsfree_rtk_node ${catkin_LIBRARIES})

install(TARGETS handsfree_rtk_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})