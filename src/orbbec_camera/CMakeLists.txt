cmake_minimum_required(VERSION 3.0.2)
project(orbbec_camera)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs image_transport camera_info_manager
)
catkin_package(CATKIN_DEPENDS roscpp sensor_msgs image_transport camera_info_manager)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(orbbec_camera_node src/orbbec_camera_node.cpp)
target_link_libraries(orbbec_camera_node ${catkin_LIBRARIES})

install(TARGETS orbbec_camera_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch cfg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})