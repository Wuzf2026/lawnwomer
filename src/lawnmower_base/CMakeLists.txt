cmake_minimum_required(VERSION 3.0.2)
project(lawnmower_base)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf2_ros
  handsfree_rtk
  orbbec_camera
  Hesai_Lidar
)
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs tf2_ros handsfree_rtk orbbec_camera Hesai_Lidar
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(lawnmower_base_node src/lawnmower_base.cpp)
target_link_libraries(lawnmower_base_node
  ${catkin_LIBRARIES}
)
# 安装规则
install(TARGETS lawnmower_base_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)