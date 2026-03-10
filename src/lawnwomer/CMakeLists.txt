cmake_minimum_required(VERSION 3.0.2)
project(lawnwomer)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros
  costmap_2d nav_core pluginlib move_base amcl robot_localization
)

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(lawnwomer_node src/lawnwomer_node.cpp)
target_link_libraries(lawnwomer_node ${catkin_LIBRARIES})

add_library(lawnwomer_planner src/planner/lawnwomer_planner.cpp)
target_link_libraries(lawnwomer_planner ${catkin_LIBRARIES})

install(TARGETS lawnwomer_node lawnwomer_planner
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY launch config rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})