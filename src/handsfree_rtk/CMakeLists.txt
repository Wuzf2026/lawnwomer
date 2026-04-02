cmake_minimum_required(VERSION 3.0.2)
project(handsfree_rtk)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  serial
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs serial
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE PROJECT_SOURCES src/*.cpp src/*.c)
add_executable(${PROJECT_NAME}_node ${PROJECT_SOURCES})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)