cmake_minimum_required(VERSION 3.0.2)
project(rk3588_sdk)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy serial
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
file(GLOB_RECURSE PROJECT_SOURCES src/*.cpp src/*.c)
add_library(${PROJECT_NAME} ${PROJECT_SOURCES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/rk3588/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)