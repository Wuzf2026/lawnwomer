cmake_minimum_required(VERSION 3.10.2)
project(tools)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2 -march=armv8-a")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_data
)

find_package(yaml-cpp REQUIRED)
set(yaml-cpp_INCLUDE_DIRS /usr/include/yaml-cpp)
set(yaml-cpp_LIBRARIES /usr/lib/aarch64-linux-gnu/libyaml-cpp.so)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES debug_tool
  CATKIN_DEPENDS roscpp std_msgs sensor_data
  DEPENDS yaml-cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_library(debug_tool src/debug_tool.cpp)
target_link_libraries(debug_tool
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

install(TARGETS debug_tool
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/tools DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")
install(PROGRAMS scripts/*.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})