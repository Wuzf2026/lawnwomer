cmake_minimum_required(VERSION 3.0.2)
project(Hesai_Lidar)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(yaml-cpp REQUIRED)
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
  DEPENDS Boost yaml-cpp
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  include
)
# 核心源文件
file(GLOB_RECURSE SRC_FILES
  src/*.cpp
  src/*.c
)
# 排除测试和示例文件
list(FILTER SRC_FILES EXCLUDE REGEX "test")
list(FILTER SRC_FILES EXCLUDE REGEX "example")
add_executable(hesai_lidar_node ${SRC_FILES})
target_link_libraries(hesai_lidar_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
# 安装规则
install(TARGETS hesai_lidar_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)