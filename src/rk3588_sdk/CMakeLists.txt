cmake_minimum_required(VERSION 3.0.2)
project(rk3588_sdk)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
)
catkin_package(
  CATKIN_DEPENDS roscpp serial
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)
# 核心源文件
file(GLOB_RECURSE SRC_FILES
  src/*.cpp
  src/*.c
)
# 排除测试文件
list(FILTER SRC_FILES EXCLUDE REGEX "test")
add_library(rk3588_sdk ${SRC_FILES})
target_link_libraries(rk3588_sdk
  ${catkin_LIBRARIES}
)
# 安装规则
install(TARGETS rk3588_sdk
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY lib/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)