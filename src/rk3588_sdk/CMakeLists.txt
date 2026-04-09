cmake_minimum_required(VERSION 3.0.2)
project(rk3588_sdk)

# 设置编译选项
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -fPIC")

# 查找ROS依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# 系统依赖（CAN驱动）
find_package(PkgConfig REQUIRED)
pkg_check_modules(SOCKETCAN REQUIRED libsocketcan)

# Catkin包配置（关键：导出头文件和库）
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}  # 库名与项目名一致
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS SOCKETCAN
)

# 包含头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SOCKETCAN_INCLUDE_DIRS}
)

# 编译共享库（必须指定源码文件）
add_library(${PROJECT_NAME}
  src/can.cpp  # 核心实现文件
)

# 链接库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${SOCKETCAN_LIBRARIES}
  pthread
)

# 安装库文件（关键：确保库被安装到devel/lib）
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
