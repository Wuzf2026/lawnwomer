cmake_minimum_required(VERSION 3.10.2)
project(ultrasonic_sensor)

# ===================== 基础配置 =====================
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2 -fPIC")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O2 -fPIC")

# ===================== 依赖查找 =====================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  rk3588_sdk
)

# ===================== Catkin包导出 =====================
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ultrasonic_sensor
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs rk3588_sdk
  # DEPENDS system_lib
)

# ===================== 头文件目录 =====================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ===================== 编译核心库 =====================
add_library(${PROJECT_NAME}
  src/ultrasonic_driver.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  rk3588_sdk  # 链接RK3588 SDK
)

# ===================== 编译节点 =====================
add_executable(${PROJECT_NAME}_node
  src/ultrasonic_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# ===================== 安装规则 =====================
# 安装库
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# 安装节点
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# 安装launch/config/scripts
install(DIRECTORY launch/ config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装Python测试脚本
catkin_install_python(PROGRAMS scripts/ultrasonic_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
