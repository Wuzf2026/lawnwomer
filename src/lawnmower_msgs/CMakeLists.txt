cmake_minimum_required(VERSION 3.0.2)
project(lawnmower_msgs)

# 引入全局RK3588工具链
include(${CMAKE_SOURCE_DIR}/rk3588_toolchain.cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

# 声明服务文件
add_service_files(
  FILES
  LawnmowerStatus.srv  # 替代原lawnwomer.service，命名规范化
)

# 生成消息依赖
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)