cmake_minimum_required(VERSION 3.10.2)
project(motor_control)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  serial
  message_generation
)

add_message_files(
  FILES
  WheelSpeed.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs serial message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(motor_node
  src/motor_node.cpp
  src/motor_driver.cpp
  src/serial_port.cpp
  src/serial_protocol.cpp
)

add_dependencies(motor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(motor_node
  ${catkin_LIBRARIES}
)

install(TARGETS motor_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)