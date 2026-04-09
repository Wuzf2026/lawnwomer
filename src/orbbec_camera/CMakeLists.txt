cmake_minimum_required(VERSION 3.0.2)
project(orbbec_camera)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  cv_bridge
)
find_package(libusb-1.0 REQUIRED)
find_package(OpenCV REQUIRED)
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs image_transport cv_bridge
  DEPENDS libusb-1.0 OpenCV
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIRS}
)
# 核心源文件
file(GLOB_RECURSE SRC_FILES
  src/*.cpp
  src/*.c
)
# 排除测试文件
list(FILTER SRC_FILES EXCLUDE REGEX "test")
list(FILTER SRC_FILES EXCLUDE REGEX "example")
add_executable(orbbec_camera_node ${SRC_FILES})
target_link_libraries(orbbec_camera_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${LIBUSB_LIBRARIES}
)
# 安装规则
install(TARGETS orbbec_camera_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)