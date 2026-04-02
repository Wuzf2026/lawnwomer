cmake_minimum_required(VERSION 3.0.2)
project(lawnmower)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  serial
  yaml-cpp
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs cv_bridge image_transport serial yaml-cpp
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/orbbec_gemini335_driver.cpp
  src/hesai_jt128_driver.cpp
  src/um982_rtk_driver.cpp
  src/api_server.cpp
  src/lawnmower_core.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/lawnmower_core.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)