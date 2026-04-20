cmake_minimum_required(VERSION 3.10)
project(lawnmower_base)
# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 查找依赖
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf
    image_transport
    cv_bridge
    pcl_ros
    serial
)
# 查找PCL库
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# 查找OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# 包含头文件路径
include_directories(
    ${catkin_INCLUDE_DIRS}
    include
    include/${PROJECT_NAME}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
# 添加可执行文件
add_executable(lawnmower_base_node
    src/lawnmower_base_node.cpp
    src/camera_driver.cpp
    src/lidar_driver.cpp
    src/rtk_driver.cpp
)
# 添加依赖
add_dependencies(lawnmower_base_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
# 链接库
target_link_libraries(lawnmower_base_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    -lobsensor  # Orbbec SDK库
)
# 安装配置
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES lawnmower_base
    CATKIN_DEPENDS roscpp 
)
# 安装文件
install(DIRECTORY
    config
    launch
    rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)