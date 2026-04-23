cmake_minimum_required(VERSION 3.0.2)
project(lawnmower_base)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 查找ROS依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  # 若用到其他ROS包，补充在此
)

# 2. 查找Orbbec SDK（解决ObSensor.hpp找不到问题）
# 方式1：手动指定SDK路径（若SDK未全局安装）
set(OBSENSOR_SDK_PATH "/usr/local/obsensor-sdk" CACHE PATH "Path to Orbbec Sensor SDK")
include_directories(${OBSENSOR_SDK_PATH}/include)
link_directories(${OBSENSOR_SDK_PATH}/lib)

# 方式2：自动查找（若SDK已全局安装）
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(OBSENSOR REQUIRED obsensor)

# 3. 声明catkin包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs
  DEPENDS OBSENSOR_SDK  # 声明第三方依赖
)

# 4. 包含头文件路径（解决自定义头文件找不到问题）
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OBSENSOR_SDK_PATH}/include  # 显式添加Orbbec SDK头文件路径
)

# 5. 编译可执行文件
add_executable(${PROJECT_NAME}_node
  src/lawnmower_base_node.cpp
  src/rtk_driver.cpp
  src/camera_driver.cpp
  src/lidar_driver.cpp
)

# 6. 链接库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  # 链接Orbbec SDK库
  ${OBSENSOR_SDK_PATH}/lib/libobsensor.so
  # 若有其他依赖库（如串口、Lidar库），补充在此
)

# 7. 安装规则（可选，规范部署）
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)