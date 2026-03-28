cmake_minimum_required(VERSION 3.10)project(hesai_lidar)# 引入RK3588工具链（禁用CUDA，RK3588不兼容）include(${CMAKE_SOURCE_DIR}/../../rk3588_toolchain.cmake)set(FIND_CUDA OFF)  # 强制禁用CUDA# ROS1 Noetic依赖find_package(catkin REQUIRED COMPONENTS    roscpp    std_msgs    sensor_msgs    message_generation    rk3588_sdk)# 生成ROS消息（仅ROS1）add_message_files(    FILES    Firetime.msg    LossPacket.msg    Ptp.msg    UdpPacket.msg    UdpFrame.msg)generate_messages(    DEPENDENCIES    std_msgs    sensor_msgs)# 导出配置catkin_package(    INCLUDE_DIRS src/driver src/utility    LIBRARIES hesai_lidar_lib    CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime rk3588_sdk)# 包含目录（ARM64适配）include_directories(    ${catkin_INCLUDE_DIRS}    src/utility    src/manager    src/driver    src/driver/HesaiLidar_SDK_2.0    src/driver/HesaiLidar_SDK_2.0/libhesai    ${CMAKE_SOURCE_DIR}/../rk3588_sdk/include)# 编译SDK核心库（JT128适配）add_subdirectory(src/driver/HesaiLidar_SDK_2.0)# 编译禾赛雷达节点add_executable(hesai_lidar_node    src/driver/hesai_driver.cpp    src/manager/lidar_manager.cpp    src/utility/lidar_utils.cpp)# 链接依赖（ARM64 + 禁用CUDA）target_link_libraries(hesai_lidar_node    ${catkin_LIBRARIES}    rk3588_sdk    hesai_sdk_lib    lidar_lib    pthread)# 编译选项（ARM64优化）target_compile_options(hesai_lidar_node PRIVATE    -march=armv8.2-a    -O2    -Wall    -std=c++14)# 安装规则install(TARGETS hesai_lidar_node    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})install(DIRECTORY config launch rviz    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})cmake_minimum_required(VERSION 3.10)project(hesai_lidar)# 引入RK3588工具链（禁用CUDA，RK3588不兼容）include(${CMAKE_SOURCE_DIR}/../../rk3588_toolchain.cmake)set(FIND_CUDA OFF)  # 强制禁用CUDA# ROS1 Noetic依赖find_package(catkin REQUIRED COMPONENTS    roscpp    std_msgs    sensor_msgs    message_generation    rk3588_sdk)# 生成ROS消息（仅ROS1）add_message_files(    FILES    Firetime.msg    LossPacket.msg    Ptp.msg    UdpPacket.msg    UdpFrame.msg)generate_messages(    DEPENDENCIES    std_msgs    sensor_msgs)# 导出配置catkin_package(    INCLUDE_DIRS src/driver src/utility    LIBRARIES hesai_lidar_lib    CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime rk3588_sdk)# 包含目录（ARM64适配）include_directories(    ${catkin_INCLUDE_DIRS}    src/utility    src/manager    src/driver    src/driver/HesaiLidar_SDK_2.0    src/driver/HesaiLidar_SDK_2.0/libhesai    ${CMAKE_SOURCE_DIR}/../rk3588_sdk/include)# 编译SDK核心库（JT128适配）add_subdirectory(src/driver/HesaiLidar_SDK_2.0)# 编译禾赛雷达节点add_executable(hesai_lidar_node    src/driver/hesai_driver.cpp    src/manager/lidar_manager.cpp    src/utility/lidar_utils.cpp)# 链接依赖（ARM64 + 禁用CUDA）target_link_libraries(hesai_lidar_node    ${catkin_LIBRARIES}    rk3588_sdk    hesai_sdk_lib    lidar_lib    pthread)# 编译选项（ARM64优化）target_compile_options(hesai_lidar_node PRIVATE    -march=armv8.2-a    -O2    -Wall    -std=c++14)# 安装规则install(TARGETS hesai_lidar_node    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})install(DIRECTORY config launch rviz    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}cmake_minimum_required(VERSION 3.10)
project(hesai_lidar)

# 引入RK3588工具链（禁用CUDA，RK3588不兼容）
include(${CMAKE_SOURCE_DIR}/../../rk3588_toolchain.cmake)
set(FIND_CUDA OFF)  # 强制禁用CUDA

# ROS1 Noetic依赖
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    message_generation
    rk3588_sdk
)

# 生成ROS消息（仅ROS1）
add_message_files(
    FILES
    Firetime.msg
    LossPacket.msg
    Ptp.msg
    UdpPacket.msg
    UdpFrame.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
)

# 导出配置
catkin_package(
    INCLUDE_DIRS src/driver src/utility
    LIBRARIES hesai_lidar_lib
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime rk3588_sdk
)

# 包含目录（ARM64适配）
include_directories(
    ${catkin_INCLUDE_DIRS}
    src/utility
    src/manager
    src/driver
    src/driver/HesaiLidar_SDK_2.0
    src/driver/HesaiLidar_SDK_2.0/libhesai
    ${CMAKE_SOURCE_DIR}/../rk3588_sdk/include
)

# 编译SDK核心库（JT128适配）
add_subdirectory(src/driver/HesaiLidar_SDK_2.0)

# 编译禾赛雷达节点
add_executable(hesai_lidar_node
    src/driver/hesai_driver.cpp
    src/manager/lidar_manager.cpp
    src/utility/lidar_utils.cpp
)

# 链接依赖（ARM64 + 禁用CUDA）
target_link_libraries(hesai_lidar_node
    ${catkin_LIBRARIES}
    rk3588_sdk
    hesai_sdk_lib
    lidar_lib
    pthread
)

# 编译选项（ARM64优化）
target_compile_options(hesai_lidar_node PRIVATE
    -march=armv8.2-a
    -O2
    -Wall
    -std=c++14
)

# 安装规则
install(TARGETS hesai_lidar_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)