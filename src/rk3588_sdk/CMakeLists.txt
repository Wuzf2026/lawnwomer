cmake_minimum_required(VERSION 3.10)
project(rk3588_sdk)

# 引入RK3588 ARM64工具链
include(${CMAKE_SOURCE_DIR}/../../rk3588_toolchain.cmake)

# ROS1 Noetic依赖
find_package(catkin REQUIRED COMPONENTS
    roscpp
    serial
    std_msgs
)

# 导出库和头文件
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES rk3588_sdk
    CATKIN_DEPENDS roscpp serial std_msgs
    DEPENDS system_lib
)

# 包含目录（ARM64系统头文件+ROS头文件）
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    /usr/include/aarch64-linux-gnu  # RK3588系统头文件
)

# 编译核心驱动（串口+485+GPIO）
add_library(${PROJECT_NAME}
    src/rk3588_serial.cpp
    src/rk3588_gpio.cpp
    src/uart_485.c
    src/gpio_control.cpp
    src/uart_driver.cpp
)

# 链接依赖（ARM64串口/485库）
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${SERIAL_LIBRARIES}
    pthread  # 485通信线程依赖
)

# 编译选项（ARM64优化）
target_compile_options(${PROJECT_NAME} PRIVATE
    -march=armv8.2-a
    -O2
    -Wall
    -fPIC
)

# 安装规则
install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)