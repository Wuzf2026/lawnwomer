cmake_minimum_required(VERSION 3.10.2)
project(rk3588_sdk)

# 1. 查找ROS依赖（仅保留实际用到的）
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# 2. ARM64编译选项（RK3588适配）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -march=armv8.2-a -O2 -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -march=armv8.2-a -O2 -Wall")

# 3. 头文件目录（包含CAN/GPIO驱动所需）
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/aarch64-linux-gnu
  /usr/include/linux  # CAN驱动头文件
)

# 4. 编译动态库（包含所有驱动源码）
add_library(${PROJECT_NAME}
  src/can.c
  src/gpio.c
  src/uart.c
)

# 5. 链接系统库（解决ioctl/socket依赖）
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  pthread
  rt
  socket
)

# 6. 安装库（补全DESTINATION，解决第一个CMake错误）
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}  # ROS标准库路径
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 7. 安装头文件（补全DESTINATION）
install(DIRECTORY launch/ config/ scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 8. catkin包配置（严格匹配find_package的依赖）
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs  # 必须包含find_package的所有依赖
)
