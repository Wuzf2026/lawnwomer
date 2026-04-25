cmake_minimum_required(VERSION 3.0.2)
project(sensor_data)

# ===================== 核心配置（适配RK3588） =====================
# 调试模式 + RK3588 aarch64架构优化
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -march=armv8-a -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -march=armv8-a -Wall")

# 库文件路径（从build.sh传入的LIB_OUTPUT_DIR）
if(DEFINED LIB_OUTPUT_DIR)
  set(LIB_BASE_DIR ${LIB_OUTPUT_DIR})
else()
  set(LIB_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libs)  # 兜底路径
endif()

# ===================== 查找依赖 =====================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  rk3588_sdk
  handsfree_rtk
  orbbec_camera
  hesai_lidar
)

# 声明catkin包（移除backward_ros相关）
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sensor_data
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs rk3588_sdk handsfree_rtk orbbec_camera hesai_lidar
  DEPENDS
)

# ===================== 头文件路径 =====================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # 依赖包头文件（RK3588适配）
  ${LIB_BASE_DIR}/rk3588_sdk
  ${LIB_BASE_DIR}/handsfree_rtk
  ${LIB_BASE_DIR}/orbbec_camera
  ${LIB_BASE_DIR}/hesai_lidar
)

# ===================== 链接库路径 =====================
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${LIB_BASE_DIR}/rk3588_sdk
  ${LIB_BASE_DIR}/handsfree_rtk
  ${LIB_BASE_DIR}/orbbec_camera
  ${LIB_BASE_DIR}/hesai_lidar
)

# ===================== 编译可执行文件 =====================
# 传感器数据融合节点
add_executable(sensor_data_node
  src/sensor_data_node.cpp
  src/sensor_fusion.cpp  # 传感器融合核心逻辑
)

# 链接依赖库（移除backward_ros）
target_link_libraries(sensor_data_node
  ${catkin_LIBRARIES}
  rk3588_sdk        # RK3588硬件SDK库
  handsfree_rtk     # RTK定位库
  orbbec_camera     # 奥比中光相机库
  hesai_lidar       # 禾赛激光雷达库
)

# ===================== 安装规则 =====================
# 安装可执行文件
install(TARGETS sensor_data_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# ===================== 单独编译调试配置 =====================
# 支持单独编译（catkin_make --cmake-target sensor_data_node）
add_custom_target(sensor_data_debug
  COMMAND gdb ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/sensor_data_node
  DEPENDS sensor_data_node
  COMMENT "启动sensor_data节点调试（GDB）"
)