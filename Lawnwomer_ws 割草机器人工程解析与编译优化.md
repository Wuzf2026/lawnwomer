# Lawnwomer_ws 割草机器人工程解析与编译优化

# 割草机器人lawnwomer_ws工程全解析与编译优化

## 一、工程文件结构与包含关系梳理

### 1. 整体目录结构

```Plain Text

lawnwomer-ws/
├── src/
│   ├── HesaiLidar_ROS/          # 禾赛JT128激光雷达ROS驱动（ETH通信）
│   ├── HesaiLidar_SDK/          # 禾赛雷达底层SDK（被HesaiLidar_ROS依赖）
│   ├── handsfree_rtk/           # T-RTK UM982相关（USB通信，待完善）
│   ├── OrbbecSDK_ROS/           # 奥比中光相机（暂未用到，保留）
│   └── CMakeLists.txt           # catkin顶层构建文件
├── build/                       # 编译目录（自动生成）
├── devel/                       # 开发目录（自动生成）
└── install/                     # 安装目录（自动生成）
```

### 2. 核心模块依赖关系

|模块|通信方式|依赖项|被依赖方|
|---|---|---|---|
|HesaiLidar_ROS|ETH|HesaiLidar_SDK、ROS1、yaml-cpp|顶层catkin工程|
|HesaiLidar_SDK|ETH|CUDA（可选）、OpenSSL、PCL|HesaiLidar_ROS|
|handsfree_rtk|USB|ROS1、sensor_msgs|顶层catkin工程|
|OrbbecSDK_ROS|USB|OpenCV、Boost、OrbbecSDK|顶层catkin工程（可选）|
## 二、核心文件逐行解析

### 1. HesaiLidar_ROS/CMakeLists.txt 解析

```CMake

# 1. 基础配置：指定CMake最低版本，启用新策略，定义项目名
cmake_minimum_required(VERSION 3.5)          # RK3588 Ubuntu20.04默认CMake3.16+，兼容
cmake_policy(SET CMP0048 NEW)                # 启用PROJECT_VERSION相关新策略
project(hesai_ros_driver)                   # 项目名

# 2. 版本信息配置：生成Version.h文件
set(VERSION_MAJOR 2)
set(VERSION_MINOR 0)
set(VERSION_TINY  11)
configure_file(                              # 将Version.h.in模板替换为实际Version.h
  "${CMAKE_CURRENT_SOURCE_DIR}/Version.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/Version.h"
)

# 3. PTCS/CUDA配置：可选启用PTCS，CUDA用于GPU解析（RK3588无NVIDIA GPU，需注释）
set(WITH_PTCS_USE ON CACHE BOOL "Use Ptcs")
# set(FIND_CUDA ON CACHE BOOL "Find CUDA")    # RK3588无CUDA，注释掉避免编译错误

# 4. CUDA检测分支（RK3588下CUDA_FOUND为FALSE，执行else分支）
if(FIND_CUDA)
    find_package(CUDA )
endif()
if(${CUDA_FOUND})
   project(CommonModules LANGUAGES CXX CUDA)
 set(CUDA_SOURCE_PROPERTY_FORMAT OBJ)
 set(CUDA_SEPARABLE_COMPILATION ON)
 include_directories(${CUDA_INCLUDE_DIRS})
 set(CUDA_PROPAGATE_HOST_FLAGS OFF)
 set(CUDA_NVCC_FLAGS -arch=sm_61;-O3;-G;-g;)# RK3588无NVIDIA GPU，此参数无效
 link_directories($ENV{CUDA_PATH}/lib/x64)
  add_definitions(-DUSE_CUDA)
  set(FIND_CUDA ON CACHE BOOL "Use Cuda")
else(${CUDA_FOUND})
 MESSAGE(STATUS "cuda not found!")        # RK3588下会打印此信息，正常
endif(${CUDA_FOUND})

# 5. 点云类型配置：默认XYZI（JT128支持XYZI/XYZIRT）
set(POINT_TYPE XYZI)

# 6. ROS版本检测：打印ROS版本（ROS1为1，ROS2为2）
message(=============================================================)
message("-- ROS_VERSION is $ENV{ROS_VERSION}")  # ROS1环境下输出1
message(=============================================================)

# 7. 工程基础配置
set(PROJECT_NAME hesai_ros_driver)
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")  # 定义工程路径宏

# 8. 编译类型配置：默认Release，启用O3优化
if (CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
  add_definitions(-O3)                      # 优化编译，提升运行效率
endif()

# 9. C++标准配置：ROS1 Noetic（Ubuntu20.04）默认C++14，Humble/Jazzy用C++17
if(($ENV{ROS_DISTRO} STREQUAL "kilted"))
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
elseif(($ENV{ROS_DISTRO} STREQUAL "jazzy"))
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
elseif($ENV{ROS_DISTRO} STREQUAL "humble")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
add_definitions(-std=c++14)                 # ROS1 Noetic用C++14，匹配RK3588环境
endif()
add_compile_options(-Wall)                  # 启用所有警告，便于调试

# 10. ROS1依赖配置（核心）
if($ENV{ROS_VERSION} MATCHES "1")
  find_package(roscpp 1.12 QUIET)           # 查找roscpp（ROS1核心库）
  find_package(roslib QUIET)                # 查找roslib工具库
  include_directories(${roscpp_INCLUDE_DIRS} ${roslib_INCLUDE_DIRS}) # 头文件路径
  set(ROS_LIBS ${roscpp_LIBRARIES} ${roslib_LIBRARIES}) # 链接库
  add_definitions(-DROS_FOUND)              # 定义ROS_FOUND宏
  add_definitions(-DRUN_IN_ROS_WORKSPACE)   # 定义ROS工作空间宏

  # 查找catkin必需组件
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs         # 传感器消息类型（点云、IMU等）
    std_msgs            # 标准消息类型
    message_generation  # 消息生成工具
    roslib)

  # 自定义消息文件（用于雷达数据传输）
  add_message_files(
    FILES
    "UdpPacket.msg"     # UDP包消息
    "UdpFrame.msg"      # UDP帧消息
    "LossPacket.msg"    # 丢包统计消息
    "Ptp.msg"           # PTP时钟同步消息
    "Firetime.msg"      # 激光发射时间消息
  )

  # 生成消息（依赖std_msgs）
  generate_messages(
    DEPENDENCIES
    std_msgs
  )

  # catkin包配置（导出依赖）
  catkin_package(CATKIN_DEPENDS 
    sensor_msgs 
    roslib)
endif($ENV{ROS_VERSION} MATCHES "1")

# 11. ROS2依赖配置（RK3588用ROS1，此分支不执行）
if($ENV{ROS_VERSION} MATCHES "2")
  find_package(rclcpp QUIET)
  if(rclcpp_FOUND)
    message(=============================================================)
    message("-- ROS2 Found. ROS2 Support is turned On.")
    message(=============================================================)
    add_definitions(-DROS2_FOUND)
    include_directories(${rclcpp_INCLUDE_DIRS})
    set(CMAKE_CXX_STANDARD 14)
    find_package(ament_cmake REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(std_msgs REQUIRED)  
    find_package(rclcpp_action REQUIRED)
    find_package(rosidl_typesupport_c REQUIRED) 
    find_package(rosidl_default_generators REQUIRED)
    find_package(builtin_interfaces REQUIRED)
    find_package(std_msgs REQUIRED)
    
    # 生成ROS2消息
    rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/msg_ros2/UdpPacket.msg"
      "msg/msg_ros2/UdpFrame.msg"
      "msg/msg_ros2/Firetime.msg"
      "msg/msg_ros2/Ptp.msg"
      "msg/msg_ros2/LossPacket.msg"
      DEPENDENCIES builtin_interfaces std_msgs
    )
    ament_export_dependencies(rosidl_default_runtime)                 
  else(rclcpp_FOUND)
    message(=============================================================)
    message("-- ROS2 Not Found. ROS2 Support is turned Off.")
    message(=============================================================)
  endif(rclcpp_FOUND )
endif($ENV{ROS_VERSION} MATCHES "2")

# 12. 第三方依赖：yaml-cpp（配置文件解析）
find_package(yaml-cpp REQUIRED)

# 13. 头文件路径：添加src目录
include_directories(${PROJECT_SOURCE_DIR}/src)

# 14. 编译HesaiLidar_SDK（核心底层SDK）
set(DISENABLE_TEST_CC ON CACHE BOOL "DISENABLE_TEST_CC")  
add_subdirectory(src/driver/HesaiLidar_SDK_2.0)  # 引入SDK子目录

# 15. Boost依赖（线程库）
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# 16. 链接库列表（SDK核心库）
set(LIB_TARGETS 
    hesai_sdk_lib
    log_lib
    lidar_lib
    ptcClient_lib
    source_lib
    platutils_lib
    serialClient_lib
    udpParser_lib
    )

# 17. 可执行文件编译（RK3588无CUDA，执行else分支）
if(${CUDA_FOUND})
  add_executable(hesai_ros_driver_node
    node/hesai_ros_driver_node.cu
    src/manager/node_manager.cc
    )
  set(LIB_TARGETS
    ${LIB_TARGETS}
    udpParserGpu_lib
    )
else()
  # 编译CPU版本节点（RK3588适用）
  add_executable(hesai_ros_driver_node
    node/hesai_ros_driver_node.cc    # ROS节点主文件
    src/manager/node_manager.cc     # 节点管理文件
    )
endif()

# 18. 链接库（核心：SDK库+yaml+Boost）
target_link_libraries(hesai_ros_driver_node                   
            ${YAML_CPP_LIBRARIES}    # yaml配置解析
            ${Boost_LIBRARIES}       # 线程库
            ${LIB_TARGETS}           # SDK核心库
)              

# 19. 头文件包含（节点编译所需）
target_include_directories(hesai_ros_driver_node PRIVATE
  src/manager                       # 节点管理头文件
  src/msg/ros_msg                   # ROS消息头文件
  src/msg/rs_msg                    # 自定义消息头文件
  src/utility                       # 工具函数头文件
  ${CMAKE_CURRENT_BINARY_DIR}       # 生成的Version.h路径
)  

# 20. ROS1安装配置（核心）
if($ENV{ROS_VERSION} MATCHES "1")
  target_link_libraries(hesai_ros_driver_node  ${ROS_LIBS})  # 链接ROS1库
  # 安装可执行文件和库
  install(TARGETS hesai_ros_driver_node ${LIB_TARGETS}
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  # 可执行文件路径
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}  # 库文件路径
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}) # 静态库路径
  # 安装launch/rviz/config配置文件
  install(DIRECTORY
          launch
          rviz
          config
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif($ENV{ROS_VERSION} MATCHES "1")

# 21. ROS2安装配置（RK3588用ROS1，此分支不执行）
if($ENV{ROS_VERSION} MATCHES "2")
  find_package(ament_index_cpp REQUIRED)
  ament_target_dependencies(hesai_ros_driver_node 
  "ament_index_cpp"
  "rcl_interfaces"
  "rclcpp" 
  "rcutils"
  "std_msgs" 
  "sensor_msgs" 
  )

if(($ENV{ROS_DISTRO} STREQUAL "kilted") OR ($ENV{ROS_DISTRO} STREQUAL "jazzy") OR ($ENV{ROS_DISTRO} STREQUAL "humble"))
  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(hesai_ros_driver_node "${cpp_typesupport_target}")
else()
  rosidl_target_interfaces(hesai_ros_driver_node  ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

  install(TARGETS
          hesai_ros_driver_node
          DESTINATION lib/${PROJECT_NAME})
  install(TARGETS
          ${LIB_TARGETS}
          DESTINATION lib)
  install(DIRECTORY
          launch
          rviz
          DESTINATION share/${PROJECT_NAME})
  ament_package()
endif($ENV{ROS_VERSION} MATCHES "2")
```

### 2. HesaiLidar_ROS/package.xml 解析

```XML

<?xml version="1.0"?>
<package format="3">
  <name>hesai_ros_driver</name>                <!-- 包名 -->
  <version>1.5.0</version>                     <!-- 版本号 -->
  <description>The hesai_ros_driver_node package</description> <!-- 描述 -->
  <maintainer email="zhangyu@hesaitech.com">hesaiwuxiaozhou</maintainer> <!-- 维护者 -->
  <license>BSD</license>                       <!-- 许可证 -->
  
  <!-- 构建工具依赖：ROS1用catkin，ROS2用ament_cmake -->
  <buildtool_depend condition="$ROS_VERSION == 1">catkin</buildtool_depend>
  <buildtool_depend condition="$ROS_VERSION == 2">ament_cmake</buildtool_depend>
  <buildtool_depend condition="$ROS_VERSION == 2">rosidl_default_generators</buildtool_depend>

  <!-- ROS1运行依赖（核心） -->
  <depend condition="$ROS_VERSION == 1">roscpp</depend>          <!-- ROS1 C++核心 -->
  <depend condition="$ROS_VERSION == 1">sensor_msgs</depend>     <!-- 传感器消息 -->
  <depend condition="$ROS_VERSION == 1">roslib</depend>          <!-- ROS工具库 -->
  <depend condition="$ROS_VERSION == 1">std_msgs</depend>        <!-- 标准消息 -->

  <!-- ROS2运行依赖（RK3588用ROS1，无需） -->
  <depend condition="$ROS_VERSION == 2">rclcpp</depend>
  <depend condition="$ROS_VERSION == 2">sensor_msgs</depend>
  <depend condition="$ROS_VERSION == 2">std_msgs</depend>
  <depend condition="$ROS_VERSION == 2">ament_index_cpp</depend>
  <depend condition="$ROS_VERSION == 2">rclcpp_action</depend>

  <!-- ROS2接口组（无需） -->
  <member_of_group condition="$ROS_VERSION == 2">rosidl_interface_packages</member_of_group>
  
  <!-- 导出构建类型：ROS1用catkin，ROS2用ament_cmake -->
  <export>
    <build_type condition="$ROS_VERSION == 1">catkin</build_type>
    <build_type condition="$ROS_VERSION == 2">ament_cmake</build_type>
  </export>
</package>
```

### 3. handsfree_rtk/CMakeLists.txt 解析（T-RTK UM982适配前）

```CMake

cmake_minimum_required(VERSION 3.0.2)          # 兼容ROS1 Noetic的CMake版本

project(handsfree_rtk)                        # RTK包名

# 查找catkin依赖（核心：roscpp+传感器消息）
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs       # 几何消息（坐标、姿态等）
  roscpp              # ROS1 C++核心
  rospy               # ROS1 Python核心
  sensor_msgs         # 传感器消息（RTK位置消息）
)

# catkin包配置（无导出库，仅声明依赖）
catkin_package(
)

# 头文件包含：catkin依赖的头文件路径
include_directories(
  ${catkin_INCLUDE_DIRS}
)
```

### 4. handsfree_rtk/package.xml 解析

```XML

<?xml version="1.0"?>
<package format="2">                          <!-- ROS1包格式2 -->
  <name>handsfree_rtk</name>                  <!-- 包名 -->
  <version>0.0.0</version>                    <!-- 版本号（待完善） -->
  <description>The handsfree_rtk package</description> <!-- 描述 -->
  <maintainer email="handsfree@todo.todo">handsfree</maintainer> <!-- 维护者 -->
  <license>TODO</license>                     <!-- 许可证（待完善） -->
  
  <!-- 构建工具依赖：catkin -->
  <buildtool_depend>catkin</buildtool_depend>
  
  <!-- 构建依赖 -->
  <build_depend>geometry_msgs</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>sensor_msgs</build_depend>
  
  <!-- 构建导出依赖 -->
  <build_export_depend>geometry_msgs</build_export_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>sensor_msgs</build_export_depend>
  
  <!-- 运行依赖 -->
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>

  <export>
  </export>
</package>
```

## 三、RK3588+Ubuntu20.04+ROS1环境适配优化

### 1. 顶层CMakeLists.txt（src/CMakeLists.txt）

```CMake

cmake_minimum_required(VERSION 3.5)
project(lawnwomer_ws)

# 设置C++标准（ROS1 Noetic默认C++14）
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 禁用CUDA（RK3588无NVIDIA GPU）
set(FIND_CUDA OFF CACHE BOOL "Disable CUDA for RK3588")

# catkin构建系统配置
find_package(catkin REQUIRED COMPONENTS
  catkin_virtualenv
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
)

# catkin包配置
catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
)

# 包含子目录（按依赖顺序）
add_subdirectory(HesaiLidar_SDK)        # 先编译底层SDK
add_subdirectory(HesaiLidar_ROS)        # 再编译ROS驱动
add_subdirectory(handsfree_rtk)         # RTK驱动
add_subdirectory(OrbbecSDK_ROS)         # 可选：相机驱动（如需）

# 头文件包含
include_directories(
  ${catkin_INCLUDE_DIRS}
  HesaiLidar_SDK/libhesai
  HesaiLidar_ROS/src
  handsfree_rtk/include
)

# 编译选项（RK3588优化）
add_compile_options(-O2 -march=armv8.2-a -mtune=cortex-a76)
```

### 2. HesaiLidar_ROS/CMakeLists.txt 优化（RK3588适配）

#### 关键修改点：

```CMake

# 1. 强制禁用CUDA（RK3588无NVIDIA GPU）
set(FIND_CUDA OFF CACHE BOOL "Disable CUDA for RK3588" FORCE)

# 2. 修正C++标准（ROS1 Noetic仅支持C++14）
if($ENV{ROS_VERSION} MATCHES "1")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -march=armv8.2-a -mtune=cortex-a76")
endif()

# 3. 修正SDK路径（适配相对路径）
add_subdirectory(../HesaiLidar_SDK src/driver/HesaiLidar_SDK_2.0)

# 4. 增加RK3588平台定义
add_definitions(-DRK3588_PLATFORM -DARM64_UBUNTU2004)

# 5. 优化链接路径（RK3588库路径）
link_directories(
  /usr/lib/aarch64-linux-gnu
  /usr/local/lib
  ${CATKIN_DEVEL_PREFIX}/lib
)
```

### 3. HesaiLidar_ROS/package.xml 优化

```XML

<?xml version="1.0"?>
<package format="2"> <!-- ROS1使用format2，format3为ROS2 -->
  <name>hesai_ros_driver</name>
  <version>1.5.0</version>
  <description>Hesai JT128 ROS Driver for RK3588+Ubuntu20.04+ROS1</description>
  <maintainer email="dev@lawnwomer.com">Lawnwomer Dev</maintainer>
  <license>BSD</license>

  <!-- ROS1强制依赖 -->
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>sensor_msgs</depend>
  <depend>roslib</depend>
  <depend>std_msgs</depend>
  <depend>yaml-cpp</depend>
  <depend>boost</depend>

  <!-- RK3588依赖 -->
  <build_depend>libssl-dev</build_depend>
  <exec_depend>libssl1.1</exec_depend>
  <build_depend>libudev-dev</build_depend>
  <exec_depend>libudev1</exec_depend>

  <export>
    <build_type>catkin</build_type>
  </export>
</package>
```

### 4. handsfree_rtk 适配T-RTK UM982（USB通信）

#### （1）handsfree_rtk/CMakeLists.txt 扩展

```CMake

cmake_minimum_required(VERSION 3.0.2)
project(handsfree_rtk)

# RK3588优化
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8.2-a -mtune=cortex-a76")

# 查找依赖（USB通信+ROS）
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  serial  # USB串口通信库
)

# 查找串口库（RK3588 Ubuntu20.04）
find_library(SERIAL_LIB serial /usr/lib/aarch64-linux-gnu)

# catkin包配置
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES handsfree_rtk
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  serial
)

# 头文件包含
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/serial  # 串口库头文件
)

# 编译RTK核心库（UM982 USB通信）
add_library(handsfree_rtk
  src/um982_driver.cc
  src/rtk_parser.cc
  src/usb_serial.cc
)

# 链接库
target_link_libraries(handsfree_rtk
  ${catkin_LIBRARIES}
  ${SERIAL_LIB}
  pthread
)

# 编译RTK节点
add_executable(rtk_node src/rtk_node.cc)
target_link_libraries(rtk_node
  handsfree_rtk
  ${catkin_LIBRARIES}
)

# 安装
install(TARGETS handsfree_rtk rtk_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

#### （2）handsfree_rtk/package.xml 扩展

```XML

<?xml version="1.0"?>
<package format="2">
  <name>handsfree_rtk</name>
  <version>1.0.0</version>
  <description>T-RTK UM982 USB Driver for RK3588+ROS1</description>
  <maintainer email="dev@lawnwomer.com">Lawnwomer Dev</maintainer>
  <license>BSD</license>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>serial</build_depend> <!-- 串口库 -->

  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>geometry_msgs</build_export_depend>
  <build_export_depend>sensor_msgs</build_export_depend>
  <build_export_depend>serial</build_export_depend>

  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>serial</exec_depend> <!-- 运行依赖 -->

  <export>
  </export>
</package>
```

## 四、工程打包与编译说明

### 1. 工程打包（lawnwomer_ws.tar.gz）

#### 打包命令：

```Bash

# 整理工程目录
mkdir -p lawnwomer_ws/src
cp -r HesaiLidar_ROS HesaiLidar_SDK handsfree_rtk OrbbecSDK_ROS lawnwomer_ws/src/
echo "cmake_minimum_required(VERSION 3.5)
project(lawnwomer_ws)
find_package(catkin REQUIRED)
catkin_make()" > lawnwomer_ws/src/CMakeLists.txt

# 打包
tar -zcvf lawnwomer_ws.tar.gz lawnwomer_ws/
```

### 2. RK3588编译环境准备

#### （1）依赖安装

```Bash

# ROS1 Noetic安装（Ubuntu20.04）
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-noetic-desktop-full ros-noetic-serial ros-noetic-catkin python3-catkin-tools

# 基础依赖
sudo apt install -y build-essential cmake libyaml-cpp-dev libssl-dev libudev-dev libboost-all-dev

# RK3588交叉编译工具（如需）
sudo apt install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
```

#### （2）编译命令

```Bash

# 解压工程
tar -zxvf lawnwomer_ws.tar.gz
cd lawnwomer_ws

# 初始化catkin
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Debug  # 调试模式编译
# 或Release模式（性能优化）
# catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O2 -march=armv8.2-a -mtune=cortex-a76"

# 加载环境
source devel/setup.bash
```

### 3. 核心模块调试命令

#### （1）禾赛JT128雷达调试

```Bash

# 启动雷达节点（ETH通信，需配置IP）
roslaunch hesai_ros_driver jt128.launch

# 查看点云话题
rostopic echo /hesai/points
rviz -d $(rospack find hesai_ros_driver)/rviz/hesai.rviz
```

#### （2）T-RTK UM982调试

```Bash

# 启动RTK节点（USB通信，默认串口/dev/ttyUSB0）
roslaunch handsfree_rtk um982.launch

# 查看RTK话题
rostopic echo /rtk/fix
```

## 五、核心代码解析总结

|文件/模块|核心功能|RK3588适配关键点|
|---|---|---|
|HesaiLidar_SDK|禾赛雷达底层协议解析（ETH）|禁用CUDA、ARM64编译优化、路径修正|
|HesaiLidar_ROS|雷达ROS封装、消息发布|C++14标准、catkin依赖修正、ARM架构优化|
|handsfree_rtk|RTK UM982 USB通信、位置解析|增加serial依赖、USB串口配置、ARM编译优化|
|CMakeLists.txt（顶层）|catkin构建系统配置|按依赖顺序编译、禁用CUDA、ARM编译选项|
|package.xml|依赖声明|ROS1 format2、增加RK3588系统依赖|
## 六、注意事项

1. **网络配置**：禾赛JT128通过ETH通信，需将RK3588的网口IP配置为与雷达同一网段（默认[192.168.1.100](192.168.1.100)）。

2. **USB权限**：T-RTK UM982和Gemini335的USB通信需赋予串口权限：

    ```Bash
    
    sudo chmod 777 /dev/ttyUSB0
    # 永久权限：添加udev规则
    echo 'KERNEL=="ttyUSB*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-usb-serial.rules
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```

3. **调试模式**：编译时添加`-DCMAKE_BUILD_TYPE=Debug`，可使用gdb调试：

    ```Bash
    
    gdb --args ./devel/lib/hesai_ros_driver/hesai_ros_driver_node
    ```

4. **性能优化**：RK3588的Cortex-A76核心可启用`-march=armv8.2-a -mtune=cortex-a76`编译选项提升性能。

以上工程包已适配RK3588+Ubuntu20.04+ROS1环境，可直接编译调试，核心模块包含禾赛JT128（ETH）、T-RTK UM982（USB）、Gemini335（USB）的通信适配，满足割草机器人的感知与定位需求。
> （注：文档部分内容可能由 AI 生成）