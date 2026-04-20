# 1 环境准备
# 1.1 系统要求
# •硬件平台：RK3588
# •操作系统：Ubuntu 20.04 LTS
# •ROS 版本：ROS1 Noetic
# •编译器：aarch64-linux-gnu-gcc (7.5.0-2019.12)
# 1.2 依赖安装
# 安装ROS Noetic基础环境
sudo apt update
sudo apt install ros-noetic-desktop-full
# 安装依赖库
sudo apt install \
    libusb-1.0-0-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libyaml-cpp-dev \
    libpcl-dev \
    libopencv-dev \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    ros-noetic-tf \
    ros-noetic-pcl-ros \
    ros-noetic-serial
# 安装RK3588交叉编译工具链
wget https://dl.rkdeveloperkit.com/tools/arm-gnu-toolchain/arm-gnu-toolchain-10.3-2021.07-x86_64-aarch64-none-linux-gnu.tar.xz
tar -xJf arm-gnu-toolchain-10.3-2021.07-x86_64-aarch64-none-linux-gnu.tar.xz
export PATH=$PATH:/path/to/arm-gnu-toolchain-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin

# 2 编译步骤
# 2.1 交叉编译配置
# 设置交叉编译环境
source /path/to/rk3588_sdk/environment-setup
# 创建ROS工作空间
mkdir -p ~/lawnmower_ws/src
# 初始化catkin工作空间
cd ..
catkin_make
# 注意：由于使用了交叉编译，需要设置以下环境变量
export CMAKE_TOOLCHAIN_FILE=src/rk3588_sdk/toolchainfile.cmake
export CMAKE_C_COMPILER=aarch64-linux-gnu-gcc
export CMAKE_CXX_COMPILER=aarch64-linux-gnu-g++
# 2.2 编译选项说明
rk3588_toolchain.cmake 配置：
# RK3588 (ARM64) 编译工具链配置
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
# RK3588 Ubuntu20.04默认gcc/g++（arm64架构）
set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)
# 编译选项（适配RK3588，关闭CUDA相关）
add_definitions(-DNO_CUDA -DROS1_NOETIC -DRK3588_PLATFORM)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O2 -Wall")
# 链接选项
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lpthread -lserial -lobsensor")

# 3 硬件连接与权限配置
# 3.1 Orbbec Gemini335 连接
硬件连接：
•通过 USB 3.0 线缆连接到 RK3588 的 USB3.0 端口（标注 "SS"）
•确保供电充足
权限配置：
# 创建udev规则
sudo nano /etc/udev/rules.d/99-orbbec.rules
添加内容：
SUBSYSTEM=="usb", ATTRS{idVendor}=="2bc5", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5750", MODE="0666", GROUP="plugdev"
# 重启udev服务
sudo udevadm control --reload-rules && sudo udevadm trigger
# 添加用户到plugdev组
sudo usermod -aG plugdev $USER
# 3.2 Hesai JT128 连接
网络配置：
•JT128 默认 IP：192.168.1.201
•RK3588 需配置同网段 IP（如 192.168.1.100）
# 临时配置IP
sudo ifconfig eth0 192.168.1.100 netmask 255.255.255.0 up
# 永久配置（netplan）
sudo nano /etc/netplan/01-hesai-eth.yaml
添加内容：
network:
  ethernets:
    eth0:
      dhcp4: no
      addresses: [192.168.1.100/24]
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
# 应用配置
sudo netplan apply
# 验证网络连通性
ping 192.168.1.201 -c 5
# 3.3 UM982 连接
串口连接：
•通过 USB 转串口模块（推荐 FT232）连接到 RK3588
•识别串口设备：ls /dev/ttyUSB* 或 ls /dev/ttyACM*
权限配置：
# 临时权限
sudo chmod 666 /dev/ttyUSB0
# 永久权限（udev规则）
sudo nano /etc/udev/rules.d/99-um982.rules
添加内容：
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", KERNEL=="ttyACM*", MODE="0666", GROUP="dialout"
# 重启udev服务
sudo udevadm control --reload-rules && sudo udevadm trigger
# 添加用户到dialout组
sudo usermod -aG dialout $USER

# 4 启动与测试
# 4.1 启动流程
# 启动 ROS master
roscore
# 启动基础节点
roslaunch lawnmower_base base.launch
# 启动传感器节点
roslaunch lawnmower_base sensors.launch
# 启动电机控制节点
roslaunch motor_control motor_control.launch
# 启动调试工具
python3 /path/to/tools/scripts/unified_tool.py
# 4.2 数据验证
话题列表验证：
rostopic list | grep lawnmower
预期输出：
/lawnmower/color/image
/lawnmower/depth/image  
/lawnmower/imu
/lawnmower/pointcloud
/lawnmower/gps/fix
/lawnmower/gps/nmea
/lawnmower/odom
/lawnmower/wheel_speed
数据监控：
# 查看点云数据频率
rostopic hz /lawnmower/pointcloud
# 查看IMU数据
rostopic echo /lawnmower/imu
# 查看GPS定位
rostopic echo /lawnmower/gps/fix
# 查看里程计
rostopic echo /lawnmower/odom
# 4.3 调试命令
传感器测试命令：
# Orbbec相机测试
rosrun orbbec_camera list_devices_node  # 枚举相机设备
roslaunch orbbec_camera gemini335.launch  # 启动相机
# Hesai激光雷达测试  
roslaunch hesai_lidar jt128.launch  # 启动激光雷达
rostopic hz /hesai_jt128/pointcloud_raw  # 检查点云频率
# UM982 RTK测试
roslaunch handsfree_rtk um982.launch  # 启动RTK
rostopic echo /um982/nmea_raw  # 查看NMEA原始数据
# 电机测试控制
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}, angular: {z: 0.2, y: 3.0}}' -r 10
# 查看轮速回传
rostopic echo /wheel_speed
系统集成测试：
# 启动所有设备
roslaunch launch all_devices.launch
# 录制所有原始数据（60秒）
rosbag record -a -O all_devices_raw.bag --duration=60
# 资源监控
top -p $(pgrep -f ob_camera_node) -p $(pgrep -f hesai_jt128_node) -p $(pgrep -f um982_node)
iftop -i eth0  # 监控ETH带宽
usbmon -t 1    # 监控USB带宽

# 5 Python 调试工具使用
# 5.1 独立工具使用
# 相机调试工具：
python3 camera_tool.py --help
功能：
•实时显示彩色和深度图像
•设置相机参数（分辨率、帧率等）
•查询相机状态
•执行相机标定
# 激光雷达调试工具：
python3 lidar_tool.py --help
功能：
•实时绘制点云俯视图
•设置激光雷达参数
•查询激光雷达状态
•录制点云数据
# RTK 调试工具：
python3 rtk_tool.py --help
功能：
•显示 GPS 定位信息
•查看 NMEA 原始数据
•设置 RTK 参数（波特率、NTRIP 配置）
•查询 RTK 状态
# 5.2 统一调试工具使用
python3 unified_tool.py --help
系统命令：
python3 unified_tool.py system start_all  # 启动所有传感器
python3 unified_tool.py system stop_all   # 停止所有传感器
python3 unified_tool.py system status     # 查询系统状态
# 传感器命令：
# 相机命令
python3 unified_tool.py camera set_params  # 设置相机参数
python3 unified_tool.py camera get_status  # 获取相机状态
python3 unified_tool.py camera calibrate   # 执行相机标定
# 激光雷达命令
python3 unified_tool.py lidar set_params   # 设置激光雷达参数
python3 unified_tool.py lidar get_status   # 获取激光雷达状态
python3 unified_tool.py lidar record_pcl   # 录制点云数据
# RTK命令
python3 unified_tool.py rtk set_params     # 设置RTK参数
python3 unified_tool.py rtk get_status     # 获取RTK状态
python3 unified_tool.py rtk config_ntrip   # 配置NTRIP客户端
