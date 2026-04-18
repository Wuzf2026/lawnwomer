## 二、Orbbec Gemini335（USB 双目）原始数据获取
### 2\.1 功能包核心解读
工程包路径：`lawnmower\_ws/src/orbbec\_camera/`
- 核心节点：`ob\_camera\_node`（发布彩色 / 深度 / IR/IMU 原始数据）；
- 测试工具：`list\_devices\_node`（枚举相机）、`rk\_mpp\_decoder`（RK3588 硬解码）；
- 关键依赖：Orbbec SDK（内置在`orbbec\_camera/SDK`，需适配 arm64）。
### 2\.2 硬件连接与权限配置
#### 2\.2\.1 硬件连接
Gemini335 通过**USB 3\.0 线缆**连接 RK3588 的 USB3\.0 端口（RK3588 USB3\.0 端口通常标注`SS`，优先选择以保证带宽）。
#### 2\.2\.2 USB 权限配置（避免访问拒绝）
# 1. 创建udev规则文件
sudo vim /etc/udev/rules.d/99-orbbec.rules
# 2. 写入以下内容（适配Orbbec设备VID/PID）
SUBSYSTEM=="usb", ATTRS{idVendor}=="2bc5", MODE="0666", GROUP="plugdev"  # Orbbec官方VID
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5750", MODE="0666", GROUP="plugdev"  # 兼容款
# 3. 重启udev服务
sudo udevadm control --reload-rules && sudo udevadm trigger
# 4. 将当前用户加入plugdev组（需重新登录生效）
sudo usermod -aG plugdev $USER
#### 2\.2\.3 验证硬件连接
# 查看USB设备是否识别
lsusb | grep Orbbec
# 运行枚举节点验证
rosrun orbbec_camera list_devices_node
# 输出类似“Found device: Gemini335”则硬件连接正常
### 2\.3 配置文件与 Launch 编写
#### 2\.3\.1 新建配置文件
路径：`lawnmower\_ws/src/orbbec\_camera/config/gemini335\.yaml`
yaml
# Gemini335原始数据配置
device_type: "gemini335"
usb_port: 0  # 多设备时指定端口
# 分辨率/帧率（原始数据维度）
color:
  width: 1920
  height: 1080
  fps: 30
  publish_raw: true  # 发布彩色原始帧
depth:
  width: 1280
  height: 720
  fps: 30
  publish_raw: true  # 发布深度原始帧
  exposure: auto     # 深度曝光模式
imu:
  publish_raw: true  # 发布IMU原始数据
ir:
  publish_raw: true  # 发布双目IR原始帧
frame_id: "gemini335_link"
#### 2\.3\.2 新建 Launch 文件
路径：`lawnmower\_ws/src/orbbec\_camera/launch/gemini335\.launch`
xml
<launch>
  <!-- 加载配置文件 -->
  <rosparam file="$(find orbbec_camera)/config/gemini335.yaml" command="load"/>
  
  <!-- 启动Gemini335节点 -->
  <node name="ob_camera_node" pkg="orbbec_camera" type="ob_camera_node" output="screen">
    <param name="device_type" value="gemini335"/>
    <param name="publish_color_raw" value="true"/>
    <param name="publish_depth_raw" value="true"/>
    <param name="publish_imu_raw" value="true"/>
    <param name="publish_ir_raw" value="true"/>
    <!-- RK3588硬解码启用 -->
    <param name="enable_rk_mpp_decoder" value="true"/>
  </node>
</launch>

### 2\.4 启动节点与原始数据验证
#### 2\.4\.1 启动节点
roslaunch orbbec_camera gemini335.launch
#### 2\.4\.2 查看原始数据话题
rostopic list | grep gemini335
# 核心原始数据话题：
# /gemini335/color/image_raw      彩色原始帧（sensor_msgs/Image）
# /gemini335/depth/image_raw      深度原始帧（sensor_msgs/Image）
# /gemini335/imu/data_raw         IMU原始数据（sensor_msgs/Imu）
# /gemini335/ir_left/image_raw    左IR原始帧（sensor_msgs/Image）
# /gemini335/ir_right/image_raw   右IR原始帧（sensor_msgs/Image）
#### 2\.4\.3 数据验证
# 1. 查看原始数据帧率（确认无丢包）
rostopic hz /gemini335/color/image_raw
rostopic hz /gemini335/depth/image_raw
# 2. 查看IMU原始数据内容
rostopic echo /gemini335/imu/data_raw
# 3. 可视化原始帧（RVIZ）
rosrun rviz rviz -d $(rospack find orbbec_camera)/rviz/ob_camera.rviz
# 在RVIZ中添加Image显示，选择对应原始帧话题
## 三、Hesai JT128 雷达（ETH）原始数据获取
### 3\.1 功能包核心解读
工程包路径：`lawnmower\_ws/src/hesai\_lidar/`
- 核心模块：
    - SDK：`HesaiLidar\_SDK\_2\.0`（UDP 解析 / 网络通信核心）；
    - 解析器：`UdpParser`（JT128 UDP 协议解析，禁用 GPU 版）；
    - 节点管理：`node\_manager\.cc`（雷达节点入口）；
- 通信方式：ETH（UDP），需配置同一网段 IP。
### 3\.2 网络配置（ETH 通信核心）
#### 3\.2\.1 硬件连接
JT128 雷达通过**千兆网线**连接 RK3588 的 ETH 网口（优先千兆网口，`ifconfig`查看网口名，如`eth0`）。
#### 3\.2\.2 IP 配置（雷达 \+ RK3588）
JT128 默认 IP：`192\.168\.1\.201`，需将 RK3588 网口配置为同网段（如`192\.168\.1\.100`）。
# 1. 临时配置IP（重启失效）
sudo ifconfig eth0 192.168.1.100 netmask 255.255.255.0 up
# 2. 永久配置IP（netplan）
sudo vim /etc/netplan/01-hesai-eth.yaml
# 写入以下内容：
network:
  ethernets:
    eth0:
      dhcp4: no
      addresses: [192.168.1.100/24]
      gateway4: 192.168.1.1
  version: 2

# 3. 应用配置
sudo netplan apply
# 4. 验证网络连通性
ping 192.168.1.201 -c 5
# 丢包率0%则网络正常
### 3\.3 配置文件与 Launch 编写
#### 3\.3\.1 新建 JT128 配置文件
路径：`lawnmower\_ws/src/hesai\_lidar/config/jt128\.yaml`
yaml
# JT128原始数据配置
lidar_type: "JT128"
source_type: "socket"  # ETH UDP模式
socket:
  ip: "0.0.0.0"        # RK3588监听所有IP
  port: 9347           # JT128默认UDP端口
  protocol: "udp"
parser:
  type: "udp"
  model: "jt128"       # 匹配JT128解析器
publish_raw_pointcloud: true  # 发布原始点云
frame_id: "hesai_jt128_link"
rate: 10               # 雷达帧率
#### 3\.3\.2 新建 Launch 文件
路径：`lawnmower\_ws/src/hesai\_lidar/launch/jt128\.launch`
xml
<launch>
  <!-- 加载JT128配置 -->
  <rosparam file="$(find hesai_lidar)/config/jt128.yaml" command="load"/>
  
  <!-- 启动JT128节点（禁用GPU解析） -->
  <node name="hesai_jt128_node" pkg="hesai_lidar" type="node_manager" output="screen">
    <param name="lidar_type" value="JT128"/>
    <param name="socket_ip" value="192.168.1.100"/>
    <param name="socket_port" value="9347"/>
    <param name="enable_gpu_parser" value="false"/>
    <param name="publish_raw_pointcloud" value="true"/>
  </node>
</launch>
### 3\.4 启动节点与原始数据验证
#### 3\.4\.1 启动节点
roslaunch hesai_lidar jt128.launch
#### 3\.4\.2 查看原始数据话题
rostopic list | grep jt128
# 核心原始数据话题：
# /hesai_jt128/pointcloud_raw  原始点云（sensor_msgs/PointCloud2）
# /hesai_jt128/status_raw      雷达状态原始数据（自定义msg）
#### 3\.4\.3 数据验证
# 1. 查看原始点云帧率
rostopic hz /hesai_jt128/pointcloud_raw
# 2. 查看原始点云内容（关键字段）
rostopic echo /hesai_jt128/pointcloud_raw --noarr
# 3. 可视化原始点云
rosrun rviz rviz -d $(rospack find hesai_lidar)/rviz/hesai_rviz.rviz
# 添加PointCloud2显示，选择/hesai_jt128/pointcloud_raw
## 四、T\-RTK UM982（USB）原始数据获取
### 4\.1 功能包核心解读
工程包路径：`lawnmower\_ws/src/handsfree\_rtk/` \+ `rk3588\_sdk/`
- 核心驱动：`rk3588\_sdk/src/uart\_driver\.cpp`（RK3588 USB 串口驱动）；
- 数据协议：NMEA\-0183（原始定位数据）、RTK 原始观测值；
- 核心节点：`rtk\_node`（解析 UM982 原始数据）。
### 4\.2 硬件连接与串口权限配置
#### 4\.2\.1 硬件连接
UM982 通过**USB 转串口模块**（RS232/485）连接 RK3588 的 USB 端口，推荐使用 FT232 模块（兼容性优于 CH340）。
#### 4\.2\.2 串口识别与权限配置
# 1. 识别串口设备（插拔前后对比）
ls /dev/ttyUSB*  # 通常为/dev/ttyUSB0
ls /dev/ttyACM*  # 部分模块为ttyACM0
# 2. 权限配置（临时+永久）
# 临时权限（立即生效）
sudo chmod 666 /dev/ttyUSB0
# 永久权限（udev规则）
sudo vim /etc/udev/rules.d/99-um982.rules
# 写入内容：
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", KERNEL=="ttyACM*", MODE="0666", GROUP="dialout"
# 3. 重启udev服务
sudo udevadm control --reload-rules && sudo udevadm trigger
# 4. 添加用户到dialout组（需重新登录）
sudo usermod -aG dialout $USER
#### 4\.2\.3 验证串口连通性
# 1. 测试串口通信（UM982默认波特率115200）
screen /dev/ttyUSB0 115200
# 若看到乱码/字符流，说明串口物理连通
# 2. 退出screen：Ctrl+A + K + Y
### 4\.3 配置文件与 Launch 编写
#### 4\.3\.1 新建 UM982 配置文件
路径：`lawnmower\_ws/src/handsfree\_rtk/config/um982\.yaml`
yaml
# UM982原始数据配置
serial:
  port: "/dev/ttyUSB0"
  baudrate: 115200
  parity: "none"
  stopbits: 1
  databits: 8
  flow_control: "none"
publish_raw_nmea: true  # 发布NMEA原始数据
publish_raw_obs: true   # 发布RTK原始观测值
frame_id: "um982_link"
rate: 10                # 数据发布帧率
#### 4\.3\.2 新建 Launch 文件
路径：`lawnmower\_ws/src/handsfree\_rtk/launch/um982\.launch`
xml
<launch>
  <!-- 加载UM982配置 -->
  <rosparam file="$(find handsfree_rtk)/config/um982.yaml" command="load"/>
  
  <!-- 启动RK3588串口驱动 -->
  <node name="uart_driver_node" pkg="rk3588_sdk" type="sdk_node" output="screen">
    <param name="uart_port" value="/dev/ttyUSB0"/>
    <param name="uart_baudrate" value="115200"/>
    <param name="uart_parity" value="none"/>
  </node>
  
  <!-- 启动UM982 RTK解析节点 -->
  <node name="um982_node" pkg="handsfree_rtk" type="rtk_node" output="screen">
    <param name="serial_port" value="/dev/ttyUSB0"/>
    <param name="publish_nmea_raw" value="true"/>
    <param name="publish_obs_raw" value="true"/>
  </node>
</launch>
### 4\.4 启动节点与原始数据验证
#### 4\.4\.1 启动节点
roslaunch handsfree_rtk um982.launch
### 4\.4\.2 查看原始数据话题
rostopic list | grep um982
# 核心原始数据话题：
# /um982/nmea_raw       NMEA原始数据（std_msgs/String，含$GPGGA/$GPRMC等）
# /um982/obs_raw        RTK原始观测值（自定义msg，含伪距/载波相位等）
#### 4\.4\.3 数据验证
# 1. 查看NMEA原始数据内容
rostopic echo /um982/nmea_raw
# 2. 查看数据帧率
rostopic hz /um982/nmea_raw
# 3. 保存原始数据到ROSBag（离线分析）
rosbag record /um982/nmea_raw /um982/obs_raw -O um982_raw.bag
## 五、多设备整合与整体调试
### 5\.1 整合 Launch 文件
路径：`lawnmower\_ws/src/launch/all\_devices\.launch`
xml
<launch>
  <!-- Orbbec Gemini335 -->
  <include file="$(find orbbec_camera)/launch/gemini335.launch"/>
  
  <!-- Hesai JT128 -->
  <include file="$(find hesai_lidar)/launch/jt128.launch"/>
  
  <!-- T-RTK UM982 -->
  <include file="$(find handsfree_rtk)/launch/um982.launch"/>
</launch>
### 5\.2 启动所有设备
roslaunch launch all_devices.launch
### 5\.3 整体数据验证
# 1. 查看所有原始数据话题
rostopic list | grep raw
# 2. 批量检查帧率
rostopic hz /gemini335/color/image_raw /hesai_jt128/pointcloud_raw /um982/nmea_raw
# 3. 录制所有原始数据（60秒）
rosbag record -a -O all_devices_raw.bag --duration=60
# 4. 资源监控（RK3588负载）
top -p $(pgrep -f ob_camera_node) -p $(pgrep -f hesai_jt128_node) -p $(pgrep -f um982_node)
iftop -i eth0  # 监控ETH带宽
usbmon -t 1    # 监控USB带宽

