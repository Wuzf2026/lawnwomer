# Lawnmower ROS1 Noetic 调试命令清单
## 1. 基础环境操作
### 1.1 ROS Core 管理
- Command: roscore
  说明: 启动ROS主节点（基础依赖）
- Command: ./run.sh start_core
  说明: 后台启动roscore（日志:/tmp/roscore.log）

### 1.2 工程编译
- Command: ./run.sh build
  说明: 全量编译工作空间（Release模式，并行编译）
- Command: ./run.sh build -p rk3588_sdk
  说明: 单独编译指定功能包（支持rk3588_sdk/hesai_lidar/orbbec_camera等）

### 1.3 工程清理
- Command: ./run.sh clean
  说明: 清理编译产物（devel/build/logs）
- Command: ./run.sh clean_source
  说明: 清理源码文件（保留配置文件，谨慎使用）

### 1.4 工程发布
- Command: ./run.sh release -v v1.0.0
  说明: 生成tar.gz发布包
- Command: ./run.sh release -v v1.0.0 --self-extractor
  说明: 生成自解压可执行发布包

## 2. 传感器节点启动
### 2.1 相机（Orbbec Gemini 335）
- Command: rosrun orbbec_camera list_devices_node
  说明: 枚举已连接的Orbbec相机设备
- Command: ./run.sh run_cmdlist roslaunch orbbec_camera gemini345.launch
  说明: 启动Gemini335相机节点（需确认launch文件路径）

### 2.2 激光雷达（Hesai JT128）
- Command: ./run.sh run_cmdlist roslaunch hesai_lidar start.launch
  说明: 启动JT128激光雷达节点（需确认launch文件路径）
- Command: ./run.sh view_topic_hz /hesai_jt128/pointcloud_raw
  说明: 检查激光雷达点云发布频率

### 2.3 RTK（UM982）
- Command: ./run.sh run_cmdlist roslaunch handsfree_rtk handsfree_rtk.launch
  说明: 启动UM982 RTK节点（需确认launch文件路径）
- Command: ./run.sh view_topic /um982/nmea_raw -n 5
  说明: 查看RTK NMEA原始数据（最多5条）

### 2.4 组合启动
- Command: roslaunch sensor_data sensors.launch
  说明: 启动所有传感器节点
- Command: roslaunch motor_control motor_control.launch
  说明: 启动电机控制节点
- Command: roslaunch launch all_devices.launch
  说明: 启动所有设备（集成启动）

## 3. 数据验证与监控
### 3.1 话题列表验证
- Command: rostopic list | grep lawnmower
  说明: 筛选lawnmower相关话题，预期输出：
        /lawnmower/color/image
        /lawnmower/depth/image  
        /lawnmower/imu
        /lawnmower/pointcloud
        /lawnmower/gps/fix
        /lawnmower/gps/nmea
        /lawnmower/odom
        /lawnmower/wheel_speed

### 3.2 话题数据监控
- Command: ./run.sh view_topic_hz /lawnmower/pointcloud
  说明: 查看点云数据发布频率
- Command: ./run.sh view_topic /lawnmower/imu
  说明: 实时查看IMU数据
- Command: ./run.sh view_topic /lawnmower/gps/fix -n 10
  说明: 查看GPS定位数据（最多10条）
- Command: ./run.sh view_topic /lawnmower/odom
  说明: 查看里程计数据

### 3.3 系统状态查看
- Command: ./run.sh view_topics
  说明: 查看所有ROS话题及类型
- Command: ./run.sh view_nodes
  说明: 查看运行中的ROS节点
- Command: ./run.sh view_services
  说明: 查看可用ROS服务列表
- Command: ./run.sh view_params
  说明: 查看ROS参数服务器配置

## 4. 数据录制与回放
- Command: ./run.sh record_bag -a -O all_devices_raw.bag --duration=60
  说明: 录制所有话题数据60秒，保存为all_devices_raw.bag
- Command: ./run.sh record_bag /lawnmower/pointcloud /lawnmower/imu -O sensor_data.bag
  说明: 录制指定传感器话题数据
- Command: ./run.sh play_bag all_devices_raw.bag -r 2 --loop
  说明: 2倍速循环播放数据包

## 5. Python调试工具（src/tools/scripts/）
### 5.1 独立工具
- Command: python3 ../scripts/camera_tool.py --help
  说明: 相机调试工具（设备检测/参数配置/状态查询）
- Command: python3 ../scripts/lidar_tool.py --help
  说明: 激光雷达调试工具（频率监控/参数配置/点云录制）
- Command: python3 ../scripts/rtk_tool.py --help
  说明: RTK调试工具（NMEA解析/参数配置/NTRIP配置）

### 5.2 统一调试工具
- Command: python3 ../scripts/unified_tool.py --help
  说明: 统一传感器调试工具
- Command: python3 ../scripts/unified_tool.py system start_all
  说明: 启动所有传感器节点
- Command: python3 ../scripts/unified_tool.py system stop_all
  说明: 停止所有传感器节点
- Command: python3 ../scripts/unified_tool.py system status
  说明: 查询所有传感器状态
- Command: python3 ../scripts/unified_tool.py camera get_status
  说明: 获取相机当前状态
- Command: python3 ../scripts/unified_tool.py lidar record_pcl -d 10 -o lidar_data.bag
  说明: 录制10秒激光雷达点云数据
- Command: python3 ../scripts/unified_tool.py rtk config_ntrip
  说明: 配置RTK的NTRIP客户端参数