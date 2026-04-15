#ifndef LAWNMOWER_BASE_NODE_H
#define LAWNMOWER_BASE_NODE_H

// ROS核心头文件
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

// PCL相关
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// 串口通信
#include <serial/serial.h>

// 标准库
#include <string>
#include <vector>
#include <mutex>

// 点云类型定义（Orbbec Gemini335）
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class LawnmowerBaseNode {
public:
  // 构造函数
  LawnmowerBaseNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  
  // 析构函数
  ~LawnmowerBaseNode();
  
  // 初始化函数
  bool init();
  
  // 主循环
  void spin();

private:
  // ROS句柄
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // 发布者
  ros::Publisher pub_pointcloud_;   // Orbbec Gemini335点云
  ros::Publisher pub_imu_;          // Hesai JT128 IMU
  ros::Publisher pub_gps_;          // UM982 GPS
  ros::Publisher pub_odom_;         // 里程计
  ros::Publisher pub_wheel_speed_;  // 轮速反馈

  // 订阅者
  ros::Subscriber sub_cmd_vel_;     // 速度指令订阅（/cmd_vel）
  ros::Subscriber sub_orbbec_raw_;  // Orbbec原始数据
  ros::Subscriber sub_hesai_raw_;   // Hesai原始IMU
  ros::Subscriber sub_um982_raw_;   // UM982原始GPS

  // TF广播器
  tf::TransformBroadcaster tf_broadcaster_;

  // 串口通信对象
  serial::Serial ser_;
  std::mutex serial_mutex_;

  // 配置参数
  std::string serial_port_;         // RK3588串口设备（如/dev/ttyUSB0）
  int serial_baudrate_;             // 串口波特率（如115200）
  std::string frame_id_base_;       // 基坐标系
  std::string frame_id_lidar_;      // 激光雷达坐标系
  std::string frame_id_imu_;        // IMU坐标系
  std::string frame_id_gps_;        // GPS坐标系

  // 数据缓存
  PointCloudT::Ptr cloud_ptr_;      // 点云缓存
  sensor_msgs::Imu imu_msg_;        // IMU缓存
  sensor_msgs::NavSatFix gps_msg_;  // GPS缓存
  geometry_msgs::Twist cmd_vel_;    // 速度指令缓存
  double wheel_speed_left_;         // 左轮速度
  double wheel_speed_right_;        // 右轮速度

  // 回调函数
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void orbbecRawCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void hesaiRawCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void um982RawCallback(const std_msgs::String::ConstPtr& msg);

  // 串口通信函数
  bool initSerial();                // 初始化串口
  void sendCmdVelSerial();          // 发送速度指令到串口
  void readSerialData();            // 读取串口数据（轮速）
  void parseWheelSpeed(const std::string& data); // 解析轮速数据

  // 传感器数据处理函数
  void processOrbbecData();         // 处理Orbbec点云
  void processHesaiIMU();           // 处理Hesai IMU
  void processUM982GPS();           // 处理UM982 GPS

  // 里程计计算
  void computeOdometry(const ros::Time& now);

  // 工具函数
  std_msgs::Header getHeader(const std::string& frame_id);
};

#endif // LAWNMOWER_BASE_NODE_H