#include "lawnmower_base/lawnmower_base_node.h"
#include <sstream>
#include <boost/algorithm/string.hpp>

// 构造函数
LawnmowerBaseNode::LawnmowerBaseNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private),
    cloud_ptr_(new PointCloudT()),
    wheel_speed_left_(0.0),
    wheel_speed_right_(0.0) {
}

// 析构函数
LawnmowerBaseNode::~LawnmowerBaseNode() {
  if (ser_.isOpen()) {
    ser_.close();
  }
}

// 初始化函数
bool LawnmowerBaseNode::init() {
  // 读取配置参数
  nh_private_.param<std::string>("serial_port", serial_port_, "/dev/ttyUSB0");
  nh_private_.param<int>("serial_baudrate", serial_baudrate_, 115200);
  nh_private_.param<std::string>("frame_id/base", frame_id_base_, "base_link");
  nh_private_.param<std::string>("frame_id/lidar", frame_id_lidar_, "lidar_link");
  nh_private_.param<std::string>("frame_id/imu", frame_id_imu_, "imu_link");
  nh_private_.param<std::string>("frame_id/gps", frame_id_gps_, "gps_link");

  // 初始化发布者
  pub_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/lawnmower/pointcloud", 10);
  pub_imu_ = nh_.advertise<sensor_msgs::Imu>("/lawnmower/imu", 10);
  pub_gps_ = nh_.advertise<sensor_msgs::NavSatFix>("/lawnmower/gps", 10);
  pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/lawnmower/odom", 10);
  pub_wheel_speed_ = nh_.advertise<std_msgs::Float64MultiArray>("/lawnmower/wheel_speed", 10);

  // 初始化订阅者
  sub_cmd_vel_ = nh_.subscribe("/cmd_vel", 10, &LawnmowerBaseNode::cmdVelCallback, this);
  sub_orbbec_raw_ = nh_.subscribe("/orbbec/raw/pointcloud", 10, &LawnmowerBaseNode::orbbecRawCallback, this);
  sub_hesai_raw_ = nh_.subscribe("/hesai/raw/imu", 10, &LawnmowerBaseNode::hesaiRawCallback, this);
  sub_um982_raw_ = nh_.subscribe("/um982/raw/gps", 10, &LawnmowerBaseNode::um982RawCallback, this);

  // 初始化串口
  if (!initSerial()) {
    ROS_ERROR("Failed to initialize serial port!");
    return false;
  }

  // 初始化点云缓存
  cloud_ptr_->header.frame_id = frame_id_lidar_;
  cloud_ptr_->height = 1;
  cloud_ptr_->width = 0;

  ROS_INFO("Lawnmower base node initialized successfully!");
  return true;
}

// 初始化串口（RK3588）
bool LawnmowerBaseNode::initSerial() {
  try {
    ser_.setPort(serial_port_);
    ser_.setBaudrate(serial_baudrate_);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser_.setTimeout(to);
    ser_.open();
  } catch (serial::IOException& e) {
    ROS_ERROR("Unable to open port: %s", e.what());
    return false;
  }

  if (ser_.isOpen()) {
    ROS_INFO("Serial port %s opened successfully (baudrate: %d)", serial_port_.c_str(), serial_baudrate_);
    return true;
  } else {
    ROS_ERROR("Serial port %s open failed!", serial_port_.c_str());
    return false;
  }
}

// 速度指令回调（/cmd_vel）
void LawnmowerBaseNode::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(serial_mutex_);
  cmd_vel_ = *msg;
  
  // 发送指令到串口
  sendCmdVelSerial();
}

// 发送速度指令到串口（RK3588串口协议）
void LawnmowerBaseNode::sendCmdVelSerial() {
  if (!ser_.isOpen()) return;

  // 串口协议格式："CMD:vx,vy,vz,rx,ry,rz\n"
  // vx: 线速度x (m/s), vy: 线速度y, vz: 线速度z
  // rx: 角速度x (rad/s), ry: 角速度y, rz: 角速度z
  std::stringstream ss;
  ss << "CMD:" 
     << cmd_vel_.linear.x << ","
     << cmd_vel_.linear.y << ","
     << cmd_vel_.linear.z << ","
     << cmd_vel_.angular.x << ","
     << cmd_vel_.angular.y << ","
     << cmd_vel_.angular.z << "\n";

  try {
    ser_.write(ss.str());
    ROS_DEBUG("Sent cmd_vel to serial: %s", ss.str().c_str());
  } catch (serial::IOException& e) {
    ROS_ERROR("Failed to write to serial port: %s", e.what());
  }
}

// 读取串口数据（轮速反馈）
void LawnmowerBaseNode::readSerialData() {
  if (!ser_.isOpen()) return;

  try {
    std::string data = ser_.readline();
    if (!data.empty()) {
      ROS_DEBUG("Received serial data: %s", data.c_str());
      parseWheelSpeed(data);
    }
  } catch (serial::IOException& e) {
    ROS_WARN("Failed to read serial data: %s", e.what());
  }
}

// 解析轮速数据（串口协议："WHEEL:left,right\n"）
void LawnmowerBaseNode::parseWheelSpeed(const std::string& data) {
  if (data.substr(0, 5) != "WHEEL") return;

  std::vector<std::string> parts;
  boost::split(parts, data, boost::is_any_of(":,\\n"));
  if (parts.size() < 3) return;

  try {
    wheel_speed_left_ = std::stod(parts[1]);
    wheel_speed_right_ = std::stod(parts[2]);

    // 发布轮速
    std_msgs::Float64MultiArray wheel_speed_msg;
    wheel_speed_msg.data.push_back(wheel_speed_left_);
    wheel_speed_msg.data.push_back(wheel_speed_right_);
    pub_wheel_speed_.publish(wheel_speed_msg);
  } catch (std::exception& e) {
    ROS_WARN("Failed to parse wheel speed: %s", e.what());
  }
}

// Orbbec Gemini335原始数据回调
void LawnmowerBaseNode::orbbecRawCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  // 转换为PCL点云
  pcl::fromROSMsg(*msg, *cloud_ptr_);
  
  // 处理点云（滤波、坐标系转换等）
  processOrbbecData();
  
  // 发布标准点云消息
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud_ptr_, cloud_msg);
  cloud_msg.header = getHeader(frame_id_lidar_);
  pub_pointcloud_.publish(cloud_msg);
}

// 处理Orbbec点云
void LawnmowerBaseNode::processOrbbecData() {
  // 示例：移除无效点
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_ptr_, *cloud_ptr_, indices);
  
  // 可添加滤波、下采样等处理逻辑
  cloud_ptr_->header.stamp = ros::Time::now().toNSec() / 1000;
}

// Hesai JT128原始IMU回调
void LawnmowerBaseNode::hesaiRawCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() < 6) {
    ROS_WARN("Invalid Hesai IMU data size!");
    return;
  }

  // 填充标准IMU消息
  imu_msg_.header = getHeader(frame_id_imu_);
  
  // 角速度（rad/s）
  imu_msg_.angular_velocity.x = msg->data[0];
  imu_msg_.angular_velocity.y = msg->data[1];
  imu_msg_.angular_velocity.z = msg->data[2];
  
  // 线加速度（m/s²）
  imu_msg_.linear_acceleration.x = msg->data[3];
  imu_msg_.linear_acceleration.y = msg->data[4];
  imu_msg_.linear_acceleration.z = msg->data[5];
  
  // 姿态四元数（示例：若Hesai无姿态，可设为单位四元数）
  imu_msg_.orientation.x = 0.0;
  imu_msg_.orientation.y = 0.0;
  imu_msg_.orientation.z = 0.0;
  imu_msg_.orientation.w = 1.0;

  // 处理IMU数据
  processHesaiIMU();
  
  // 发布IMU
  pub_imu_.publish(imu_msg_);
}

// 处理Hesai IMU
void LawnmowerBaseNode::processHesaiIMU() {
  // 示例：添加协方差（根据传感器手册配置）
  imu_msg_.angular_velocity_covariance[0] = 0.001;
  imu_msg_.angular_velocity_covariance[4] = 0.001;
  imu_msg_.angular_velocity_covariance[8] = 0.001;
  
  imu_msg_.linear_acceleration_covariance[0] = 0.01;
  imu_msg_.linear_acceleration_covariance[4] = 0.01;
  imu_msg_.linear_acceleration_covariance[8] = 0.01;
}

// UM982原始GPS回调
void LawnmowerBaseNode::um982RawCallback(const std_msgs::String::ConstPtr& msg) {
  // 示例：解析NMEA格式（$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47）
  std::vector<std::string> parts;
  boost::split(parts, msg->data, boost::is_any_of(",*"));
  if (parts.size() < 10) {
    ROS_WARN("Invalid UM982 GPS data!");
    return;
  }

  // 填充标准GPS消息
  gps_msg_.header = getHeader(frame_id_gps_);
  
  // 纬度（度）
  double lat_deg = std::stod(parts[2].substr(0, 2));
  double lat_min = std::stod(parts[2].substr(2));
  gps_msg_.latitude = lat_deg + lat_min / 60.0;
  if (parts[3] == "S") gps_msg_.latitude *= -1;
  
  // 经度（度）
  double lon_deg = std::stod(parts[4].substr(0, 3));
  double lon_min = std::stod(parts[4].substr(3));
  gps_msg_.longitude = lon_deg + lon_min / 60.0;
  if (parts[5] == "W") gps_msg_.longitude *= -1;
  
  // 高度（米）
  gps_msg_.altitude = std::stod(parts[9]);
  
  // GPS状态
  gps_msg_.status.status = std::stoi(parts[6]); // 0:无定位,1:GPS定位,2:DGPS定位
  gps_msg_.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

  // 处理GPS数据
  processUM982GPS();
  
  // 发布GPS
  pub_gps_.publish(gps_msg_);
}

// 处理UM982 GPS
void LawnmowerBaseNode::processUM982GPS() {
  // 示例：添加协方差（根据传感器手册配置）
  gps_msg_.position_covariance[0] = 1.0;
  gps_msg_.position_covariance[4] = 1.0;
  gps_msg_.position_covariance[8] = 1.0;
  gps_msg_.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
}

// 计算里程计
void LawnmowerBaseNode::computeOdometry(const ros::Time& now) {
  static ros::Time last_time = now;
  double dt = (now - last_time).toSec();
  last_time = now;

  // 示例：基于轮速计算里程计
  double v = (wheel_speed_left_ + wheel_speed_right_) / 2.0; // 平均速度
  double w = (wheel_speed_right_ - wheel_speed_left_) / 0.5; // 角速度（轮距0.5m）

  static double x = 0.0, y = 0.0, yaw = 0.0;
  x += v * cos(yaw) * dt;
  y += v * sin(yaw) * dt;
  yaw += w * dt;

  // 发布里程计
  nav_msgs::Odometry odom_msg;
  odom_msg.header = getHeader(frame_id_base_);
  odom_msg.child_frame_id = "base_footprint";

  // 位置
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

  // 速度
  odom_msg.twist.twist.linear.x = v;
  odom_msg.twist.twist.angular.z = w;

  pub_odom_.publish(odom_msg);

  // 发布TF
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, yaw);
  transform.setRotation(q);
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform, now, "odom", "base_footprint"));
}

// 获取标准Header
std_msgs::Header LawnmowerBaseNode::getHeader(const std::string& frame_id) {
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = frame_id;
  return header;
}

// 主循环
void LawnmowerBaseNode::spin() {
  ros::Rate rate(50); // 50Hz
  while (ros::ok()) {
    // 读取串口数据（轮速）
    readSerialData();
    
    // 计算里程计
    computeOdometry(ros::Time::now());
    
    // 处理回调
    ros::spinOnce();
    
    // 睡眠
    rate.sleep();
  }
}

// 主函数
int main(int argc, char** argv) {
  ros::init(argc, argv, "lawnmower_base_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  LawnmowerBaseNode node(nh, nh_private);
  if (!node.init()) {
    ROS_ERROR("Lawnmower base node initialization failed!");
    return -1;
  }

  node.spin();
  return 0;
}