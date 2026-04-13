#include "lawnmower_base/hesai_driver.h"
#include <tf/transform_datatypes.h>

namespace lawnmower_base {

HesaiDriver::HesaiDriver(ros::NodeHandle& nh, const std::string& config_path)
  : nh_(nh), is_running_(false) {
  // 加载配置参数
  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("hesai/ip_address", ip_address_, "192.168.1.200");
  private_nh.param<int>("hesai/port", port_, 9347);
  private_nh.param<std::string>("hesai/frame_id", frame_id_, "hesai_imu");
  private_nh.param<int>("hesai/imu_rate", imu_rate_, 200);
  
  // 加载协方差矩阵
  std::vector<double> ori_cov, ang_vel_cov, lin_acc_cov;
  private_nh.param<std::vector<double>>("hesai/orientation_covariance", ori_cov, 
    {0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001});
  private_nh.param<std::vector<double>>("hesai/angular_velocity_covariance", ang_vel_cov, 
    {0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001});
  private_nh.param<std::vector<double>>("hesai/linear_acceleration_covariance", lin_acc_cov, 
    {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01});
  
  memcpy(orientation_covariance_, ori_cov.data(), 9*sizeof(double));
  memcpy(angular_velocity_covariance_, ang_vel_cov.data(), 9*sizeof(double));
  memcpy(linear_acceleration_covariance_, lin_acc_cov.data(), 9*sizeof(double));

  // 初始化RK3588 ETH驱动
  rk_eth_ = new RKETH();
}

HesaiDriver::~HesaiDriver() {
  stopStream();
  if (rk_eth_) {
    delete rk_eth_;
  }
  if (hesai_driver_) {
    delete hesai_driver_;
  }
}

bool HesaiDriver::init() {
  // 初始化RK3588 ETH
  if (!rk_eth_->init(ip_address_, port_)) {
    ROS_ERROR("RK3588 ETH init failed for Hesai");
    return false;
  }

  // 初始化禾赛雷达驱动
  hesai_driver_ = new HesaiLidarDriver();
  HesaiLidarConfig config;
  config.ip_address = ip_address_;
  config.port = port_;
  config.imu_rate = imu_rate_;
  config.imu_callback = std::bind(&HesaiDriver::imuCallback, this, std::placeholders::_1);

  if (!hesai_driver_->init(config)) {
    ROS_ERROR("Hesai JT128 init failed");
    return false;
  }

  ROS_INFO("Hesai JT128 init success");
  return true;
}

void HesaiDriver::startStream() {
  if (is_running_) return;
  
  if (hesai_driver_->startIMUStream()) {
    is_running_ = true;
    ROS_INFO("Hesai IMU stream started");
  } else {
    ROS_ERROR("Hesai start IMU stream failed");
  }
}

void HesaiDriver::stopStream() {
  if (!is_running_) return;
  
  hesai_driver_->stopIMUStream();
  is_running_ = false;
  ROS_INFO("Hesai IMU stream stopped");
}

void HesaiDriver::imuCallback(const HesaiImuData& raw_imu) {
  std::lock_guard<std::mutex> lock(imu_mutex_);
  
  // 填充IMU消息
  imu_data_.header.stamp = ros::Time::now();
  imu_data_.header.frame_id = frame_id_;

  // 姿态四元数
  imu_data_.orientation.x = raw_imu.quat_x;
  imu_data_.orientation.y = raw_imu.quat_y;
  imu_data_.orientation.z = raw_imu.quat_z;
  imu_data_.orientation.w = raw_imu.quat_w;

  // 协方差矩阵
  memcpy(imu_data_.orientation_covariance.data(), orientation_covariance_, 9*sizeof(double));
  memcpy(imu_data_.angular_velocity_covariance.data(), angular_velocity_covariance_, 9*sizeof(double));
  memcpy(imu_data_.linear_acceleration_covariance.data(), linear_acceleration_covariance_, 9*sizeof(double));

  // 角速度 (rad/s)
  imu_data_.angular_velocity.x = raw_imu.gyro_x * M_PI / 180.0;
  imu_data_.angular_velocity.y = raw_imu.gyro_y * M_PI / 180.0;
  imu_data_.angular_velocity.z = raw_imu.gyro_z * M_PI / 180.0;

  // 线加速度 (m/s²)
  imu_data_.linear_acceleration.x = raw_imu.acc_x * 9.81;
  imu_data_.linear_acceleration.y = raw_imu.acc_y * 9.81;
  imu_data_.linear_acceleration.z = raw_imu.acc_z * 9.81;
}

void HesaiDriver::getIMUData(sensor_msgs::Imu& imu_data) {
  std::lock_guard<std::mutex> lock(imu_mutex_);
  imu_data = imu_data_;
}

void HesaiDriver::setIPAddress(const std::string& ip) {
  ip_address_ = ip;
  if (is_running_) {
    stopStream();
    init();
    startStream();
  }
}

void HesaiDriver::setPort(int port) {
  port_ = port;
  if (is_running_) {
    stopStream();
    init();
    startStream();
  }
}

void HesaiDriver::setFrameId(const std::string& frame_id) {
  frame_id_ = frame_id;
}

void HesaiDriver::setIMURate(int rate) {
  imu_rate_ = rate;
  if (is_running_) {
    stopStream();
    init();
    startStream();
  }
}

void HesaiDriver::setCovariance(const double* orientation_cov, 
                               const double* angular_vel_cov,
                               const double* linear_acc_cov) {
  memcpy(orientation_covariance_, orientation_cov, 9*sizeof(double));
  memcpy(angular_velocity_covariance_, angular_vel_cov, 9*sizeof(double));
  memcpy(linear_acceleration_covariance_, linear_acc_cov, 9*sizeof(double));
}

} // namespace lawnmower_base