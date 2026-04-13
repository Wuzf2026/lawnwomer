#ifndef HESAI_DRIVER_H
#define HESAI_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <rk3588_sdk/rk_eth.h>
#include <hesai_lidar_driver.h>
#include <mutex>

namespace lawnmower_base {

class HesaiDriver {
public:
  HesaiDriver(ros::NodeHandle& nh, const std::string& config_path);
  ~HesaiDriver();

  // 初始化禾赛雷达
  bool init();

  // 获取IMU数据
  void getIMUData(sensor_msgs::Imu& imu_data);

  // 启动/停止数据流
  void startStream();
  void stopStream();

  // 配置参数API
  void setIPAddress(const std::string& ip);
  void setPort(int port);
  void setFrameId(const std::string& frame_id);
  void setIMURate(int rate);
  void setCovariance(const double* orientation_cov, 
                     const double* angular_vel_cov,
                     const double* linear_acc_cov);

private:
  // IMU数据回调函数
  void imuCallback(const HesaiImuData& imu_data);

  // ROS节点句柄
  ros::NodeHandle nh_;
  
  // 禾赛雷达驱动句柄
  HesaiLidarDriver* hesai_driver_;
  
  // RK3588 ETH驱动
  RKETH* rk_eth_;
  
  // 配置参数
  std::string ip_address_;
  int port_;
  std::string frame_id_;
  int imu_rate_;
  double orientation_covariance_[9];
  double angular_velocity_covariance_[9];
  double linear_acceleration_covariance_[9];
  
  // 数据缓存
  sensor_msgs::Imu imu_data_;
  std::mutex imu_mutex_;
  bool is_running_;
};

} // namespace lawnmower_base

#endif // HESAI_DRIVER_H