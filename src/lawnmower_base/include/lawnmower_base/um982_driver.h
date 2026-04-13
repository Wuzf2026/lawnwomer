#ifndef UM982_DRIVER_H
#define UM982_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <rk3588_sdk/rk_usb.h>
#include <handsfree_rtk/um982.h>
#include <mutex>

namespace lawnmower_base {

class UM982Driver {
public:
  UM982Driver(ros::NodeHandle& nh, const std::string& config_path);
  ~UM982Driver();

  // 初始化UM982 RTK
  bool init();

  // 获取GPS数据
  void getGPSData(sensor_msgs::NavSatFix& gps_data);

  // 启动/停止数据流
  void startStream();
  void stopStream();

  // 配置参数API
  void setSerialPort(const std::string& port);
  void setBaudRate(int baud);
  void setFrameId(const std::string& frame_id);
  void setPositionCovariance(const double* cov);
  void setCovarianceType(uint8_t type);

private:
  // GPS数据回调函数
  void gpsCallback(const UM982Data& gps_data);

  // ROS节点句柄
  ros::NodeHandle nh_;
  
  // UM982驱动句柄
  UM982Device* um982_device_;
  
  // RK3588 USB驱动
  RKUSB* rk_usb_;
  
  // 配置参数
  std::string serial_port_;
  int baud_rate_;
  std::string frame_id_;
  double position_covariance_[9];
  uint8_t position_covariance_type_;
  
  // 数据缓存
  sensor_msgs::NavSatFix gps_data_;
  std::mutex gps_mutex_;
  bool is_running_;
};

} // namespace lawnmower_base

#endif // UM982_DRIVER_H