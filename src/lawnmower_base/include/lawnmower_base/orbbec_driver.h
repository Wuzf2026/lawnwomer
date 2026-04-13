#ifndef ORBBEC_DRIVER_H
#define ORBBEC_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rk3588_sdk/rk_usb.h>
#include <ObSensor.h>
#include <ObTypes.h>
#include <mutex>

namespace lawnmower_base {

class OrbbecDriver {
public:
  OrbbecDriver(ros::NodeHandle& nh, const std::string& config_path);
  ~OrbbecDriver();

  // 初始化Orbbec相机
  bool init();

  // 获取点云数据
  void getPointCloud(sensor_msgs::PointCloud2& cloud);

  // 启动/停止数据流
  void startStream();
  void stopStream();

  // 配置参数API
  void setResolution(int width, int height);
  void setFPS(int fps);
  void setFrameId(const std::string& frame_id);

private:
  // 点云数据回调函数
  void pointCloudCallback(ob::FrameSet::Ptr frame_set);

  // ROS节点句柄
  ros::NodeHandle nh_;
  
  // Orbbec设备句柄
  ob::DevicePtr device_;
  ob::PipelinePtr pipeline_;
  ob::ConfigPtr config_;
  
  // RK3588 USB驱动
  RKUSB* rk_usb_;
  
  // 配置参数
  std::string device_id_;
  std::string usb_port_;
  std::string frame_id_;
  int width_;
  int height_;
  int fps_;
  int point_step_;
  bool is_dense_;
  bool is_bigendian_;
  
  // 数据缓存
  sensor_msgs::PointCloud2 cloud_data_;
  std::mutex cloud_mutex_;
  bool is_running_;
};

} // namespace lawnmower_base

#endif // ORBBEC_DRIVER_H