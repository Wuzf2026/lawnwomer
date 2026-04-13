#include "lawnmower_base/um982_driver.h"
#include <sensor_msgs/NavSatStatus.h>

namespace lawnmower_base {

UM982Driver::UM982Driver(ros::NodeHandle& nh, const std::string& config_path)
  : nh_(nh), is_running_(false) {
  // 加载配置参数
  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("um982/serial_port", serial_port_, "/dev/ttyUSB0");
  private_nh.param<int>("um982/baud_rate", baud_rate_, 115200);
  private_nh.param<std::string>("um982/frame_id", frame_id_, "um982_gps");
  
  // 加载协方差矩阵
  std::vector<double> pos_cov;
  private_nh.param<std::vector<double>>("um982/position_covariance", pos_cov, 
    {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01});
  memcpy(position_covariance_, pos_cov.data(), 9*sizeof(double));
  
  private_nh.param<uint8_t>("um982/position_covariance_type", position_covariance_type_, 2);

  // 初始化RK3588 USB驱动
  rk_usb_ = new RKUSB();
}

UM982Driver::~UM982Driver() {
  stopStream();
  if (rk_usb_) {
    delete rk_usb_;
  }
  if (um982_device_) {
    delete um982_device_;
  }
}

bool UM982Driver::init() {
  // 初始化RK3588 USB串口
  if (!rk_usb_->initSerial(serial_port_, baud_rate_)) {
    ROS_ERROR("RK3588 USB serial init failed for UM982");
    return false;
  }

  // 初始化UM982设备
  um982_device_ = new UM982Device();
  UM982Config config;
  config.serial_port = serial_port_;
  config.baud_rate = baud_rate_;
  config.gps_callback = std::bind(&UM982Driver::gpsCallback, this, std::placeholders::_1);

  if (!um982_device_->init(config)) {
    ROS_ERROR("UM982 init failed");
    return false;
  }

  ROS_INFO("UM982 RTK init success");
  return true;
}

void UM982Driver::startStream() {
  if (is_running_) return;
  
  if (um982_device_->startGPSStream()) {
    is_running_ = true;
    ROS_INFO("UM982 GPS stream started");
  } else {
    ROS_ERROR("UM982 start GPS stream failed");
  }
}

void UM982Driver::stopStream() {
  if (!is_running_) return;
  
  um982_device_->stopGPSStream();
  is_running_ = false;
  ROS_INFO("UM982 GPS stream stopped");
}

void UM982Driver::gpsCallback(const UM982Data& raw_gps) {
  std::lock_guard<std::mutex> lock(gps_mutex_);
  
  // 填充NavSatFix消息
  gps_data_.header.stamp = ros::Time::now();
  gps_data_.header.frame_id = frame_id_;

  // GPS状态
  gps_data_.status.status = raw_gps.status;
  gps_data_.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS | sensor_msgs::NavSatStatus::SERVICE_GLONASS | 
                             sensor_msgs::NavSatStatus::SERVICE_COMPASS | sensor_msgs::NavSatStatus::SERVICE_GALILEO;

  // 经纬度高度
  gps_data_.latitude = raw_gps.latitude;
  gps_data_.longitude = raw_gps.longitude;
  gps_data_.altitude = raw_gps.altitude;

  // 协方差矩阵
  memcpy(gps_data_.position_covariance.data(), position_covariance_, 9*sizeof(double));
  gps_data_.position_covariance_type = position_covariance_type_;
}

void UM982Driver::getGPSData(sensor_msgs::NavSatFix& gps_data) {
  std::lock_guard<std::mutex> lock(gps_mutex_);
  gps_data = gps_data_;
}

void UM982Driver::setSerialPort(const std::string& port) {
  serial_port_ = port;
  if (is_running_) {
    stopStream();
    init();
    startStream();
  }
}

void UM982Driver::setBaudRate(int baud) {
  baud_rate_ = baud;
  if (is_running_) {
    stopStream();
    init();
    startStream();
  }
}

void UM982Driver::setFrameId(const std::string& frame_id) {
  frame_id_ = frame_id;
}

void UM982Driver::setPositionCovariance(const double* cov) {
  memcpy(position_covariance_, cov, 9*sizeof(double));
}

void UM982Driver::setCovarianceType(uint8_t type) {
  position_covariance_type_ = type;
}

} // namespace lawnmower_base