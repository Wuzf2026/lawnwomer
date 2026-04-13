#include "lawnmower_base/sensor_data_fusion.h"
#include <ros/rate.h>

namespace lawnmower_base {

SensorDataFusion::SensorDataFusion(ros::NodeHandle& nh, const std::string& config_path)
  : nh_(nh), config_path_(config_path), is_running_(false),
    orbbec_ready_(false), hesai_ready_(false), um982_ready_(false) {
  // 加载发布频率
  ros::NodeHandle private_nh("~");
  private_nh.param<double>("publish_rate", publish_rate_, 50.0);

  // 创建ROS发布者
  pub_pointcloud_ = nh.advertise<sensor_msgs::PointCloud2>("/lawnmower/pointcloud", 10);
  pub_imu_ = nh.advertise<sensor_msgs::Imu>("/lawnmower/imu", 10);
  pub_gps_ = nh.advertise<sensor_msgs::NavSatFix>("/lawnmower/gps", 10);

  // 创建传感器驱动实例
  orbbec_driver_ = new OrbbecDriver(nh, config_path);
  hesai_driver_ = new HesaiDriver(nh, config_path);
  um982_driver_ = new UM982Driver(nh, config_path);
}

SensorDataFusion::~SensorDataFusion() {
  stop();
  if (orbbec_driver_) delete orbbec_driver_;
  if (hesai_driver_) delete hesai_driver_;
  if (um982_driver_) delete um982_driver_;
}

bool SensorDataFusion::initAllSensors() {
  ROS_INFO("Initializing all sensors...");
  
  // 初始化Orbbec
  orbbec_ready_ = orbbec_driver_->init();
  if (!orbbec_ready_) ROS_ERROR("Orbbec init failed");

  // 初始化Hesai
  hesai_ready_ = hesai_driver_->init();
  if (!hesai_ready_) ROS_ERROR("Hesai init failed");

  // 初始化UM982
  um982_ready_ = um982_driver_->init();
  if (!um982_ready_) ROS_ERROR("UM982 init failed");

  // 启动所有传感器数据流
  if (orbbec_ready_) orbbec_driver_->startStream();
  if (hesai_ready_) hesai_driver_->startStream();
  if (um982_ready_) um982_driver_->startStream();

  return (orbbec_ready_ && hesai_ready_ && um982_ready_);
}

void SensorDataFusion::start() {
  if (is_running_) return;
  
  is_running_ = true;
  publish_thread_ = std::thread(&SensorDataFusion::publishThreadFunc, this);
  ROS_INFO("Sensor data fusion started");
}

void SensorDataFusion::stop() {
  if (!is_running_) return;
  
  is_running_ = false;
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }

  // 停止所有传感器
  if (orbbec_ready_) orbbec_driver_->stopStream();
  if (hesai_ready_) hesai_driver_->stopStream();
  if (um982_ready_) um982_driver_->stopStream();

  ROS_INFO("Sensor data fusion stopped");
}

void SensorDataFusion::publishThreadFunc() {
  ros::Rate rate(publish_rate_);
  
  while (ros::ok() && is_running_) {
    // 发布点云数据
    if (orbbec_ready_) {
      sensor_msgs::PointCloud2 cloud;
      orbbec_driver_->getPointCloud(cloud);
      pub_pointcloud_.publish(cloud);
    }

    // 发布IMU数据
    if (hesai_ready_) {
      sensor_msgs::Imu imu;
      hesai_driver_->getIMUData(imu);
      pub_imu_.publish(imu);
    }

    // 发布GPS数据
    if (um982_ready_) {
      sensor_msgs::NavSatFix gps;
      um982_driver_->getGPSData(gps);
      pub_gps_.publish(gps);
    }

    rate.sleep();
  }
}

bool SensorDataFusion::getOrbbecStatus() {
  return orbbec_ready_;
}

bool SensorDataFusion::getHesaiStatus() {
  return hesai_ready_;
}

bool SensorDataFusion::getUM982Status() {
  return um982_ready_;
}

void SensorDataFusion::resetOrbbec() {
  if (orbbec_ready_) {
    orbbec_driver_->stopStream();
    orbbec_ready_ = orbbec_driver_->init();
    if (orbbec_ready_) orbbec_driver_->startStream();
  }
}

void SensorDataFusion::resetHesai() {
  if (hesai_ready_) {
    hesai_driver_->stopStream();
    hesai_ready_ = hesai_driver_->init();
    if (hesai_ready_) hesai_driver_->startStream();
  }
}

void SensorDataFusion::resetUM982() {
  if (um982_ready_) {
    um982_driver_->stopStream();
    um982_ready_ = um982_driver_->init();
    if (um982_ready_) um982_driver_->startStream();
  }
}

} // namespace lawnmower_base