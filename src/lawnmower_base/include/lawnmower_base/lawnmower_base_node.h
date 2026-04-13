#ifndef LAWNMOWER_BASE_NODE_H
#define LAWNMOWER_BASE_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>

// 传感器配置结构体
struct SensorCommonConfig {
  double publish_rate;
  std::string frame_id;
};

struct OrbbecConfig {
  bool enable;
  std::string topic_name;
  int width;
  int height;
  std::string frame_id;
  int point_step;
  bool is_dense;
  bool is_bigendian;
};

struct HesaiConfig {
  bool enable;
  std::string topic_name;
  std::string frame_id;
  std::vector<double> angular_velocity_cov;
  std::vector<double> linear_acceleration_cov;
  std::vector<double> orientation_cov;
};

struct UM982Config {
  bool enable;
  std::string topic_name;
  std::string frame_id;
  int position_covariance_type;
  std::vector<double> position_cov;
  uint8_t status;
};

// 核心节点类
class LawnmowerBaseNode {
public:
  LawnmowerBaseNode(ros::NodeHandle& nh);
  ~LawnmowerBaseNode();

  // 初始化配置
  bool initConfig(const std::string& config_path);

  // 定时器回调: 发布所有传感器数据
  void publishSensorData(const ros::TimerEvent& event);

  // 模拟传感器数据生成（实际场景替换为真实传感器订阅）
  void generateSimulatedPointCloud(sensor_msgs::PointCloud2& pc2);
  void generateSimulatedIMU(sensor_msgs::Imu& imu);
  void generateSimulatedGPS(sensor_msgs::NavSatFix& gps);

  // 系统状态查询回调
  void motorStatusCallback(const std_msgs::Bool::ConstPtr& msg);
  void sensorConnCallback(const std_msgs::String::ConstPtr& msg);

private:
  ros::NodeHandle nh_;
  ros::Timer publish_timer_;

  // 发布者
  ros::Publisher pc2_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher gps_pub_;

  // 订阅者（系统状态）
  ros::Subscriber motor_status_sub_;
  ros::Subscriber sensor_conn_sub_;

  // TF广播器
  tf::TransformBroadcaster tf_broadcaster_;

  // 配置参数
  SensorCommonConfig common_cfg_;
  OrbbecConfig orbbec_cfg_;
  HesaiConfig hesai_cfg_;
  UM982Config um982_cfg_;

  // 系统状态缓存
  bool motor_running_ = false;
  std::string sensor_conn_status_ = "disconnected";
};

#endif // LAWNMOWER_BASE_NODE_H