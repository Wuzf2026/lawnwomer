#include <ros/ros.h>
#include "lawnmower_base/sensor_data_fusion.h"

int main(int argc, char** argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "lawnmower_base_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ROS_INFO("Starting Lawnmower Base Node...");

  // 获取配置文件路径
  std::string config_path;
  private_nh.param<std::string>("config_path", config_path, 
    ros::package::getPath("lawnmower_base") + "/config/sensor_params.yaml");

  // 创建数据融合实例
  lawnmower_base::SensorDataFusion fusion(nh, config_path);

  // 初始化所有传感器
  if (!fusion.initAllSensors()) {
    ROS_ERROR("Failed to initialize all sensors, exiting...");
    return -1;
  }

  // 启动数据融合和发布
  fusion.start();

  // 保持节点运行
  ros::spin();

  // 停止所有传感器
  fusion.stop();

  ROS_INFO("Lawnmower Base Node stopped");
  return 0;
}