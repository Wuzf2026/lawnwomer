#include "lawnmower_base/lawnmower_base_node.hpp"

LawnmowerBaseNode::LawnmowerBaseNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private) {
  camera_driver_ = std::make_unique<CameraDriver>(nh, nh_private);
  lidar_driver_ = std::make_unique<LiDARDriver>(nh, nh_private);
  rtk_driver_ = std::make_unique<RTKDriver>(nh, nh_private);
}
#ifdef OBSENSOR_SDK_FOUND
bool LawnmowerBaseNode::init() {
  nh_private_.param<std::string>("base_frame", base_frame_, "base_link");
  nh_private_.param<std::string>("lidar_frame", lidar_frame_, "lidar_link");
  nh_private_.param<std::string>("imu_frame", imu_frame_, "imu_link");
  nh_private_.param<std::string>("gps_frame", gps_frame_, "gps_link");
  pub_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/lawnmower/merge/pointcloud", 10);
  pub_imu_ = nh_.advertise<sensor_msgs::Imu>("/lawnmower/merge/imu", 10);
  pub_gps_ = nh_.advertise<sensor_msgs::NavSatFix>("/lawnmower/merge/gps", 10);
  pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/lawnmower/odom", 10);
  sub_cmd_vel_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &LawnmowerBaseNode::cmdVelCallback, this);
  if (!camera_driver_->init()) return false;
  if (!lidar_driver_->init()) return false;
  if (!rtk_driver_->init()) return false;
  ROS_INFO("lawnmower base main node init ok");
  return true;
}
#else
ROS_WARN("Orbbec SDK not found, camera driver disabled");
#endif

void LawnmowerBaseNode::spin() {
  ros::Rate r(50);
  while (ros::ok()) {
    ros::spinOnce();
    camera_driver_->readData();
    lidar_driver_->readData();
    rtk_driver_->readData();
    processData();
    publishTf();
    r.sleep();
  }
}
void LawnmowerBaseNode::processData() {
  if (camera_driver_->hasNewImage()) pub_imu_.publish(camera_driver_->getImuData());
  if (lidar_driver_->hasNewPointCloud()) pub_pointcloud_.publish(lidar_driver_->getPointCloud());
  if (rtk_driver_->hasNewGpsData()) pub_gps_.publish(rtk_driver_->getGpsFix());
  calculateOdom();
}
void LawnmowerBaseNode::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  // 速度控制逻辑
}
void LawnmowerBaseNode::publishTf() {
  tf::StampedTransform tf_lidar, tf_imu, tf_gps;
  tf_lidar.frame_id_ = base_frame_;
  tf_lidar.child_frame_id_ = lidar_frame_;
  tf_lidar.stamp_ = ros::Time::now();
  tf_lidar.setOrigin(tf::Vector3(0.2, 0, 0.4));
  tf_lidar.setRotation(tf::Quaternion(0, 0, 0, 1));
  tf_imu.frame_id_ = base_frame_;
  tf_imu.child_frame_id_ = imu_frame_;
  tf_imu.stamp_ = ros::Time::now();
  tf_imu.setOrigin(tf::Vector3(0.1, 0, 0.3));
  tf_gps.frame_id_ = base_frame_;
  tf_gps.child_frame_id_ = gps_frame_;
  tf_gps.stamp_ = ros::Time::now();
  tf_gps.setOrigin(tf::Vector3(0.0, 0, 0.5));
  tf_broadcaster_.sendTransform(tf_lidar);
  tf_broadcaster_.sendTransform(tf_imu);
  tf_broadcaster_.sendTransform(tf_gps);
}
void LawnmowerBaseNode::calculateOdom() {
  // 里程计计算逻辑
}
std_msgs::Header LawnmowerBaseNode::getHeader(std::string frame_id) {
  std_msgs::Header h;
  h.stamp = ros::Time::now();
  h.frame_id = frame_id;
  return h;
}