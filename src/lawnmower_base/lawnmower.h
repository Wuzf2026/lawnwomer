#ifndef LAWNMOWER_BASE_LAWNMOWER_H
#define LAWNMOWER_BASE_LAWNMOWER_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
namespace lawnmower_base {
class Lawnmower {
public:
    Lawnmower(ros::NodeHandle& nh);
    ~Lawnmower();
    
    void run();
    
private:
    ros::NodeHandle nh_;
    
    // 传感器订阅者
    ros::Subscriber orbbec_depth_sub_;
    ros::Subscriber orbbec_imu_sub_;
    ros::Subscriber hesai_lidar_sub_;
    ros::Subscriber rtk_fix_sub_;
    
    // 数据发布者
    ros::Publisher combined_sensor_pub_;
    
    // 传感器数据缓冲区
    sensor_msgs::PointCloud2 orbbec_depth_cloud_;
    sensor_msgs::Imu orbbec_imu_;
    sensor_msgs::PointCloud2 hesai_lidar_cloud_;
    sensor_msgs::NavSatFix rtk_fix_;
    
    // 数据融合方法
    void sensor_data_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
    void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void rtk_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void publish_combined_data();
};
} // namespace lawnmower_base
#endif // LAWNMOWER_BASE_LAWNMOWER_H