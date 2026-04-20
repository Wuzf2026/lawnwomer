#ifndef LIDAR_DRIVER_H
#define LIDAR_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <hesai_lidar/hesai_driver_api.h>

class LiDARDriver
{
public:
    LiDARDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~LiDARDriver();

    bool init();
    void readData();
    bool hasNewPointCloud();

    sensor_msgs::PointCloud2Ptr getPointCloud();
    sensor_msgs::ImuPtr getLidarImu();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher pub_lidar_pointcloud_;
    ros::Publisher pub_lidar_imu_;

    std::string lidar_ip_;
    int lidar_port_;
    std::string lidar_frame_id_;
    bool remove_nan_;

    std::shared_ptr<hesai::LidarInterface> lidar_handler_;
    bool new_pointcloud_;

    sensor_msgs::PointCloud2Ptr point_cloud_msg_;
    sensor_msgs::ImuPtr lidar_imu_msg_;
};

#endif