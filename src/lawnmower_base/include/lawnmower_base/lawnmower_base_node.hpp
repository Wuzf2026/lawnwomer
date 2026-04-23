#ifndef LAWNMOWER_BASE_NODE_H
#define LAWNMOWER_BASE_NODE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <memory>

#include "camera_driver.hpp"
#include "lidar_driver.hpp"
#include "rtk_driver.hpp"

class LawnmowerBaseNode
{
public:
    LawnmowerBaseNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    bool init();
    void spin();

private:
    void processData();
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void publishTf();
    void calculateOdom();
    std_msgs::Header getHeader(std::string frame_id);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    tf::TransformBroadcaster tf_broadcaster_;

    ros::Publisher pub_pointcloud_;
    ros::Publisher pub_imu_;
    ros::Publisher pub_gps_;
    ros::Publisher pub_odom_;

    ros::Subscriber sub_cmd_vel_;

    std::unique_ptr<CameraDriver> camera_driver_;
    std::unique_ptr<LiDARDriver> lidar_driver_;
    std::unique_ptr<RTKDriver> rtk_driver_;

    std::string base_frame_, lidar_frame_, imu_frame_, gps_frame_;
};

#endif