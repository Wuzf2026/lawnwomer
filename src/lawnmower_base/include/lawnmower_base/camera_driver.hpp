#ifndef CAMERA_DRIVER_H
#define CAMERA_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <libobsensor/ObSensor.hpp>

class CameraDriver
{
public:
    CameraDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~CameraDriver();

    bool init();
    void readData();
    bool hasNewImage();

    sensor_msgs::ImagePtr getColorImage();
    sensor_msgs::ImagePtr getDepthImage();
    sensor_msgs::ImuPtr getImuData();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher pub_color_;
    ros::Publisher pub_depth_;
    ros::Publisher pub_ir_left_;
    ros::Publisher pub_ir_right_;
    ros::Publisher pub_imu_;

    std::string camera_model_;
    int width_, height_, fps_;
    bool publish_raw_;

    std::shared_ptr<ob::Device> camera_;
    bool new_image_;

    sensor_msgs::ImagePtr color_image_;
    sensor_msgs::ImagePtr depth_image_;
    sensor_msgs::ImuPtr imu_data_;
};

#endif