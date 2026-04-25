#ifndef SENSOR_COMMON_H
#define SENSOR_COMMON_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>

#define FRAME_ORBBEC    "camera_link"
#define FRAME_HESAI     "lidar_link"
#define FRAME_UM982     "gps_link"

struct SensorConfig
{
    int cloud_width;
    int cloud_height;
    double imu_rate;
    std::string gps_tty;
    int gps_baud;
};

#endif