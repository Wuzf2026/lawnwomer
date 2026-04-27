#ifndef ORBBEC_DRIVER_H
#define ORBBEC_DRIVER_H

#include "sensor_common_config.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

// PCL C++14兼容声明
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class OrbbecDriver {
public:
    OrbbecDriver() : m_online(false) {}
    ~OrbbecDriver() {}

    bool Init(const SensorConfig& cfg);
    bool GetColorImage(sensor_msgs::Image& img_msg);
    bool GetPointCloud(sensor_msgs::PointCloud2& pc_msg);

private:
    bool m_online;
    SensorConfig m_cfg;
    PointCloudT::Ptr m_cloud;
};

#endif // ORBBEC_DRIVER_H