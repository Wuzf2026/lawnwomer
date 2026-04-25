#ifndef ORBBEC_DRIVER_H
#define ORBBEC_DRIVER_H
#include "common.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class OrbbecDriver
{
public:
    OrbbecDriver();
    ~OrbbecDriver();
    bool Init(const SensorConfig& cfg);
    bool GetPointCloud2(sensor_msgs::PointCloud2& msg);
    bool IsOnline();

private:
    bool m_online;
    SensorConfig m_cfg;
};
#endif