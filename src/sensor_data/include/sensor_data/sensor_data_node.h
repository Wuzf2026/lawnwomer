#ifndef SENSOR_DATA_NODE_H
#define SENSOR_DATA_NODE_H

#include <ros/ros.h>
#include "sensor_data/hesai_imu_driver.h"
#include "sensor_data/um982_gps_driver.h"
#include "sensor_data/orbbec_driver.h"
#include "sensor_data/sensor_common_config.h"

class SensorDataNode {
public:
    SensorDataNode(ros::NodeHandle& nh);
    void Run();

private:
    void LoadConfig();
    void ImuPublishLoop();
    void GpsPublishLoop();
    void CameraPublishLoop();

    ros::NodeHandle m_nh;
    ros::Publisher m_imu_pub, m_gps_pub, m_img_pub, m_pc_pub;

    SensorConfig m_cfg;
    HesaiImuDriver m_imu_driver;
    UM982GpsDriver m_gps_driver;
    OrbbecDriver m_camera_driver;

    bool m_imu_enable = true;
    bool m_gps_enable = true;
    bool m_camera_enable = true;
    double m_pub_freq = 10.0;
};

#endif // SENSOR_DATA_NODE_H