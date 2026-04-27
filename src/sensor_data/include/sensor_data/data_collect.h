#ifndef DATA_COLLECT_H
#define DATA_COLLECT_H
#include "sensor_common_config.h"
#include "orbbec_driver.h"
#include "hesai_imu_driver.h"
#include "um982_gps_driver.h"

class DataCollectManager
{
public:
    DataCollectManager(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);
    void LoopOnce();

private:
    void LoadRosParam();
    void PublishAllSensor();

    ros::NodeHandle m_nh;
    ros::NodeHandle m_nh_priv;
    ros::Publisher pub_orbbec_cloud;
    ros::Publisher pub_hesai_imu;
    ros::Publisher pub_um982_gps;

    OrbbecDriver m_orbbec;
    HesaiImuDriver m_hesai;
    UM982GpsDriver m_gps;
    SensorConfig m_cfg;
};
#endif