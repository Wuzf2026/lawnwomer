#ifndef HESAI_IMU_DRIVER_H
#define HESAI_IMU_DRIVER_H
#include "common.h"

class HesaiImuDriver
{
public:
    HesaiImuDriver();
    ~HesaiImuDriver();
    bool Init(const SensorConfig& cfg);
    bool GetImuData(sensor_msgs::Imu& msg);
    bool IsOnline();

private:
    bool m_online;
    SensorConfig m_cfg;
};
#endif