#ifndef HESAI_IMU_DRIVER_H
#define HESAI_IMU_DRIVER_H

#include "sensor_common_config.h"
#include <sensor_msgs/Imu.h>

class HesaiImuDriver {
public:
    HesaiImuDriver() : m_online(false) {}
    ~HesaiImuDriver();

    bool Init(const SensorConfig& cfg);
    bool GetImuData(sensor_msgs::Imu& imu_msg);

private:
    bool m_online;
    SensorConfig m_cfg;
};

// 补齐缺失的函数声明
int hesai_eth_init();
void hesai_eth_close();
bool hesai_get_imu_raw(double q[4], double w[3], double a[3]);

#endif // HESAI_IMU_DRIVER_H