#ifndef UM982_GPS_DRIVER_H
#define UM982_GPS_DRIVER_H

#include "sensor_common_config.h"
#include <sensor_msgs/NavSatFix.h>

class UM982GpsDriver {
public:
    UM982GpsDriver() : m_online(false) {}
    ~UM982GpsDriver();

    bool Init(const SensorConfig& cfg);
    bool GetNavSatFix(sensor_msgs::NavSatFix& gps_msg);

private:
    bool m_online;
    SensorConfig m_cfg;
};

// 补齐缺失的函数声明
int um982_serial_open(const std::string& tty, int baud);
void um982_close();
bool um982_get_gps_data(double& lat, double& lon, double& alt, int& stat, int& srv, double& cov);

#endif // UM982_GPS_DRIVER_H