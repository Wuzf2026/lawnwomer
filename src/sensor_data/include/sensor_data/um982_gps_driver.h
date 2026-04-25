#ifndef UM982_GPS_DRIVER_H
#define UM982_GPS_DRIVER_H
#include "common.h"

class UM982GpsDriver
{
public:
    UM982GpsDriver();
    ~UM982GpsDriver();
    bool Init(const SensorConfig& cfg);
    bool GetNavSatFix(sensor_msgs::NavSatFix& msg);
    bool IsOnline();

private:
    bool m_online;
    SensorConfig m_cfg;
};
#endif