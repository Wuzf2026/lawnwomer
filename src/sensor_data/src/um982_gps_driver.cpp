#include "sensor_data/um982_gps_driver.h"
#include <handsfree_rtk/um982_usb_sdk.h>

UM982GpsDriver::UM982GpsDriver() : m_online(false) {}
UM982GpsDriver::~UM982GpsDriver()
{
    if(m_online) um982_close();
}

bool UM982GpsDriver::Init(const SensorConfig& cfg)
{
    m_cfg = cfg;
    int ret = um982_serial_open(m_cfg.gps_tty, m_cfg.gps_baud);
    if(ret != 0)
    {
        ROS_ERROR("UM982 GPS USB Serial fail");
        return false;
    }
    m_online = true;
    ROS_INFO("UM982 RTK GPS Online");
    return true;
}

bool UM982GpsDriver::GetNavSatFix(sensor_msgs::NavSatFix& msg)
{
    if(!m_online) return false;
    double lat,lon,alt;
    uint8_t stat,srv;
    double cov[9];
    if(!um982_get_gps_data(lat,lon,alt,stat,srv,cov)) return false;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = FRAME_UM982;
    msg.status.status = stat;
    msg.status.service = srv;
    msg.latitude = lat;
    msg.longitude = lon;
    msg.altitude = alt;
    msg.position_covariance_type = 2;
    for(int i=0;i<9;i++) msg.position_covariance[i] = cov[i];
    return true;
}

bool UM982GpsDriver::IsOnline(){return m_online;}