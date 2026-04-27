#include "sensor_data/um982_gps_driver.h"
#include <cmath>

UM982GpsDriver::~UM982GpsDriver() {
    if(m_online) um982_close();
}

bool UM982GpsDriver::Init(const SensorConfig& cfg) {
    m_cfg = cfg;
    int ret = um982_serial_open(m_cfg.gps_tty, m_cfg.gps_baud);
    if(ret != 0) {
        m_online = false;
        return false;
    }
    m_online = true;
    return true;
}

bool UM982GpsDriver::GetNavSatFix(sensor_msgs::NavSatFix& gps_msg) {
    double lat = 0.0, lon = 0.0, alt = 0.0;
    int stat = 0, srv = 0;
    double cov = 0.0;

    if(!um982_get_gps_data(lat,lon,alt,stat,srv,cov)) return false;

    // 填充ROS GPS消息
    gps_msg.header.stamp = ros::Time::now();
    gps_msg.header.frame_id = m_cfg.frame_id;
    gps_msg.latitude = lat;
    gps_msg.longitude = lon;
    gps_msg.altitude = alt;
    gps_msg.status.status = stat;
    gps_msg.status.service = srv;

    // 协方差
    for(int i=0; i<9; i++) gps_msg.position_covariance[i] = 0.0;
    gps_msg.position_covariance[0] = cov;
    gps_msg.position_covariance[4] = cov;
    gps_msg.position_covariance[8] = cov*2;
    gps_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    return true;
}