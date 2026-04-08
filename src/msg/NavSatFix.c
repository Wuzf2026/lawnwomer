#include "NavSatFix.h"
#include <string.h>
// 初始化NavSatFix消息
void nav_sat_fix_init(sensor_msgs::NavSatFix* msg) {
    msg->header = std_msgs::Header();
    msg->latitude = 0.0;
    msg->longitude = 0.0;
    msg->altitude = 0.0;
    
    // 初始化协方差矩阵为单位矩阵
    memset(msg->position_covariance, 0, sizeof(msg->position_covariance));
    msg->position_covariance[0] = 1.0;
    msg->position_covariance[4] = 1.0;
    msg->position_covariance[8] = 1.0;
    
    msg->position_covariance_type = 0; // COVARIANCE_TYPE_UNKNOWN
}
// 设置位置信息
void nav_sat_fix_set_position(sensor_msgs::NavSatFix* msg, double lat, double lon, double alt) {
    msg->latitude = lat;
    msg->longitude = lon;
    msg->altitude = alt;
}
// 设置协方差矩阵
void nav_sat_fix_set_covariance(sensor_msgs::NavSatFix* msg, const double* covariance) {
    memcpy(msg->position_covariance, covariance, sizeof(msg->position_covariance));
}
// 设置协方差类型
void nav_sat_fix_set_covariance_type(sensor_msgs::NavSatFix* msg, uint8_t type) {
    msg->position_covariance_type = type;
}