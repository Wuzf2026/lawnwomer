#include "sensor_data/um982_gps_driver.h"
#include <sensor_msgs/NavSatFix.h>
#include <cstdio>

// 模拟串口打开
int um982_serial_open(const std::string& tty, int baud) {
    printf("[UM982 GPS] Open serial: %s (baud: %d)\n", tty.c_str(), baud);
    return 0;
}

// 模拟串口关闭
void um982_close() {
    printf("[UM982 GPS] Serial closed\n");
}

// 模拟获取GPS数据（示例：北京坐标）
bool um982_get_gps_data(double& lat, double& lon, double& alt, int& stat, int& srv, double& cov) {
    lat = 39.908823;
    lon = 116.397470;
    alt = 50.0;
    stat = sensor_msgs::NavSatStatus::STATUS_FIX;
    srv = sensor_msgs::NavSatStatus::SERVICE_GPS;
    cov = 0.5;
    return true;
}