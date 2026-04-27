#include "sensor_data/hesai_imu_driver.h"
#include <cstdio>

// 模拟ETH初始化（实际需对接硬件SDK）
int hesai_eth_init() {
    printf("[Hesai IMU] ETH init success (sim)\n");
    return 0;
}

// 模拟ETH关闭
void hesai_eth_close() {
    printf("[Hesai IMU] ETH closed\n");
}

// 模拟获取IMU原始数据
bool hesai_get_imu_raw(double q[4], double w[3], double a[3]) {
    q[0] = 0.0; q[1] = 0.0; q[2] = 0.0; q[3] = 1.0;
    w[0] = 0.01; w[1] = 0.02; w[2] = 0.005;
    a[0] = 0.1; a[1] = 0.05; a[2] = 9.81;
    return true;
}