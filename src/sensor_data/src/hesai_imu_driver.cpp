#include "sensor_data/hesai_imu_driver.h"
#include <cmath>

HesaiImuDriver::~HesaiImuDriver() {
    if(m_online) hesai_eth_close();
}

bool HesaiImuDriver::Init(const SensorConfig& cfg) {
    m_cfg = cfg;
    int ret = hesai_eth_init();
    if(ret != 0) {
        m_online = false;
        return false;
    }
    m_online = true;
    return true;
}

bool HesaiImuDriver::GetImuData(sensor_msgs::Imu& imu_msg) {
    double q[4] = {0.0, 0.0, 0.0, 1.0}; // 默认四元数
    double w[3] = {0.0, 0.0, 0.0};      // 角速度
    double a[3] = {0.0, 0.0, 9.81};     // 重力加速度

    if(!hesai_get_imu_raw(q,w,a)) return false;

    // 填充ROS IMU消息
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = m_cfg.frame_id;
    imu_msg.orientation.x = q[0];
    imu_msg.orientation.y = q[1];
    imu_msg.orientation.z = q[2];
    imu_msg.orientation.w = q[3];
    imu_msg.angular_velocity.x = w[0];
    imu_msg.angular_velocity.y = w[1];
    imu_msg.angular_velocity.z = w[2];
    imu_msg.linear_acceleration.x = a[0];
    imu_msg.linear_acceleration.y = a[1];
    imu_msg.linear_acceleration.z = a[2];

    // 协方差初始化
    for(int i=0; i<9; i++) {
        imu_msg.orientation_covariance[i] = 0.01;
        imu_msg.angular_velocity_covariance[i] = 0.01;
        imu_msg.linear_acceleration_covariance[i] = 0.01;
    }
    return true;
}