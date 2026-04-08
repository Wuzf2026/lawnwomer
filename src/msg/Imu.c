#include "Imu.h"
#include <math.h>
// 初始化Imu消息
void imu_init(sensor_msgs::Imu* msg) {
    msg->header = std_msgs::Header();
    
    // 初始化四元数（单位四元数）
    msg->orientation.x = 0.0f;
    msg->orientation.y = 0.0f;
    msg->orientation.z = 0.0f;
    msg->orientation.w = 1.0f;
    
    // 初始化协方差矩阵为单位矩阵
    msg->orientation_covariance.x = 1.0f;
    msg->orientation_covariance.y = 0.0f;
    msg->orientation_covariance.z = 0.0f;
    
    msg->angular_velocity.x = 0.0f;
    msg->angular_velocity.y = 0.0f;
    msg->angular_velocity.z = 0.0f;
    
    msg->linear_acceleration.x = 0.0f;
    msg->linear_acceleration.y = 0.0f;
    msg->linear_acceleration.z = 0.0f;
}
// 设置姿态（欧拉角转四元数）
void imu_set_orientation(sensor_msgs::Imu* msg, float roll, float pitch, float yaw) {
    // 欧拉角转四元数算法
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    
    msg->orientation.w = cr * cp * cy + sr * sp * sy;
    msg->orientation.x = sr * cp * cy - cr * sp * sy;
    msg->orientation.y = cr * sp * cy + sr * cp * sy;
    msg->orientation.z = cr * cp * sy - sr * sp * cy;
}
// 设置角速度
void imu_set_angular_velocity(sensor_msgs::Imu* msg, float x, float y, float z) {
    msg->angular_velocity.x = x;
    msg->angular_velocity.y = y;
    msg->angular_velocity.z = z;
}
// 设置线性加速度
void imu_set_linear_acceleration(sensor_msgs::Imu* msg, float x, float y, float z) {
    msg->linear_acceleration.x = x;
    msg->linear_acceleration.y = y;
    msg->linear_acceleration.z = z;
}