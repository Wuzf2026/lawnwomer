#ifndef SENSOR_MSGS_IMU_H
#define SENSOR_MSGS_IMU_H
#include <std_msgs/Header.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
namespace sensor_msgs {
struct Imu {
    std_msgs::Header header;
    geometry_msgs::Quaternion orientation;
    geometry_msgs::Vector3 orientation_covariance;
    geometry_msgs::Vector3 angular_velocity;
    geometry_msgs::Vector3 angular_velocity_covariance;
    geometry_msgs::Vector3 linear_acceleration;
    geometry_msgs::Vector3 linear_acceleration_covariance;
};
} // namespace sensor_msgs
#endif // SENSOR_MSGS_IMU_H