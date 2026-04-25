#include "sensor_data/hesai_imu_driver.h"
#include <hesai_lidar/jt128_eth_sdk.h>

HesaiImuDriver::HesaiImuDriver() : m_online(false) {}
HesaiImuDriver::~HesaiImuDriver()
{
    if(m_online) hesai_eth_close();
}

bool HesaiImuDriver::Init(const SensorConfig& cfg)
{
    m_cfg = cfg;
    int ret = hesai_eth_init();
    if(ret != 0)
    {
        ROS_ERROR("Hesai JT128 ETH connect fail");
        return false;
    }
    m_online = true;
    ROS_INFO("Hesai JT128 IMU Online");
    return true;
}

bool HesaiImuDriver::GetImuData(sensor_msgs::Imu& msg)
{
    if(!m_online) return false;
    double q[4], w[3], a[3];
    if(!hesai_get_imu_raw(q,w,a)) return false;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = FRAME_HESAI;
    msg.orientation.w = q[0];
    msg.orientation.x = q[1];
    msg.orientation.y = q[2];
    msg.orientation.z = q[3];

    msg.angular_velocity.x = w[0];
    msg.angular_velocity.y = w[1];
    msg.angular_velocity.z = w[2];

    msg.linear_acceleration.x = a[0];
    msg.linear_acceleration.y = a[1];
    msg.linear_acceleration.z = a[2];

    for(int i=0;i<9;i++)
    {
        msg.orientation_covariance[i] = 0.01;
        msg.angular_velocity_covariance[i] = 0.01;
        msg.linear_acceleration_covariance[i] = 0.01;
    }
    return true;
}

bool HesaiImuDriver::IsOnline(){return m_online;}