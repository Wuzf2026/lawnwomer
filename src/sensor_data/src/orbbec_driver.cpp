#include "sensor_data/orbbec_driver.h"
#include <orbbec_camera/orbbec_sdk.h>
#include <pcl_conversions/pcl_conversions.h>

OrbbecDriver::OrbbecDriver() : m_online(false) {}
OrbbecDriver::~OrbbecDriver()
{
    if(m_online) orbbec_close_device();
}

bool OrbbecDriver::Init(const SensorConfig& cfg)
{
    m_cfg = cfg;
    int ret = orbbec_usb_init(m_cfg.cloud_width, m_cfg.cloud_height);
    if(ret != 0)
    {
        ROS_ERROR("Orbbec Gemini335 USB init fail");
        return false;
    }
    m_online = true;
    ROS_INFO("Orbbec Gemini335 Online");
    return true;
}

bool OrbbecDriver::GetPointCloud2(sensor_msgs::PointCloud2& msg)
{
    if(!m_online) return false;
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    if(!orbbec_get_cloud_data(pcl_cloud)) return false;

    pcl::toROSMsg(pcl_cloud, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = FRAME_ORBBEC;
    msg.height = pcl_cloud.height;
    msg.width = pcl_cloud.width;
    msg.is_bigendian = false;
    msg.point_step = 24;
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = true;
    return true;
}

bool OrbbecDriver::IsOnline(){return m_online;}