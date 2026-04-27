#include "sensor_data/orbbec_driver.h"
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// 注：真实使用Orbbec相机需安装驱动：
// sudo apt-get install ros-noetic-orbbec-camera

bool OrbbecDriver::Init(const SensorConfig& cfg) {
    m_cfg = cfg;
    m_cloud.reset(new PointCloudT);
    m_cloud->width = cfg.camera_width;
    m_cloud->height = cfg.camera_height;
    m_cloud->resize(m_cloud->width * m_cloud->height);
    m_online = true;
    printf("[Orbbec] Init success (sim: %dx%d@%dfps)\n", cfg.camera_width, cfg.camera_height, cfg.camera_fps);
    return true;
}

bool OrbbecDriver::GetColorImage(sensor_msgs::Image& img_msg) {
    if(!m_online) return false;
    img_msg.header.stamp = ros::Time::now();
    img_msg.header.frame_id = m_cfg.frame_id;
    img_msg.width = m_cfg.camera_width;
    img_msg.height = m_cfg.camera_height;
    img_msg.encoding = "bgr8";
    img_msg.step = img_msg.width * 3;
    img_msg.data.resize(img_msg.step * img_msg.height, 0);
    // 模拟图像数据
    for(size_t i=0; i<img_msg.data.size(); i++) img_msg.data[i] = rand()%255;
    return true;
}

bool OrbbecDriver::GetPointCloud(sensor_msgs::PointCloud2& pc_msg) {
    if(!m_online) return false;
    // 模拟点云数据
    for(auto& p : *m_cloud) {
        p.x = (rand()%1000)/100.0 -5.0;
        p.y = (rand()%1000)/100.0 -5.0;
        p.z = (rand()%500)/100.0;
        p.r = rand()%255; p.g = rand()%255; p.b = rand()%255;
    }
    // 转换为ROS消息
    pcl::toROSMsg(*m_cloud, pc_msg);
    pc_msg.header.stamp = ros::Time::now();
    pc_msg.header.frame_id = m_cfg.frame_id;
    return true;
}