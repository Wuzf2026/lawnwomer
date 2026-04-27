#include "sensor_data/sensor_data_node.h"
#include <thread>

SensorDataNode::SensorDataNode(ros::NodeHandle& nh) : m_nh(nh) {
    // 初始化发布者
    m_imu_pub = m_nh.advertise<sensor_msgs::Imu>("/sensor/imu", 10);
    m_gps_pub = m_nh.advertise<sensor_msgs::NavSatFix>("/sensor/gps", 10);
    m_img_pub = m_nh.advertise<sensor_msgs::Image>("/sensor/color_img", 10);
    m_pc_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/sensor/point_cloud", 10);

    // 加载配置
    LoadConfig();

    // 初始化传感器
    if(m_imu_enable && !m_imu_driver.Init(m_cfg)) ROS_ERROR("IMU init failed");
    if(m_gps_enable && !m_gps_driver.Init(m_cfg)) ROS_ERROR("GPS init failed");
    if(m_camera_enable && !m_camera_driver.Init(m_cfg)) ROS_ERROR("Camera init failed");
}

void SensorDataNode::LoadConfig() {
    m_nh.param<bool>("imu/enable", m_imu_enable, true);
    m_nh.param<bool>("gps/enable", m_gps_enable, true);
    m_nh.param<bool>("camera/enable", m_camera_enable, true);
    m_nh.param<double>("pub_freq", m_pub_freq, 10.0);
    m_nh.param<std::string>("frame_id", m_cfg.frame_id, "base_link");
    m_nh.param<std::string>("gps/tty", m_cfg.gps_tty, "/dev/ttyUSB0");
    m_nh.param<int>("gps/baud", m_cfg.gps_baud, 115200);
}

void SensorDataNode::ImuPublishLoop() {
    ros::Rate rate(m_pub_freq);
    while(ros::ok() && m_imu_enable) {
        sensor_msgs::Imu imu;
        if(m_imu_driver.GetImuData(imu)) m_imu_pub.publish(imu);
        rate.sleep();
    }
}

void SensorDataNode::GpsPublishLoop() {
    ros::Rate rate(m_pub_freq);
    while(ros::ok() && m_gps_enable) {
        sensor_msgs::NavSatFix gps;
        if(m_gps_driver.GetNavSatFix(gps)) m_gps_pub.publish(gps);
        rate.sleep();
    }
}

void SensorDataNode::CameraPublishLoop() {
    ros::Rate rate(m_pub_freq);
    while(ros::ok() && m_camera_enable) {
        sensor_msgs::Image img;
        if(m_camera_driver.GetColorImage(img)) m_img_pub.publish(img);
        
        sensor_msgs::PointCloud2 pc;
        if(m_camera_driver.GetPointCloud(pc)) m_pc_pub.publish(pc);
        rate.sleep();
    }
}

void SensorDataNode::Run() {
    std::thread imu_th(&SensorDataNode::ImuPublishLoop, this);
    std::thread gps_th(&SensorDataNode::GpsPublishLoop, this);
    std::thread cam_th(&SensorDataNode::CameraPublishLoop, this);
    imu_th.join();
    gps_th.join();
    cam_th.join();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_data_node");
    ros::NodeHandle nh("~");
    SensorDataNode node(nh);
    node.Run();
    return 0;
}