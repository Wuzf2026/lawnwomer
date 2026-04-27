#include "sensor_data/data_collect.h"

DataCollectManager::DataCollectManager(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
:m_nh(nh),m_nh_priv(nh_priv)
{
    pub_orbbec_cloud = m_nh.advertise<sensor_msgs::PointCloud2>("/sensor_data/orbbec/pointcloud2", 10);
    pub_hesai_imu    = m_nh.advertise<sensor_msgs::Imu>("/sensor_data/hesai/imu", 10);
    pub_um982_gps    = m_nh.advertise<sensor_msgs::NavSatFix>("/sensor_data/um982/gps", 10);
    LoadRosParam();
    m_orbbec.Init(m_cfg);
    m_hesai.Init(m_cfg);
    m_gps.Init(m_cfg);
}

void DataCollectManager::LoadRosParam()
{
    m_nh_priv.param<int>("cloud_width", m_cfg.cloud_width, 1280);
    m_nh_priv.param<int>("cloud_height", m_cfg.cloud_height, 720);
    m_nh_priv.param<double>("imu_rate", m_cfg.imu_rate, 100.0);
    m_nh_priv.param<std::string>("gps_tty", m_cfg.gps_tty, "/dev/ttyUSB0");
    m_nh_priv.param<int>("gps_baud", m_cfg.gps_baud, 115200);
}

void DataCollectManager::PublishAllSensor()
{
    sensor_msgs::PointCloud2 cloud_msg;
    sensor_msgs::Imu imu_msg;
    sensor_msgs::NavSatFix gps_msg;

    if(m_orbbec.GetPointCloud(cloud_msg)) pub_orbbec_cloud.publish(cloud_msg);
    if(m_hesai.GetImuData(imu_msg))        pub_hesai_imu.publish(imu_msg);
    if(m_gps.GetNavSatFix(gps_msg))        pub_um982_gps.publish(gps_msg);
}

void DataCollectManager::LoopOnce()
{
    PublishAllSensor();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_data_collect");
    ros::NodeHandle n;
    ros::NodeHandle np("~");
    DataCollectManager app(n,np);
    ros::Rate rate(50);
    while(ros::ok())
    {
        app.LoopOnce();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}