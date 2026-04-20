#include "lawnmower_base/lidar_driver.h"

LiDARDriver::LiDARDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      lidar_handler_(nullptr),
      new_pointcloud_(false)
{
    // 读取雷达网络参数
    nh_private_.param<std::string>("lidar_ip", lidar_ip_, "192.168.1.201");
    nh_private_.param<int>("lidar_port", lidar_port_, 2368);
    nh_private_.param<std::string>("frame_id", lidar_frame_id_, "hesai_jt128_link");
    nh_private_.param<bool>("remove_nan_point", remove_nan_, true);
}

bool LiDARDriver::init()
{
    // 初始化禾赛JT128网口雷达SDK
    try
    {
        // 初始化禾赛雷达驱动实例
        lidar_handler_ = hesai::LidarFactory::CreateLidar("JT128");
        if(!lidar_handler_)
        {
            ROS_ERROR("Create hesai JT128 lidar driver failed");
            return false;
        }

        // 网口ETH通信参数配置
        hesai::LidarParam param;
        param.lidar_ip = lidar_ip_;
        param.data_port = lidar_port_;
        param.device_type = hesai::LidarType::JT128;

        lidar_handler_->LoadParam(param);
        lidar_handler_->StartRecv();

        ROS_INFO("Hesai JT128 lidar eth driver init success");
    }
    catch(std::exception& e)
    {
        ROS_ERROR("Lidar init exception:%s",e.what());
        return false;
    }

    // ROS标准PointCloud2发布器
    pub_lidar_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/hesai_jt128/pointcloud2", 10);
    pub_lidar_imu_ = nh_.advertise<sensor_msgs::Imu>("/hesai_jt128/imu", 10);

    return true;
}

void LiDARDriver::readData()
{
    if(!lidar_handler_) return;

    hesai::PointCloudMsg hs_cloud;
    hesai::ImuMsg hs_imu;

    // 获取雷达整帧点云
    bool get_cloud = lidar_handler_->GetPointCloud(hs_cloud);
    bool get_imu = lidar_handler_->GetLidarImu(hs_imu);

    // 封装ROS标准PointCloud2
    if(get_cloud)
    {
        sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);

        cloud_msg->header.stamp = ros::Time().fromSec(hs_cloud.timestamp);
        cloud_msg->header.frame_id = lidar_frame_id_;
        cloud_msg->height = 1;
        cloud_msg->width = hs_cloud.points_num;
        cloud_msg->is_bigendian = false;
        cloud_msg->point_step = 16;
        cloud_msg->row_step = cloud_msg->width * cloud_msg->point_step;
        cloud_msg->is_dense = !remove_nan_;

        // XYZINTENSITY标准点域
        cloud_msg->fields.resize(4);
        cloud_msg->fields[0].name = "x"; cloud_msg->fields[0].offset=0; cloud_msg->fields[0].datatype=7; cloud_msg->fields[0].count=1;
        cloud_msg->fields[1].name = "y"; cloud_msg->fields[1].offset=4; cloud_msg->fields[1].datatype=7; cloud_msg->fields[1].count=1;
        cloud_msg->fields[2].name = "z"; cloud_msg->fields[2].offset=8; cloud_msg->fields[2].datatype=7; cloud_msg->fields[2].count=1;
        cloud_msg->fields[3].name = "intensity"; cloud_msg->fields[3].offset=12; cloud_msg->fields[3].datatype=2; cloud_msg->fields[3].count=1;

        cloud_msg->data.resize(cloud_msg->width * cloud_msg->point_step);
        memcpy(&cloud_msg->data[0], hs_cloud.point_data, cloud_msg->data.size());

        point_cloud_msg_ = cloud_msg;
        new_pointcloud_ = true;
        pub_lidar_pointcloud_.publish(cloud_msg);
    }

    // 封装雷达内置IMU标准sensor_msgs/Imu
    if(get_imu)
    {
        sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);
        imu_msg->header.stamp = ros::Time().fromSec(hs_imu.timestamp);
        imu_msg->header.frame_id = lidar_frame_id_;

        imu_msg->angular_velocity.x = hs_imu.gyro_x;
        imu_msg->angular_velocity.y = hs_imu.gyro_y;
        imu_msg->angular_velocity.z = hs_imu.gyro_z;

        imu_msg->linear_acceleration.x = hs_imu.acc_x;
        imu_msg->linear_acceleration.y = hs_imu.acc_y;
        imu_msg->linear_acceleration.z = hs_imu.acc_z;

        // 姿态四元数
        imu_msg->orientation.x = hs_imu.qx;
        imu_msg->orientation.y = hs_imu.qy;
        imu_msg->orientation.z = hs_imu.qz;
        imu_msg->orientation.w = hs_imu.qw;

        // 固定协方差矩阵
        memset(imu_msg->orientation_covariance.data(),0,72);
        memset(imu_msg->angular_velocity_covariance.data(),0,72);
        memset(imu_msg->linear_acceleration_covariance.data(),0,72);

        imu_msg->orientation_covariance[0]=0.001;
        imu_msg->orientation_covariance[4]=0.001;
        imu_msg->orientation_covariance[8]=0.001;
        imu_msg->angular_velocity_covariance[0]=0.0005;
        imu_msg->angular_velocity_covariance[4]=0.0005;
        imu_msg->angular_velocity_covariance[8]=0.0005;
        imu_msg->linear_acceleration_covariance[0]=0.01;
        imu_msg->linear_acceleration_covariance[4]=0.01;
        imu_msg->linear_acceleration_covariance[8]=0.01;

        lidar_imu_msg_ = imu_msg;
        pub_lidar_imu_.publish(imu_msg);
    }
}