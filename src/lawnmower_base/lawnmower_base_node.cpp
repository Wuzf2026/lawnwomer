#include "lawnmower_base/lawnmower_base.h"
int main(int argc, char** argv) {
    ros::init(argc, argv, "lawnmower_base");
    
    // 创建节点句柄
    ros::NodeHandle nh("~");
    
    // 初始化参数服务器
    LawnmowerBaseParams params;
    if (!params.loadParams(nh)) {
        ROS_ERROR("Failed to load parameters. Exiting...");
        return 1;
    }
    
    // 创建主控制节点实例
    LawnmowerBase lawnmower_base(nh, params);
    
    // 启动传感器接口
    if (!lawnmower_base.initSensors()) {
        ROS_ERROR("Failed to initialize sensors. Exiting...");
        return 1;
    }
    
    // 启动控制循环
    lawnmower_base.startControlLoop();
    
    // 保持节点运行
    ros::spin();
    
    return 0;
}
核心类实现 - lawnmower_base.h：

#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include "lawnmower_base/LawnmowerBaseConfig.h"
#include "third_party/handsfree_rtk/include/rtk.h"
#include "third_party/orbbec_camera/include/orbbec_camera.h"
#include "third_party/hesai_lidar/include/hesai_lidar.h"
class LawnmowerBase {
public:
    struct LawnmowerBaseParams {
        // 传感器配置参数
        std::string rtk_port;
        int rtk_baudrate;
        std::string orbbec_camera_id;
        std::string hesai_lidar_ip;
        int hesai_lidar_port;
        
        // 坐标系配置
        std::string base_frame_id;
        std::string odom_frame_id;
        
        // 发布频率
        double pointcloud_publish_rate;
        double imu_publish_rate;
        double gps_publish_rate;
        
        // 数据融合参数
        bool enable_ekf;
        bool enable_tf_broadcast;
        
        // 从参数服务器加载参数
        bool loadParams(ros::NodeHandle& nh) {
            if (!nh.getParam("rtk_port", rtk_port)) return false;
            if (!nh.getParam("rtk_baudrate", rtk_baudrate)) return false;
            if (!nh.getParam("orbbec_camera_id", orbbec_camera_id)) return false;
            if (!nh.getParam("hesai_lidar_ip", hesai_lidar_ip)) return false;
            if (!nh.getParam("hesai_lidar_port", hesai_lidar_port)) return false;
            if (!nh.getParam("base_frame_id", base_frame_id)) return false;
            if (!nh.getParam("odom_frame_id", odom_frame_id)) return false;
            if (!nh.getParam("pointcloud_publish_rate", pointcloud_publish_rate)) return false;
            if (!nh.getParam("imu_publish_rate", imu_publish_rate)) return false;
            if (!nh.getParam("gps_publish_rate", gps_publish_rate)) return false;
            if (!nh.getParam("enable_ekf", enable_ekf)) return false;
            if (!nh.getParam("enable_tf_broadcast", enable_tf_broadcast)) return false;
            return true;
        }
    };
    
    LawnmowerBase(ros::NodeHandle& nh, const LawnmowerBaseParams& params)
        : nh_(nh), params_(params),
          pointcloud_publisher_(nh.advertise<sensor_msgs::PointCloud2>("/lawnmower/pointcloud", 10)),
          imu_publisher_(nh.advertise<sensor_msgs::Imu>("/lawnmower/imu", 10)),
          gps_publisher_(nh.advertise<sensor_msgs::NavSatFix>("/lawnmower/gps", 10)),
          tf_broadcaster_(new tf2_ros::TransformBroadcaster()),
          dynamic_reconfigure_server_(nh) {
        
        // 初始化动态参数配置
        dynamic_reconfigure_server_.setCallback(
            boost::bind(&LawnmowerBase::dynamicReconfigureCallback, this, _1, _2));
        
        // 初始化TF广播
        if (params.enable_tf_broadcast) {
            tf_timer_ = nh.createTimer(ros::Duration(1.0/100), &LawnmowerBase::tfBroadcastCallback, this);
        }
    }
    
    // 初始化传感器接口
    bool initSensors() {
        // 初始化RTK模块
        rtk_.reset(new RTKInterface(params_.rtk_port, params_.rtk_baudrate));
        if (!rtk_->init()) {
            ROS_ERROR("Failed to initialize RTK module");
            return false;
        }
        
        // 初始化Orbbec相机
        orbbec_camera_.reset(new OrbbecCamera(params_.orbbec_camera_id));
        if (!orbbec_camera_->init()) {
            ROS_ERROR("Failed to initialize Orbbec camera");
            return false;
        }
        
        // 初始化禾赛激光雷达
        hesai_lidar_.reset(new HesaiLidar(params_.hesai_lidar_ip, params_.hesai_lidar_port));
        if (!hesai_lidar_->init()) {
            ROS_ERROR("Failed to initialize Hesai lidar");
            return false;
        }
        
        return true;
    }
    
    // 启动控制循环
    void startControlLoop() {
        // 创建数据采集线程
        data_thread_ = std::thread(&LawnmowerBase::dataCollectionLoop, this);
        
        // 创建数据发布线程
        publish_thread_ = std::thread(&LawnmowerBase::dataPublishLoop, this);
    }
    
private:
    ros::NodeHandle nh_;
    LawnmowerBaseParams params_;
    
    // 传感器接口指针
    std::unique_ptr<RTKInterface> rtk_;
    std::unique_ptr<OrbbecCamera> orbbec_camera_;
    std::unique_ptr<HesaiLidar> hesai_lidar_;
    
    // 发布器
    ros::Publisher pointcloud_publisher_;
    ros::Publisher imu_publisher_;
    ros::Publisher gps_publisher_;
    
    // TF相关
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    ros::Timer tf_timer_;
    
    // 动态参数配置
    dynamic_reconfigure::Server<lawnmower_base::LawnmowerBaseConfig> dynamic_reconfigure_server_;
    
    // 线程
    std::thread data_thread_;
    std::thread publish_thread_;
    
    // 数据缓冲区
    std::mutex data_mutex_;
    sensor_msgs::PointCloud2 latest_pointcloud_;
    sensor_msgs::Imu latest_imu_;
    sensor_msgs::NavSatFix latest_gps_;
    
    // 动态参数回调
    void dynamicReconfigureCallback(lawnmower_base::LawnmowerBaseConfig& config, uint32_t level) {
        ROS_INFO("Dynamic reconfigure request: pointcloud_rate=%f, imu_rate=%f, gps_rate=%f",
                 config.pointcloud_publish_rate, config.imu_publish_rate, config.gps_publish_rate);
        
        // 更新发布频率
        params_.pointcloud_publish_rate = config.pointcloud_publish_rate;
        params_.imu_publish_rate = config.imu_publish_rate;
        params_.gps_publish_rate = config.gps_publish_rate;
        params_.enable_ekf = config.enable_ekf;
        params_.enable_tf_broadcast = config.enable_tf_broadcast;
    }
    
    // 数据采集循环
    void dataCollectionLoop() {
        ros::Rate rate(100); // 100Hz采集频率
        
        while (ros::ok()) {
            // 采集RTK数据
            if (rtk_->readGPS(latest_gps_)) {
                ROS_DEBUG("Received RTK data: %f, %f, %f",
                          latest_gps_.latitude, latest_gps_.longitude, latest_gps_.altitude);
            }
            
            // 采集激光雷达IMU数据
            if (hesai_lidar_->readIMU(latest_imu_)) {
                ROS_DEBUG("Received IMU data: %f, %f, %f",
                          latest_imu_.angular_velocity.x, latest_imu_.angular_velocity.y, latest_imu_.angular_velocity.z);
            }
            
            // 采集点云数据
            sensor_msgs::PointCloud2 new_pointcloud;
            if (orbbec_camera_->readPointCloud(new_pointcloud)) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_pointcloud_ = new_pointcloud;
                ROS_DEBUG("Received pointcloud with %d points", new_pointcloud.width * new_pointcloud.height);
            }
            
            rate.sleep();
        }
    }
    
    // 数据发布循环
    void dataPublishLoop() {
        ros::Rate pointcloud_rate(params_.pointcloud_publish_rate);
        ros::Rate imu_rate(params_.imu_publish_rate);
        ros::Rate gps_rate(params_.gps_publish_rate);
        
        ros::Time last_pointcloud_time = ros::Time::now();
        ros::Time last_imu_time = ros::Time::now();
        ros::Time last_gps_time = ros::Time::now();
        
        while (ros::ok()) {
            // 发布点云数据
            if ((ros::Time::now() - last_pointcloud_time).toSec() > 1.0/params_.pointcloud_publish_rate) {
                std::lock_guard<std::mutex> lock(data_mutex_);
                pointcloud_publisher_.publish(latest_pointcloud_);
                last_pointcloud_time = ros::Time::now();
            }
            
            // 发布IMU数据
            if ((ros::Time::now() - last_imu_time).toSec() > 1.0/params_.imu_publish_rate) {
                imu_publisher_.publish(latest_imu_);
                last_imu_time = ros::Time::now();
            }
            
            // 发布GPS数据
            if ((ros::Time::now() - last_gps_time).toSec() > 1.0/params_.gps_publish_rate) {
                gps_publisher_.publish(latest_gps_);
                last_gps_time = ros::Time::now();
            }
            
            // 短暂休眠避免CPU占用过高
            ros::Duration(0.01).sleep();
        }
    }
    
    // TF广播回调
    void tfBroadcastCallback(const ros::TimerEvent& event) {
        // 创建TF变换（示例：base_link到odom的变换）
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = params_.odom_frame_id;
        transform.child_frame_id = params_.base_frame_id;
        
        // 设置变换（这里是示例数据，实际需要根据定位结果更新）
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        
        // 单位四元数表示没有旋转
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        
        tf_broadcaster_->sendTransform(transform);
    }
};