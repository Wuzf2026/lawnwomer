#ifndef SENSOR_COMMON_CONFIG_H
#define SENSOR_COMMON_CONFIG_H

// 保留原 common.h 中的 ROS/传感器消息头文件依赖
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>

// 保留原 common.h 中的坐标系宏定义
#define FRAME_ORBBEC    "camera_link"
#define FRAME_HESAI     "lidar_link"
#define FRAME_UM982     "gps_link"

// 合并后的 SensorConfig 结构体（包含两个文件的所有字段，保留默认值）
struct SensorConfig {
    // 通用配置（来自 sensor_config.h）
    bool enable = false;
    std::string frame_id = "base_link";

    // GPS配置（两个文件重复字段，保留 sensor_config.h 的默认值）
    std::string gps_tty = "/dev/ttyUSB0";
    int gps_baud = 115200;

    // IMU配置（来自 sensor_config.h + common.h 的 imu_rate）
    std::string imu_ip = "192.168.1.200";
    int imu_port = 9000;
    double imu_rate;  // common.h 中无默认值，保持原生定义

    // Orbbec相机配置（字段名对齐：common.h 的 cloud_* → sensor_config.h 的 camera_*）
    int camera_width = 640;    // 对应 common.h 的 cloud_width
    int camera_height = 480;   // 对应 common.h 的 cloud_height
    int camera_fps = 30;       // 来自 sensor_config.h
};

// 兼容原 common.h 中 cloud_width/cloud_height 的字段名（可选，保证旧代码不报错）
// 若不需要兼容旧代码，可删除以下两行
#define cloud_width camera_width
#define cloud_height camera_height

#endif // SENSOR_COMMON_CONFIG_H