#include "sensor_data/sensor_fusion.h"

// 数据融合实现
Eigen::Isometry3d SensorFusion::fusion(const std::vector<double>& rtk_data,
                                       const std::vector<float>& lidar_data,
                                       const std::vector<uchar>& camera_data,
                                       const std::vector<double>& imu_data) {
    // 初始化位姿
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

    // 示例：融合RTK位置数据
    if (!rtk_data.empty()) {
        pose.translation() = Eigen::Vector3d(rtk_data[0], rtk_data[1], rtk_data[2]) * rtk_weight_;
    }

    // 示例：融合IMU姿态数据
    if (!imu_data.empty()) {
        Eigen::Quaterniond q(imu_data[3], imu_data[0], imu_data[1], imu_data[2]);
        pose.rotate(q * imu_weight_);
    }

    return pose;
}