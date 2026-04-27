#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

// 传感器数据融合核心类
class SensorFusion {
public:
    /**
     * @brief 融合RTK+激光雷达+相机+IMU数据
     * @param rtk_data RTK定位数据
     * @param lidar_data 激光雷达点云数据
     * @param camera_data 相机图像数据
     * @param imu_data IMU惯性数据
     * @return 融合后的位姿（Eigen矩阵）
     */
    Eigen::Isometry3d fusion(const std::vector<double>& rtk_data,
                             const std::vector<float>& lidar_data,
                             const std::vector<unsigned char>& camera_data,
                             const std::vector<double>& imu_data);

private:
    // 融合权重（可配置）
    double rtk_weight_ = 0.4;
    double lidar_weight_ = 0.3;
    double camera_weight_ = 0.1;
    double imu_weight_ = 0.2;
};

#endif  // SENSOR_FUSION_H