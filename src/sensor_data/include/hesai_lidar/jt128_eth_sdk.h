#ifndef HESAI_LIDAR_JT128_ETH_SDK_H
#define HESAI_LIDAR_JT128_ETH_SDK_H

// 空框架：适配hesai_imu_driver.cpp的基础调用（可根据实际SDK补充）
namespace hesai_lidar {
    class JT128EthSDK {
    public:
        // 构造/析构
        JT128EthSDK() = default;
        ~JT128EthSDK() = default;
        // 基础接口（根据cpp文件调用补充）
        bool initIMU(const std::string& ip = "192.168.1.201");
        void readIMUData(double& imu_x, double& imu_y, double& imu_z);
    };
}

#endif // HESAI_LIDAR_JT128_ETH_SDK_H