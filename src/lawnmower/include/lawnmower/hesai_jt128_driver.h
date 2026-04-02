#ifndef HESAI_JT128_DRIVER_H
#define HESAI_JT128_DRIVER_H

#include <ros/ros.h>
#include <socketcan_interface/socketcan.h>
#include "msg/LidarData.h"
#include "HesaiJT128SDK.h"

namespace lawnmower {

class HesaiJT128Driver {
public:
    HesaiJT128Driver(ros::NodeHandle& nh, const std::string& config_path);
    ~HesaiJT128Driver();

    bool init();
    void startCapture();
    void stopCapture();
    bool getStatus(std::string& status_detail);
    bool setParam(const std::string& param_key, const std::string& param_value);

private:
    void lidarCallback(const std::vector<PointXYZIT>& points);
    void loadConfig(const std::string& config_path);

    ros::NodeHandle nh_;
    ros::Publisher lidar_data_pub_;

    HesaiJT128Handle* sensor_handle_;
    int eth_socket_;

    std::string eth_ip_;
    std::string host_ip_;
    int port_;
    int scan_frequency_;
    std::string return_mode_;
    float filter_distance_;
    std::string topic_name_;

    bool is_running_;
    bool is_connected_;
};

}
#endif