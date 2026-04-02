#ifndef UM982_RTK_DRIVER_H
#define UM982_RTK_DRIVER_H

#include <ros/ros.h>
#include <serial/serial.h>
#include "msg/RTKData.h"
#include "UM982SDK.h"

namespace lawnmower {

class UM982RTKDriver {
public:
    UM982RTKDriver(ros::NodeHandle& nh, const std::string& config_path);
    ~UM982RTKDriver();

    bool init();
    void startCapture();
    void stopCapture();
    bool getStatus(std::string& status_detail);
    bool setParam(const std::string& param_key, const std::string& param_value);

private:
    void rtkCallback(const RTKFrame& frame);
    void loadConfig(const std::string& config_path);

    ros::NodeHandle nh_;
    ros::Publisher rtk_data_pub_;

    UM982Handle* sensor_handle_;
    serial::Serial* usb_serial_;

    std::string usb_port_;
    int baudrate_;
    std::string rtk_mode_;
    std::string base_station_ip_;
    int base_station_port_;
    int update_rate_;
    std::string topic_name_;

    bool is_running_;
    bool is_connected_;
};

}
#endif