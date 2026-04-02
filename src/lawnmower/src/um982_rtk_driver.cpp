#include "lawnmower/um982_rtk_driver.h"
#include <yaml-cpp/yaml.h>
#include <sstream>

namespace lawnmower {

UM982RTKDriver::UM982RTKDriver(ros::NodeHandle& nh, const std::string& config_path)
    : nh_(nh), sensor_handle_(nullptr), usb_serial_(nullptr), is_running_(false), is_connected_(false) {
    loadConfig(config_path);
    rtk_data_pub_ = nh_.advertise<msg::RTKData>(topic_name_, 10);
}

UM982RTKDriver::~UM982RTKDriver() {
    stopCapture();
    if (sensor_handle_) UM982_Close(sensor_handle_);
    if (usb_serial_) delete usb_serial_;
}

bool UM982RTKDriver::init() {
    try {
        usb_serial_ = new serial::Serial(usb_port_, baudrate_, serial::Timeout::simpleTimeout(1000));
        if (!usb_serial_->isOpen()) {
            ROS_ERROR("UM982 USB serial open failed");
            delete usb_serial_;
            usb_serial_ = nullptr;
            return false;
        }
    } catch (serial::IOException& e) {
        ROS_ERROR("UM982 USB serial exception: %s", e.what());
        return false;
    }

    sensor_handle_ = UM982_Open(usb_serial_->getFD(), baudrate_);
    if (!sensor_handle_) {
        ROS_ERROR("UM982 SDK init failed");
        delete usb_serial_;
        usb_serial_ = nullptr;
        return false;
    }

    UM982_SetRTKMode(sensor_handle_, rtk_mode_.c_str());
    UM982_SetBaseStation(sensor_handle_, base_station_ip_.c_str(), base_station_port_);
    UM982_SetUpdateRate(sensor_handle_, update_rate_);

    is_connected_ = true;
    ROS_INFO("UM982 RTK init success");
    return true;
}

void UM982RTKDriver::startCapture() {
    if (!is_connected_ || is_running_) return;
    is_running_ = true;

    UM982_RegisterRTKCallback(sensor_handle_,
        [](const RTKFrame& frame, void* user_data) {
            static_cast<UM982RTKDriver*>(user_data)->rtkCallback(frame);
        }, this);
    UM982_StartCapture(sensor_handle_);
    ROS_INFO("UM982 capture started");
}

void UM982RTKDriver::stopCapture() {
    if (!is_running_) return;
    UM982_StopCapture(sensor_handle_);
    is_running_ = false;
    ROS_INFO("UM982 capture stopped");
}

bool UM982RTKDriver::getStatus(std::string& status_detail) {
    std::stringstream ss;
    ss << "USB Port: " << usb_port_ 
       << ", RTK Mode: " << rtk_mode_
       << ", Base IP: " << base_station_ip_
       << ", Connected: " << (is_connected_ ? "Yes" : "No")
       << ", Running: " << (is_running_ ? "Yes" : "No")
       << ", Update Rate: " << update_rate_ << "Hz";
    status_detail = ss.str();
    return is_connected_;
}

bool UM982RTKDriver::setParam(const std::string& param_key, const std::string& param_value) {
    if (!is_connected_) {
        ROS_ERROR("UM982 not connected, cannot set param");
        return false;
    }

    if (param_key == "rtk_mode") {
        rtk_mode_ = param_value;
        UM982_SetRTKMode(sensor_handle_, rtk_mode_.c_str());
    } else if (param_key == "base_station_ip") {
        base_station_ip_ = param_value;
        UM982_SetBaseStation(sensor_handle_, base_station_ip_.c_str(), base_station_port_);
    } else if (param_key == "base_station_port") {
        base_station_port_ = std::stoi(param_value);
        UM982_SetBaseStation(sensor_handle_, base_station_ip_.c_str(), base_station_port_);
    } else if (param_key == "update_rate") {
        update_rate_ = std::stoi(param_value);
        UM982_SetUpdateRate(sensor_handle_, update_rate_);
    } else {
        ROS_ERROR("UM982 unknown param: %s", param_key.c_str());
        return false;
    }
    ROS_INFO("UM982 set param %s = %s success", param_key.c_str(), param_value.c_str());
    return true;
}

void UM982RTKDriver::rtkCallback(const RTKFrame& frame) {
    if (!is_running_) return;

    msg::RTKData msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "um982_rtk";
    msg.latitude = frame.latitude;
    msg.longitude = frame.longitude;
    msg.altitude = frame.altitude;
    msg.heading = frame.heading;
    msg.fix_status = frame.fix_status;
    msg.hdop = frame.hdop;

    rtk_data_pub_.publish(msg);
}

void UM982RTKDriver::loadConfig(const std::string& config_path) {
    YAML::Node config = YAML::LoadFile(config_path);
    usb_port_ = config["um982_rtk"]["usb_port"].as<std::string>();
    baudrate_ = config["um982_rtk"]["baudrate"].as<int>();
    rtk_mode_ = config["um982_rtk"]["rtk_mode"].as<std::string>();
    base_station_ip_ = config["um982_rtk"]["base_station_ip"].as<std::string>();
    base_station_port_ = config["um982_rtk"]["base_station_port"].as<int>();
    update_rate_ = config["um982_rtk"]["update_rate"].as<int>();
    topic_name_ = config["um982_rtk"]["topic_name"].as<std::string>();
}

}