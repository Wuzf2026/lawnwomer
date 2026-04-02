#include "lawnmower/hesai_jt128_driver.h"
#include <yaml-cpp/yaml.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sstream>

namespace lawnmower {

HesaiJT128Driver::HesaiJT128Driver(ros::NodeHandle& nh, const std::string& config_path)
    : nh_(nh), sensor_handle_(nullptr), eth_socket_(-1), is_running_(false), is_connected_(false) {
    loadConfig(config_path);
    lidar_data_pub_ = nh_.advertise<msg::LidarData>(topic_name_, 10);
}

HesaiJT128Driver::~HesaiJT128Driver() {
    stopCapture();
    if (sensor_handle_) HesaiJT128_Close(sensor_handle_);
    if (eth_socket_ >= 0) close(eth_socket_);
}

bool HesaiJT128Driver::init() {
    eth_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (eth_socket_ < 0) {
        ROS_ERROR("Hesai ETH socket create failed");
        return false;
    }

    struct sockaddr_in host_addr;
    memset(&host_addr, 0, sizeof(host_addr));
    host_addr.sin_family = AF_INET;
    host_addr.sin_addr.s_addr = inet_addr(host_ip_.c_str());
    host_addr.sin_port = htons(port_);

    if (bind(eth_socket_, (struct sockaddr*)&host_addr, sizeof(host_addr)) < 0) {
        ROS_ERROR("Hesai ETH bind failed");
        close(eth_socket_);
        eth_socket_ = -1;
        return false;
    }

    sensor_handle_ = HesaiJT128_Open(eth_socket_, eth_ip_.c_str(), port_);
    if (!sensor_handle_) {
        ROS_ERROR("Hesai SDK init failed");
        close(eth_socket_);
        eth_socket_ = -1;
        return false;
    }

    HesaiJT128_SetScanFrequency(sensor_handle_, scan_frequency_);
    HesaiJT128_SetReturnMode(sensor_handle_, return_mode_.c_str());
    HesaiJT128_SetMinDistance(sensor_handle_, filter_distance_);

    is_connected_ = true;
    ROS_INFO("Hesai JT128 init success");
    return true;
}

void HesaiJT128Driver::startCapture() {
    if (!is_connected_ || is_running_) return;
    is_running_ = true;

    HesaiJT128_RegisterPointCloudCallback(sensor_handle_,
        [](const std::vector<PointXYZIT>& points, void* user_data) {
            static_cast<HesaiJT128Driver*>(user_data)->lidarCallback(points);
        }, this);
    HesaiJT128_StartCapture(sensor_handle_);
    ROS_INFO("Hesai capture started");
}

void HesaiJT128Driver::stopCapture() {
    if (!is_running_) return;
    HesaiJT128_StopCapture(sensor_handle_);
    is_running_ = false;
    ROS_INFO("Hesai capture stopped");
}

bool HesaiJT128Driver::getStatus(std::string& status_detail) {
    std::stringstream ss;
    ss << "ETH IP: " << eth_ip_ 
       << ", Host IP: " << host_ip_
       << ", Port: " << port_
       << ", Connected: " << (is_connected_ ? "Yes" : "No")
       << ", Running: " << (is_running_ ? "Yes" : "No")
       << ", Scan Freq: " << scan_frequency_ << "Hz";
    status_detail = ss.str();
    return is_connected_;
}

bool HesaiJT128Driver::setParam(const std::string& param_key, const std::string& param_value) {
    if (!is_connected_) {
        ROS_ERROR("Hesai not connected, cannot set param");
        return false;
    }

    if (param_key == "scan_frequency") {
        scan_frequency_ = std::stoi(param_value);
        HesaiJT128_SetScanFrequency(sensor_handle_, scan_frequency_);
    } else if (param_key == "return_mode") {
        return_mode_ = param_value;
        HesaiJT128_SetReturnMode(sensor_handle_, return_mode_.c_str());
    } else if (param_key == "filter_distance") {
        filter_distance_ = std::stof(param_value);
        HesaiJT128_SetMinDistance(sensor_handle_, filter_distance_);
    } else {
        ROS_ERROR("Hesai unknown param: %s", param_key.c_str());
        return false;
    }
    ROS_INFO("Hesai set param %s = %s success", param_key.c_str(), param_value.c_str());
    return true;
}

void HesaiJT128Driver::lidarCallback(const std::vector<PointXYZIT>& points) {
    if (!is_running_) return;

    msg::LidarData msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "hesai_lidar";
    msg.point_num = points.size();

    msg.x.resize(msg.point_num);
    msg.y.resize(msg.point_num);
    msg.z.resize(msg.point_num);
    msg.intensity.resize(msg.point_num);

    for (size_t i = 0; i < points.size(); ++i) {
        msg.x[i] = points[i].x;
        msg.y[i] = points[i].y;
        msg.z[i] = points[i].z;
        msg.intensity[i] = points[i].intensity;
    }

    lidar_data_pub_.publish(msg);
}

void HesaiJT128Driver::loadConfig(const std::string& config_path) {
    YAML::Node config = YAML::LoadFile(config_path);
    eth_ip_ = config["hesai_jt128"]["eth_ip"].as<std::string>();
    host_ip_ = config["hesai_jt128"]["host_ip"].as<std::string>();
    port_ = config["hesai_jt128"]["port"].as<int>();
    scan_frequency_ = config["hesai_jt128"]["scan_frequency"].as<int>();
    return_mode_ = config["hesai_jt128"]["return_mode"].as<std::string>();
    filter_distance_ = config["hesai_jt128"]["filter_distance"].as<float>();
    topic_name_ = config["hesai_jt128"]["topic_name"].as<std::string>();
}

}