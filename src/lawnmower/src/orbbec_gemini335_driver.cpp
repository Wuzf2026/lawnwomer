#include "lawnmower/orbbec_gemini335_driver.h"
#include <yaml-cpp/yaml.h>
#include <sstream>

namespace lawnmower {

OrbbecGemini335Driver::OrbbecGemini335Driver(ros::NodeHandle& nh, const std::string& config_path)
    : nh_(nh), sensor_handle_(nullptr), usb_handle_(nullptr), is_running_(false), is_connected_(false) {
    loadConfig(config_path);
    stereo_img_pub_ = nh_.advertise<msg::StereoImage>(topic_name_, 10);
}

OrbbecGemini335Driver::~OrbbecGemini335Driver() {
    stopCapture();
    if (sensor_handle_) OBGemini335_Close(sensor_handle_);
    if (usb_handle_) libusb_close(usb_handle_);
    libusb_exit(nullptr);
}

bool OrbbecGemini335Driver::init() {
    int ret = libusb_init(nullptr);
    if (ret < 0) {
        ROS_ERROR("Orbbec USB init failed: %d", ret);
        return false;
    }

    usb_handle_ = libusb_open_device_with_vid_pid(nullptr, 0x0483, 0x5750);
    if (!usb_handle_) {
        ROS_ERROR("Orbbec USB open failed: %s", usb_port_.c_str());
        return false;
    }

    sensor_handle_ = OBGemini335_Open(usb_handle_, baudrate_);
    if (!sensor_handle_) {
        ROS_ERROR("Orbbec SDK init failed");
        libusb_close(usb_handle_);
        return false;
    }

    OBGemini335_SetFrameRate(sensor_handle_, frame_rate_);
    OBGemini335_SetResolution(sensor_handle_, resolution_.c_str());
    OBGemini335_SetAutoExposure(sensor_handle_, auto_exposure_);
    if (!auto_exposure_) OBGemini335_SetExposure(sensor_handle_, exposure_value_);
    OBGemini335_SetIMUEnable(sensor_handle_, true);

    is_connected_ = true;
    ROS_INFO("Orbbec Gemini335 init success");
    return true;
}

void OrbbecGemini335Driver::startCapture() {
    if (!is_connected_ || is_running_) return;
    is_running_ = true;

    OBGemini335_RegisterImageCallback(sensor_handle_, 
        [](const cv::Mat& left, const cv::Mat& right, void* user_data) {
            static_cast<OrbbecGemini335Driver*>(user_data)->imageCallback(left, right);
        }, this);
    OBGemini335_StartCapture(sensor_handle_);
    ROS_INFO("Orbbec capture started");
}

void OrbbecGemini335Driver::stopCapture() {
    if (!is_running_) return;
    OBGemini335_StopCapture(sensor_handle_);
    is_running_ = false;
    ROS_INFO("Orbbec capture stopped");
}

bool OrbbecGemini335Driver::getStatus(std::string& status_detail) {
    std::stringstream ss;
    ss << "USB Port: " << usb_port_ 
       << ", Connected: " << (is_connected_ ? "Yes" : "No")
       << ", Running: " << (is_running_ ? "Yes" : "No")
       << ", Frame Rate: " << frame_rate_
       << ", Resolution: " << resolution_;
    status_detail = ss.str();
    return is_connected_;
}

bool OrbbecGemini335Driver::setParam(const std::string& param_key, const std::string& param_value) {
    if (!is_connected_) {
        ROS_ERROR("Orbbec not connected, cannot set param");
        return false;
    }

    if (param_key == "frame_rate") {
        frame_rate_ = std::stoi(param_value);
        OBGemini335_SetFrameRate(sensor_handle_, frame_rate_);
    } else if (param_key == "auto_exposure") {
        auto_exposure_ = (param_value == "true");
        OBGemini335_SetAutoExposure(sensor_handle_, auto_exposure_);
    } else if (param_key == "exposure_value") {
        exposure_value_ = std::stoi(param_value);
        OBGemini335_SetExposure(sensor_handle_, exposure_value_);
    } else if (param_key == "resolution") {
        resolution_ = param_value;
        OBGemini335_SetResolution(sensor_handle_, resolution_.c_str());
    } else {
        ROS_ERROR("Orbbec unknown param: %s", param_key.c_str());
        return false;
    }
    ROS_INFO("Orbbec set param %s = %s success", param_key.c_str(), param_value.c_str());
    return true;
}

void OrbbecGemini335Driver::imageCallback(const cv::Mat& left_img, const cv::Mat& right_img) {
    if (!is_running_) return;

    msg::StereoImage msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "orbbec_stereo";
    msg.baseline = baseline_;

    cv_bridge::CvImage left_cv_img, right_cv_img;
    left_cv_img.encoding = sensor_msgs::image_encodings::BGR8;
    left_cv_img.image = left_img;
    left_cv_img.toImageMsg(msg.left_image);

    right_cv_img.encoding = sensor_msgs::image_encodings::BGR8;
    right_cv_img.image = right_img;
    right_cv_img.toImageMsg(msg.right_image);

    stereo_img_pub_.publish(msg);
}

void OrbbecGemini335Driver::loadConfig(const std::string& config_path) {
    YAML::Node config = YAML::LoadFile(config_path);
    usb_port_ = config["orbbec_gemini335"]["usb_port"].as<std::string>();
    baudrate_ = config["orbbec_gemini335"]["baudrate"].as<int>();
    frame_rate_ = config["orbbec_gemini335"]["frame_rate"].as<int>();
    resolution_ = config["orbbec_gemini335"]["resolution"].as<std::string>();
    auto_exposure_ = config["orbbec_gemini335"]["auto_exposure"].as<bool>();
    exposure_value_ = config["orbbec_gemini335"]["exposure_value"].as<int>();
    baseline_ = config["orbbec_gemini335"]["baseline"].as<float>();
    topic_name_ = config["orbbec_gemini335"]["topic_name"].as<std::string>();
}

}