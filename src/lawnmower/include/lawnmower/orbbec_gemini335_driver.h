#ifndef ORBBEC_GEMINI335_DRIVER_H
#define ORBBEC_GEMINI335_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "msg/StereoImage.h"
#include <libusb-1.0/libusb.h>
#include "OrbbecGemini335SDK.h"

namespace lawnmower {

class OrbbecGemini335Driver {
public:
    OrbbecGemini335Driver(ros::NodeHandle& nh, const std::string& config_path);
    ~OrbbecGemini335Driver();

    bool init();
    void startCapture();
    void stopCapture();
    bool getStatus(std::string& status_detail);
    bool setParam(const std::string& param_key, const std::string& param_value);

private:
    void imageCallback(const cv::Mat& left_img, const cv::Mat& right_img);
    void loadConfig(const std::string& config_path);

    ros::NodeHandle nh_;
    ros::Publisher stereo_img_pub_;

    OBGemini335Handle* sensor_handle_;
    libusb_device_handle* usb_handle_;

    std::string usb_port_;
    int baudrate_;
    int frame_rate_;
    std::string resolution_;
    bool auto_exposure_;
    int exposure_value_;
    float baseline_;
    std::string topic_name_;

    bool is_running_;
    bool is_connected_;
};

}
#endif