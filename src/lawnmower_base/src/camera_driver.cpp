#include "lawnmower_base/camera_driver.h"
CameraDriver::CameraDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      camera_(nullptr),
      new_image_(false) {
    // 读取相机配置参数
    nh_private_.param<std::string>("camera_model", camera_model_, "gemini335");
    nh_private_.param<int>("width", width_, 1920);
    nh_private_.param<int>("height", height_, 1080);
    nh_private_.param<int>("fps", fps_, 30);
    nh_private_.param<bool>("publish_raw", publish_raw_, true);
}
bool CameraDriver::init() {
    // 初始化Orbbec相机
    try {
        // 查找并打开相机设备
        std::vector<ob::DeviceInfo> devices = ob::DeviceManager::Instance().GetDevices();
        if (devices.empty()) {
            ROS_ERROR("No Orbbec devices found!");
            return false;
        }
        // 打开第一个设备
        camera_ = ob::DeviceManager::Instance().OpenDevice(devices[0]);
        if (!camera_) {
            ROS_ERROR("Failed to open Orbbec device!");
            return false;
        }
        // 创建流配置
        ob::StreamProfile color_profile = camera_->GetStreamProfile(ob::OB_SENSOR_COLOR, width_, height_, ob::OB_FORMAT_RGB888, fps_);
        ob::StreamProfile depth_profile = camera_->GetStreamProfile(ob::OB_SENSOR_DEPTH, 1280, 720, ob::OB_FORMAT_Y16, fps_);
        ob::StreamProfile ir_profile = camera_->GetStreamProfile(ob::OB_SENSOR_IR, 1280, 720, ob::OB_FORMAT_Y16, fps_);
        ob::StreamProfile imu_profile = camera_->GetStreamProfile(ob::OB_SENSOR_IMU, 0, 0, ob::OB_FORMAT_MOTION_6DOF, 100);
        // 配置并启动流
        camera_->Start(color_profile);
        camera_->Start(depth_profile);
        camera_->Start(ir_profile);
        camera_->Start(imu_profile);
        ROS_INFO("Orbbec camera initialized successfully!");
    } catch (const std::exception& e) {
        ROS_ERROR("Orbbec camera initialization failed: %s", e.what());
        return false;
    }
    // 初始化ROS发布者
    pub_color_ = nh_.advertise<sensor_msgs::Image>("/gemini335/color/image_raw", 10);
    pub_depth_ = nh_.advertise<sensor_msgs::Image>("/gemini335/depth/image_raw", 10);
    pub_ir_left_ = nh_.advertise<sensor_msgs::Image>("/gemini335/ir_left/image_raw", 10);
    pub_ir_right_ = nh_.advertise<sensor_msgs::Image>("/gemini335/ir_right/image_raw", 10);
    pub_imu_ = nh_.advertise<sensor_msgs::Imu>("/gemini335/imu/data_raw", 10);
    return true;
}
void CameraDriver::readData() {
    if (!camera_) return;
    try {
        // 等待新的帧数据
        ob::FrameSet frames = camera_->WaitForFrame(100);
        // 处理彩色图像
        if (frames.ColorFrame()) {
            const ob::Frame* frame = frames.ColorFrame();
            if (frame->IsValid()) {
                // 转换为ROS Image消息
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
                    frame->GetTimestamp(), "rgb8",
                    cv::Mat(frame->GetHeight(), frame->GetWidth(), CV_8UC3, const_cast<uint8_t*>(static_cast<const uint8_t*>(frame->GetData())))).toImageMsg();
                msg->header.stamp = ros::Time::now();
                msg->header.frame_id = "gemini335_color_link";
                color_image_ = msg;
            }
        }
        // 处理深度图像
        if (frames.DepthFrame()) {
            const ob::Frame* frame = frames.DepthFrame();
            if (frame->IsValid()) {
                // 转换为ROS Image消息
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
                    frame->GetTimestamp(), "16UC1",
                    cv::Mat(frame->GetHeight(), frame->GetWidth(), CV_16UC1, const_cast<uint8_t*>(static_cast<const uint8_t*>(frame->GetData())))).toImageMsg();
                msg->header.stamp = ros::Time::now();
                msg->header.frame_id = "gemini335_depth_link";
                depth_image_ = msg;
            }
        }
        // 处理IMU数据
        if (frames.IMUFrame()) {
            const ob::Frame* frame = frames.IMUFrame();
            if (frame->IsValid()) {
                // 转换为ROS Imu消息
                sensor_msgs::ImuPtr msg(new sensor_msgs::Imu());
                msg->header.stamp = ros::Time::now();
                msg->header.frame_id = "gemini335_imu_link";
                // 加速度数据（m/s²）
                msg->linear_acceleration.x = static_cast<const ob::MotionData*>(frame->GetData())->linear_acceleration.x;
                msg->linear_acceleration.y = static_cast<const ob::MotionData*>(frame->GetData())->linear_acceleration.y;
                msg->linear_acceleration.z = static_cast<const ob::MotionData*>(frame->GetData())->linear_acceleration.z;
                // 角速度数据（rad/s）
                msg->angular_velocity.x = static_cast<const ob::MotionData*>(frame->GetData())->angular_velocity.x;
                msg->angular_velocity.y = static_cast<const ob::MotionData*>(frame->GetData())->angular_velocity.y;
                msg->angular_velocity.z = static_cast<const ob::MotionData*>(frame->GetData())->angular_velocity.z;
                // 设置协方差（根据传感器规格）
                msg->linear_acceleration_covariance[0] = 0.01;
                msg->linear_acceleration_covariance[4] = 0.01;
                msg->linear_acceleration_covariance[8] = 0.01;
                msg->angular_velocity_covariance[0] = 0.001;
                msg->angular_velocity_covariance[4] = 0.001;
                msg->angular_velocity_covariance[8] = 0.001;
                imu_data_ = msg;
            }
        }
        new_image_ = true;
    } catch (const std::exception& e) {
        ROS_WARN("Failed to read camera data: %s", e.what());
        new_image_ = false;
    }
}