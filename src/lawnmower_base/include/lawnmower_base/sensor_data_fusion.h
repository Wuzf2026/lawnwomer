#include "lawnmower_base/orbbec_driver.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <ObException.h>

namespace lawnmower_base {

OrbbecDriver::OrbbecDriver(ros::NodeHandle& nh, const std::string& config_path)
  : nh_(nh), is_running_(false) {
  // 加载配置参数
  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("orbbec/device_id", device_id_, "0");
  private_nh.param<std::string>("orbbec/usb_port", usb_port_, "/dev/bus/usb/001/005");
  private_nh.param<std::string>("orbbec/frame_id", frame_id_, "orbbec_pointcloud");
  private_nh.param<int>("orbbec/width", width_, 640);
  private_nh.param<int>("orbbec/height", height_, 480);
  private_nh.param<int>("orbbec/fps", fps_, 30);
  private_nh.param<int>("orbbec/point_step", point_step_, 32);
  private_nh.param<bool>("orbbec/is_dense", is_dense_, true);
  private_nh.param<bool>("orbbec/is_bigendian", is_bigendian_, false);

  // 初始化RK3588 USB驱动
  rk_usb_ = new RKUSB();
}

OrbbecDriver::~OrbbecDriver() {
  stopStream();
  if (rk_usb_) {
    delete rk_usb_;
  }
}

bool OrbbecDriver::init() {
  try {
    // 初始化RK3588 USB
    if (!rk_usb_->init(usb_port_)) {
      ROS_ERROR("RK3588 USB init failed for Orbbec");
      return false;
    }

    // 创建Orbbec管道
    pipeline_ = std::make_shared<ob::Pipeline>();
    config_ = std::make_shared<ob::Config>();

    // 配置流参数
    auto depth_profile_list = pipeline_->getStreamProfileList(OB_STREAM_DEPTH);
    auto depth_profile = depth_profile_list->getVideoStreamProfile(width_, height_, OB_FORMAT_Y16, fps_);
    config_->enableStream(depth_profile);

    // 配置点云流
    auto pc_profile_list = pipeline_->getStreamProfileList(OB_STREAM_POINT_CLOUD);
    auto pc_profile = pc_profile_list->getPointCloudStreamProfile(width_, height_, OB_FORMAT_POINT_CLOUD_XYZRGBA, fps_);
    config_->enableStream(pc_profile);

    // 设置回调函数
    pipeline_->setFrameSetCallback(std::bind(&OrbbecDriver::pointCloudCallback, this, std::placeholders::_1));

    ROS_INFO("Orbbec Gemini335 init success");
    return true;
  } catch (ob::Exception& e) {
    ROS_ERROR("Orbbec init exception: %s", e.what());
    return false;
  }
}

void OrbbecDriver::startStream() {
  if (is_running_) return;
  
  try {
    pipeline_->start(config_);
    is_running_ = true;
    ROS_INFO("Orbbec stream started");
  } catch (ob::Exception& e) {
    ROS_ERROR("Orbbec start stream failed: %s", e.what());
  }
}

void OrbbecDriver::stopStream() {
  if (!is_running_) return;
  
  pipeline_->stop();
  is_running_ = false;
  ROS_INFO("Orbbec stream stopped");
}

void OrbbecDriver::pointCloudCallback(ob::FrameSet::Ptr frame_set) {
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  
  // 获取点云帧
  auto pc_frame = frame_set->getPointCloudFrame();
  if (!pc_frame) {
    ROS_WARN("Empty Orbbec point cloud frame");
    return;
  }

  // 填充PointCloud2消息
  cloud_data_.header.stamp = ros::Time::now();
  cloud_data_.header.frame_id = frame_id_;
  cloud_data_.height = height_;
  cloud_data_.width = width_;
  cloud_data_.is_bigendian = is_bigendian_;
  cloud_data_.point_step = point_step_;
  cloud_data_.row_step = cloud_data_.point_step * cloud_data_.width;
  cloud_data_.is_dense = is_dense_;

  // 清除旧字段
  cloud_data_.fields.clear();
  // 添加点云字段
  sensor_msgs::PointField x_field, y_field, z_field, rgb_field;
  x_field.name = "x"; x_field.offset = 0; x_field.datatype = sensor_msgs::PointField::FLOAT32; x_field.count = 1;
  y_field.name = "y"; y_field.offset = 4; y_field.datatype = sensor_msgs::PointField::FLOAT32; y_field.count = 1;
  z_field.name = "z"; z_field.offset = 8; z_field.datatype = sensor_msgs::PointField::FLOAT32; z_field.count = 1;
  rgb_field.name = "rgb"; rgb_field.offset = 16; rgb_field.datatype = sensor_msgs::PointField::UINT32; rgb_field.count = 1;
  
  cloud_data_.fields.push_back(x_field);
  cloud_data_.fields.push_back(y_field);
  cloud_data_.fields.push_back(z_field);
  cloud_data_.fields.push_back(rgb_field);

  // 复制点云数据
  cloud_data_.data.resize(pc_frame->dataSize());
  memcpy(&cloud_data_.data[0], pc_frame->data(), pc_frame->dataSize());
}

void OrbbecDriver::getPointCloud(sensor_msgs::PointCloud2& cloud) {
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  cloud = cloud_data_;
}

void OrbbecDriver::setResolution(int width, int height) {
  width_ = width;
  height_ = height;
  if (is_running_) {
    stopStream();
    init();
    startStream();
  }
}

void OrbbecDriver::setFPS(int fps) {
  fps_ = fps;
  if (is_running_) {
    stopStream();
    init();
    startStream();
  }
}

void OrbbecDriver::setFrameId(const std::string& frame_id) {
  frame_id_ = frame_id;
}

} // namespace lawnmower_base