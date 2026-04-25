#include "sensor_data/sensor_data_node.h"
#include "sensor_data/sensor_fusion.h"

using namespace sensor_data;

int main(int argc, char** argv) {
    // 初始化ROS节点（适配RK3588）
    ros::init(argc, argv, "sensor_data_node");
    ros::NodeHandle nh("~");

    ROS_INFO("【SensorDataNode】启动传感器数据融合节点（RK3588 ROS1 Noetic）");

    // 创建节点实例
    SensorDataNode node(nh);

    // 初始化传感器
    if (!node.initSensors()) {
        ROS_ERROR("【SensorDataNode】传感器初始化失败！");
        return -1;
    }

    // 数据融合主循环
    ros::Rate rate(100);  // 100Hz（RK3588硬件适配）
    while (ros::ok()) {
        node.fusionData();
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("【SensorDataNode】节点正常退出");
    return 0;
}

// 构造函数实现
SensorDataNode::SensorDataNode(ros::NodeHandle& nh) : nh_(nh) {
    // 订阅各传感器话题
    sub_rtk_ = nh_.subscribe("/handsfree_rtk/data", 10, &SensorDataNode::rtkDataCallback, this);
    sub_lidar_ = nh_.subscribe("/hesai_lidar/points", 10, &SensorDataNode::lidarDataCallback, this);
    sub_camera_ = nh_.subscribe("/orbbec_camera/image_raw", 10, &SensorDataNode::cameraDataCallback, this);
    sub_imu_ = nh_.subscribe("/rk3588/imu/data", 10, &SensorDataNode::imuDataCallback, this);

    // 发布融合后数据
    pub_fusion_data_ = nh_.advertise<std_msgs::String>("/sensor_data/fusion", 10);
}

// 析构函数实现
SensorDataNode::~SensorDataNode() {
    // 释放RK3588硬件资源
    rk3588_gpio_close(&gpio_handle_);
    ROS_INFO("【SensorDataNode】释放硬件资源完成");
}

// 传感器初始化实现
bool SensorDataNode::initSensors() {
    // 初始化RK3588 GPIO
    if (rk3588_gpio_init(&gpio_handle_, RK3588_GPIO_PIN_18) != 0) {
        ROS_ERROR("【SensorDataNode】RK3588 GPIO初始化失败");
        return false;
    }

    // 初始化奥比中光相机
    if (!ob_camera_init()) {
        ROS_ERROR("【SensorDataNode】Orbbec相机初始化失败");
        return false;
    }

    // 初始化禾赛激光雷达
    if (!hesai_lidar_init()) {
        ROS_ERROR("【SensorDataNode】禾赛激光雷达初始化失败");
        return false;
    }

    // 初始化RTK模块
    if (!handsfree_rtk_init()) {
        ROS_ERROR("【SensorDataNode】RTK模块初始化失败");
        return false;
    }

    ROS_INFO("【SensorDataNode】所有传感器初始化成功");
    return true;
}

// 数据融合主逻辑实现
void SensorDataNode::fusionData() {
    // 传感器数据融合核心逻辑（示例）
    std_msgs::String fusion_msg;
    fusion_msg.data = "fusion_data: rtk+lidar+camera+imu (RK3588)";
    pub_fusion_data_.publish(fusion_msg);
}

// RTK数据回调实现
void SensorDataNode::rtkDataCallback(const handsfree_rtk::RTKData::ConstPtr& msg) {
    ROS_DEBUG("【SensorDataNode】收到RTK数据：%s", msg->data.c_str());
}

// 激光雷达数据回调实现
void SensorDataNode::lidarDataCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    ROS_DEBUG("【SensorDataNode】收到激光雷达点云：%d点", msg->width * msg->height);
}

// 相机数据回调实现
void SensorDataNode::cameraDataCallback(const sensor_msgs::Image::ConstPtr& msg) {
    ROS_DEBUG("【SensorDataNode】收到相机图像：%dx%d", msg->width, msg->height);
}

// IMU数据回调实现
void SensorDataNode::imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ROS_DEBUG("【SensorDataNode】收到IMU数据：角速度=%.2f", msg->angular_velocity.x);
}