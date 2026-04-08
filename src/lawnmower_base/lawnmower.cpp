#include "lawnmower.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
namespace lawnmower_base {
Lawnmower::Lawnmower(ros::NodeHandle& nh) : nh_(nh) {
    // 初始化订阅者
    orbbec_depth_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        "/orbbec/camera/depth_points", 10, &Lawnmower::sensor_data_callback, this);
    
    orbbec_imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(
        "/orbbec/camera/imu", 10, &Lawnmower::imu_callback, this);
    
    hesai_lidar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        "/hesai_lidar/points", 10, &Lawnmower::lidar_callback, this);
    
    rtk_fix_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>(
        "/handsfree_rtk/fix", 10, &Lawnmower::rtk_callback, this);
    
    // 初始化发布者
    combined_sensor_pub_ = nh_.publish<sensor_msgs::PointCloud2>(
        "/lawnmower/sensor_combined", 10);
    
    // 初始化传感器数据缓冲区
    point_cloud2_init(&orbbec_depth_cloud_, 1, 0, false, 32, 0, true);
    point_cloud2_init(&hesai_lidar_cloud_, 1, 0, false, 32, 0, true);
    imu_init(&orbbec_imu_);
    nav_sat_fix_init(&rtk_fix_);
}
Lawnmower::~Lawnmower() {
    point_cloud2_destroy(&orbbec_depth_cloud_);
    point_cloud2_destroy(&hesai_lidar_cloud_);
}
void Lawnmower::sensor_data_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // 深度相机数据回调
    if (msg->width > 0) {
        orbbec_depth_cloud_.width = msg->width;
        orbbec_depth_cloud_.row_step = msg->row_step;
        point_cloud2_set_data(&orbbec_depth_cloud_, msg->data);
    }
}
void Lawnmower::imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    // IMU数据回调
    orbbec_imu_ = *msg;
}
void Lawnmower::lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // 激光雷达数据回调
    if (msg->width > 0) {
        hesai_lidar_cloud_.width = msg->width;
        hesai_lidar_cloud_.row_step = msg->row_step;
        point_cloud2_set_data(&hesai_lidar_cloud_, msg->data);
    }
}
void Lawnmower::rtk_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    // RTK定位数据回调
    rtk_fix_ = *msg;
}
void Lawnmower::publish_combined_data() {
    // 创建组合点云消息
    sensor_msgs::PointCloud2 combined_cloud;
    
    // 设置消息头
    combined_cloud.header = orbbec_depth_cloud_.header;
    combined_cloud.header.frame_id = "base_link";
    
    // 计算总点数
    uint32_t total_points = orbbec_depth_cloud_.width + hesai_lidar_cloud_.width;
    
    // 初始化组合点云
    point_cloud2_init(&combined_cloud, 1, total_points, false, 32, 32 * total_points, true);
    
    // 合并数据（先深度相机，后激光雷达）
    memcpy(combined_cloud.data, orbbec_depth_cloud_.data, orbbec_depth_cloud_.row_step);
    memcpy(combined_cloud.data + orbbec_depth_cloud_.row_step, 
           hesai_lidar_cloud_.data, hesai_lidar_cloud_.row_step);
    
    // 发布组合数据
    combined_sensor_pub_.publish(combined_cloud);
    
    // 发布TF变换
    static tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped transform;
    
    // 设置变换时间戳
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "base_link";
    transform.child_frame_id = "laser";
    
    // 设置激光雷达位置（示例值）
    transform.transform.translation.x = 0.5;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 1.0;
    
    // 设置旋转（示例值）
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    
    tf_broadcaster.sendTransform(transform);
    
    // 清理内存
    point_cloud2_destroy(&combined_cloud);
}
void Lawnmower::run() {
    ros::Rate rate(30); // 30Hz循环
    
    while (ros::ok()) {
        ros::spinOnce();
        
        // 定期发布组合数据
        publish_combined_data();
        
        rate.sleep();
    }
}
} // namespace lawnmower_base
int main(int argc, char** argv) {
    ros::init(argc, argv, "lawnmower_base_node");
    ros::NodeHandle nh("~");
    
    lawnmower_base::Lawnmower lawnmower(nh);
    lawnmower.run();
    
    return 0;
}