#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_broadcaster.h>
class LawnmowerBase {
public:
    LawnmowerBase(ros::NodeHandle& nh) : nh_(nh) {
        // 初始化订阅者
        orbbec_depth_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            "/orbbec/camera/depth_points", 10, 
            &LawnmowerBase::orbbec_depth_callback, this);
        
        orbbec_imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(
            "/orbbec/camera/imu", 10, 
            &LawnmowerBase::orbbec_imu_callback, this);
        
        hesai_lidar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            "/hesai_lidar/points", 10, 
            &LawnmowerBase::hesai_lidar_callback, this);
        
        hesai_imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(
            "/hesai_lidar/imu", 10, 
            &LawnmowerBase::hesai_imu_callback, this);
        
        rtk_fix_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>(
            "/handsfree/rtk/fix", 10, 
            &LawnmowerBase::rtk_fix_callback, this);
        
        // 初始化发布者
        combined_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            "/lawnmower/sensor_combined", 10);
        
        orbbec_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            "/lawnmower/orbbec_cloud", 10);
        
        hesai_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            "/lawnmower/hesai_cloud", 10);
        
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>(
            "/lawnmower/imu", 10);
        
        gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(
            "/lawnmower/gps", 10);
        
        // 初始化TF广播器
        tf_broadcaster_ = new tf2_ros::TransformBroadcaster();
    }
    
    ~LawnmowerBase() {
        if (tf_broadcaster_) {
            delete tf_broadcaster_;
        }
    }
    
private:
    void orbbec_depth_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // 存储Orbbec深度数据
        orbbec_depth_cloud_ = *msg;
        
        // 发布Orbbec点云
        orbbec_cloud_pub_.publish(orbbec_depth_cloud_);
        
        // 广播TF变换
        publish_tf("camera_depth_frame", "base_link", 
            0.0, 0.0, 0.5, 0.0, 0.0, 0.0);
    }
    
    void orbbec_imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
        // 存储Orbbec IMU数据
        orbbec_imu_ = *msg;
        
        // 发布IMU数据（融合后的）
        publish_imu();
    }
    
    void hesai_lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // 存储Hesai激光雷达数据
        hesai_lidar_cloud_ = *msg;
        
        // 发布Hesai点云
        hesai_cloud_pub_.publish(hesai_lidar_cloud_);
        
        // 广播TF变换
        publish_tf("laser", "base_link", 
            0.5, 0.0, 1.0, 0.0, 0.0, 0.0);
    }
    
    void hesai_imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
        // 存储Hesai IMU数据
        hesai_imu_ = *msg;
        
        // 发布IMU数据（融合后的）
        publish_imu();
    }
    
    void rtk_fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        // 存储RTK定位数据
        rtk_fix_ = *msg;
        
        // 发布GPS数据
        gps_pub_.publish(rtk_fix_);
    }
    
    void publish_imu() {
        // 融合IMU数据（简化实现，这里使用Orbbec的IMU）
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu_link";
        
        // 可以实现更复杂的IMU融合逻辑
        imu_msg = orbbec_imu_;
        
        imu_pub_.publish(imu_msg);
    }
    
    void publish_combined_cloud() {
        // 合并Orbbec和Hesai的点云数据
        sensor_msgs::PointCloud2 combined_cloud;
        
        // 设置消息头
        combined_cloud.header.stamp = ros::Time::now();
        combined_cloud.header.frame_id = "base_link";
        
        // 计算总点数
        uint32_t total_points = orbbec_depth_cloud_.width + hesai_lidar_cloud_.width;
        
        // 设置点云格式（假设两者格式相同）
        combined_cloud.fields = orbbec_depth_cloud_.fields;
        combined_cloud.point_step = orbbec_depth_cloud_.point_step;
        combined_cloud.width = total_points;
        combined_cloud.height = 1;
        combined_cloud.row_step = total_points * combined_cloud.point_step;
        combined_cloud.is_bigendian = orbbec_depth_cloud_.is_bigendian;
        combined_cloud.is_dense = false;  // 合并后可能有无效点
        
        // 分配内存
        combined_cloud.data.resize(combined_cloud.row_step);
        
        // 复制Orbbec数据
        memcpy(combined_cloud.data.data(), 
            orbbec_depth_cloud_.data.data(), 
            orbbec_depth_cloud_.row_step);
        
        // 复制Hesai数据
        memcpy(combined_cloud.data.data() + orbbec_depth_cloud_.row_step, 
            hesai_lidar_cloud_.data.data(), 
            hesai_lidar_cloud_.row_step);
        
        combined_cloud_pub_.publish(combined_cloud);
    }
    
    void publish_tf(const std::string& child_frame, const std::string& parent_frame,
                   double x, double y, double z, 
                   double roll, double pitch, double yaw) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = parent_frame;
        transformStamped.child_frame_id = child_frame;
        
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = z;
        
        // 转换欧拉角为四元数
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(transformStamped);
    }
    
    ros::NodeHandle nh_;
    
    // 订阅者
    ros::Subscriber orbbec_depth_sub_;
    ros::Subscriber orbbec_imu_sub_;
    ros::Subscriber hesai_lidar_sub_;
    ros::Subscriber hesai_imu_sub_;
    ros::Subscriber rtk_fix_sub_;
    
    // 发布者
    ros::Publisher combined_cloud_pub_;
    ros::Publisher orbbec_cloud_pub_;
    ros::Publisher hesai_cloud_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher gps_pub_;
    
    // 存储数据
    sensor_msgs::PointCloud2 orbbec_depth_cloud_;
    sensor_msgs::Imu orbbec_imu_;
    sensor_msgs::PointCloud2 hesai_lidar_cloud_;
    sensor_msgs::Imu hesai_imu_;
    sensor_msgs::NavSatFix rtk_fix_;
    
    tf2_ros::TransformBroadcaster* tf_broadcaster_;
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "lawnmower_base_node");
    ros::NodeHandle nh;
    
    LawnmowerBase lawnmower_base(nh);
    
    // 主循环
    ros::Rate rate(30);  // 30Hz
    while (ros::ok()) {
        ros::spinOnce();
        
        // 定期发布组合点云
        lawnmower_base.publish_combined_cloud();
        
        rate.sleep();
    }
    
    return 0;
}