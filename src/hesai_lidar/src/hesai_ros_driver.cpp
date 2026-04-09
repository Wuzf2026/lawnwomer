#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <yaml-cpp/yaml.h>
// Hesai SDK
#include "hesai_sdk.h"
// 激光雷达参数配置
struct LiDARParams {
    std::string device_ip;
    int udp_port;
    int ptc_port;
    std::string correction_file;
    std::string firetimes_file;
    bool use_gpu;
    int source_type;
    int thread_num;
    bool transform_flag;
    double x, y, z;
    double roll, pitch, yaw;
    bool enable_packet_loss_tool;
    bool distance_correction_flag;
    int device_udp_src_port;
    int device_fault_port;
    double frame_frequency;
    int echo_mode_filter;
};
class HesaiLiDAR {
public:
    HesaiLiDAR(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
        // 读取参数
        read_params();
        
        // 初始化激光雷达
        initialize_lidar();
        
        // 发布者
        point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("hesai_lidar/points", 10);
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("hesai_lidar/imu", 10);
        
        // TF广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
        
        // 诊断更新器
        updater_.setHardwareID("hesai_lidar");
        updater_.add("LiDAR Status", this, &HesaiLiDAR::diagnosticStatus);
        timer_ = nh_.createTimer(ros::Duration(1.0), &HesaiLiDAR::diagnosticTimerCallback, this);
    }
    ~HesaiLiDAR() {
        stop_lidar();
    }
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    LiDARParams params;
    hesai::LiDAR* lidar_;
    hesai::PacketQueue packet_queue_;
    
    ros::Publisher point_cloud_pub_;
    ros::Publisher imu_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    diagnostic_updater::Updater updater_;
    ros::Timer timer_;
    void read_params() {
        // 读取参数（简化版）
        params.device_ip = pnh_.param<std::string>("device_ip", "192.168.1.200");
        params.udp_port = pnh_.param<int>("udp_port", 2368);
        params.ptc_port = pnh_.param<int>("ptc_port", 9347);
        params.correction_file = pnh_.param<std::string>("correction_file", "");
        params.firetimes_file = pnh_.param<std::string>("firetimes_file", "");
        params.use_gpu = pnh_.param<bool>("use_gpu", false);
        params.source_type = pnh_.param<int>("source_type", 1);  // 1:实时数据
        params.thread_num = pnh_.param<int>("thread_num", 4);
        params.transform_flag = pnh_.param<bool>("transform_flag", false);
        params.x = pnh_.param<double>("x", 0.0);
        params.y = pnh_.param<double>("y", 0.0);
        params.z = pnh_.param<double>("z", 1.0);
        params.roll = pnh_.param<double>("roll", 0.0);
        params.pitch = pnh_.param<double>("pitch", 0.0);
        params.yaw = pnh_.param<double>("yaw", 0.0);
    }
    void initialize_lidar() {
        try {
            // 初始化SDK
            hesai::init();
            
            // 创建激光雷达对象
            lidar_ = new hesai::LiDAR();
            
            // 配置参数
            hesai::LiDARConfig config;
            config.device_ip = params.device_ip;
            config.udp_port = params.udp_port;
            config.ptc_port = params.ptc_port;
            config.correction_file = params.correction_file;
            config.firetimes_file = params.firetimes_file;
            config.use_gpu = params.use_gpu;
            config.source_type = params.source_type;
            config.thread_num = params.thread_num;
            
            // 初始化
            if (!lidar_->init(config)) {
                throw std::runtime_error("Failed to initialize LiDAR");
            }
            
            // 启动接收线程
            lidar_->start();
            
            ROS_INFO("Hesai LiDAR initialized successfully");
        } catch (const std::exception& e) {
            ROS_FATAL("Failed to initialize Hesai LiDAR: %s", e.what());
            ros::shutdown();
        }
    }
    void stop_lidar() {
        if (lidar_) {
            lidar_->stop();
            delete lidar_;
            lidar_ = nullptr;
        }
        hesai::shutdown();
    }
    void publish_data() {
        try {
            // 获取数据包
            hesai::Packet packet;
            while (lidar_->getPacket(packet)) {
                // 解析点云
                if (packet.type == hesai::PacketType::POINT_CLOUD) {
                    publish_point_cloud(packet.point_cloud);
                }
                
                // 解析IMU
                if (packet.type == hesai::PacketType::IMU) {
                    publish_imu(packet.imu);
                }
                
                // 解析故障信息
                if (packet.type == hesai::PacketType::FAULT) {
                    ROS_WARN("LiDAR fault: %s", packet.fault_message.c_str());
                }
            }
            
            // 发布TF
            publish_transform();
        } catch (const std::exception& e) {
            ROS_WARN("Error publishing LiDAR data: %s", e.what());
        }
    }
    void publish_point_cloud(const hesai::PointCloud& pc) {
        sensor_msgs::PointCloud2 point_cloud;
        
        // 设置消息头
        point_cloud.header.stamp = ros::Time::now();
        point_cloud.header.frame_id = "laser";
        
        // 点云参数
        point_cloud.width = pc.points.size();
        point_cloud.height = 1;
        point_cloud.is_dense = false;
        point_cloud.is_bigendian = false;
        
        // 点云格式（XYZ + 强度）
        sensor_msgs::PointField x_field;
        x_field.name = "x";
        x_field.offset = 0;
        x_field.datatype = sensor_msgs::PointField::FLOAT32;
        x_field.count = 1;
        
        sensor_msgs::PointField y_field;
        y_field.name = "y";
        y_field.offset = 4;
        y_field.datatype = sensor_msgs::PointField::FLOAT32;
        y_field.count = 1;
        
        sensor_msgs::PointField z_field;
        z_field.name = "z";
        z_field.offset = 8;
        z_field.datatype = sensor_msgs::PointField::FLOAT32;
        z_field.count = 1;
        
        sensor_msgs::PointField intensity_field;
        intensity_field.name = "intensity";
        intensity_field.offset = 12;
        intensity_field.datatype = sensor_msgs::PointField::FLOAT32;
        intensity_field.count = 1;
        
        point_cloud.fields.push_back(x_field);
        point_cloud.fields.push_back(y_field);
        point_cloud.fields.push_back(z_field);
        point_cloud.fields.push_back(intensity_field);
        
        // 步长
        point_cloud.point_step = 16;
        point_cloud.row_step = point_cloud.point_step * point_cloud.width;
        
        // 分配内存
        point_cloud.data.resize(point_cloud.row_step * point_cloud.height);
        
        // 填充数据
        uint8_t* data_ptr = &point_cloud.data[0];
        for (const auto& point : pc.points) {
            float* float_ptr = reinterpret_cast<float*>(data_ptr);
            float_ptr[0] = point.x;
            float_ptr[1] = point.y;
            float_ptr[2] = point.z;
            float_ptr[3] = point.intensity;
            data_ptr += point_cloud.point_step;
        }
        
        point_cloud_pub_.publish(point_cloud);
    }
    void publish_imu(const hesai::IMU& imu_data) {
        sensor_msgs::Imu imu_msg;
        
        // 设置消息头
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu_link";
        
        // 线性加速度（m/s²）
        imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x;
        imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y;
        imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z;
        
        // 角速度（rad/s）
        imu_msg.angular_velocity.x = imu_data.angular_velocity.x;
        imu_msg.angular_velocity.y = imu_data.angular_velocity.y;
        imu_msg.angular_velocity.z = imu_data.angular_velocity.z;
        
        // 方向（四元数）
        imu_msg.orientation.x = imu_data.orientation.x;
        imu_msg.orientation.y = imu_data.orientation.y;
        imu_msg.orientation.z = imu_data.orientation.z;
        imu_msg.orientation.w = imu_data.orientation.w;
        
        // 协方差（简化设置）
        imu_msg.linear_acceleration_covariance[0] = 0.01;
        imu_msg.angular_velocity_covariance[0] = 0.001;
        imu_msg.orientation_covariance[0] = 0.0001;
        
        imu_pub_.publish(imu_msg);
    }
    void publish_transform() {
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "base_link";
        transformStamped.child_frame_id = "laser";
        
        // 激光雷达相对于base_link的变换
        transformStamped.transform.translation.x = params.x;
        transformStamped.transform.translation.y = params.y;
        transformStamped.transform.translation.z = params.z;
        
        // 旋转（RPY转四元数）
        tf2::Quaternion q;
        q.setRPY(params.roll, params.pitch, params.yaw);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        
        br.sendTransform(transformStamped);
    }
    void diagnosticStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "LiDAR running");
        stat.add("Device IP", params.device_ip);
        stat.add("UDP Port", std::to_string(params.udp_port));
        stat.add("PTC Port", std::to_string(params.ptc_port));
        stat.add("Correction File", params.correction_file);
        stat.add("Firetimes File", params.firetimes_file);
        stat.add("Use GPU", params.use_gpu ? "Yes" : "No");
    }
    void diagnosticTimerCallback(const ros::TimerEvent&) {
        updater_.update();
    }
    void run() {
        ros::Rate rate(10);  // 10Hz
        while (ros::ok()) {
            publish_data();
            ros::spinOnce();
            rate.sleep();
        }
    }
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "hesai_lidar_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    HesaiLiDAR lidar(nh, pnh);
    lidar.run();
    
    return 0;
}