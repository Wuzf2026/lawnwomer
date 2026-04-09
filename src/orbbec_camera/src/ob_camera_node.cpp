#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <libusb-1.0/libusb.h>
#include <opencv2/opencv.hpp>
// Orbbec Gemini335设备信息
#define VENDOR_ID 0x2BC5
#define PRODUCT_ID 0x0402
class OrbbecCamera {
public:
    OrbbecCamera(ros::NodeHandle& nh) : nh_(nh), it_(nh) {
        // 初始化ROS发布者
        depth_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/orbbec/camera/depth_points", 10);
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/orbbec/camera/imu", 10);
        left_image_pub_ = it_.advertise("/orbbec/camera/left/image_raw", 10);
        right_image_pub_ = it_.advertise("/orbbec/camera/right/image_raw", 10);
        
        // 初始化USB设备
        if (libusb_init(&ctx_) != 0) {
            ROS_ERROR("Failed to initialize libusb.");
            return;
        }
        
        device_ = libusb_open_device_with_vid_pid(ctx_, VENDOR_ID, PRODUCT_ID);
        if (!device_) {
            ROS_ERROR("Failed to open Orbbec Gemini335 device.");
            return;
        }
        
        // 配置设备（简化实现）
        configure_device();
        
        // 启动数据采集线程
        data_thread_ = std::thread(&OrbbecCamera::capture_data, this);
    }
    
    ~OrbbecCamera() {
        stop();
        if (device_) {
            libusb_close(device_);
        }
        libusb_exit(ctx_);
    }
    
    void stop() {
        running_ = false;
        if (data_thread_.joinable()) {
            data_thread_.join();
        }
    }
    
private:
    void configure_device() {
        // 设置配置（简化实现）
        ROS_INFO("Configuring Orbbec Gemini335...");
        
        // 设置分辨率和帧率
        // 这里需要根据实际的USB通信协议实现
        // 示例：设置为1280x720分辨率，30fps帧率
        
        // 启用设备
        if (libusb_claim_interface(device_, 0) != 0) {
            ROS_ERROR("Failed to claim interface 0.");
        }
    }
    
    void capture_data() {
        running_ = true;
        
        while (running_) {
            // 模拟数据采集（需要根据实际USB协议实现）
            
            // 创建深度点云数据
            sensor_msgs::PointCloud2 depth_msg;
            depth_msg.header.stamp = ros::Time::now();
            depth_msg.header.frame_id = "camera_depth_frame";
            depth_msg.width = 640;  // 示例值
            depth_msg.height = 480; // 示例值
            depth_msg.fields.resize(3);
            
            // 设置点云字段
            depth_msg.fields[0].name = "x";
            depth_msg.fields[0].offset = 0;
            depth_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
            depth_msg.fields[0].count = 1;
            
            depth_msg.fields[1].name = "y";
            depth_msg.fields[1].offset = 4;
            depth_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
            depth_msg.fields[1].count = 1;
            
            depth_msg.fields[2].name = "z";
            depth_msg.fields[2].offset = 8;
            depth_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
            depth_msg.fields[2].count = 1;
            
            depth_msg.is_bigendian = false;
            depth_msg.point_step = 12;
            depth_msg.row_step = depth_msg.width * depth_msg.point_step;
            depth_msg.is_dense = true;
            
            // 模拟深度数据（需要实际实现）
            depth_msg.data.resize(depth_msg.width * depth_msg.height * depth_msg.point_step);
            
            // 发布深度点云
            depth_pub_.publish(depth_msg);
            
            // 创建IMU数据
            sensor_msgs::Imu imu_msg;
            imu_msg.header = depth_msg.header;
            imu_msg.header.frame_id = "camera_imu_frame";
            
            // 模拟IMU数据（需要实际实现）
            imu_msg.orientation.w = 1.0;
            imu_msg.angular_velocity.x = 0.0;
            imu_msg.angular_velocity.y = 0.0;
            imu_msg.angular_velocity.z = 0.0;
            
            imu_pub_.publish(imu_msg);
            
            // 创建左右图像数据
            cv::Mat left_image(480, 640, CV_8UC1, cv::Scalar(0));
            cv::Mat right_image(480, 640, CV_8UC1, cv::Scalar(0));
            
            // 转换为ROS图像消息
            sensor_msgs::ImagePtr left_img_msg = cv_bridge::CvImage(
                depth_msg.header, "mono8", left_image).toImageMsg();
            sensor_msgs::ImagePtr right_img_msg = cv_bridge::CvImage(
                depth_msg.header, "mono8", right_image).toImageMsg();
            
            left_image_pub_.publish(left_img_msg);
            right_image_pub_.publish(right_img_msg);
            
            // 控制采集频率
            ros::Duration(0.033).sleep();  // 30Hz
        }
    }
    
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Publisher depth_pub_;
    ros::Publisher imu_pub_;
    image_transport::Publisher left_image_pub_;
    image_transport::Publisher right_image_pub_;
    
    libusb_context* ctx_ = nullptr;
    libusb_device_handle* device_ = nullptr;
    std::thread data_thread_;
    bool running_ = false;
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "orbbec_camera_node");
    ros::NodeHandle nh("~");
    
    OrbbecCamera camera(nh);
    
    // 等待关闭
    ros::spin();
    
    camera.stop();
    return 0;
}