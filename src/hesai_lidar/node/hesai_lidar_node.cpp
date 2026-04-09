#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
// Hesai JT128默认配置
const std::string DEFAULT_IP = "192.168.1.200";
const int DEFAULT_PORT = 2368;
const int DEFAULT_PTC_PORT = 9347;
class HesaiLidar {
public:
    HesaiLidar(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
        // 读取配置参数
        pnh_.param<std::string>("device_ip", device_ip_, DEFAULT_IP);
        pnh_.param<int>("udp_port", udp_port_, DEFAULT_PORT);
        pnh_.param<int>("ptc_port", ptc_port_, DEFAULT_PTC_PORT);
        pnh_.param<std::string>("correction_file", correction_file_, "");
        
        // 初始化ROS发布者
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/hesai_lidar/points", 10);
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/hesai_lidar/imu", 10);
        
        // 初始化UDP通信
        try {
            io_service_ = boost::asio::io_service();
            udp_socket_ = new boost::asio::ip::udp::socket(io_service_, 
                boost::asio::ip::udp::v4());
            
            // 设置接收缓冲区大小（根据需要调整）
            size_t receive_buffer_size = 1024 * 1024;  // 1MB
            udp_socket_->set_option(boost::asio::socket_base::receive_buffer_size(
                receive_buffer_size));
            
            // 绑定到任意本地端口
            udp_socket_->bind(boost::asio::ip::udp::endpoint(
                boost::asio::ip::udp::v4(), 0));
            
            // 目标地址
            target_endpoint_ = boost::asio::ip::udp::endpoint(
                boost::asio::ip::address::from_string(device_ip_), udp_port_);
            
            ROS_INFO("Hesai JT128 driver initialized.");
            ROS_INFO("Connecting to %s:%d", device_ip_.c_str(), udp_port_);
            
        } catch (std::exception& e) {
            ROS_ERROR("Failed to initialize UDP socket: %s", e.what());
            return;
        }
        
        // 启动数据接收线程
        data_thread_ = boost::thread(boost::bind(&HesaiLidar::receive_data, this));
        
        // 启动PTC连接（如果需要）
        if (ptc_port_ > 0) {
            ptc_thread_ = boost::thread(boost::bind(&HesaiLidar::ptc_connect, this));
        }
    }
    
    ~HesaiLidar() {
        stop();
        if (udp_socket_) {
            udp_socket_->close();
            delete udp_socket_;
        }
    }
    
    void stop() {
        running_ = false;
        if (data_thread_.joinable()) {
            data_thread_.join();
        }
        if (ptc_thread_.joinable()) {
            ptc_thread_.join();
        }
    }
    
private:
    void receive_data() {
        running_ = true;
        
        try {
            while (running_) {
                // 接收数据报
                char buffer[1024 * 10];  // 10KB缓冲区
                boost::asio::ip::udp::endpoint remote_endpoint;
                
                size_t bytes_received = udp_socket_->receive_from(
                    boost::asio::buffer(buffer, sizeof(buffer)), remote_endpoint);
                
                // 解析数据（简化实现）
                if (bytes_received > 0) {
                    parse_packet(buffer, bytes_received);
                }
                
            }
        } catch (std::exception& e) {
            if (running_) {
                ROS_ERROR("Error receiving data: %s", e.what());
            }
        }
    }
    
    void parse_packet(const char* data, size_t length) {
        // 简化的数据包解析（需要根据Hesai协议实现）
        // 这里模拟生成点云和IMU数据
        
        // 创建点云消息
        sensor_msgs::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "laser";
        
        // 模拟点云数据（128线，每线100个点）
        cloud_msg.width = 128 * 100;
        cloud_msg.height = 1;
        cloud_msg.fields.resize(4);
        
        // 设置点云字段
        cloud_msg.fields[0].name = "x";
        cloud_msg.fields[0].offset = 0;
        cloud_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        cloud_msg.fields[0].count = 1;
        
        cloud_msg.fields[1].name = "y";
        cloud_msg.fields[1].offset = 4;
        cloud_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        cloud_msg.fields[1].count = 1;
        
        cloud_msg.fields[2].name = "z";
        cloud_msg.fields[2].offset = 8;
        cloud_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        cloud_msg.fields[2].count = 1;
        
        cloud_msg.fields[3].name = "intensity";
        cloud_msg.fields[3].offset = 12;
        cloud_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
        cloud_msg.fields[3].count = 1;
        
        cloud_msg.is_bigendian = false;
        cloud_msg.point_step = 16;
        cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
        cloud_msg.is_dense = true;
        
        // 模拟点数据（需要实际协议解析）
        cloud_msg.data.resize(cloud_msg.width * cloud_msg.point_step);
        
        pointcloud_pub_.publish(cloud_msg);
        
        // 创建IMU消息
        sensor_msgs::Imu imu_msg;
        imu_msg.header = cloud_msg.header;
        imu_msg.header.frame_id = "imu_link";
        
        // 模拟IMU数据（需要实际协议解析）
        imu_msg.orientation.w = 1.0;
        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;
        imu_msg.angular_velocity.z = 0.0;
        
        imu_pub_.publish(imu_msg);
    }
    
    void ptc_connect() {
        try {
            // PTC连接（简化实现）
            boost::asio::ip::tcp::socket ptc_socket(io_service_);
            boost::asio::ip::tcp::resolver resolver(io_service_);
            
            boost::asio::ip::tcp::resolver::query query(
                device_ip_, std::to_string(ptc_port_));
            
            boost::asio::connect(ptc_socket, resolver.resolve(query));
            
            ROS_INFO("PTC connection established to %s:%d", 
                device_ip_.c_str(), ptc_port_);
            
            // 保持连接
            while (running_) {
                boost::this_thread::sleep(boost::posix_time::seconds(1));
            }
            
            ptc_socket.close();
            ROS_INFO("PTC connection closed.");
            
        } catch (std::exception& e) {
            if (running_) {
                ROS_ERROR("PTC connection failed: %s", e.what());
            }
        }
    }
    
    ros::NodeHandle nh_, pnh_;
    ros::Publisher pointcloud_pub_;
    ros::Publisher imu_pub_;
    
    boost::asio::io_service io_service_;
    boost::asio::ip::udp::socket* udp_socket_ = nullptr;
    boost::asio::ip::udp::endpoint target_endpoint_;
    
    boost::thread data_thread_;
    boost::thread ptc_thread_;
    
    std::string device_ip_;
    int udp_port_;
    int ptc_port_;
    std::string correction_file_;
    bool running_ = false;
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "hesai_lidar_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    HesaiLidar lidar(nh, pnh);
    
    // 等待关闭
    ros::spin();
    
    lidar.stop();
    return 0;
}