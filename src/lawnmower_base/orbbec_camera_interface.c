#include "orbbec_camera_interface.h"
#include <orbbec_camera/ob_camera.h>
#include <sensor_msgs/PointCloud2.h>
class OrbbecCamera {
public:
    OrbbecCamera(const std::string& camera_id) : camera_id_(camera_id), camera_(nullptr) {
        // 初始化Orbbec相机SDK
        ob_error result = ob_init();
        if (result != OB_SUCCESS) {
            ROS_ERROR("Orbbec SDK initialization failed: %d", result);
        }
    }
    
    ~OrbbecCamera() {
        if (camera_) {
            ob_close_camera(camera_);
        }
        ob_shutdown();
    }
    
    bool init() {
        // 查找相机设备
        ob_device_list* devices = nullptr;
        ob_error result = ob_enumerate_devices(&devices);
        if (result != OB_SUCCESS) {
            ROS_ERROR("Failed to enumerate devices: %d", result);
            return false;
        }
        
        // 查找指定ID的相机
        for (int i = 0; i < devices->count; ++i) {
            ob_device* device = devices->devices[i];
            const char* device_id = ob_device_get_id(device);
            
            if (std::string(device_id) == camera_id_) {
                // 打开相机
                result = ob_device_open(device, &camera_);
                if (result != OB_SUCCESS) {
                    ROS_ERROR("Failed to open camera %s: %d", camera_id_, result);
                    return false;
                }
                
                // 配置流（设置分辨率、帧率等）
                ob_stream_profile_list* profile_list = ob_camera_get_stream_profile_list(camera_);
                if (!profile_list) {
                    ROS_ERROR("Failed to get stream profile list");
                    return false;
                }
                
                // 选择深度流配置（示例：1280x720@30fps）
                ob_video_stream_profile* depth_profile = nullptr;
                result = ob_stream_profile_list_get_video_stream_profile_by_resolution(
                    profile_list, OB_FORMAT_Y16, 1280, 720, 30, &depth_profile);
                
                if (result != OB_SUCCESS) {
                    ROS_ERROR("Failed to find depth stream profile: %d", result);
                    return false;
                }
                
                // 选择彩色流配置（示例：1920x1080@30fps）
                ob_video_stream_profile* color_profile = nullptr;
                result = ob_stream_profile_list_get_video_stream_profile_by_resolution(
                    profile_list, OB_FORMAT_RGB888, 1920, 1080, 30, &color_profile);
                
                if (result != OB_SUCCESS) {
                    ROS_ERROR("Failed to find color stream profile: %d", result);
                    return false;
                }
                
                // 启动流
                result = ob_camera_start_streaming(camera_, depth_profile, nullptr);
                if (result != OB_SUCCESS) {
                    ROS_ERROR("Failed to start depth stream: %d", result);
                    return false;
                }
                
                result = ob_camera_start_streaming(camera_, color_profile, nullptr);
                if (result != OB_SUCCESS) {
                    ROS_ERROR("Failed to start color stream: %d", result);
                    return false;
                }
                
                ob_stream_profile_list_release(profile_list);
                ob_device_list_release(devices);
                
                return true;
            }
        }
        
        ob_device_list_release(devices);
        ROS_ERROR("Camera with ID %s not found", camera_id_);
        return false;
    }
    
    bool readPointCloud(sensor_msgs::PointCloud2& pointcloud) {
        // 等待获取帧
        ob_frame* frames[2] = {nullptr, nullptr};
        ob_error result = ob_camera_get_frames(camera_, frames, 2, OB_WAIT_FOREVER);
        
        if (result != OB_SUCCESS) {
            ROS_ERROR("Failed to get frames: %d", result);
            return false;
        }
        
        ob_frame* depth_frame = frames[0];
        ob_frame* color_frame = frames[1];
        
        // 检查帧类型
        if (ob_frame_get_type(depth_frame) != OB_FRAME_TYPE_DEPTH ||
            ob_frame_get_type(color_frame) != OB_FRAME_TYPE_COLOR) {
            ROS_ERROR("Invalid frame types received");
            ob_frame_release(depth_frame);
            ob_frame_release(color_frame);
            return false;
        }
        
        // 获取深度数据
        const uint16_t* depth_data = (const uint16_t*)ob_frame_get_data(depth_frame);
        int depth_width = ob_frame_get_width(depth_frame);
        int depth_height = ob_frame_get_height(depth_frame);
        
        // 获取彩色数据
        const uint8_t* color_data = (const uint8_t*)ob_frame_get_data(color_frame);
        int color_width = ob_frame_get_width(color_frame);
        int color_height = ob_frame_get_height(color_frame);
        
        // 计算点云（简化实现）
        pointcloud.header.stamp = ros::Time::now();
        pointcloud.header.frame_id = "camera_depth_frame";
        pointcloud.width = depth_width * depth_height;
        pointcloud.height = 1;
        pointcloud.is_dense = false;
        
        // 设置点云字段（X, Y, Z, RGB）
        pointcloud.fields.resize(4);
        pointcloud.fields[0].name = "x";
        pointcloud.fields[0].offset = 0;
        pointcloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        pointcloud.fields[0].count = 1;
        
        pointcloud.fields[1].name = "y";
        pointcloud.fields[1].offset = 4;
        pointcloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        pointcloud.fields[1].count = 1;
        
        pointcloud.fields[2].name = "z";
        pointcloud.fields[2].offset = 8;
        pointcloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        pointcloud.fields[2].count = 1;
        
        pointcloud.fields[3].name = "rgb";
        pointcloud.fields[3].offset = 12;
        pointcloud.fields[3].datatype = sensor_msgs::PointField::UINT32;
        pointcloud.fields[3].count = 1;
        
        pointcloud.point_step = 16;
        pointcloud.row_step = pointcloud.point_step * pointcloud.width;
        
        // 分配点云数据缓冲区
        pointcloud.data.resize(pointcloud.row_step * pointcloud.height);
        
        // 相机内参（示例数据，需要根据实际标定结果设置）
        float fx = 952.5;  // 焦距x
        float fy = 952.5;  // 焦距y
        float cx = 640.0;  // 主点x
        float cy = 360.0;  // 主点y
        float depth_scale = 1000.0;  // 深度缩放因子
        
        uint8_t* data_ptr = &pointcloud.data[0];
        
        for (int v = 0; v < depth_height; ++v) {
            for (int u = 0; u < depth_width; ++u) {
                int index = v * depth_width + u;
                uint16_t depth_value = depth_data[index];
                
                if (depth_value > 0) {
                    // 计算3D坐标
                    float z = depth_value / depth_scale;
                    float x = (u - cx) * z / fx;
                    float y = (v - cy) * z / fy;
                    
                    // 设置XYZ
                    memcpy(data_ptr, &x, sizeof(float));
                    data_ptr += 4;
                    memcpy(data_ptr, &y, sizeof(float));
                    data_ptr += 4;
                    memcpy(data_ptr, &z, sizeof(float));
                    data_ptr += 4;
                    
                    // 设置RGB（从彩色图像中获取，需要坐标映射）
                    // 这里是简化实现，直接使用深度值生成颜色
                    uint32_t rgb = (depth_value & 0xFF) << 16 | (depth_value & 0xFF) << 8 | (depth_value & 0xFF);
                    memcpy(data_ptr, &rgb, sizeof(uint32_t));
                    data_ptr += 4;
                } else {
                    // 无效点，填充0
                    memset(data_ptr, 0, 16);
                    data_ptr += 16;
                }
            }
        }
        
        ob_frame_release(depth_frame);
        ob_frame_release(color_frame);
        
        return true;
    }
    
private:
    std::string camera_id_;
    ob_camera* camera_;
};

禾赛激光雷达驱动集成：

#include "hesai_lidar_interface.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <netinet/in.h>
class HesaiLidar {
public:
    HesaiLidar(const std::string& ip, int port)
        : ip_(ip), port_(port), sockfd_(-1), imu_msg_(), initialized_(false) {
        // 初始化IMU消息
        imu_msg_.header.frame_id = "laser_imu_link";
        imu_msg_.orientation_covariance[0] = -1.0;  // 未使用
        imu_msg_.angular_velocity_covariance[0] = 0.01;  // 陀螺仪不确定性
        imu_msg_.linear_acceleration_covariance[0] = 0.01;  // 加速度计不确定性
    }
    
    bool init() {
        // 创建UDP套接字
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            ROS_ERROR("Failed to create socket: %s", strerror(errno));
            return false;
        }
        
        // 设置套接字选项（允许重用地址）
        int opt = 1;
        if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            ROS_ERROR("Failed to set socket options: %s", strerror(errno));
            close(sockfd_);
            return false;
        }
        
        // 绑定到本地端口（用于接收数据）
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port_);
        addr.sin_addr.s_addr = INADDR_ANY;
        
        if (bind(sockfd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            ROS_ERROR("Failed to bind socket: %s", strerror(errno));
            close(sockfd_);
            return false;
        }
        
        // 设置非阻塞模式
        fcntl(sockfd_, F_SETFL, O_NONBLOCK);
        
        initialized_ = true;
        return true;
    }
    
    bool readIMU(sensor_msgs::Imu& imu) {
        if (!initialized_) return false;
        
        // 接收UDP数据包
        char buffer[1500];
        struct sockaddr_in sender_addr;
        socklen_t addr_len = sizeof(sender_addr);
        
        int n = recvfrom(sockfd_, buffer, sizeof(buffer), 0,
                         (struct sockaddr*)&sender_addr, &addr_len);
        
        if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                ROS_ERROR("recvfrom failed: %s", strerror(errno));
            }
            return false;
        }
        
        // 解析数据包（简化实现，假设是IMU数据格式）
        if (n >= 42) {  // 假设IMU数据包至少42字节
            // 解析时间戳（前8字节）
            uint64_t timestamp = *((uint64_t*)buffer);
            
            // 解析加速度（接下来的12字节）
            float accel_x = *((float*)(buffer + 8));
            float accel_y = *((float*)(buffer + 12));
            float accel_z = *((float*)(buffer + 16));
            
            // 解析角速度（接下来的12字节）
            float gyro_x = *((float*)(buffer + 28));
            float gyro_y = *((float*)(buffer + 32));
            float gyro_z = *((float*)(buffer + 36));
            
            // 解析温度（最后4字节）
            float temperature = *((float*)(buffer + 42));
            
            // 更新IMU消息
            imu_msg_.header.stamp = ros::Time((timestamp >> 32), (timestamp & 0xFFFFFFFF));
            imu_msg_.linear_acceleration.x = accel_x;
            imu_msg_.linear_acceleration.y = accel_y;
            imu_msg_.linear_acceleration.z = accel_z;
            imu_msg_.angular_velocity.x = gyro_x;
            imu_msg_.angular_velocity.y = gyro_y;
            imu_msg_.angular_velocity.z = gyro_z;
            
            imu = imu_msg_;
            return true;
        }
        
        return false;
    }
    
private:
    std::string ip_;
    int port_;
    int sockfd_;
    sensor_msgs::Imu imu_msg_;
    bool initialized_;
};
