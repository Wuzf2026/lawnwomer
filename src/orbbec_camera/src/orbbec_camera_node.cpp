#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
// Orbbec SDK
#include <orbbecsdk.h>
// 相机参数配置
struct CameraParams {
    std::string device_type;
    std::string camera_name;
    int connection_delay;
    std::string log_level;
    std::string log_file_name;
    bool publish_tf;
    double tf_publish_rate;
    bool enable_frame_sync;
    std::string time_domain;
    std::string uvc_backend;
    bool enable_sync_host_time;
    std::string device_preset;
    std::string color_preset;
    double diagnostics_frequency;
    bool enable_laser;
    std::string depth_precision;
    std::string sync_mode;
    std::string disparity_to_depth_mode;
    int depth_delay_us;
    int color_delay_us;
    int trigger2image_delay_us;
    int trigger_out_delay_us;
    bool trigger_out_enabled;
    int frames_per_trigger;
    int software_trigger_period;
    bool enable_ptp_config;
    std::string upgrade_firmware;
    std::string preset_firmware_path;
    std::string config_file_path;
    std::string load_config_json_file_path;
    std::string export_config_json_file_path;
    bool enumerate_net_device;
    std::string device_access_mode;
    std::string ip_address;
    int port;
    std::string exposure_range_mode;
    std::string ir_info_uri;
    std::string color_info_uri;
    int color_width;
    int color_height;
    int color_fps;
    bool enable_color;
    std::string color_format;
    int color_rotation;
    bool color_flip;
    bool color_mirror;
    bool enable_color_auto_exposure_priority;
    bool enable_color_auto_exposure;
    int color_exposure;
    int color_ae_roi_left;
    int color_ae_roi_right;
    int color_ae_roi_top;
    int color_ae_roi_bottom;
    int color_gain;
    bool enable_color_auto_white_balance;
    int color_white_balance;
    int color_ae_max_exposure;
    int color_brightness;
    int color_sharpness;
    int color_gamma;
    int color_saturation;
    int color_contrast;
    int color_hue;
    int color_backlight_compensation;
    std::string color_powerline_freq;
    bool enable_color_decimation_filter;
    int color_decimation_filter_scale;
    int color_denoising_level;
    int depth_width;
    int depth_height;
    int depth_fps;
    bool enable_depth;
    std::string depth_format;
    bool depth_registration;
    int depth_rotation;
    bool depth_flip;
    bool depth_mirror;
    bool enable_accel_data_correction;
    std::string accel_rate;
    bool enable_gyro_data_correction;
    double linear_accel_cov;
    std::string usb_port;
    std::string serial_number;
    int device_num;
    bool retry_on_usb3_detection_failure;
    bool enable_d2c_viewer;
    int laser_energy_level;
    bool enable_ldp;
    bool enable_heartbeat;
    bool enable_hardware_reset;
    std::string frame_aggregate_mode;
    std::string disparity_range_mode;
    int disparity_search_offset;
    std::string disparity_offset_config;
    int offset_index0;
    int offset_index1;
    std::string interleave_ae_mode;
    bool interleave_frame_enable;
    bool interleave_skip_enable;
    int interleave_skip_index;
    std::string hdr_index1_laser_control;
    int hdr_index1_depth_exposure;
    int hdr_index1_depth_gain;
    int hdr_index1_ir_brightness;
    int hdr_index1_ir_ae_max_exposure;
    std::string hdr_index0_laser_control;
    int hdr_index0_depth_exposure;
    int hdr_index0_depth_gain;
    int hdr_index0_ir_brightness;
    int hdr_index0_ir_ae_max_exposure;
    std::string laser_index1_laser_control;
    int laser_index1_depth_exposure;
    int laser_index1_depth_gain;
    int laser_index1_ir_brightness;
    int laser_index1_ir_ae_max_exposure;
    std::string laser_index0_laser_control;
    int laser_index0_depth_exposure;
    int laser_index0_depth_gain;
    int laser_index0_ir_brightness;
    int laser_index0_ir_ae_max_exposure;
    bool force_ip_enable;
    std::string force_ip_mac;
    std::string force_ip_address;
    std::string force_ip_subnet_mask;
    std::string force_ip_gateway;
    std::string intra_camera_sync_reference;
    bool enable_point_cloud;
    int point_cloud_decimation_filter_factor;
    bool ordered_pc;
    bool enable_threshold_filter;
    bool enable_temporal_filter;
    bool enable_spatial_fast_filter;
    int threshold_filter_max;
    int threshold_filter_min;
    double spatial_filter_alpha;
    int spatial_filter_diff_threshold;
    int spatial_filter_magnitude;
    int spatial_filter_radius;
    int temporal_filter_diff_threshold;
    double temporal_filter_weight;
    int hdr_merge_exposure_2;
    int hdr_merge_gain_2;
    int spatial_fast_filter_radius;
    int spatial_moderate_filter_diff_threshold;
    int spatial_moderate_filter_radius;
    bool enable_sync_output_accel_gyro;
    bool enable_accel;
    bool enable_gyro;
    std::string accel_range;
    std::string gyro_range;
    std::string gyro_rate;
};
class OrbbecCameraNode {
public:
    OrbbecCameraNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
        // 读取参数
        read_params();
        
        // 初始化相机
        initialize_camera();
        
        // 启动发布线程
        if (params.enable_point_cloud) {
            point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("orbbec/camera/depth_points", 10);
        }
        if (params.enable_accel || params.enable_gyro) {
            imu_pub_ = nh_.advertise<sensor_msgs::Imu>("orbbec/camera/imu", 10);
        }
        
        // TF广播器
        if (params.publish_tf) {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
        }
        
        // 诊断更新器
        updater_.setHardwareID("orbbec_camera");
        updater_.add("Camera Status", this, &OrbbecCameraNode::diagnosticCameraStatus);
        timer_ = nh_.createTimer(ros::Duration(params.diagnostics_frequency), 
                                &OrbbecCameraNode::diagnosticTimerCallback, this);
    }
    ~OrbbecCameraNode() {
        stop_camera();
    }
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    CameraParams params;
    orbbec::Camera cam_;
    orbbec::FrameSet frames_;
    orbbec::VideoStream depth_stream_;
    orbbec::VideoStream color_stream_;
    orbbec::VideoStream ir_stream_;
    orbbec::ImuStream imu_stream_;
    
    ros::Publisher point_cloud_pub_;
    ros::Publisher imu_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    diagnostic_updater::Updater updater_;
    ros::Timer timer_;
    void read_params() {
        // 读取参数（简化版，只读取关键参数）
        params.camera_name = pnh_.param<std::string>("camera_name", "camera");
        params.device_type = pnh_.param<std::string>("device_type", "usb");
        params.color_width = pnh_.param<int>("color_width", 1280);
        params.color_height = pnh_.param<int>("color_height", 720);
        params.color_fps = pnh_.param<int>("color_fps", 30);
        params.depth_width = pnh_.param<int>("depth_width", 1280);
        params.depth_height = pnh_.param<int>("depth_height", 720);
        params.depth_fps = pnh_.param<int>("depth_fps", 30);
        params.enable_point_cloud = pnh_.param<bool>("enable_point_cloud", true);
        params.enable_accel = pnh_.param<bool>("enable_accel", true);
        params.enable_gyro = pnh_.param<bool>("enable_gyro", true);
        params.publish_tf = pnh_.param<bool>("publish_tf", true);
    }
    void initialize_camera() {
        try {
            // 初始化SDK
            orbbec::Initialize();
            
            // 查找设备
            std::vector<orbbec::DeviceInfo> devices = orbbec::DeviceFinder::getInstance().getDevices();
            if (devices.empty()) {
                throw std::runtime_error("No Orbbec device found");
            }
            
            // 打开设备
            cam_ = orbbec::DeviceFinder::getInstance().openDevice(devices[0]);
            
            // 配置流
            depth_stream_ = cam_.createStream(orbbec::StreamType::DEPTH, 
                                             params.depth_width, 
                                             params.depth_height, 
                                             orbbec::PixelFormat::DEPTH_1_MM, 
                                             params.depth_fps);
            
            color_stream_ = cam_.createStream(orbbec::StreamType::COLOR, 
                                             params.color_width, 
                                             params.color_height, 
                                             orbbec::PixelFormat::RGB888, 
                                             params.color_fps);
            
            if (params.enable_accel || params.enable_gyro) {
                imu_stream_ = cam_.createImuStream();
            }
            
            // 启动流
            depth_stream_.start();
            color_stream_.start();
            if (params.enable_accel || params.enable_gyro) {
                imu_stream_.start();
            }
            
            ROS_INFO("Orbbec camera initialized successfully");
        } catch (const std::exception& e) {
            ROS_FATAL("Failed to initialize Orbbec camera: %s", e.what());
            ros::shutdown();
        }
    }
    void stop_camera() {
        try {
            if (depth_stream_.isOpened()) {
                depth_stream_.stop();
            }
            if (color_stream_.isOpened()) {
                color_stream_.stop();
            }
            if (imu_stream_.isOpened()) {
                imu_stream_.stop();
            }
            if (cam_.isOpened()) {
                cam_.close();
            }
            orbbec::Shutdown();
        } catch (const std::exception& e) {
            ROS_WARN("Error stopping camera: %s", e.what());
        }
    }
    void publish_data() {
        try {
            // 获取帧数据
            frames_ = cam_.waitForFrame(100);
            
            // 发布点云
            if (params.enable_point_cloud && frames_.exist(orbbec::StreamType::DEPTH)) {
                const orbbec::Frame& depth_frame = frames_.getDepthFrame();
                publish_point_cloud(depth_frame);
            }
            
            // 发布IMU
            if ((params.enable_accel || params.enable_gyro) && frames_.exist(orbbec::StreamType::IMU)) {
                const orbbec::ImuFrame& imu_frame = frames_.getImuFrame();
                publish_imu(imu_frame);
            }
            
            // 发布TF
            if (params.publish_tf) {
                publish_transform();
            }
        } catch (const std::exception& e) {
            ROS_WARN("Error publishing data: %s", e.what());
        }
    }
    void publish_point_cloud(const orbbec::Frame& depth_frame) {
        sensor_msgs::PointCloud2 point_cloud;
        
        // 设置消息头
        point_cloud.header.stamp = ros::Time::now();
        point_cloud.header.frame_id = "camera_depth_frame";
        
        // 点云参数
        point_cloud.width = depth_frame.getWidth();
        point_cloud.height = depth_frame.getHeight();
        point_cloud.is_dense = false;
        point_cloud.is_bigendian = false;
        
        // 点云格式（XYZ + RGB）
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
        
        sensor_msgs::PointField rgb_field;
        rgb_field.name = "rgb";
        rgb_field.offset = 12;
        rgb_field.datatype = sensor_msgs::PointField::FLOAT32;
        rgb_field.count = 1;
        
        point_cloud.fields.push_back(x_field);
        point_cloud.fields.push_back(y_field);
        point_cloud.fields.push_back(z_field);
        point_cloud.fields.push_back(rgb_field);
        
        // 计算步长
        point_cloud.point_step = 16;
        point_cloud.row_step = point_cloud.point_step * point_cloud.width;
        
        // 分配内存
        point_cloud.data.resize(point_cloud.row_step * point_cloud.height);
        
        // 填充点云数据（简化实现，仅使用深度数据）
        const uint16_t* depth_data = static_cast<const uint16_t*>(depth_frame.getData());
        for (int i = 0; i < point_cloud.width * point_cloud.height; ++i) {
            uint16_t depth = depth_data[i];
            
            // 转换为米
            float z = depth / 1000.0f;
            
            // 简化的坐标计算（假设相机内参）
            int u = i % point_cloud.width;
            int v = i / point_cloud.width;
            
            float x = (u - point_cloud.width/2) * z / 525.0f;  // 假设焦距525px
            float y = (v - point_cloud.height/2) * z / 525.0f;
            
            // 填充数据
            float* point_data = reinterpret_cast<float*>(&point_cloud.data[i * point_cloud.point_step]);
            point_data[0] = x;
            point_data[1] = y;
            point_data[2] = z;
            point_data[3] = 0x00ffffff;  // 白色
        }
        
        point_cloud_pub_.publish(point_cloud);
    }
    void publish_imu(const orbbec::ImuFrame& imu_frame) {
        sensor_msgs::Imu imu_msg;
        
        // 设置消息头
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "camera_imu_frame";
        
        // 加速度
        orbbec::Vector3f accel = imu_frame.getLinearAcceleration();
        imu_msg.linear_acceleration.x = accel.x;
        imu_msg.linear_acceleration.y = accel.y;
        imu_msg.linear_acceleration.z = accel.z;
        
        // 角速度
        orbbec::Vector3f gyro = imu_frame.getAngularVelocity();
        imu_msg.angular_velocity.x = gyro.x;
        imu_msg.angular_velocity.y = gyro.y;
        imu_msg.angular_velocity.z = gyro.z;
        
        // 协方差（简化设置）
        imu_msg.linear_acceleration_covariance[0] = params.linear_accel_cov;
        imu_msg.angular_velocity_covariance[0] = 0.01;
        
        imu_pub_.publish(imu_msg);
    }
    void publish_transform() {
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "camera_link";
        transformStamped.child_frame_id = "camera_depth_frame";
        
        // 相机深度帧相对于相机链接帧的变换（示例值）
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        
        br.sendTransform(transformStamped);
    }
    void diagnosticCameraStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Camera running");
        stat.add("Device Type", params.device_type);
        stat.add("Resolution", 
                std::to_string(params.color_width) + "x" + 
                std::to_string(params.color_height) + " (color), " +
                std::to_string(params.depth_width) + "x" + 
                std::to_string(params.depth_height) + " (depth)");
        stat.add("FPS", 
                std::to_string(params.color_fps) + " (color), " +
                std::to_string(params.depth_fps) + " (depth)");
    }
    void diagnosticTimerCallback(const ros::TimerEvent&) {
        updater_.update();
    }
    void run() {
        ros::Rate rate(30);  // 30Hz
        while (ros::ok()) {
            publish_data();
            ros::spinOnce();
            rate.sleep();
        }
    }
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "orbbec_camera_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    OrbbecCameraNode node(nh, pnh);
    node.run();
    
    return 0;
}
launch/gemini_330_series.launch

<?xml version="1.0"?>
<launch>
    <arg name="device_type" default="usb"/>
    <arg name="camera_name" default="camera"/>
    <arg name="color_width" default="1280"/>
    <arg name="color_height" default="720"/>
    <arg name="color_fps" default="30"/>
    <arg name="depth_width" default="1280"/>
    <arg name="depth_height" default="720"/>
    <arg name="depth_fps" default="30"/>
    <arg name="enable_point_cloud" default="true"/>
    <arg name="enable_accel" default="true"/>
    <arg name="enable_gyro" default="true"/>
    <arg name="publish_tf" default="true"/>
    
    <node name="orbbec_camera_node" pkg="orbbec_camera" type="orbbec_camera_node" output="screen">
        <param name="device_type" value="$(arg device_type)"/>
        <param name="camera_name" value="$(arg camera_name)"/>
        <param name="color_width" value="$(arg color_width)"/>
        <param name="color_height" value="$(arg color_height)"/>
        <param name="color_fps" value="$(arg color_fps)"/>
        <param name="depth_width" value="$(arg depth_width)"/>
        <param name="depth_height" value="$(arg depth_height)"/>
        <param name="depth_fps" value="$(arg depth_fps)"/>
        <param name="enable_point_cloud" value="$(arg enable_point_cloud)"/>
        <param name="enable_accel" value="$(arg enable_accel)"/>
        <param name="enable_gyro" value="$(arg enable_gyro)"/>
        <param name="publish_tf" value="$(arg publish_tf)"/>
    </node>
</launch>