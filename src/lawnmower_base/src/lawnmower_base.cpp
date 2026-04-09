#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_broadcaster.h>
class LawnmowerBase {
public:
    LawnmowerBase(ros::NodeHandle& nh) : nh_(nh) {
        // 订阅传感器数据
        orbbec_depth_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
            "/orbbec/camera/depth_points", 10, &LawnmowerBase::orbbec_depth_callback, this);
        
        hesai_lidar_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
            "/hesai_lidar/points", 10, &LawnmowerBase::hesai_lidar_callback, this);
        
        rtk_fix_sub_ = nh.subscribe<sensor_msgs::NavSatFix>(
            "/handsfree/rtk/gnss", 10, &LawnmowerBase::rtk_callback, this);
        
        hesai_imu_sub_ = nh.subscribe<sensor_msgs::Imu>(
            "/hesai_lidar/imu", 10, &LawnmowerBase::hesai_imu_callback, this);
        
        // 发布融合后的数据
        combined_point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
            "/lawnmower/sensor_combined", 10);
        
        // TF广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
        
        // 初始化数据缓冲区
        reset_data();
    }
    void run() {
        ros::Rate rate(30);  // 30Hz
        while (ros::ok()) {
            ros::spinOnce();
            
            // 发布TF变换
            publish_transforms();
            
            // 发布融合数据
            publish_combined_data();
            
            rate.sleep();
        }
    }
private:
    ros::NodeHandle nh_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // 订阅者
    ros::Subscriber orbbec_depth_sub_;
    ros::Subscriber hesai_lidar_sub_;
    ros::Subscriber rtk_fix_sub_;
    ros::Subscriber hesai_imu_sub_;
    
    // 发布者
    ros::Publisher combined_point_cloud_pub_;
    
    // 数据缓冲区
    sensor_msgs::PointCloud2 orbbec_depth_cloud_;
    sensor_msgs::PointCloud2 hesai_lidar_cloud_;
    sensor_msgs::NavSatFix rtk_fix_;
    sensor_msgs::Imu hesai_imu_;
    bool new_orbbec_data_ = false;
    bool new_hesai_data_ = false;
    bool new_rtk_data_ = false;
    bool new_hesai_imu_data_ = false;
    void reset_data() {
        orbbec_depth_cloud_.data.clear();
        hesai_lidar_cloud_.data.clear();
        new_orbbec_data_ = false;
        new_hesai_data_ = false;
        new_rtk_data_ = false;
        new_hesai_imu_data_ = false;
    }
    void orbbec_depth_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        orbbec_depth_cloud_ = *msg;
        new_orbbec_data_ = true;
    }
    void hesai_lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        hesai_lidar_cloud_ = *msg;
        new_hesai_data_ = true;
    }
    void rtk_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        rtk_fix_ = *msg;
        new_rtk_data_ = true;
    }
    void hesai_imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
        hesai_imu_ = *msg;
        new_hesai_imu_data_ = true;
    }
    void publish_combined_data() {
        sensor_msgs::PointCloud2 combined_cloud;
        
        // 合并点云数据
        if (new_orbbec_data_ || new_hesai_data_) {
            // 设置消息头
            combined_cloud.header.stamp = ros::Time::now();
            combined_cloud.header.frame_id = "base_link";
            
            // 计算总点数
            uint32_t total_points = 0;
            if (new_orbbec_data_) {
                total_points += orbbec_depth_cloud_.width * orbbec_depth_cloud_.height;
            }
            if (new_hesai_data_) {
                total_points += hesai_lidar_cloud_.width * hesai_lidar_cloud_.height;
            }
            
            if (total_points > 0) {
                // 初始化点云
                combined_cloud.width = total_points;
                combined_cloud.height = 1;
                combined_cloud.is_dense = false;
                combined_cloud.is_bigendian = false;
                
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
                
                combined_cloud.fields.push_back(x_field);
                combined_cloud.fields.push_back(y_field);
                combined_cloud.fields.push_back(z_field);
                combined_cloud.fields.push_back(intensity_field);
                
                combined_cloud.point_step = 16;
                combined_cloud.row_step = combined_cloud.point_step * combined_cloud.width;
                
                combined_cloud.data.resize(combined_cloud.row_step * combined_cloud.height);
                
                // 填充数据
                uint8_t* data_ptr = &combined_cloud.data[0];
                
                // 先添加Orbbec点云
                if (new_orbbec_data_) {
                    const sensor_msgs::PointCloud2& cloud = orbbec_depth_cloud_;
                    for (size_t i = 0; i < cloud.width * cloud.height; ++i) {
                        const float* point = reinterpret_cast<const float*>(&cloud.data[i * cloud.point_step]);
                        float* out_point = reinterpret_cast<float*>(data_ptr);
                        out_point[0] = point[0];
                        out_point[1] = point[1];
                        out_point[2] = point[2];
                        out_point[3] = 0.5f;  // Orbbec点云强度设为0.5
                        data_ptr += combined_cloud.point_step;
                    }
                }
                
                // 再添加Hesai激光雷达点云
                if (new_hesai_data_) {
                    const sensor_msgs::PointCloud2& cloud = hesai_lidar_cloud_;
                    for (size_t i = 0; i < cloud.width * cloud.height; ++i) {
                        const float* point = reinterpret_cast<const float*>(&cloud.data[i * cloud.point_step]);
                        float* out_point = reinterpret_cast<float*>(data_ptr);
                        out_point[0] = point[0];
                        out_point[1] = point[1];
                        out_point[2] = point[2];
                        out_point[3] = point[3];  // 使用激光雷达的原始强度
                        data_ptr += combined_cloud.point_step;
                    }
                }
                
                combined_point_cloud_pub_.publish(combined_cloud);
            }
        }
        
        // 重置数据标志
        new_orbbec_data_ = false;
        new_hesai_data_ = false;
    }
    void publish_transforms() {
        // 发布各个传感器的TF变换
        publish_tf("base_link", "camera_link", 0.0, 0.0, 0.5, 0, 0, 0);
        publish_tf("base_link", "laser", 0.5, 0.0, 1.0, 0, 0, 0);
        publish_tf("laser", "imu_link", 0.0, 0.0, 0.0, 0, 0, 0);
        publish_tf("base_link", "gps", 0.0, 0.0, 1.5, 0, 0, 0);
    }
    void publish_tf(const std::string& parent_frame, const std::string& child_frame,
                   double x, double y, double z,
                   double roll, double pitch, double yaw) {
        geometry_msgs::TransformStamped transformStamped;
        
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = parent_frame;
        transformStamped.child_frame_id = child_frame;
        
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = z;
        
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(transformStamped);
    }
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "lawnmower_base_node");
    ros::NodeHandle nh("~");
    
    LawnmowerBase base(nh);
    base.run();
    
    return 0;
}