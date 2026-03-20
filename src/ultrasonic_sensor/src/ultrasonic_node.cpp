#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Float32.h"
#include "ultrasonic_sensor/ultrasonic_driver.h"

using namespace ultrasonic_sensor;

class UltrasonicSensorNode {
public:
    UltrasonicSensorNode() {
        ros::NodeHandle nh_private("~");
        
        // 读取参数
        nh_private.param<int>("trig_pin", trig_pin_, 23);
        nh_private.param<int>("echo_pin", echo_pin_, 24);
        nh_private.param<float>("frequency", frequency_, 10.0f); // 10Hz

        // 初始化超声波驱动
        ultrasonic_driver_ = std::make_unique<UltrasonicDriver>(trig_pin_, echo_pin_);
        if (ultrasonic_driver_->init()) {
            ROS_INFO("Ultrasonic sensor initialized (Trig: %d, Echo: %d)", trig_pin_, echo_pin_);
        } else {
            ROS_ERROR("Failed to initialize ultrasonic sensor");
            ros::shutdown();
            return;
        }

        // 发布器
        range_pub_ = nh_.advertise<sensor_msgs::Range>("ultrasonic_range", 10);
        distance_pub_ = nh_.advertise<std_msgs::Float32>("ultrasonic_distance", 10);

        // 定时器
        timer_ = nh_.createTimer(ros::Duration(1.0/frequency_), &UltrasonicSensorNode::timerCallback, this);
    }

    void timerCallback(const ros::TimerEvent& e) {
        float distance = ultrasonic_driver_->getDistance();

        // 发布Range消息（ROS标准）
        sensor_msgs::Range range_msg;
        range_msg.header.stamp = ros::Time::now();
        range_msg.header.frame_id = "ultrasonic_sensor";
        range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
        range_msg.field_of_view = 0.1; // 10度视场角
        range_msg.min_range = 0.02; // 最小2cm
        range_msg.max_range = 4.0; // 最大4m
        range_msg.range = std::isnan(distance) ? range_msg.max_range + 1 : distance;

        // 发布Float32消息
        std_msgs::Float32 distance_msg;
        distance_msg.data = distance;

        range_pub_.publish(range_msg);
        distance_pub_.publish(distance_msg);

        if (std::isnan(distance)) {
            ROS_WARN("Ultrasonic sensor read timeout");
        } else {
            ROS_DEBUG("Ultrasonic distance: %.2f m", distance);
        }
    }

private:
    ros::NodeHandle nh_;
    std::unique_ptr<UltrasonicDriver> ultrasonic_driver_;
    ros::Publisher range_pub_;
    ros::Publisher distance_pub_;
    ros::Timer timer_;

    // 参数
    int trig_pin_;
    int echo_pin_;
    float frequency_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ultrasonic_sensor_node");
    UltrasonicSensorNode node;
    ros::spin();
    return 0;
}