#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "motor_control/motor_driver.h"

using namespace motor_control;

class MotorControlNode {
public:
    MotorControlNode() {
        ros::NodeHandle nh_private("~");
        
        // 读取参数
        nh_private.param<std::string>("uart_dev", uart_dev_, "/dev/ttyS3");
        nh_private.param<int>("uart_baud", uart_baud_, 115200);
        nh_private.param<float>("kp", kp_, 1.0f);
        nh_private.param<float>("ki", ki_, 0.1f);
        nh_private.param<float>("kd", kd_, 0.05f);

        // 初始化电机驱动
        motor_driver_ = std::make_unique<MotorDriver>(uart_dev_, uart_baud_, kp_, ki_, kd_);
        if (motor_driver_->init()) {
            ROS_INFO("Motor driver initialized successfully");
        } else {
            ROS_ERROR("Failed to initialize motor driver");
            ros::shutdown();
            return;
        }

        // 发布/订阅
        speed_pub_ = nh_.advertise<std_msgs::Float32>("motor_speed", 10);
        cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &MotorControlNode::cmdVelCallback, this);
        speed_sub_ = nh_.subscribe("set_speed", 10, &MotorControlNode::speedCallback, this);

        // 定时器（100Hz更新）
        timer_ = nh_.createTimer(ros::Duration(0.01), &MotorControlNode::timerCallback, this);
        last_time_ = ros::Time::now();
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // 将线速度转换为电机转速（示例：1m/s = 100rpm）
        float target_rpm = msg->linear.x * 100.0f;
        motor_driver_->setSpeed(target_rpm);
    }

    void speedCallback(const std_msgs::Float32::ConstPtr& msg) {
        motor_driver_->setSpeed(msg->data);
    }

    void timerCallback(const ros::TimerEvent& e) {
        ros::Time now = ros::Time::now();
        float dt = (now - last_time_).toSec();
        last_time_ = now;

        // 更新电机控制
        motor_driver_->update(dt);

        // 发布当前转速
        std_msgs::Float32 speed_msg;
        speed_msg.data = motor_driver_->getCurrentSpeed();
        speed_pub_.publish(speed_msg);
    }

private:
    ros::NodeHandle nh_;
    std::unique_ptr<MotorDriver> motor_driver_;
    ros::Publisher speed_pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber speed_sub_;
    ros::Timer timer_;
    ros::Time last_time_;

    // 参数
    std::string uart_dev_;
    int uart_baud_;
    float kp_;
    float ki_;
    float kd_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_control_node");
    MotorControlNode node;
    ros::spin();
    return 0;
}