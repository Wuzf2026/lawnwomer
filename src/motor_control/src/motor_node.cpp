#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>
#include "serial_protocol.h"
#include "rk3588_serial.h"

using namespace motor_control;
using namespace rk3588_serial;

class MotorControlNode {
public:
    MotorControlNode() : nh_("~"), serial_port_() {
        // 读取配置
        std::string serial_port;
        int baudrate, cutter_vel;
        nh_.param<std::string>("serial_port", serial_port, "/dev/ttyS2");
        nh_.param<int>("baudrate", baudrate, 115200);
        nh_.param<int>("cutter_vel", cutter_vel, 0);
        cutter_vel_ = static_cast<uint16_t>(cutter_vel);

        // 初始化串口
        if (!serial_port_.init(serial_port, baudrate)) {
            ROS_FATAL("Serial port init failed!");
            ros::shutdown();
        }

        // 订阅/cmd_vel话题
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &MotorControlNode::cmdVelCb, this);

        // 发布轮速话题
        wheel_speed_pub_ = nh_.advertise<std_msgs::Int16MultiArray>("/wheel_speed", 10);

        // 启动回采线程
        recv_thread_ = std::thread(&MotorControlNode::recvWheelSpeed, this);

        ROS_INFO("Motor control node init success (RK3588)");
    }

    ~MotorControlNode() {
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
        serial_port_.closePort();
    }

    void cmdVelCb(const geometry_msgs::Twist::ConstPtr& twist) {
        // 打包控制帧并发送
        auto frame_data = packControlFrame(twist, cutter_vel_);
        if (serial_port_.sendData(frame_data)) {
            ROS_DEBUG("Send control frame: linear=%.2f m/s, angular=%.2f rad/s",
                      twist->linear.x, twist->angular.z);
        } else {
            ROS_ERROR("Failed to send control frame");
        }
    }

    void recvWheelSpeed() {
        ros::Rate rate(100); // 100Hz回采
        while (ros::ok()) {
            std::vector<uint8_t> recv_data;
            // 读取9字节轮速帧
            if (serial_port_.recvData(recv_data, 9, 10)) {
                WheelSpeedFrame frame;
                if (unpackWheelSpeedFrame(recv_data, frame)) {
                    // 发布轮速话题（left, right）
                    std_msgs::Int16MultiArray wheel_speed_msg;
                    wheel_speed_msg.data.push_back(frame.left_wheel_vel);
                    wheel_speed_msg.data.push_back(frame.right_wheel_vel);
                    wheel_speed_pub_.publish(wheel_speed_msg);
                    ROS_DEBUG("Recv wheel speed: left=%d cm/s, right=%d cm/s",
                              frame.left_wheel_vel, frame.right_wheel_vel);
                } else {
                    ROS_WARN("Invalid wheel speed frame");
                }
            }
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    SerialPort serial_port_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher wheel_speed_pub_;
    std::thread recv_thread_;
    uint16_t cutter_vel_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_control_node");
    MotorControlNode node;
    ros::spin();
    return 0;
}