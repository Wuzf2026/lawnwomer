#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "serial_port.h"
#include "serial_protocol.h"
#include "motor_control/WheelSpeed.h"

class MotorDriver {
public:
    MotorDriver(ros::NodeHandle& nh);
    ~MotorDriver();
    bool init();
    void run();

private:
    void cmdVelCB(const geometry_msgs::Twist::ConstPtr& msg);
    void readSerial();

    ros::Subscriber sub_cmd_;
    ros::Publisher pub_wheel_;
    SerialPort serial_;
    SerialProtocol proto_;

    std::string port_;
    int baud_;
    float wheel_base_;
    float wheel_radius_;
};

#endif