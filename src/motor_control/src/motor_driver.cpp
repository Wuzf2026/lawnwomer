#include "motor_control/motor_driver.h"

MotorDriver::MotorDriver(ros::NodeHandle& nh) {
    nh.param<std::string>("port", port_, "/dev/ttyS2");
    nh.param<int>("baud", baud_, 115200);
    sub_cmd_ = nh.subscribe("/cmd_vel", 10, &MotorDriver::cmdVelCB, this);
    pub_wheel_ = nh.advertise<motor_control::WheelSpeed>("/wheel_speed", 10);
}

MotorDriver::~MotorDriver() {}

bool MotorDriver::init() {
    if (!serial_.init(port_, baud_)) return false;
    ROS_INFO("Serial OK");
    return true;
}

void MotorDriver::cmdVelCB(const geometry_msgs::Twist::ConstPtr& msg) {
    auto frame = proto_.pack(msg->linear.x, msg->angular.z, msg->angular.y);
    serial_.send(frame.data(), frame.size());
}

void MotorDriver::readSerial() {
    uint8_t buf[18];
    int len = serial_.receive(buf, 18);
    if (len != 18) return;

    float l, r, c;
    if (proto_.unpack(buf, l, r, c)) {
        motor_control::WheelSpeed msg;
        msg.header.stamp = ros::Time::now();
        msg.left_wheel = l;
        msg.right_wheel = r;
        msg.cutter = c;
        pub_wheel_.publish(msg);
    }
}

void MotorDriver::run() {
    ros::Rate rate(50);
    while (ros::ok()) {
        readSerial();
        ros::spinOnce();
        rate.sleep();
    }
}