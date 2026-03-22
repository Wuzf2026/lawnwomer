#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

class UltrasonicSensor {
public:
    UltrasonicSensor(ros::NodeHandle &nh);
    bool init();
    void run();

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_dist_;
    int uart_fd_;
};

#endif