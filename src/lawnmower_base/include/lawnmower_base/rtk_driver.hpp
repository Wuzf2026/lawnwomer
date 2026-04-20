#ifndef RTK_DRIVER_H
#define RTK_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <handsfree_rtk/rtk_parse.h>

class RTKDriver
{
public:
    RTKDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~RTKDriver();

    bool init();
    void readData();
    bool hasNewGpsData();

    sensor_msgs::NavSatFixPtr getGpsFix();
    std_msgs::StringPtr getNmeaMsg();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher pub_gps_fix_;
    ros::Publisher pub_nmea_raw_;

    std::string uart_dev_;
    int baudrate_;
    std::string gps_frame_id_;

    serial::Serial* serial_port_;
    RtkParser rtk_parser_;
    bool new_gps_data_;

    sensor_msgs::NavSatFixPtr gps_fix_msg_;
    std_msgs::StringPtr nmea_msg_;
};

#endif