#ifndef DEBUG_TOOL_H
#define DEBUG_TOOL_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "sensor_data/sensor_data_node.h"
//#include "motor_control/motor_control_node.h"

namespace tools {

class DebugTool {
public:
    DebugTool(ros::NodeHandle& nh);
    void monitorAllData();
    void printSystemInfo();

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_sensor_fusion_;
    ros::Subscriber sub_motor_status_;

    void fusionDataCallback(const std_msgs::String::ConstPtr& msg);
    void motorStatusCallback(const std_msgs::String::ConstPtr& msg);
};

}  // namespace tools

#endif  // DEBUG_TOOL_H