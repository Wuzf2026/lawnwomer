#include "tools/debug_tool.h"
#include <sys/utsname.h>

using namespace tools;

int main(int argc, char** argv) {
    ros::init(argc, argv, "debug_tool");
    ros::NodeHandle nh("~");

    ROS_INFO("【DebugTool】启动调试工具（RK3588 ROS1 Noetic）");

    DebugTool tool(nh);
    tool.printSystemInfo();

    ros::Rate rate(10);  // 10Hz
    while (ros::ok()) {
        tool.monitorAllData();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

DebugTool::DebugTool(ros::NodeHandle& nh) : nh_(nh) {
    sub_sensor_fusion_ = nh_.subscribe("/sensor_data/fusion", 10, &DebugTool::fusionDataCallback, this);
    sub_motor_status_ = nh_.subscribe("/motor_control/status", 10, &DebugTool::motorStatusCallback, this);
}

void DebugTool::monitorAllData() {
    ROS_INFO("【DebugTool】监控所有传感器/电机数据...");
}

void DebugTool::printSystemInfo() {
    // 打印RK3588系统信息
    struct utsname sys_info;
    uname(&sys_info);
    ROS_INFO("【DebugTool】系统信息：");
    ROS_INFO("  架构: %s", sys_info.machine);
    ROS_INFO("  系统: %s", sys_info.sysname);
    ROS_INFO("  版本: %s", sys_info.version);
}

void DebugTool::fusionDataCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("【DebugTool】传感器融合数据：%s", msg->data.c_str());
}

void DebugTool::motorStatusCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("【DebugTool】电机状态：%s", msg->data.c_str());
}