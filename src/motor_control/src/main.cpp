#include <ros/ros.h>
#include "motor_control/motor_controller.h"

int main(int argc, char** argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "motor_control_node");
  ros::NodeHandle nh("~");

  // 创建电机控制器
  motor_control::MotorController controller(nh);

  // 初始化控制器
  if (!controller.init()) {
    ROS_FATAL("Motor controller initialization failed");
    return -1;
  }

  // 运行主循环
  controller.spin();

  return 0;
}