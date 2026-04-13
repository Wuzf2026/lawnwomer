#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <std_msgs/Float64.h>
#include <lawnmower_msgs/LawnmowerStatus.h>
#include <sstream>
#include <string>

// 全局变量
serial::Serial ser;
double g_max_linear = 0.5;
double g_max_angular = 1.0;
double g_wheel_base = 0.5;
double g_wheel_radius = 0.1;

// 解析串口回采的轮速数据
bool parseWheelSpeed(const std::string& data, double& left_speed, double& right_speed) {
  // 串口协议示例："L:0.2,R:0.25\n"
  size_t l_pos = data.find("L:");
  size_t r_pos = data.find("R:");
  size_t nl_pos = data.find("\n");
  if (l_pos == std::string::npos || r_pos == std::string::npos || nl_pos == std::string::npos) {
    return false;
  }
  left_speed = std::atof(data.substr(l_pos+2, r_pos - l_pos - 2).c_str());
  right_speed = std::atof(data.substr(r_pos+2, nl_pos - r_pos - 2).c_str());
  return true;
}

// 回调函数：/cmd_vel话题
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  // 限幅
  double linear_x = std::clamp(msg->linear.x, -g_max_linear, g_max_linear);
  double angular_z = std::clamp(msg->angular.z, -g_max_angular, g_max_angular);

  // 运动学逆解：线速度/角速度 -> 左右轮速 (rad/s)
  double left_rad = (linear_x - angular_z * g_wheel_base / 2) / g_wheel_radius;
  double right_rad = (linear_x + angular_z * g_wheel_base / 2) / g_wheel_radius;

  // 构造串口指令（自定义协议，示例："CMD:L1.2,R1.3\n"）
  std::stringstream cmd;
  cmd << "CMD:L" << left_rad << ",R" << right_rad << "\n";
  
  // 发送串口指令
  try {
    ser.write(cmd.str());
    ROS_DEBUG("Send serial cmd: %s", cmd.str().c_str());
  } catch (serial::IOException& e) {
    ROS_ERROR("Serial write error: %s", e.what());
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "motor_control_node");
  ros::NodeHandle nh("~");

  // 加载参数
  nh.param<double>("max_linear_speed", g_max_linear, 0.5);
  nh.param<double>("max_angular_speed", g_max_angular, 1.0);
  nh.param<double>("motor/wheel_base", g_wheel_base, 0.5);
  nh.param<double>("motor/wheel_radius", g_wheel_radius, 0.1);

  // 串口配置
  std::string port;
  int baudrate;
  nh.param<std::string>("serial/port", port, "/dev/ttyS2");
  nh.param<int>("serial/baudrate", baudrate, 115200);

  // 初始化串口
  try {
    ser.setPort(port);
    ser.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    ser.setTimeout(to);
    ser.open();
  } catch (serial::IOException& e) {
    ROS_FATAL("Failed to open serial port %s: %s", port.c_str(), e.what());
    return -1;
  }

  if (ser.isOpen()) {
    ROS_INFO("Serial port %s opened (baudrate: %d)", port.c_str(), baudrate);
  } else {
    ROS_FATAL("Serial port %s not open", port.c_str());
    return -1;
  }

  // 订阅/cmd_vel话题
  ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmdVelCallback);

  // 发布轮速话题
  ros::Publisher left_wheel_pub = nh.advertise<std_msgs::Float64>("/lawnmower/left_wheel_speed", 10);
  ros::Publisher right_wheel_pub = nh.advertise<std_msgs::Float64>("/lawnmower/right_wheel_speed", 10);

  // 循环频率
  ros::Rate rate(50); // 50Hz处理串口数据

  ROS_INFO("Motor control node started (RK3588 ROS1 Noetic)");

  while (ros::ok()) {
    // 读取串口回采数据
    if (ser.available() > 0) {
      std::string recv_data = ser.read(ser.available());
      double left_speed, right_speed;
      if (parseWheelSpeed(recv_data, left_speed, right_speed)) {
        // 发布轮速（转换为m/s：rad/s * 轮径）
        std_msgs::Float64 left_msg, right_msg;
        left_msg.data = left_speed * g_wheel_radius;
        right_msg.data = right_speed * g_wheel_radius;
        left_wheel_pub.publish(left_msg);
        right_wheel_pub.publish(right_msg);
        ROS_DEBUG("Recv wheel speed: L=%.2f m/s, R=%.2f m/s", left_msg.data, right_msg.data);
      } else {
        ROS_WARN("Invalid serial data: %s", recv_data.c_str());
      }
    }

    ros::spinOnce();
    rate.sleep();
  }

  // 关闭串口
  ser.close();
  return 0;
}