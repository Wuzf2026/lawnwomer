#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include "serial_port.h"
#include <vector>
#include <cstring>
#include <cmath>

namespace motor_control {

// 通信协议常量
const uint8_t FRAME_HEAD_CMD[2] = {0xAA, 0x55};    // 指令帧头
const uint8_t FRAME_TAIL_CMD[2] = {0xEE, 0xFF};    // 指令帧尾
const uint8_t FRAME_HEAD_RESP[2] = {0x55, 0xAA};   // 响应帧头
const uint8_t FRAME_TAIL_RESP[2] = {0xFF, 0xEE};   // 响应帧尾
const uint8_t DATA_LEN_CMD = 16;                   // 指令帧数据长度（线速度x/y+角速度z+刀盘速度）
const uint8_t DATA_LEN_RESP = 8;                   // 响应帧数据长度（左/右轮速）

class MotorController {
public:
  MotorController(ros::NodeHandle& nh);
  ~MotorController();

  /**
   * @brief 初始化控制器（打开串口、订阅话题）
   * @return 成功返回true，失败返回false
   */
  bool init();

  /**
   * @brief 主循环（接收串口数据、处理指令）
   */
  void spin();

private:
  // ROS相关
  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher wheel_speed_pub_;

  // 串口相关
  SerialPort serial_port_;
  std::string serial_port_;
  int baudrate_;
  std::string parity_;
  int stopbits_;
  int bytesize_;
  int timeout_;

  // 电机参数
  float max_linear_speed_;
  float max_angular_speed_;
  float max_cutter_speed_;

  // 缓存数据
  std::vector<uint8_t> recv_buffer_;

  /**
   * @brief cmd_vel话题回调函数
   * @param msg 速度指令消息
   */
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

  /**
   * @brief 封装指令帧并发送给下位机
   * @param linear_x 线速度x（m/s）
   * @param linear_y 线速度y（m/s）
   * @param angular_z 角速度z（rad/s）
   * @param cutter_speed 刀盘速度（rpm）
   * @return 成功返回true，失败返回false
   */
  bool sendMotorCmd(float linear_x, float linear_y, float angular_z, float cutter_speed);

  /**
   * @brief 解析下位机响应帧（轮速数据）
   * @param data 接收到的字节数组
   * @param len 数据长度
   * @param left_speed 解析出的左轮速
   * @param right_speed 解析出的右轮速
   * @return 解析成功返回true，失败返回false
   */
  bool parseWheelSpeed(const uint8_t* data, size_t len, float& left_speed, float& right_speed);

  /**
   * @brief 计算和校验
   * @param data 数据数组
   * @param len 数据长度
   * @return 校验和（低8位）
   */
  uint8_t calculateChecksum(const uint8_t* data, size_t len);

  /**
   * @brief 将float转换为小端序字节数组
   * @param val 浮点数
   * @param bytes 输出字节数组（长度4）
   */
  void floatToBytes(float val, uint8_t* bytes);

  /**
   * @brief 将小端序字节数组转换为float
   * @param bytes 字节数组（长度4）
   * @return 浮点数
   */
  float bytesToFloat(const uint8_t* bytes);
};

} // namespace motor_control

#endif // MOTOR_CONTROLLER_H