#include "motor_control/motor_controller.h"

namespace motor_control {

MotorController::MotorController(ros::NodeHandle& nh) : nh_(nh), fd_(-1) {
  // 读取参数
  nh_.param<std::string>("serial/port", serial_port_, "/dev/ttyS2");
  nh_.param<int>("serial/baudrate", baudrate_, 115200);
  nh_.param<std::string>("serial/parity", parity_, "none");
  nh_.param<int>("serial/stopbits", stopbits_, 1);
  nh_.param<int>("serial/bytesize", bytesize_, 8);
  nh_.param<int>("serial/timeout", timeout_, 100);

  nh_.param<float>("motor/max_linear_speed", max_linear_speed_, 0.5);
  nh_.param<float>("motor/max_angular_speed", max_angular_speed_, 1.0);
  nh_.param<float>("motor/max_cutter_speed", max_cutter_speed_, 2000.0);

  // 初始化发布/订阅
  cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &MotorController::cmdVelCallback, this);
  wheel_speed_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/wheel_speed", 10);

  recv_buffer_.reserve(1024); // 预分配接收缓冲区
}

MotorController::~MotorController() {
  serial_port_.closePort();
}

bool MotorController::init() {
  // 打开串口
  if (!serial_port_.openPort(serial_port_, baudrate_, parity_, stopbits_, bytesize_, timeout_)) {
    ROS_ERROR("Failed to initialize serial port");
    return false;
  }

  ROS_INFO("Motor controller initialized successfully");
  return true;
}

void MotorController::spin() {
  ros::Rate rate(100); // 100Hz循环
  while (ros::ok()) {
    // 读取串口数据
    uint8_t buffer[256];
    ssize_t len = serial_port_.readData(buffer, sizeof(buffer));
    if (len > 0) {
      // 将新数据加入接收缓冲区
      recv_buffer_.insert(recv_buffer_.end(), buffer, buffer + len);

      // 解析缓冲区中的响应帧
      size_t frame_start = 0;
      while (frame_start + 14 <= recv_buffer_.size()) { // 响应帧总长度14字节
        // 查找帧头
        if (recv_buffer_[frame_start] == FRAME_HEAD_RESP[0] && 
            recv_buffer_[frame_start + 1] == FRAME_HEAD_RESP[1]) {
          // 验证帧尾
          size_t frame_end = frame_start + 13;
          if (recv_buffer_[frame_end - 1] == FRAME_TAIL_RESP[0] && 
              recv_buffer_[frame_end] == FRAME_TAIL_RESP[1]) {
            // 验证数据长度
            if (recv_buffer_[frame_start + 2] == DATA_LEN_RESP) {
              // 验证校验位
              uint8_t checksum = calculateChecksum(&recv_buffer_[frame_start + 2], 9); // 数据长度+左轮速+右轮速
              if (recv_buffer_[frame_start + 2 + DATA_LEN_RESP + 1] == checksum) {
                // 解析轮速
                float left_speed = bytesToFloat(&recv_buffer_[frame_start + 3]);
                float right_speed = bytesToFloat(&recv_buffer_[frame_start + 7]);

                // 发布轮速话题
                std_msgs::Float32MultiArray msg;
                msg.data.push_back(left_speed);
                msg.data.push_back(right_speed);
                wheel_speed_pub_.publish(msg);

                ROS_DEBUG("Received wheel speed: left=%.2f m/s, right=%.2f m/s", left_speed, right_speed);
              } else {
                ROS_WARN("Invalid checksum for wheel speed frame");
              }
            } else {
              ROS_WARN("Invalid data length for wheel speed frame");
            }
            // 跳过已解析的帧
            frame_start += 14;
          } else {
            frame_start++;
          }
        } else {
          frame_start++;
        }
      }

      // 清理已处理的缓冲区数据
      if (frame_start > 0) {
        recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + frame_start);
      }

      // 防止缓冲区过大
      if (recv_buffer_.size() > 1024) {
        recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.end() - 1024);
        ROS_WARN("Serial receive buffer overflow, truncated");
      }
    }

    ros::spinOnce();
    rate.sleep();
  }
}

void MotorController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  // 限制速度范围
  float linear_x = std::clamp(msg->linear.x, -max_linear_speed_, max_linear_speed_);
  float linear_y = std::clamp(msg->linear.y, -max_linear_speed_, max_linear_speed_);
  float angular_z = std::clamp(msg->angular.z, -max_angular_speed_, max_angular_speed_);
  float cutter_speed = 0.0; // 示例：刀盘速度可从自定义话题/参数读取，此处暂设0

  // 发送电机指令
  if (!sendMotorCmd(linear_x, linear_y, angular_z, cutter_speed)) {
    ROS_WARN("Failed to send motor command");
  }

  ROS_DEBUG("Send cmd_vel: linear_x=%.2f, angular_z=%.2f", linear_x, angular_z);
}

bool MotorController::sendMotorCmd(float linear_x, float linear_y, float angular_z, float cutter_speed) {
  std::vector<uint8_t> frame;
  frame.reserve(22); // 指令帧总长度22字节

  // 帧头
  frame.push_back(FRAME_HEAD_CMD[0]);
  frame.push_back(FRAME_HEAD_CMD[1]);

  // 数据长度
  frame.push_back(DATA_LEN_CMD);

  // 线速度x（float→小端序字节）
  uint8_t linear_x_bytes[4];
  floatToBytes(linear_x, linear_x_bytes);
  frame.insert(frame.end(), linear_x_bytes, linear_x_bytes + 4);

  // 线速度y
  uint8_t linear_y_bytes[4];
  floatToBytes(linear_y, linear_y_bytes);
  frame.insert(frame.end(), linear_y_bytes, linear_y_bytes + 4);

  // 角速度z
  uint8_t angular_z_bytes[4];
  floatToBytes(angular_z, angular_z_bytes);
  frame.insert(frame.end(), angular_z_bytes, angular_z_bytes + 4);

  // 刀盘速度
  uint8_t cutter_speed_bytes[4];
  floatToBytes(cutter_speed, cutter_speed_bytes);
  frame.insert(frame.end(), cutter_speed_bytes, cutter_speed_bytes + 4);

  // 校验位（帧头后到刀盘速度的所有字节之和）
  uint8_t checksum = calculateChecksum(&frame[2], 17); // 数据长度+4*4=17字节
  frame.push_back(checksum);

  // 帧尾
  frame.push_back(FRAME_TAIL_CMD[0]);
  frame.push_back(FRAME_TAIL_CMD[1]);

  // 发送帧
  ssize_t len = serial_port_.writeData(frame);
  return len == frame.size();
}

bool MotorController::parseWheelSpeed(const uint8_t* data, size_t len, float& left_speed, float& right_speed) {
  if (len < 14) return false;

  // 验证帧头/帧尾
  if (data[0] != FRAME_HEAD_RESP[0] || data[1] != FRAME_HEAD_RESP[1] ||
      data[12] != FRAME_TAIL_RESP[0] || data[13] != FRAME_TAIL_RESP[1]) {
    return false;
  }

  // 验证数据长度
  if (data[2] != DATA_LEN_RESP) return false;

  // 验证校验位
  uint8_t checksum = calculateChecksum(&data[2], 9);
  if (data[2 + DATA_LEN_RESP + 1] != checksum) return false;

  // 解析轮速
  left_speed = bytesToFloat(&data[3]);
  right_speed = bytesToFloat(&data[7]);

  return true;
}

uint8_t MotorController::calculateChecksum(const uint8_t* data, size_t len) {
  uint16_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum & 0xFF; // 取低8位
}

void MotorController::floatToBytes(float val, uint8_t* bytes) {
  memcpy(bytes, &val, 4);
  // 小端序（x86/RK3588默认小端，如需大端需字节交换）
}

float MotorController::bytesToFloat(const uint8_t* bytes) {
  float val;
  memcpy(&val, bytes, 4);
  return val;
}

} // namespace motor_control