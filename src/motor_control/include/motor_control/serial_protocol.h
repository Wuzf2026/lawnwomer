#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <vector>
#include <stdint.h>
#include <geometry_msgs/Twist.h>

namespace motor_control {

// 上位机→下位机 控制帧结构
struct ControlFrame {
    uint8_t header = 0xAA;
    uint8_t dev_addr = 0x01;
    uint8_t cmd_type = 0x01;
    int16_t linear_vel;  // cm/s (-500~500)
    int16_t angular_vel; // °/s (-180~180)
    uint16_t cutter_vel; // rpm (0~300)
    uint8_t checksum;
    uint8_t tail = 0x55;
};

// 下位机→上位机 轮速回采帧结构
struct WheelSpeedFrame {
    uint8_t header = 0xBB;
    uint8_t dev_addr = 0x01;
    uint8_t data_len = 0x04;
    int16_t left_wheel_vel;  // cm/s
    int16_t right_wheel_vel; // cm/s
    uint8_t checksum;
    uint8_t tail = 0x66;
};

// 打包控制帧（Twist→协议帧）
std::vector<uint8_t> packControlFrame(const geometry_msgs::Twist::ConstPtr& twist, uint16_t cutter_vel = 0);

// 解析轮速回采帧
bool unpackWheelSpeedFrame(const std::vector<uint8_t>& data, WheelSpeedFrame& frame);

// 计算异或校验和
uint8_t calcChecksum(const uint8_t* data, size_t len);

// 16位有符号数转大端字节
std::vector<uint8_t> int16ToBigEndian(int16_t val);

// 16位无符号数转大端字节
std::vector<uint8_t> uint16ToBigEndian(uint16_t val);

// 大端字节转16位有符号数
int16_t bigEndianToInt16(const uint8_t* data);

} // namespace motor_control

#endif // SERIAL_PROTOCOL_H