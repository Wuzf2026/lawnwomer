#include "serial_protocol.h"
#include <cstring>

namespace motor_control {

uint8_t calcChecksum(const uint8_t* data, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; ++i) {
        checksum ^= data[i];
    }
    return checksum;
}

std::vector<uint8_t> int16ToBigEndian(int16_t val) {
    std::vector<uint8_t> bytes(2);
    bytes[0] = (val >> 8) & 0xFF;
    bytes[1] = val & 0xFF;
    return bytes;
}

std::vector<uint8_t> uint16ToBigEndian(uint16_t val) {
    std::vector<uint8_t> bytes(2);
    bytes[0] = (val >> 8) & 0xFF;
    bytes[1] = val & 0xFF;
    return bytes;
}

int16_t bigEndianToInt16(const uint8_t* data) {
    return (static_cast<int16_t>(data[0]) << 8) | data[1];
}

std::vector<uint8_t> packControlFrame(const geometry_msgs::Twist::ConstPtr& twist, uint16_t cutter_vel) {
    ControlFrame frame;
    
    // 转换Twist到协议速度（linear.x=线速度cm/s，angular.z=角速度°/s）
    frame.linear_vel = static_cast<int16_t>(twist->linear.x * 100); // m/s → cm/s
    frame.angular_vel = static_cast<int16_t>(twist->angular.z * 180 / M_PI); // rad/s → °/s
    
    // 限制范围
    frame.linear_vel = std::clamp(frame.linear_vel, (int16_t)-500, (int16_t)500);
    frame.angular_vel = std::clamp(frame.angular_vel, (int16_t)-180, (int16_t)180);
    frame.cutter_vel = std::clamp(cutter_vel, (uint16_t)0, (uint16_t)300);

    // 打包字节
    std::vector<uint8_t> data;
    data.push_back(frame.header);
    data.push_back(frame.dev_addr);
    data.push_back(frame.cmd_type);
    
    auto linear_bytes = int16ToBigEndian(frame.linear_vel);
    data.insert(data.end(), linear_bytes.begin(), linear_bytes.end());
    
    auto angular_bytes = int16ToBigEndian(frame.angular_vel);
    data.insert(data.end(), angular_bytes.begin(), angular_bytes.end());
    
    auto cutter_bytes = uint16ToBigEndian(frame.cutter_vel);
    data.insert(data.end(), cutter_bytes.begin(), cutter_bytes.end());

    // 计算校验和
    frame.checksum = calcChecksum(data.data(), data.size());
    data.push_back(frame.checksum);
    data.push_back(frame.tail);

    return data;
}

bool unpackWheelSpeedFrame(const std::vector<uint8_t>& data, WheelSpeedFrame& frame) {
    if (data.size() != 9) return false; // 帧长度固定9字节

    // 校验帧头/帧尾
    if (data[0] != 0xBB || data[8] != 0x66) return false;

    // 解析字段
    frame.dev_addr = data[1];
    frame.data_len = data[2];
    frame.left_wheel_vel = bigEndianToInt16(&data[3]);
    frame.right_wheel_vel = bigEndianToInt16(&data[5]);
    frame.checksum = data[7];

    // 校验和验证
    uint8_t calc_check = calcChecksum(data.data(), 7); // 帧头~右轮速共7字节
    if (calc_check != frame.checksum) return false;

    return true;
}

} // namespace motor_control