#include "motor_control/motor_driver.h"
#include <cstring>

namespace motor_control {

MotorDriver::MotorDriver(const std::string& uart_dev, int baudrate, float kp, float ki, float kd)
    : uart_(uart_dev, baudrate), pid_(kp, ki, kd, 100.0f, -100.0f),
      target_rpm_(0.0f), current_rpm_(0.0f) {}

bool MotorDriver::init() {
    return uart_.openPort();
}

void MotorDriver::setSpeed(float target_rpm) {
    target_rpm_ = target_rpm;
}

float MotorDriver::getCurrentSpeed() {
    return current_rpm_;
}

void MotorDriver::update(float dt) {
    // 读取电机当前转速
    readMotorStatus();
    
    // PID计算控制量
    float control = pid_.compute(target_rpm_, current_rpm_, dt);
    
    // 发送控制指令
    sendMotorCmd(control);
}

bool MotorDriver::sendMotorCmd(float rpm) {
    // 示例协议：帧头(0xAA) + 指令(0x01) + 转速(4字节float) + 校验和(1字节) + 帧尾(0x55)
    uint8_t buf[8];
    buf[0] = 0xAA;
    buf[1] = 0x01;
    memcpy(&buf[2], &rpm, sizeof(float));
    buf[6] = buf[0] ^ buf[1] ^ buf[2] ^ buf[3] ^ buf[4] ^ buf[5];
    buf[7] = 0x55;
    
    return uart_.sendData(buf, 8) == 8;
}

bool MotorDriver::readMotorStatus() {
    uint8_t buf[8];
    int len = uart_.readData(buf, 8);
    if (len != 8 || buf[0] != 0xBB || buf[7] != 0x66) {
        return false;
    }
    
    // 校验和
    uint8_t checksum = buf[0] ^ buf[1] ^ buf[2] ^ buf[3] ^ buf[4] ^ buf[5];
    if (checksum != buf[6]) {
        return false;
    }
    
    // 解析转速
    memcpy(&current_rpm_, &buf[2], sizeof(float));
    return true;
}

} // namespace motor_control