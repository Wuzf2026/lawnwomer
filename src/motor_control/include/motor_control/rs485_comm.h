#ifndef RS485_COMM_H
#define RS485_COMM_H

#include <ros/ros.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <cstdint>
#include <string>

namespace motor_control {

class RS485Comm {
public:
    // 构造/析构
    RS485Comm();
    ~RS485Comm();

    // 初始化串口
    bool init(const std::string& port, int baudrate, int databits = 8, int stopbits = 1, char parity = 'N');
    
    // 关闭串口
    void closePort();

    // 发送控制帧（线速度、角速度、刀盘速度）
    bool sendControlFrame(float linear_x, float angular_z, uint16_t cutter_speed);

    // 接收轮速回采帧（返回左/右轮速）
    bool recvWheelSpeedFrame(float& left_speed, float& right_speed);

    // 检查串口是否打开
    bool isOpen() const { return fd_ >= 0; }

private:
    // 串口文件描述符
    int fd_;

    // 打包控制帧
    std::vector<uint8_t> packControlFrame(float linear_x, float angular_z, uint16_t cutter_speed);
    
    // 解包轮速帧
    bool unpackWheelSpeedFrame(const std::vector<uint8_t>& frame, float& left_speed, float& right_speed);
    
    // 计算校验和
    uint8_t calcChecksum(const std::vector<uint8_t>& data);
    
    // 字节序转换（主机→大端）
    uint16_t hostToBigEndian(int16_t val);
    uint16_t hostToBigEndian(uint16_t val);
    
    // 字节序转换（大端→主机）
    int16_t bigEndianToHost(const uint8_t* bytes);
};

} // namespace motor_control

#endif // RS485_COMM_H