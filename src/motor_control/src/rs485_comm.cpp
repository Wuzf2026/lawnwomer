#include "motor_control/rs485_comm.h"
#include <cstring>
#include <iostream>

namespace motor_control {

RS485Comm::RS485Comm() : fd_(-1) {}

RS485Comm::~RS485Comm() {
    closePort();
}

bool RS485Comm::init(const std::string& port, int baudrate, int databits, int stopbits, char parity) {
    // 打开串口（非阻塞）
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        ROS_ERROR("Failed to open RS485 port: %s", port.c_str());
        return false;
    }

    // 配置串口
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd_, &tty) != 0) {
        ROS_ERROR("Failed to get RS485 port attributes");
        closePort();
        return false;
    }

    // 设置波特率
    speed_t baud;
    switch (baudrate) {
        case 9600: baud = B9600; break;
        case 19200: baud = B19200; break;
        case 38400: baud = B38400; break;
        case 115200: baud = B115200; break;
        default: ROS_ERROR("Unsupported baudrate: %d", baudrate); closePort(); return false;
    }
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    // 设置数据位
    tty.c_cflag &= ~CSIZE;
    switch (databits) {
        case 8: tty.c_cflag |= CS8; break;
        case 7: tty.c_cflag |= CS7; break;
        default: ROS_ERROR("Unsupported databits: %d", databits); closePort(); return false;
    }

    // 设置校验位
    switch (parity) {
        case 'N': case 'n': tty.c_cflag &= ~PARENB; break;
        case 'O': case 'o': tty.c_cflag |= (PARENB | PARODD); break;
        case 'E': case 'e': tty.c_cflag |= PARENB; tty.c_cflag &= ~PARODD; break;
        default: ROS_ERROR("Unsupported parity: %c", parity); closePort(); return false;
    }

    // 设置停止位
    switch (stopbits) {
        case 1: tty.c_cflag &= ~CSTOPB; break;
        case 2: tty.c_cflag |= CSTOPB; break;
        default: ROS_ERROR("Unsupported stopbits: %d", stopbits); closePort(); return false;
    }

    // 禁用流控
    tty.c_cflag &= ~CRTSCTS;
    // 启用接收器、本地模式
    tty.c_cflag |= CREAD | CLOCAL;
    // 禁用规范模式、回显、信号
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // 禁用软件流控
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    // 原始输出
    tty.c_oflag &= ~OPOST;

    // 设置超时（100ms）
    tty.c_cc[VTIME] = 1;  // 100ms
    tty.c_cc[VMIN] = 0;   // 非阻塞

    // 应用配置
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        ROS_ERROR("Failed to set RS485 port attributes");
        closePort();
        return false;
    }

    ROS_INFO("RS485 port %s initialized successfully (baudrate: %d)", port.c_str(), baudrate);
    return true;
}

void RS485Comm::closePort() {
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
        ROS_INFO("RS485 port closed");
    }
}

bool RS485Comm::sendControlFrame(float linear_x, float angular_z, uint16_t cutter_speed) {
    if (!isOpen()) {
        ROS_ERROR("RS485 port not open, cannot send control frame");
        return false;
    }

    // 打包帧
    std::vector<uint8_t> frame = packControlFrame(linear_x, angular_z, cutter_speed);
    
    // 发送帧
    ssize_t sent = write(fd_, frame.data(), frame.size());
    if (sent != frame.size()) {
        ROS_ERROR("Failed to send control frame: sent %zd bytes, expected %zu", sent, frame.size());
        return false;
    }

    ROS_DEBUG("Sent control frame: linear=%.2f cm/s, angular=%.2f °/s, cutter=%d rpm", 
              linear_x, angular_z, cutter_speed);
    return true;
}

bool RS485Comm::recvWheelSpeedFrame(float& left_speed, float& right_speed) {
    if (!isOpen()) {
        ROS_ERROR("RS485 port not open, cannot receive wheel speed frame");
        return false;
    }

    // 读取串口数据（最多10字节，对应回采帧长度）
    uint8_t buf[10] = {0};
    ssize_t recv_len = read(fd_, buf, sizeof(buf));
    if (recv_len <= 0) {
        return false; // 无数据
    }

    // 转换为vector
    std::vector<uint8_t> frame(buf, buf + recv_len);
    
    // 解包帧
    return unpackWheelSpeedFrame(frame, left_speed, right_speed);
}

std::vector<uint8_t> RS485Comm::packControlFrame(float linear_x, float angular_z, uint16_t cutter_speed) {
    std::vector<uint8_t> frame;
    
    // 帧头
    frame.push_back(0xAA);
    // 设备地址
    frame.push_back(0x01);
    // 指令类型
    frame.push_back(0x01);
    
    // 线速度（转换为16位有符号数，大端）
    int16_t linear_val = static_cast<int16_t>(linear_x);
    uint16_t linear_be = hostToBigEndian(linear_val);
    frame.push_back((linear_be >> 8) & 0xFF);
    frame.push_back(linear_be & 0xFF);
    
    // 角速度（转换为16位有符号数，大端）
    int16_t angular_val = static_cast<int16_t>(angular_z);
    uint16_t angular_be = hostToBigEndian(angular_val);
    frame.push_back((angular_be >> 8) & 0xFF);
    frame.push_back(angular_be & 0xFF);
    
    // 刀盘速度（转换为16位无符号数，大端）
    uint16_t cutter_be = hostToBigEndian(cutter_speed);
    frame.push_back((cutter_be >> 8) & 0xFF);
    frame.push_back(cutter_be & 0xFF);
    
    // 校验和
    frame.push_back(calcChecksum(frame));
    // 帧尾
    frame.push_back(0x55);

    return frame;
}

bool RS485Comm::unpackWheelSpeedFrame(const std::vector<uint8_t>& frame, float& left_speed, float& right_speed) {
    // 检查帧长度（至少9字节）
    if (frame.size() < 9) {
        ROS_WARN("Wheel speed frame too short: %zu bytes", frame.size());
        return false;
    }

    // 检查帧头/帧尾
    if (frame[0] != 0xBB || frame[8] != 0x66) {
        ROS_WARN("Invalid wheel speed frame header/tail: 0x%02X / 0x%02X", frame[0], frame[8]);
        return false;
    }

    // 检查设备地址
    if (frame[1] != 0x01) {
        ROS_WARN("Invalid wheel speed frame device address: 0x%02X", frame[1]);
        return false;
    }

    // 检查数据长度
    if (frame[2] != 0x04) {
        ROS_WARN("Invalid wheel speed frame data length: 0x%02X", frame[2]);
        return false;
    }

    // 校验和验证
    std::vector<uint8_t> data_to_check(frame.begin(), frame.begin() + 7); // 帧头~右轮速
    uint8_t checksum = calcChecksum(data_to_check);
    if (checksum != frame[7]) {
        ROS_WARN("Wheel speed frame checksum error: calc=0x%02X, recv=0x%02X", checksum, frame[7]);
        return false;
    }

    // 解析左轮速（大端→主机）
    uint8_t left_bytes[2] = {frame[3], frame[4]};
    int16_t left_val = bigEndianToHost(left_bytes);
    left_speed = static_cast<float>(left_val);

    // 解析右轮速（大端→主机）
    uint8_t right_bytes[2] = {frame[5], frame[6]};
    int16_t right_val = bigEndianToHost(right_bytes);
    right_speed = static_cast<float>(right_val);

    ROS_DEBUG("Received wheel speed: left=%.2f cm/s, right=%.2f cm/s", left_speed, right_speed);
    return true;
}

uint8_t RS485Comm::calcChecksum(const std::vector<uint8_t>& data) {
    uint8_t checksum = 0;
    for (uint8_t byte : data) {
        checksum ^= byte;
    }
    return checksum;
}

uint16_t RS485Comm::hostToBigEndian(int16_t val) {
    return htons(static_cast<uint16_t>(val));
}

uint16_t RS485Comm::hostToBigEndian(uint16_t val) {
    return htons(val);
}

int16_t RS485Comm::bigEndianToHost(const uint8_t* bytes) {
    uint16_t be_val = (static_cast<uint16_t>(bytes[0]) << 8) | bytes[1];
    return static_cast<int16_t>(ntohs(be_val));
}

} // namespace motor_control