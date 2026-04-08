#include "rk3588_serial.h"
#include <ros/ros.h>

namespace rk3588_serial {

SerialPort::SerialPort() : is_init_(false) {}

SerialPort::~SerialPort() {
    closePort();
}

bool SerialPort::init(const std::string& port, int baudrate) {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
        ser_.setPort(port);
        ser_.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        ser_.setTimeout(to);
        ser_.open();
        is_init_ = ser_.isOpen();
        if (is_init_) {
            ROS_INFO("RK3588 Serial init success: %s (baudrate: %d)", port.c_str(), baudrate);
        } else {
            ROS_ERROR("RK3588 Serial init failed!");
        }
    } catch (serial::IOException& e) {
        ROS_ERROR("Serial exception: %s", e.what());
        is_init_ = false;
    }
    return is_init_;
}

bool SerialPort::sendData(const std::vector<uint8_t>& data) {
    if (!is_init_) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    try {
        size_t sent = ser_.write(data);
        return sent == data.size();
    } catch (serial::IOException& e) {
        ROS_ERROR("Send data failed: %s", e.what());
        return false;
    }
}

bool SerialPort::recvData(std::vector<uint8_t>& data, size_t len, int timeout_ms) {
    if (!is_init_) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    data.clear();
    ser_.setTimeout(serial::Timeout::simpleTimeout(timeout_ms));
    try {
        data = ser_.read(len);
        return data.size() == len;
    } catch (serial::IOException& e) {
        ROS_ERROR("Recv data failed: %s", e.what());
        return false;
    }
}

void SerialPort::closePort() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ser_.isOpen()) {
        ser_.close();
    }
    is_init_ = false;
}

} // namespace rk3588_serial