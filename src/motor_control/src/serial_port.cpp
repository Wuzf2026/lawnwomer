#include "motor_control/serial_port.h"

SerialPort::SerialPort() {}
SerialPort::~SerialPort() { close(); }

bool SerialPort::init(const std::string& port, int baudrate) {
    try {
        ser_.setPort(port);
        ser_.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        ser_.setTimeout(to);
        ser_.open();
        return ser_.isOpen();
    } catch (...) {
        ROS_ERROR("Serial open failed");
        return false;
    }
}

bool SerialPort::send(const uint8_t* data, size_t len) {
    return ser_.write(data, len) == len;
}

int SerialPort::receive(uint8_t* buf, size_t max_len) {
    return ser_.read(buf, max_len);
}

bool SerialPort::isOpen() { return ser_.isOpen(); }
void SerialPort::close() { if (ser_.isOpen()) ser_.close(); }