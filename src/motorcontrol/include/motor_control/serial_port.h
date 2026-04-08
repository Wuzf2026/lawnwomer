#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <serial/serial.h>
#include <string>
#include <vector>
#include <ros/ros.h>

class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    bool init(const std::string& port, int baudrate);
    bool send(const uint8_t* data, size_t len);
    int receive(uint8_t* buf, size_t max_len);
    bool isOpen();
    void close();

private:
    serial::Serial ser_;
};

#endif