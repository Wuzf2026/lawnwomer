#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <geometry_msgs/Twist.h>
#include <vector>
#include <stdint.h>

#define FRAME_LEN      18
#define HEAD1          0xAA
#define HEAD2          0x55
#define CMD_MOTOR      0x01
#define TAIL1          0x55
#define TAIL2          0xAA

class SerialProtocol {
public:
    std::vector<uint8_t> pack(float vx, float vz, float cutter);
    bool unpack(uint8_t* buf, float& left, float& right, float& cutter);

private:
    uint8_t xorCheck(const uint8_t* data, int len);
    void f2b(float f, uint8_t* bytes);
    float b2f(const uint8_t* bytes);
};

#endif