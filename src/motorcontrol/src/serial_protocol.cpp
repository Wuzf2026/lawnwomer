#include "motor_control/serial_protocol.h"
#include <cstring>

void SerialProtocol::f2b(float f, uint8_t* bytes) { memcpy(bytes, &f, 4); }
float SerialProtocol::b2f(const uint8_t* bytes) { float f; memcpy(&f, bytes, 4); return f; }

uint8_t SerialProtocol::xorCheck(const uint8_t* data, int len) {
    uint8_t c = 0;
    for (int i=0; i<len; i++) c ^= data[i];
    return c;
}

std::vector<uint8_t> SerialProtocol::pack(float vx, float vz, float cutter) {
    std::vector<uint8_t> buf(FRAME_LEN, 0);
    buf[0] = HEAD1;
    buf[1] = HEAD2;
    buf[2] = CMD_MOTOR;

    float L = (vx - vz*0.25) / 0.15;
    float R = (vx + vz*0.25) / 0.15;

    f2b(L, buf.data()+3);
    f2b(R, buf.data()+7);
    f2b(cutter, buf.data()+11);
    buf[15] = xorCheck(buf.data(),15);
    buf[16] = TAIL1;
    buf[17] = TAIL2;
    return buf;
}

bool SerialProtocol::unpack(uint8_t* buf, float& left, float& right, float& cutter) {
    if (buf[0]!=HEAD1 || buf[1]!=HEAD2 || buf[16]!=TAIL1 || buf[17]!=TAIL2) return false;
    if (xorCheck(buf,15)!=buf[15]) return false;
    left = b2f(buf+3);
    right = b2f(buf+7);
    cutter = b2f(buf+11);
    return true;
}