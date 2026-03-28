#ifndef RK3588_SERIAL_H
#define RK3588_SERIAL_H

#include <serial/serial.h>
#include <string>
#include <vector>
#include <mutex>

namespace rk3588_serial {

class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    // 初始化串口（RK3588串口：/dev/ttyS2 波特率115200）
    bool init(const std::string& port = "/dev/ttyS2", int baudrate = 115200);

    // 发送数据
    bool sendData(const std::vector<uint8_t>& data);

    // 接收数据（阻塞，超时ms）
    bool recvData(std::vector<uint8_t>& data, size_t len, int timeout_ms = 100);

    // 关闭串口
    void closePort();

private:
    serial::Serial ser_;
    std::mutex mutex_;  // 线程安全锁
    bool is_init_;
};

} // namespace rk3588_serial

#endif // RK3588_SERIAL_H