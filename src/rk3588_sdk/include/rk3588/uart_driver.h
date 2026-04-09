#ifndef RK3588_SDK_UART_DRIVER_H
#define RK3588_SDK_UART_DRIVER_H

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>

namespace rk3588_sdk {

class UARTDriver {
public:
    UARTDriver(const std::string& dev_path, int baudrate);
    ~UARTDriver();

    bool openPort();
    void closePort();
    int sendData(const uint8_t* data, int len);
    int readData(uint8_t* buffer, int max_len, int timeout_ms = 100);

private:
    std::string dev_path_;
    int baudrate_;
    int fd_;
    struct termios old_tio_;
};

} // namespace rk3588_sdk

#endif // RK3588_SDK_UART_DRIVER_H