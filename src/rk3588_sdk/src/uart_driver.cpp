#include "rk3588/uart_driver.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
namespace rk3588_sdk {
UARTDriver::UARTDriver(const std::string& dev_path, int baudrate)
    : dev_path_(dev_path), baudrate_(baudrate), fd_(-1) {
}
UARTDriver::~UARTDriver() {
    if (fd_ >= 0) {
        closePort();
    }
}
bool UARTDriver::openPort() {
    fd_ = open(dev_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "Failed to open UART port: " << dev_path_ << std::endl;
        return false;
    }
    
    struct termios tio;
    tcgetattr(fd_, &old_tio_);
    bzero(&tio, sizeof(tio));
    
    // 设置波特率
    speed_t baud;
    switch (baudrate_) {
        case 9600:   baud = B9600; break;
        case 19200:  baud = B19200; break;
        case 38400:  baud = B38400; break;
        case 57600:  baud = B57600; break;
        case 115200: baud = B115200; break;
        default:     baud = B115200; break;
    }
    
    cfsetispeed(&tio, baud);
    cfsetospeed(&tio, baud);
    
    // 设置8N1
    tio.c_cflag |= CS8 | CLOCAL | CREAD;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;
    
    // 禁用规范模式，禁用回显
    tio.c_lflag &= ~ICANON;
    tio.c_lflag &= ~ECHO;
    tio.c_lflag &= ~ECHOE;
    tio.c_lflag &= ~ISIG;
    
    // 禁用软件流控
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_iflag &= ~(ICRNL | INLCR);
    tio.c_oflag &= ~OPOST;
    
    // 设置超时
    tio.c_cc[VTIME] = 1;  // 100ms超时
    tio.c_cc[VMIN] = 0;
    
    tcsetattr(fd_, TCSANOW, &tio);
    tcflush(fd_, TCIFLUSH);
    
    return true;
}
void UARTDriver::closePort() {
    if (fd_ >= 0) {
        tcsetattr(fd_, TCSANOW, &old_tio_);
        close(fd_);
        fd_ = -1;
    }
}
int UARTDriver::sendData(const uint8_t* data, int len) {
    if (fd_ < 0) return -1;
    return write(fd_, data, len);
}
int UARTDriver::readData(uint8_t* buffer, int max_len, int timeout_ms) {
    if (fd_ < 0) return -1;
    
    fd_set read_fds;
    struct timeval tv;
    
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);
    
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    
    int ret = select(fd_ + 1, &read_fds, NULL, NULL, &tv);
    if (ret <= 0) return ret;
    
    return read(fd_, buffer, max_len);
}
}  // namespace rk3588_sdk