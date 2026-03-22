#include "motor_control/serial_port.h"
#include <ros/ros.h>

namespace motor_control {

SerialPort::SerialPort() : fd_(-1), timeout_(100) {}

SerialPort::~SerialPort() {
  closePort();
}

bool SerialPort::openPort(const std::string& port, int baudrate, 
                          const std::string& parity, int stopbits, int bytesize, int timeout) {
  // 关闭已有串口
  if (fd_ >= 0) closePort();

  // 打开串口（O_RDWR：读写，O_NOCTTY：不成为控制终端，O_NDELAY：非阻塞）
  fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    ROS_ERROR("Failed to open serial port %s: %s", port.c_str(), strerror(errno));
    return false;
  }

  // 保存原有配置
  tcgetattr(fd_, &old_tio_);

  // 配置串口
  struct termios new_tio;
  bzero(&new_tio, sizeof(new_tio));

  // 输入输出波特率
  speed_t baud = convertBaudrate(baudrate);
  cfsetispeed(&new_tio, baud);
  cfsetospeed(&new_tio, baud);

  // 启用接收器，设置本地模式
  new_tio.c_cflag |= CREAD | CLOCAL;

  // 数据位
  new_tio.c_cflag &= ~CSIZE;
  switch (bytesize) {
    case 5: new_tio.c_cflag |= CS5; break;
    case 6: new_tio.c_cflag |= CS6; break;
    case 7: new_tio.c_cflag |= CS7; break;
    case 8: new_tio.c_cflag |= CS8; break;
    default: 
      ROS_ERROR("Invalid bytesize: %d", bytesize);
      closePort();
      return false;
  }

  // 校验位
  if (parity == "none") {
    new_tio.c_cflag &= ~PARENB;
  } else if (parity == "odd") {
    new_tio.c_cflag |= PARENB | PARODD;
  } else if (parity == "even") {
    new_tio.c_cflag |= PARENB;
    new_tio.c_cflag &= ~PARODD;
  } else {
    ROS_ERROR("Invalid parity: %s", parity.c_str());
    closePort();
    return false;
  }

  // 停止位
  switch (stopbits) {
    case 1: new_tio.c_cflag &= ~CSTOPB; break;
    case 2: new_tio.c_cflag |= CSTOPB; break;
    default:
      ROS_ERROR("Invalid stopbits: %d", stopbits);
      closePort();
      return false;
  }

  // 禁用流控
  new_tio.c_cflag &= ~CRTSCTS;

  // 原始模式（无回车换行转换）
  new_tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  new_tio.c_oflag &= ~OPOST;

  // 超时设置
  new_tio.c_cc[VTIME] = timeout / 100;  // 单位：0.1秒
  new_tio.c_cc[VMIN] = 0;               // 最小读取字节数

  // 应用配置
  tcflush(fd_, TCIFLUSH);
  if (tcsetattr(fd_, TCSANOW, &new_tio) != 0) {
    ROS_ERROR("Failed to set serial attributes: %s", strerror(errno));
    closePort();
    return false;
  }

  timeout_ = timeout;
  ROS_INFO("Serial port %s opened successfully (baudrate: %d)", port.c_str(), baudrate);
  return true;
}

void SerialPort::closePort() {
  if (fd_ >= 0) {
    // 恢复原有配置
    tcsetattr(fd_, TCSANOW, &old_tio_);
    close(fd_);
    fd_ = -1;
    ROS_INFO("Serial port closed");
  }
}

ssize_t SerialPort::writeData(const std::vector<uint8_t>& data) {
  if (!isOpen()) {
    ROS_ERROR("Serial port not open");
    return -1;
  }

  ssize_t len = write(fd_, data.data(), data.size());
  if (len < 0) {
    ROS_ERROR("Failed to write to serial port: %s", strerror(errno));
    return -1;
  }

  // 确保数据发送完成
  tcdrain(fd_);
  return len;
}

ssize_t SerialPort::readData(uint8_t* buffer, size_t max_len) {
  if (!isOpen()) {
    ROS_ERROR("Serial port not open");
    return -1;
  }

  // 设置非阻塞读取，带超时
  fd_set read_fds;
  struct timeval tv;
  FD_ZERO(&read_fds);
  FD_SET(fd_, &read_fds);

  tv.tv_sec = timeout_ / 1000;
  tv.tv_usec = (timeout_ % 1000) * 1000;

  int ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
  if (ret < 0) {
    ROS_ERROR("Select error: %s", strerror(errno));
    return -1;
  } else if (ret == 0) {
    // 超时
    return 0;
  }

  // 有数据可读
  ssize_t len = read(fd_, buffer, max_len);
  if (len < 0) {
    ROS_ERROR("Failed to read from serial port: %s", strerror(errno));
    return -1;
  }

  return len;
}

speed_t SerialPort::convertBaudrate(int baudrate) {
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: 
      ROS_WARN("Unsupported baudrate %d, use 115200", baudrate);
      return B115200;
  }
}

} // namespace motor_control