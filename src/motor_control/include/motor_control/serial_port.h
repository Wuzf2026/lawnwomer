#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <ros/ros.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <stdint.h>
#include <errno.h>
#include <sys/ioctl.h>

namespace motor_control {

class SerialPort {
public:
  SerialPort();
  ~SerialPort();

  /**
   * @brief 打开串口
   * @param port 串口路径（如/dev/ttyS2）
   * @param baudrate 波特率
   * @param parity 校验位（none/odd/even）
   * @param stopbits 停止位
   * @param bytesize 数据位
   * @param timeout 超时时间（ms）
   * @return 成功返回true，失败返回false
   */
  bool openPort(const std::string& port, int baudrate, 
                const std::string& parity, int stopbits, int bytesize, int timeout);

  /**
   * @brief 关闭串口
   */
  void closePort();

  /**
   * @brief 写入数据到串口
   * @param data 待写入的字节数组
   * @return 成功返回写入字节数，失败返回-1
   */
  ssize_t writeData(const std::vector<uint8_t>& data);

  /**
   * @brief 从串口读取数据
   * @param buffer 接收缓冲区
   * @param max_len 最大读取长度
   * @return 成功返回读取字节数，失败返回-1
   */
  ssize_t readData(uint8_t* buffer, size_t max_len);

  /**
   * @brief 检查串口是否打开
   * @return 打开返回true，否则false
   */
  bool isOpen() const { return fd_ >= 0; }

private:
  int fd_;                  // 串口文件描述符
  struct termios old_tio_;  // 保存原有串口配置
  int timeout_;             // 超时时间（ms）

  /**
   * @brief 转换波特率为termios格式
   */
  speed_t convertBaudrate(int baudrate);
};

} // namespace motor_control

#endif // SERIAL_PORT_H