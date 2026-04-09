#include "rk3588/serial_port.h"
#include <serial/serial.h>
namespace rk3588 {
class SerialPort::impl {
public:
    serial::Serial serial_port;
};
SerialPort::SerialPort() : impl_(new impl) {}
SerialPort::~SerialPort() {
    delete impl_;
}
bool SerialPort::open(const std::string& port, int baudrate) {
    try {
        impl_->serial_port.setPort(port);
        impl_->serial_port.setBaudrate(baudrate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100);  // 100ms超时
        impl_->serial_port.setTimeout(timeout);
        impl_->serial_port.open();
        return true;
    } catch (const serial::IOException& e) {
        return false;
    }
}
void SerialPort::close() {
    if (is_open()) {
        impl_->serial_port.close();
    }
}
bool SerialPort::is_open() const {
    return impl_->serial_port.isOpen();
}
int SerialPort::write(const std::vector<uint8_t>& data) {
    if (!is_open()) {
        return -1;
    }
    return impl_->serial_port.write(data);
}
int SerialPort::read(std::vector<uint8_t>& data, int max_size) {
    if (!is_open()) {
        return -1;
    }
    return impl_->serial_port.read(data, max_size);
}
} // namespace rk3588