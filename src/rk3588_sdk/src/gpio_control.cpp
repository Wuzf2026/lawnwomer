#include "rk3588_sdk/gpio_control.h"

namespace rk3588_sdk {

GPIOControl::GPIOControl(int gpio_pin) : gpio_pin_(gpio_pin), is_exported_(false) {
    gpio_path_ = "/sys/class/gpio/gpio" + std::to_string(gpio_pin);
}

GPIOControl::~GPIOControl() {
    if (is_exported_) {
        unexportGPIO();
    }
}

bool GPIOControl::exportGPIO() {
    std::ofstream export_file("/sys/class/gpio/export");
    if (!export_file.is_open()) {
        std::cerr << "Failed to export GPIO" << gpio_pin_ << std::endl;
        return false;
    }
    export_file << gpio_pin_;
    export_file.close();
    usleep(10000); // 等待GPIO节点创建
    is_exported_ = true;
    return true;
}

bool GPIOControl::unexportGPIO() {
    std::ofstream unexport_file("/sys/class/gpio/unexport");
    if (!unexport_file.is_open()) {
        std::cerr << "Failed to unexport GPIO" << gpio_pin_ << std::endl;
        return false;
    }
    unexport_file << gpio_pin_;
    unexport_file.close();
    is_exported_ = false;
    return true;
}

bool GPIOControl::setDirection(const std::string& direction) {
    if (!is_exported_) return false;
    std::ofstream dir_file(gpio_path_ + "/direction");
    if (!dir_file.is_open()) {
        std::cerr << "Failed to set direction for GPIO" << gpio_pin_ << std::endl;
        return false;
    }
    dir_file << direction;
    dir_file.close();
    return true;
}

bool GPIOControl::setValue(int value) {
    if (!is_exported_) return false;
    std::ofstream value_file(gpio_path_ + "/value");
    if (!value_file.is_open()) {
        std::cerr << "Failed to set value for GPIO" << gpio_pin_ << std::endl;
        return false;
    }
    value_file << value;
    value_file.close();
    return true;
}

int GPIOControl::getValue() {
    if (!is_exported_) return -1;
    std::ifstream value_file(gpio_path_ + "/value");
    if (!value_file.is_open()) {
        std::cerr << "Failed to get value for GPIO" << gpio_pin_ << std::endl;
        return -1;
    }
    int value;
    value_file >> value;
    value_file.close();
    return value;
}

} // namespace rk3588_sdk