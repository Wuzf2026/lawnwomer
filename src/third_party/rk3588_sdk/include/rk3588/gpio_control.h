#ifndef RK3588_SDK_GPIO_CONTROL_H
#define RK3588_SDK_GPIO_CONTROL_H

#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>

namespace rk3588_sdk {

class GPIOControl {
public:
    GPIOControl(int gpio_pin);
    ~GPIOControl();

    bool exportGPIO();
    bool unexportGPIO();
    bool setDirection(const std::string& direction); // "in" / "out"
    bool setValue(int value); // 0 / 1
    int getValue();

private:
    int gpio_pin_;
    std::string gpio_path_;
    bool is_exported_;
};

} // namespace rk3588_sdk

#endif // RK3588_SDK_GPIO_CONTROL_H