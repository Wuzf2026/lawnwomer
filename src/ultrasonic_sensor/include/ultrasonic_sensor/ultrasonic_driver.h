#ifndef ULTRASONIC_SENSOR_ULTRASONIC_DRIVER_H
#define ULTRASONIC_SENSOR_ULTRASONIC_DRIVER_H

#include "rk3588_sdk/gpio_control.h"
#include <chrono>

namespace ultrasonic_sensor {

class UltrasonicDriver {
public:
    UltrasonicDriver(int trig_pin, int echo_pin);
    ~UltrasonicDriver() = default;

    bool init();
    float getDistance(); // 返回距离（单位：米）

private:
    rk3588_sdk::GPIOControl trig_gpio_;
    rk3588_sdk::GPIOControl echo_gpio_;
    int trig_pin_;
    int echo_pin_;
};

} // namespace ultrasonic_sensor

#endif // ULTRASONIC_SENSOR_ULTRASONIC_DRIVER_H