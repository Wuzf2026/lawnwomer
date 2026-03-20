#include "ultrasonic_sensor/ultrasonic_driver.h"
#include <cmath>

namespace ultrasonic_sensor {

UltrasonicDriver::UltrasonicDriver(int trig_pin, int echo_pin)
    : trig_gpio_(trig_pin), echo_gpio_(echo_pin),
      trig_pin_(trig_pin), echo_pin_(echo_pin) {}

bool UltrasonicDriver::init() {
    // 初始化Trig（输出）和Echo（输入）
    if (!trig_gpio_.exportGPIO() || !echo_gpio_.exportGPIO()) {
        return false;
    }
    trig_gpio_.setDirection("out");
    echo_gpio_.setDirection("in");
    
    // 初始化为低电平
    trig_gpio_.setValue(0);
    usleep(2000); // 稳定
    return true;
}

float UltrasonicDriver::getDistance() {
    // 发送触发信号（10us高电平）
    trig_gpio_.setValue(1);
    usleep(10);
    trig_gpio_.setValue(0);

    // 等待Echo上升沿
    auto start_time = std::chrono::high_resolution_clock::now();
    auto timeout = start_time + std::chrono::milliseconds(100); // 超时100ms
    while (echo_gpio_.getValue() == 0) {
        if (std::chrono::high_resolution_clock::now() > timeout) {
            return NAN; // 超时返回无效值
        }
    }
    start_time = std::chrono::high_resolution_clock::now();

    // 等待Echo下降沿
    while (echo_gpio_.getValue() == 1) {
        if (std::chrono::high_resolution_clock::now() > timeout) {
            return NAN;
        }
    }
    auto end_time = std::chrono::high_resolution_clock::now();

    // 计算时间差（秒）
    std::chrono::duration<float> duration = end_time - start_time;
    float time_us = duration.count() * 1e6;

    // 距离 = 声速(343m/s) * 时间 / 2
    float distance = (343.0f * time_us) / (2.0f * 1e6);
    return distance;
}

} // namespace ultrasonic_sensor