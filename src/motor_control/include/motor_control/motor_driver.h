#ifndef MOTOR_CONTROL_MOTOR_DRIVER_H
#define MOTOR_CONTROL_MOTOR_DRIVER_H

#include "rk3588_sdk/uart_driver.h"
#include "motor_control/pid_controller.h"

namespace motor_control {

class MotorDriver {
public:
    MotorDriver(const std::string& uart_dev, int baudrate, float kp, float ki, float kd);
    ~MotorDriver() = default;

    bool init();
    void setSpeed(float target_rpm);
    float getCurrentSpeed();
    void update(float dt);

private:
    rk3588_sdk::UARTDriver uart_;
    PIDController pid_;
    float target_rpm_;
    float current_rpm_;

    // 串口通信协议解析/封装
    bool sendMotorCmd(float rpm);
    bool readMotorStatus();
};

} // namespace motor_control

#endif // MOTOR_CONTROL_MOTOR_DRIVER_H