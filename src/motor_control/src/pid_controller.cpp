#include "motor_control/pid_controller.h"

namespace motor_control {

PIDController::PIDController(float kp, float ki, float kd, float max_out, float min_out)
    : kp_(kp), ki_(ki), kd_(kd), max_out_(max_out), min_out_(min_out),
      integral_(0.0f), last_error_(0.0f) {}

float PIDController::compute(float setpoint, float feedback, float dt) {
    float error = setpoint - feedback;
    
    // 比例项
    float p_term = kp_ * error;
    
    // 积分项（限幅）
    integral_ += error * dt;
    integral_ = (integral_ > max_out_) ? max_out_ : (integral_ < min_out_) ? min_out_ : integral_;
    float i_term = ki_ * integral_;
    
    // 微分项
    float d_term = kd_ * (error - last_error_) / dt;
    last_error_ = error;
    
    // 输出限幅
    float output = p_term + i_term + d_term;
    output = (output > max_out_) ? max_out_ : (output < min_out_) ? min_out_ : output;
    
    return output;
}

void PIDController::reset() {
    integral_ = 0.0f;
    last_error_ = 0.0f;
}

} // namespace motor_control