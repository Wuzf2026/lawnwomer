#ifndef MOTOR_CONTROL_PID_CONTROLLER_H
#define MOTOR_CONTROL_PID_CONTROLLER_H

namespace motor_control {

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float max_out, float min_out);
    float compute(float setpoint, float feedback, float dt);
    void reset();

private:
    float kp_;
    float ki_;
    float kd_;
    float max_out_;
    float min_out_;
    float integral_;
    float last_error_;
};

} // namespace motor_control

#endif // MOTOR_CONTROL_PID_CONTROLLER_H