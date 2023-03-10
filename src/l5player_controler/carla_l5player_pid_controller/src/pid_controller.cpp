#include "carla_l5player_pid_controller/pid_controller.h"

#include <assert.h>

#include <iostream>
namespace l5player {
namespace control {

PIDController::PIDController(const double kp, const double ki, const double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    integral_ = 0.0;
    first_hit_ = true;
}

// /**to-do**/ 实现PID控制
double PIDController::Control(const double error, const double dt) {
    assert(dt > 0 && "dt must be positive!!!");    // 需要添加一下头文件：#include <assert.h>

    if (first_hit_) {
        first_hit_ = false;
        proportional_part = error;
        integral_part += error * dt;
        derivative_part = 0;
    } else {
        proportional_part = error;
        integral_part += error * dt;
        derivative_part = (error - previous_error_) / dt;
    }

    // 积分限幅
    if (integral_part > 40.0) {
        integral_part = 40.0;
    }
    if (integral_part < -40.0) {
        integral_part = -40.0;
    }
    // std::cout << "integral_part: " << integral_part << std::endl;
    current_output = kp_ * proportional_part + ki_ * integral_part + kd_ * derivative_part;

    previous_error_ = error;
    previous_output_ = current_output;

    return current_output;
}

// /**to-do**/ 重置PID参数
void PIDController::Reset() {
    kp_ = 0.0;
    ki_ = 0.0;
    kd_ = 0.0;
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    integral_ = 0.0;
    first_hit_ = false;
}

}    // namespace control
}    // namespace l5player