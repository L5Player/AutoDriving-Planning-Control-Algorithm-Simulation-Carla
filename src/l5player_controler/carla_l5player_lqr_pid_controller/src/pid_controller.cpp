#include "carla_l5player_lqr_pid_controller/pid_controller.h"

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

double PIDController::Control(const double error, const double dt) {
    if (dt <= 0) {
        return previous_output_;
    }
    double diff = 0;
    double output = 0;

    if (first_hit_)    // first_hit_: 用来选择是否计算diff
    {
        first_hit_ = false;
    } else {
        diff = (error - previous_error_) / dt;
    }

    integral_ += ki_ * error * dt;    // 积分环节

    output = kp_ * error + integral_ + diff * kd_;
    previous_output_ = output;
    previous_error_ = error;
    return output;
}

void PIDController::Reset() {
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    integral_ = 0.0;
    first_hit_ = true;
}

}    // namespace control
}    // namespace l5player