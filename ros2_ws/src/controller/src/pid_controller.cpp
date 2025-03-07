#include "pid_controller.hpp"

double PIDController::compute(double error)
{
    integral_ += error;
    double derivative = error - prev_error_;  
    prev_error_ = error;

    return p_gain_ * error + i_gain_ * integral_ + d_gain_ * derivative;
}
