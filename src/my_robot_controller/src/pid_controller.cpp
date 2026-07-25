#include "my_robot_controller/pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace my_robot_controller
{

PIDController::PIDController(double kp, double ki, double kd, double max_i)
{
  configure(kp, ki, kd, max_i);
}

void PIDController::configure(double kp, double ki, double kd, double max_i)
{
  if (!std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd) ||
    !std::isfinite(max_i))
  {
    throw std::invalid_argument("PID gains and integral limit must be finite");
  }

  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  // Treat a negative limit as its magnitude so clamp bounds remain ordered.
  max_i_ = std::abs(max_i);
  reset();
}

double PIDController::calculate(double error, double dt)
{
  // Invalid data must never be propagated into a velocity command.
  if (!std::isfinite(error) || !std::isfinite(dt) || dt <= 0.0) {
    return 0.0;
  }

  // Clamp the stored state rather than only the final contribution. This
  // prevents a controller from retaining an arbitrarily large hidden integral
  // while its actuator or heading-correction output is saturated.
  integral_ = std::clamp(integral_ + error * dt, -max_i_, max_i_);

  // The first update after reset has no valid previous error. A zero derivative
  // avoids a large artificial kick when a run starts or odometry recovers.
  double derivative = 0.0;
  if (has_previous_error_) {
    derivative = (error - previous_error_) / dt;
  }

  previous_error_ = error;
  has_previous_error_ = true;
  return kp_ * error + ki_ * integral_ + kd_ * derivative;
}

void PIDController::reset()
{
  integral_ = 0.0;
  previous_error_ = 0.0;
  has_previous_error_ = false;
}

}  // namespace my_robot_controller
