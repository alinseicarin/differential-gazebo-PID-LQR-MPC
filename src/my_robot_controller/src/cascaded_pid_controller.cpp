#include "my_robot_controller/cascaded_pid_controller.hpp"

#include "my_robot_controller/path_reference_manager.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace my_robot_controller
{

// This class contains only control mathematics. ROS messages, trajectory
// lookup, feedforward addition, and actuator saturation are handled elsewhere,
// which makes the three PID loops independently unit-testable.
CascadedPidController::CascadedPidController(const CascadedPidConfig & config)
{
  configure(config);
}

void CascadedPidController::validate_config(const CascadedPidConfig & config) const
{
  // Check all three sets of gains in one pass. Zero and negative gains remain
  // legal because experiments may intentionally disable or invert a term; the
  // essential numerical requirement here is that every value is finite.
  const bool gains_are_finite =
    std::isfinite(config.longitudinal_kp) && std::isfinite(config.longitudinal_ki) &&
    std::isfinite(config.longitudinal_kd) &&
    std::isfinite(config.longitudinal_integral_limit) &&
    std::isfinite(config.cross_track_kp) && std::isfinite(config.cross_track_ki) &&
    std::isfinite(config.cross_track_kd) &&
    std::isfinite(config.cross_track_integral_limit) &&
    std::isfinite(config.heading_kp) && std::isfinite(config.heading_ki) &&
    std::isfinite(config.heading_kd) && std::isfinite(config.heading_integral_limit);
  if (!gains_are_finite) {
    throw std::invalid_argument("Cascaded PID gains and integral limits must be finite");
  }

  if (!std::isfinite(config.maximum_heading_correction) ||
    config.maximum_heading_correction <= 0.0)
  {
    throw std::invalid_argument("maximum_heading_correction must be finite and positive");
  }
}

void CascadedPidController::configure(const CascadedPidConfig & config)
{
  // Each physical error channel owns separate PID memory. Sharing one PID
  // object would incorrectly mix integrals and previous errors with different
  // units (metres, radians, metres per second, and radians per second).
  validate_config(config);
  config_ = config;
  longitudinal_pid_.configure(
    config.longitudinal_kp, config.longitudinal_ki, config.longitudinal_kd,
    config.longitudinal_integral_limit);
  cross_track_pid_.configure(
    config.cross_track_kp, config.cross_track_ki, config.cross_track_kd,
    config.cross_track_integral_limit);
  heading_pid_.configure(
    config.heading_kp, config.heading_ki, config.heading_kd,
    config.heading_integral_limit);
}

CascadedPidOutput CascadedPidController::calculate(
  double longitudinal_error,
  double lateral_error,
  double reference_heading,
  double robot_heading,
  double dt)
{
  // The controller is deliberately strict: unlike the generic PID primitive,
  // this higher-level block reports invalid state/timing data to its caller.
  if (!std::isfinite(longitudinal_error) || !std::isfinite(lateral_error) ||
    !std::isfinite(reference_heading) ||
    !std::isfinite(robot_heading) || !std::isfinite(dt) || dt <= 0.0)
  {
    throw std::invalid_argument("Cascaded PID input and dt must be finite, with dt positive");
  }

  CascadedPidOutput output;

  // A positive body-frame longitudinal error means that the virtual reference
  // is ahead, so positive feedback asks the robot to accelerate and catch up.
  output.longitudinal_pid_output = longitudinal_pid_.calculate(longitudinal_error, dt);

  // Body-frame e_y is positive when the reference lies to the robot's left.
  // The desired heading correction therefore has the same sign as e_y.
  output.lateral_pid_output = cross_track_pid_.calculate(lateral_error, dt);
  output.heading_correction = std::clamp(
    output.lateral_pid_output,
    -config_.maximum_heading_correction,
    config_.maximum_heading_correction);
  output.desired_heading = wrap_angle(reference_heading + output.heading_correction);

  // This inner error includes both path-tangent alignment and the outer-loop
  // lateral correction. A sudden robot rotation changes it immediately even
  // if cross-track error has not had time to grow.
  output.heading_error = wrap_angle(output.desired_heading - robot_heading);
  output.heading_pid_output = heading_pid_.calculate(output.heading_error, dt);

  // The returned diagnostics preserve every intermediate signal. The ROS node
  // can log them without recomputing the controller or exposing internal PID
  // state, while the shared motion policy later adds trajectory feedforward.
  return output;
}

void CascadedPidController::reset()
{
  // All loops must start from a clean state after settling, reset, or timeout.
  longitudinal_pid_.reset();
  cross_track_pid_.reset();
  heading_pid_.reset();
}

}  // namespace my_robot_controller
