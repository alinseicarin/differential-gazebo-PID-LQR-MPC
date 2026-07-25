#include "my_robot_controller/command_disturbance.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace my_robot_controller
{
namespace
{
// ROS timestamps are converted from integer nanoseconds to double seconds.
// This small tolerance makes a nominal 6.0 s boundary behave as 6.0 s even if
// floating-point subtraction represents it as 5.999999999999 s.
constexpr double kScheduleTimeTolerance = 1.0e-6;
}  // namespace

CommandDisturbance::CommandDisturbance(const CommandDisturbanceConfig & config)
{
  configure(config);
}

void CommandDisturbance::validate_config(const CommandDisturbanceConfig & config) const
{
  if (!std::isfinite(config.start_delay) || config.start_delay < 0.0) {
    throw std::invalid_argument("Command-fault start_delay must be finite and non-negative");
  }
  if (!std::isfinite(config.duration) || config.duration <= 0.0) {
    throw std::invalid_argument("Command-fault duration must be finite and positive");
  }
  if (!std::isfinite(config.linear_velocity_bias) ||
    !std::isfinite(config.angular_velocity_bias))
  {
    throw std::invalid_argument("Command-fault velocity biases must be finite");
  }
  if (!std::isfinite(config.maximum_abs_linear_velocity) ||
    config.maximum_abs_linear_velocity <= 0.0 ||
    !std::isfinite(config.maximum_abs_angular_velocity) ||
    config.maximum_abs_angular_velocity <= 0.0)
  {
    throw std::invalid_argument("Command-fault safety limits must be finite and positive");
  }
}

void CommandDisturbance::configure(const CommandDisturbanceConfig & config)
{
  validate_config(config);
  config_ = config;
}

CommandDisturbanceOutput CommandDisturbance::apply(
  double nominal_linear_velocity,
  double nominal_angular_velocity,
  double elapsed_time) const
{
  if (!std::isfinite(nominal_linear_velocity) ||
    !std::isfinite(nominal_angular_velocity) ||
    !std::isfinite(elapsed_time) || elapsed_time < 0.0)
  {
    throw std::invalid_argument(
            "Nominal command and elapsed command-fault time must be finite and non-negative");
  }

  CommandDisturbanceOutput output;
  output.nominal_linear_velocity = nominal_linear_velocity;
  output.nominal_angular_velocity = nominal_angular_velocity;
  output.fault_active =
    elapsed_time + kScheduleTimeTolerance >= config_.start_delay &&
    elapsed_time + kScheduleTimeTolerance < config_.start_delay + config_.duration;

  const double linear_bias = output.fault_active ? config_.linear_velocity_bias : 0.0;
  const double angular_bias = output.fault_active ? config_.angular_velocity_bias : 0.0;
  output.applied_linear_velocity = std::clamp(
    nominal_linear_velocity + linear_bias,
    -config_.maximum_abs_linear_velocity,
    config_.maximum_abs_linear_velocity);
  output.applied_angular_velocity = std::clamp(
    nominal_angular_velocity + angular_bias,
    -config_.maximum_abs_angular_velocity,
    config_.maximum_abs_angular_velocity);
  return output;
}

const CommandDisturbanceConfig & CommandDisturbance::config() const
{
  return config_;
}

}  // namespace my_robot_controller
