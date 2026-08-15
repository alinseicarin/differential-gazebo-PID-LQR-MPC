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
  if (config.persistent && config.start_delays.size() > 1u) {
    throw std::invalid_argument(
            "A persistent command fault may have only one start time");
  }
  double previous_start = -config.duration;
  for (const double start : config.start_delays) {
    if (!std::isfinite(start) || start < 0.0) {
      throw std::invalid_argument(
              "Command-fault repeated start times must be finite and non-negative");
    }
    if (start + kScheduleTimeTolerance < previous_start + config.duration) {
      throw std::invalid_argument(
              "Command-fault windows must be ordered and non-overlapping");
    }
    previous_start = start;
  }
  if (!std::isfinite(config.linear_velocity_bias) ||
    !std::isfinite(config.angular_velocity_bias))
  {
    throw std::invalid_argument("Command-fault velocity biases must be finite");
  }
  if (!std::isfinite(config.left_wheel_effectiveness) ||
    !std::isfinite(config.right_wheel_effectiveness) ||
    config.left_wheel_effectiveness < 0.0 || config.left_wheel_effectiveness > 1.0 ||
    config.right_wheel_effectiveness < 0.0 || config.right_wheel_effectiveness > 1.0)
  {
    throw std::invalid_argument(
            "Wheel-effectiveness factors must be finite values in [0, 1]");
  }
  if (!std::isfinite(config.wheel_separation) || config.wheel_separation <= 0.0) {
    throw std::invalid_argument("Wheel separation must be finite and positive");
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
  output.active_window_index = active_window_index(elapsed_time);
  output.fault_active = output.active_window_index >= 0;

  // Convert the body command to equivalent wheel-ground velocities. Applying
  // effectiveness here emulates a weakened drive side while keeping the
  // downstream Gazebo diff-drive interface unchanged and controller agnostic.
  const double half_separation = 0.5 * config_.wheel_separation;
  output.nominal_left_wheel_velocity =
    nominal_linear_velocity - half_separation * nominal_angular_velocity;
  output.nominal_right_wheel_velocity =
    nominal_linear_velocity + half_separation * nominal_angular_velocity;
  output.effective_left_wheel_velocity = output.nominal_left_wheel_velocity;
  output.effective_right_wheel_velocity = output.nominal_right_wheel_velocity;
  const bool wheel_fault_active = output.fault_active &&
    (config_.left_wheel_effectiveness != 1.0 ||
    config_.right_wheel_effectiveness != 1.0);
  if (wheel_fault_active) {
    output.effective_left_wheel_velocity *= config_.left_wheel_effectiveness;
    output.effective_right_wheel_velocity *= config_.right_wheel_effectiveness;
  }

  // Preserve the controller command exactly when no wheel fault is active;
  // an unnecessary forward/inverse conversion would change its last floating
  // point bit and would make a nominal pass-through needlessly non-transparent.
  double effective_linear_velocity = nominal_linear_velocity;
  double effective_angular_velocity = nominal_angular_velocity;
  if (wheel_fault_active) {
    effective_linear_velocity = 0.5 *
      (output.effective_left_wheel_velocity + output.effective_right_wheel_velocity);
    effective_angular_velocity =
      (output.effective_right_wheel_velocity - output.effective_left_wheel_velocity) /
      config_.wheel_separation;
  }

  const double linear_bias = output.fault_active ? config_.linear_velocity_bias : 0.0;
  const double angular_bias = output.fault_active ? config_.angular_velocity_bias : 0.0;
  output.applied_linear_velocity = std::clamp(
    effective_linear_velocity + linear_bias,
    -config_.maximum_abs_linear_velocity,
    config_.maximum_abs_linear_velocity);
  output.applied_angular_velocity = std::clamp(
    effective_angular_velocity + angular_bias,
    -config_.maximum_abs_angular_velocity,
    config_.maximum_abs_angular_velocity);
  return output;
}

bool CommandDisturbance::is_active(double elapsed_time) const
{
  return active_window_index(elapsed_time) >= 0;
}

int CommandDisturbance::active_window_index(double elapsed_time) const
{
  if (!std::isfinite(elapsed_time) || elapsed_time < 0.0) {
    throw std::invalid_argument("Command-fault elapsed time must be finite and non-negative");
  }
  if (!config_.enabled) {
    return -1;
  }

  if (config_.start_delays.empty()) {
    if (elapsed_time + kScheduleTimeTolerance < config_.start_delay) {
      return -1;
    }
    if (config_.persistent ||
      elapsed_time + kScheduleTimeTolerance < config_.start_delay + config_.duration)
    {
      return 0;
    }
    return -1;
  }

  for (std::size_t index = 0u; index < config_.start_delays.size(); ++index) {
    const double start = config_.start_delays[index];
    if (elapsed_time + kScheduleTimeTolerance >= start &&
      (config_.persistent ||
      elapsed_time + kScheduleTimeTolerance < start + config_.duration))
    {
      return static_cast<int>(index);
    }
  }
  return -1;
}

const CommandDisturbanceConfig & CommandDisturbance::config() const
{
  return config_;
}

CommandDelay::CommandDelay(double delay)
{
  configure(delay);
}

void CommandDelay::configure(double delay)
{
  if (!std::isfinite(delay) || delay < 0.0) {
    throw std::invalid_argument("Command delay must be finite and non-negative");
  }
  delay_ = delay;
  reset();
}

void CommandDelay::reset()
{
  history_.clear();
}

DelayedCommandOutput CommandDelay::apply(
  double current_linear_velocity,
  double current_angular_velocity,
  double elapsed_time,
  bool fault_active)
{
  if (!std::isfinite(current_linear_velocity) ||
    !std::isfinite(current_angular_velocity) ||
    !std::isfinite(elapsed_time) || elapsed_time < 0.0)
  {
    throw std::invalid_argument(
            "Delayed command and elapsed time must be finite and non-negative");
  }

  if (!history_.empty() && elapsed_time + kScheduleTimeTolerance < history_.back().time) {
    reset();
  }
  history_.push_back({elapsed_time, current_linear_velocity, current_angular_velocity});

  DelayedCommandOutput output;
  output.current_linear_velocity = current_linear_velocity;
  output.current_angular_velocity = current_angular_velocity;
  output.source_linear_velocity = current_linear_velocity;
  output.source_angular_velocity = current_angular_velocity;
  output.source_time = elapsed_time;
  output.delay_active = fault_active && delay_ > 0.0;

  if (output.delay_active) {
    const double target_time = elapsed_time - delay_;
    bool source_found = false;
    for (auto iterator = history_.rbegin(); iterator != history_.rend(); ++iterator) {
      if (iterator->time <= target_time + kScheduleTimeTolerance) {
        output.source_linear_velocity = iterator->linear_velocity;
        output.source_angular_velocity = iterator->angular_velocity;
        output.source_time = iterator->time;
        source_found = true;
        break;
      }
    }
    // This can occur only if a delay is activated before enough history has
    // accumulated. A zero command is safer and deterministic in that case.
    if (!source_found) {
      output.source_linear_velocity = 0.0;
      output.source_angular_velocity = 0.0;
      output.source_time = elapsed_time;
    }
  }

  const double oldest_useful_time = elapsed_time - delay_ - 1.0;
  while (history_.size() > 2u && history_[1].time < oldest_useful_time) {
    history_.pop_front();
  }
  return output;
}

double CommandDelay::delay() const
{
  return delay_;
}

}  // namespace my_robot_controller
