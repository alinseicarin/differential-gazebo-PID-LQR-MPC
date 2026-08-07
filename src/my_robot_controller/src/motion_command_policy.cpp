#include "my_robot_controller/motion_command_policy.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace my_robot_controller
{

MotionCommandPolicy::MotionCommandPolicy(const MotionCommandPolicyConfig & config)
{
  configure(config);
}

void MotionCommandPolicy::validate_config(const MotionCommandPolicyConfig & config) const
{
  if (!std::isfinite(config.maximum_linear_velocity) ||
    config.maximum_linear_velocity <= 0.0 ||
    !std::isfinite(config.maximum_angular_velocity) ||
    config.maximum_angular_velocity <= 0.0)
  {
    throw std::invalid_argument("Motion-command velocity limits must be finite and positive");
  }
  if (!std::isfinite(config.translation_stop_lateral_error) ||
    config.translation_stop_lateral_error <= 0.0 ||
    !std::isfinite(config.translation_stop_heading_error) ||
    config.translation_stop_heading_error <= 0.0)
  {
    throw std::invalid_argument("Translation-stop thresholds must be finite and positive");
  }
}

void MotionCommandPolicy::configure(const MotionCommandPolicyConfig & config)
{
  validate_config(config);
  config_ = config;
}

MotionCommand MotionCommandPolicy::calculate(
  double reference_linear_velocity,
  double reference_angular_velocity,
  double lateral_error,
  double heading_error,
  double linear_feedback_command,
  double angular_feedback_command) const
{
  if (!std::isfinite(reference_linear_velocity) || reference_linear_velocity < 0.0 ||
    !std::isfinite(reference_angular_velocity) || !std::isfinite(lateral_error) ||
    !std::isfinite(heading_error) || !std::isfinite(linear_feedback_command) ||
    !std::isfinite(angular_feedback_command))
  {
    throw std::invalid_argument(
            "Motion-command inputs must be finite and reference speed non-negative");
  }

  MotionCommand output;
  output.linear_feedforward_command = reference_linear_velocity;
  output.linear_feedback_command = linear_feedback_command;
  output.angular_feedforward_command = reference_angular_velocity;
  output.angular_feedback_command = angular_feedback_command;

  // The guard is shared by all controllers and is inactive during ordinary
  // trajectory tracking. At a gross deviation it suppresses reference motion
  // while retaining angular feedback so the robot can realign in place.
  output.translation_safety_stop =
    std::abs(lateral_error) >= config_.translation_stop_lateral_error ||
    std::abs(heading_error) >= config_.translation_stop_heading_error;

  if (!output.translation_safety_stop) {
    output.linear_command = std::clamp(
      output.linear_feedforward_command + output.linear_feedback_command,
      0.0, config_.maximum_linear_velocity);
    output.angular_command = std::clamp(
      output.angular_feedforward_command + output.angular_feedback_command,
      -config_.maximum_angular_velocity,
      config_.maximum_angular_velocity);
  } else {
    output.linear_feedforward_command = 0.0;
    output.angular_feedforward_command = 0.0;
    output.angular_command = std::clamp(
      output.angular_feedback_command,
      -config_.maximum_angular_velocity,
      config_.maximum_angular_velocity);
  }

  return output;
}

}  // namespace my_robot_controller
