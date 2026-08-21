#include "my_robot_controller/wheel_slip_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace my_robot_controller
{
namespace
{

// Normalize a velocity mismatch without dividing by a value close to zero.
// The floor keeps the diagnostic finite while the robot starts and stops.
double relative_discrepancy(double expected, double measured, double floor)
{
  const double denominator = std::max({std::abs(expected), std::abs(measured), floor});
  return (expected - measured) / denominator;
}

}  // namespace

WheelSlipResult calculate_wheel_slip_metrics(
  const WheelSlipInput & input, const WheelSlipConfig & config)
{
  if (!std::isfinite(config.wheel_radius) || config.wheel_radius <= 0.0 ||
    !std::isfinite(config.wheel_separation) || config.wheel_separation <= 0.0 ||
    !std::isfinite(config.minimum_speed_denominator) ||
    config.minimum_speed_denominator <= 0.0)
  {
    throw std::invalid_argument("Wheel-slip geometry and speed floor must be finite and positive");
  }
  if (!std::isfinite(input.left_wheel_angular_velocity) ||
    !std::isfinite(input.right_wheel_angular_velocity) ||
    !std::isfinite(input.truth_world_linear_x) ||
    !std::isfinite(input.truth_world_linear_y) ||
    !std::isfinite(input.truth_yaw) ||
    !std::isfinite(input.truth_angular_velocity))
  {
    throw std::invalid_argument("Wheel-slip inputs must be finite");
  }

  WheelSlipResult result;
  // Ideal rolling converts wheel angular speeds to tangential speeds. Their
  // mean and difference are the differential-drive predictions for chassis
  // longitudinal velocity and yaw rate, respectively.
  result.left_wheel_tangential_velocity =
    config.wheel_radius * input.left_wheel_angular_velocity;
  result.right_wheel_tangential_velocity =
    config.wheel_radius * input.right_wheel_angular_velocity;
  result.wheel_kinematic_linear_velocity = 0.5 * (
    result.left_wheel_tangential_velocity + result.right_wheel_tangential_velocity);
  result.wheel_kinematic_angular_velocity =
    (result.right_wheel_tangential_velocity - result.left_wheel_tangential_velocity) /
    config.wheel_separation;

  const double cosine = std::cos(input.truth_yaw);
  const double sine = std::sin(input.truth_yaw);
  // Rotate Gazebo world-frame velocity into the robot body frame. This yields
  // physically comparable longitudinal/lateral speeds at the chassis centre.
  result.truth_body_longitudinal_velocity =
    cosine * input.truth_world_linear_x + sine * input.truth_world_linear_y;
  result.truth_body_lateral_velocity =
    -sine * input.truth_world_linear_x + cosine * input.truth_world_linear_y;

  // A rigid body rotating about its centre has different longitudinal speeds
  // at the two wheel centres. Comparing against those local speeds prevents a
  // normal turn from being misclassified as wheel slip.
  const double truth_left_wheel_velocity =
    result.truth_body_longitudinal_velocity -
    0.5 * config.wheel_separation * input.truth_angular_velocity;
  const double truth_right_wheel_velocity =
    result.truth_body_longitudinal_velocity +
    0.5 * config.wheel_separation * input.truth_angular_velocity;

  // Positive ratios mean the wheel-based prediction exceeds measured ground
  // motion under this expected-minus-measured convention. These are evaluation
  // diagnostics only and never feed back into a controller.
  result.left_longitudinal_slip_ratio = relative_discrepancy(
    result.left_wheel_tangential_velocity, truth_left_wheel_velocity,
    config.minimum_speed_denominator);
  result.right_longitudinal_slip_ratio = relative_discrepancy(
    result.right_wheel_tangential_velocity, truth_right_wheel_velocity,
    config.minimum_speed_denominator);
  result.center_longitudinal_slip_ratio = relative_discrepancy(
    result.wheel_kinematic_linear_velocity, result.truth_body_longitudinal_velocity,
    config.minimum_speed_denominator);
  result.yaw_velocity_discrepancy_ratio = relative_discrepancy(
    result.wheel_kinematic_angular_velocity, input.truth_angular_velocity,
    config.minimum_speed_denominator);
  result.sideslip_angle = std::atan2(
    result.truth_body_lateral_velocity, result.truth_body_longitudinal_velocity);
  return result;
}

}  // namespace my_robot_controller
