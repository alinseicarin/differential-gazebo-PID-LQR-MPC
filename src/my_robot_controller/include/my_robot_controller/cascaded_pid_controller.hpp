#ifndef MY_ROBOT_CONTROLLER__CASCADED_PID_CONTROLLER_HPP_
#define MY_ROBOT_CONTROLLER__CASCADED_PID_CONTROLLER_HPP_

#include "my_robot_controller/pid_controller.hpp"

namespace my_robot_controller
{

/// Tunable parameters for the two-loop path-error PID controller.
struct CascadedPidConfig
{
  // Outer loop: signed cross-track error [m] -> heading correction [rad].
  double cross_track_kp{1.2};
  double cross_track_ki{0.0};
  double cross_track_kd{0.15};
  double cross_track_integral_limit{0.5};

  // Inner loop: corrected heading error [rad] -> yaw-rate feedback [rad/s].
  double heading_kp{2.5};
  double heading_ki{0.0};
  double heading_kd{0.15};
  double heading_integral_limit{0.5};

  // Safety and actuator limits used after both PID calculations.
  double maximum_heading_correction{0.7};
  double cross_track_speed_gain{1.5};
  double maximum_linear_velocity{1.0};
  double maximum_angular_velocity{1.5};
};

/// Values produced during one cascade update and retained in experiment logs.
struct CascadedPidOutput
{
  double linear_command{0.0};
  double angular_command{0.0};
  double desired_heading{0.0};
  double heading_correction{0.0};
  double cross_track_pid_output{0.0};
  double heading_error{0.0};
  double heading_pid_output{0.0};
  double heading_speed_factor{0.0};
  double cross_track_speed_factor{0.0};
};

/// Cascaded path-following PID independent of ROS and Gazebo.
///
/// The outer loop converts signed lateral displacement into a desired heading.
/// The inner loop regulates that desired heading and adds the yaw-rate
/// feedforward supplied by the common path-reference manager. Forward speed is
/// the common reference speed reduced when the robot is badly displaced or
/// misaligned, so a spun robot turns back toward the path before translating.
class CascadedPidController
{
public:
  explicit CascadedPidController(const CascadedPidConfig & config = {});

  /// Validate and apply all gains/limits, then clear both loop states.
  void configure(const CascadedPidConfig & config);

  /// Calculate one pair of differential-drive body commands.
  CascadedPidOutput calculate(
    double cross_track_error,
    double path_heading,
    double robot_heading,
    double reference_linear_velocity,
    double reference_angular_velocity,
    double dt);

  /// Clear both PID memories after a timing gap or experiment reset.
  void reset();

private:
  void validate_config(const CascadedPidConfig & config) const;

  CascadedPidConfig config_;
  PIDController cross_track_pid_;
  PIDController heading_pid_;
};

}  // namespace my_robot_controller

#endif  // MY_ROBOT_CONTROLLER__CASCADED_PID_CONTROLLER_HPP_
