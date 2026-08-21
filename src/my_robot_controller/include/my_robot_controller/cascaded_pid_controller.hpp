#ifndef MY_ROBOT_CONTROLLER__CASCADED_PID_CONTROLLER_HPP_
#define MY_ROBOT_CONTROLLER__CASCADED_PID_CONTROLLER_HPP_

#include "my_robot_controller/pid_controller.hpp"

namespace my_robot_controller
{

/// Tunable parameters for the longitudinal plus cascaded lateral PID controller.
struct CascadedPidConfig
{
  // Longitudinal loop: body-frame e_x [m] -> linear-velocity feedback [m/s].
  double longitudinal_kp{0.8};
  double longitudinal_ki{0.05};
  double longitudinal_kd{0.05};
  double longitudinal_integral_limit{0.5};

  // Outer lateral loop: body-frame e_y [m] -> heading correction [rad].
  double cross_track_kp{1.5};
  double cross_track_ki{0.0};
  double cross_track_kd{0.20};
  double cross_track_integral_limit{0.5};

  // Inner loop: corrected heading error [rad] -> yaw-rate feedback [rad/s].
  double heading_kp{3.0};
  double heading_ki{0.0};
  double heading_kd{0.20};
  double heading_integral_limit{0.5};

  // The outer loop may request only a physically meaningful heading change.
  // Command limits and forward speed belong to the common motion policy.
  double maximum_heading_correction{0.7};
};

/// Values produced during one cascade update and retained in experiment logs.
struct CascadedPidOutput
{
  double longitudinal_pid_output{0.0};
  double desired_heading{0.0};
  double heading_correction{0.0};
  double lateral_pid_output{0.0};
  double heading_error{0.0};
  double heading_pid_output{0.0};
};

/// Two-input trajectory-tracking PID independent of ROS and Gazebo.
///
/// One loop maps longitudinal pose error to linear-speed feedback. The outer
/// lateral loop converts body-frame lateral error into a desired heading, and
/// the inner loop returns yaw-rate feedback. The resulting pair
/// [delta_v, delta_omega] matches the LQR and MPC controller interface.
class CascadedPidController
{
public:
  explicit CascadedPidController(const CascadedPidConfig & config = {});

  /// Validate and apply all gains/limits, then clear all three PID states.
  void configure(const CascadedPidConfig & config);

  /// Calculate both velocity corrections from the common pose-error state.
  CascadedPidOutput calculate(
    double longitudinal_error,
    double lateral_error,
    double reference_heading,
    double robot_heading,
    double dt);

  /// Clear all three PID memories after a timing gap or experiment reset.
  void reset();

private:
  void validate_config(const CascadedPidConfig & config) const;

  CascadedPidConfig config_;
  PIDController longitudinal_pid_;
  PIDController cross_track_pid_;
  PIDController heading_pid_;
};

}  // namespace my_robot_controller

#endif  // MY_ROBOT_CONTROLLER__CASCADED_PID_CONTROLLER_HPP_
