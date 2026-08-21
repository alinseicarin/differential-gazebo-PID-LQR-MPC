#ifndef MY_ROBOT_CONTROLLER__MOTION_COMMAND_POLICY_HPP_
#define MY_ROBOT_CONTROLLER__MOTION_COMMAND_POLICY_HPP_

namespace my_robot_controller
{

/// Controller-independent command limits and emergency translation thresholds.
struct MotionCommandPolicyConfig
{
  /// Forward-speed limit. The current encoder plugin cannot report signed
  /// reverse velocity, so every compared controller uses the same [0, max]
  /// admissible interval.
  double maximum_linear_velocity{1.0};
  double maximum_angular_velocity{1.5};

  // These are emergency guards, not continuous performance modifiers.
  double translation_stop_lateral_error{0.75};
  double translation_stop_heading_error{1.2};
};

/// Final body command assembled from a common reference and controller output.
struct MotionCommand
{
  double linear_command{0.0};
  double angular_command{0.0};
  double linear_feedforward_command{0.0};
  double linear_feedback_command{0.0};
  double angular_feedforward_command{0.0};
  double angular_feedback_command{0.0};
  bool translation_safety_stop{false};
};

/// Add common trajectory feedforward to a controller's two feedback outputs,
/// then apply identical safety and actuator limits to PID, LQR, and MPC.
class MotionCommandPolicy
{
public:
  explicit MotionCommandPolicy(const MotionCommandPolicyConfig & config = {});

  void configure(const MotionCommandPolicyConfig & config);

  MotionCommand calculate(
    double reference_linear_velocity,
    double reference_angular_velocity,
    double lateral_error,
    double heading_error,
    double linear_feedback_command,
    double angular_feedback_command) const;

private:
  void validate_config(const MotionCommandPolicyConfig & config) const;

  MotionCommandPolicyConfig config_;
};

}  // namespace my_robot_controller

#endif  // MY_ROBOT_CONTROLLER__MOTION_COMMAND_POLICY_HPP_
